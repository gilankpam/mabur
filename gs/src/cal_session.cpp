#include "cal_session.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <string>
#include <tuple>
#include <variant>

#include "cal_log.h"
#include "cal_picker.h"
#include "cal_plan.h"
#include "mabur/cal_park.h"

namespace maburgs {
namespace cal = mabur::cal;

namespace {

// How often an unanswered SweepCmd or ApplyCmd is offered again. The uplink
// loses a large share of frames, so one send is not enough; both are
// idempotent on the drone by (nonce, seq), so a repeat costs nothing there.
constexpr uint32_t kResendMs = 200;

int median(std::vector<int> v) {
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

const char* ack_word(cal::AckResult r) {
  switch (r) {
    case cal::AckResult::Accepted: return "accepted";
    case cal::AckResult::AnchorUnreadable: return "anchor_unreadable";
    case cal::AckResult::Busy: return "busy";
    case cal::AckResult::Unsupported: return "unsupported";
  }
  return "unknown";
}

// Power before capacity within a width, so a capacity sweep flies at the
// power its own width's walls were just parked at.
int kind_order(cal::Kind k) { return k == cal::Kind::Power ? 0 : 1; }

}  // namespace

bool CalSession::start(uint32_t nonce, std::vector<cal::SweepSpec> sweeps, uint64_t now_ms,
                       std::string* err) {
  // Picking is a run too: a second start would fight the first one's pick.
  if (running() || state_ == State::Picking) {
    if (err) *err = "a calibration session is already running";
    return false;
  }
  if (!linked_) {
    if (err) *err = "refused: link is down";
    return false;
  }
  if (!cal_capable_) {
    if (err) *err = "refused: peer does not advertise CAP_CALIBRATE";
    return false;
  }
  if (sweeps.empty()) {
    if (err) *err = "refused: no sweeps";
    return false;
  }
  for (const auto& s : sweeps) {
    if (s.kind != cal::Kind::Power && s.kind != cal::Kind::Capacity) {
      if (err) *err = "refused: a sweep is power or capacity";
      return false;
    }
    if (s.bw != 20 && s.bw != 40) {
      if (err) *err = "refused: a sweep's width is 20 or 40";
      return false;
    }
  }
  auto key = [](const cal::SweepSpec& s) { return std::make_tuple(s.bw, kind_order(s.kind)); };
  std::sort(sweeps.begin(), sweeps.end(),
            [&](const cal::SweepSpec& a, const cal::SweepSpec& b) { return key(a) < key(b); });
  sweeps.erase(std::unique(sweeps.begin(), sweeps.end(),
                           [&](const cal::SweepSpec& a, const cal::SweepSpec& b) {
                             return key(a) == key(b);
                           }),
               sweeps.end());

  nonce_ = nonce;
  sweeps_ = std::move(sweeps);
  sweep_i_ = 0;
  fail_reason_.clear();
  run_logged_ = false;
  reached_drone_ = false;
  stamp_mismatch_ = 0;
  cell_i_ = 0;
  ack_flags_ = 0;
  done_.reset();
  width_ = {};
  reloc_ordered_ = false;
  // pending_abort_ is deliberately kept: an Abort for the previous run
  // that has not gone out yet still must.

  channel_ = picker_ ? picker_->op() : 0;
  if (picker_ == nullptr || picker_->mode() != CalPicker::Mode::Auto) {
    pick_ = (picker_ && picker_->mode() == CalPicker::Mode::OneCard) ? "onecard" : "pinned";
    pick_note_ = pick_;
    begin_sweep(0, now_ms);
    return true;
  }
  pick_ = "auto";
  pick_note_ = "measuring";
  pick_start_ms_ = now_ms;
  state_ = State::Picking;
  picker_->begin_measure(now_ms);
  return true;
}

void CalSession::on_mav(const cal::CalMsg& m, uint64_t now_ms) {
  step(now_ms);
  if (const auto* a = std::get_if<cal::SweepAck>(&m)) {
    if (state_ != State::AwaitAck || a->nonce != nonce_ || a->seq != cmd_.seq) return;
    if (a->result != cal::AckResult::Accepted) {
      fail(std::string("sweep refused by the drone: ") + ack_word(a->result));
      return;
    }
    on_accepted(a->flags, a->start_in_ms, now_ms);
  } else if (const auto* d = std::get_if<cal::SweepDone>(&m)) {
    if (state_ != State::Sweep && state_ != State::AwaitDone) return;
    if (d->nonce != nonce_ || d->seq != cmd_.seq) return;
    done_ = *d;
    finish_sweep(now_ms);
  } else if (const auto* ap = std::get_if<cal::Applied>(&m)) {
    // Matched on the sweep's seq as well as the width: a run applies twice
    // at one width (power, then capacity), and the drone keeps answering
    // repeats of the first from its cache.
    if (state_ != State::Apply || ap->nonce != nonce_ || ap->seq != cmd_.seq ||
        ap->bw != cmd_.bw)
      return;
    on_applied(*ap, now_ms);
  }
}

std::optional<cal::CalMsg> CalSession::due_mav(uint64_t now_ms) {
  step(now_ms);
  if (pending_abort_) {
    const cal::Abort a = *pending_abort_;
    pending_abort_.reset();
    return cal::CalMsg{a};
  }
  if (state_ != State::AwaitAck && state_ != State::Apply) return std::nullopt;
  if (sent_once_ && now_ms - last_sent_ms_ < kResendMs) return std::nullopt;
  sent_once_ = true;
  last_sent_ms_ = now_ms;
  if (state_ == State::AwaitAck) {
    reached_drone_ = true;
    return cal::CalMsg{cmd_};
  }
  return cal::CalMsg{apply_};
}

void CalSession::on_cal_frame(int card, const cal::CalFrameInfo& f, int rssi_dbm, bool crc_ok,
                              size_t air_bytes, uint64_t now_ms) {
  step(now_ms);
  if (card < 0 || card > 1 || f.rate > 7) return;
  if (state_ == State::AwaitAck) {
    // The ack rides a lossy uplink; a frame of the commanded sweep proves
    // what it would have -- the drone accepted this sweep and is airing it
    // -- and its stamp carries the flags the ack would have announced.
    // Without this the GS would keep sending SweepCmd repeats into the
    // live sweep and drop every frame of it. Only a CRC-clean frame: a
    // corrupted one still parses with a flipped LDPC/STBC bit, and the
    // flags it would adopt filter every frame of the sweep.
    if (f.kind != cmd_.kind || f.bw != cmd_.bw) {
      ++stamp_mismatch_;
      return;
    }
    if (!crc_ok) return;
    on_accepted(f.flags, 0, now_ms);
  }
  if (state_ == State::Sweep || state_ == State::AwaitDone) {
    if (f.kind != cmd_.kind || f.bw != cmd_.bw || f.flags != ack_flags_) {
      ++stamp_mismatch_;
      return;
    }
    if (cmd_.kind == cal::Kind::Power)
      tally_power(card, f, rssi_dbm, crc_ok);
    else
      tally_capacity(card, f, rssi_dbm, crc_ok, air_bytes);
  } else if (state_ == State::Verify) {
    if (f.kind != cal::Kind::Verify || f.bw != cmd_.bw || f.flags != ack_flags_) {
      ++stamp_mismatch_;
      return;
    }
    tally_verify(card, f, crc_ok);
  }
}

bool CalSession::radio_silent(uint64_t now_ms) const {
  if (state_ == State::Sweep || state_ == State::AwaitDone || state_ == State::Verify)
    return now_ms < window_end_ms_;
  return false;
}

bool CalSession::running() const {
  switch (state_) {
    case State::AwaitAck:
    case State::Sweep:
    case State::AwaitDone:
    case State::Analyze:
    case State::Apply:
    case State::Verify:
      return true;
    default:
      return false;
  }
}

void CalSession::abort(const char* why) {
  if (running() || state_ == State::Picking) {
    if (picker_) picker_->cancel();
    // Only a run that offered the drone a SweepCmd can have opened one
    // there; the drone closes it (restoring its power table) at once
    // rather than at its own await timeout.
    if (reached_drone_) pending_abort_ = cal::Abort{nonce_};
    fail_reason_ = why ? why : "aborted";
    if (log_) log_->flush();
  }
  state_ = State::Idle;
}

void CalSession::set_peer(bool linked, bool cal_capable, mabur::rc::TagCtx ctx) {
  linked_ = linked;
  cal_capable_ = cal_capable;
  peer_ctx_ = ctx;
}

CalSnapshot CalSession::snapshot() const {
  CalSnapshot s;
  s.state = state_;
  s.sweep_i = sweep_i_;
  s.sweep_n = sweeps_.size();
  if (!sweeps_.empty()) {
    s.cur = sweeps_[sweep_i_];
    if (state_ == State::Verify) {
      s.cell_n = static_cast<uint32_t>(verify_cells_.size());
    } else {
      const uint32_t rates = static_cast<uint32_t>(std::bitset<8>(cmd_.mcs_mask).count());
      s.cell_n = cmd_.kind == cal::Kind::Capacity
                     ? rates
                     : rates * static_cast<uint32_t>(cells_per_row());
    }
  }
  s.cell_i = cell_i_;
  s.channel = channel_;
  s.pick_note = pick_note_;
  s.flags = ack_flags_;
  s.stamp_mismatch = stamp_mismatch_;
  s.width = width_;
  s.fail_reason = fail_reason_;
  return s;
}

uint16_t CalSession::cell_received(uint8_t rate, int idx, int card) const {
  if (rate > 7 || card < 0 || card > 1) return 0;
  const auto& row = cells_[rate];
  const int j = idx - cmd_.idx_lo;
  if (j < 0 || static_cast<size_t>(j) >= row.size()) return 0;
  return row[static_cast<size_t>(j)].received[static_cast<size_t>(card)];
}

// --- private ---------------------------------------------------------

void CalSession::step(uint64_t now_ms) {
  switch (state_) {
    case State::Picking:
      step_picking(now_ms);
      break;
    case State::AwaitAck:
      if (now_ms - phase_start_ms_ >= cfg_.ack_timeout_ms) fail("calibration ack timeout");
      break;
    case State::Sweep:
      if (now_ms < sweep_end_ms_) break;
      state_ = State::AwaitDone;
      [[fallthrough]];
    case State::AwaitDone:
      // The drone's SweepDone normally ends this early; its loss costs at
      // most phase_slack_ms.
      if (now_ms >= window_end_ms_) finish_sweep(now_ms);
      break;
    case State::Apply:
      if (now_ms - phase_start_ms_ >= cfg_.apply_timeout_ms) fail("calibration apply timeout");
      break;
    case State::Verify:
      if (now_ms >= window_end_ms_) finish_verify(now_ms);
      break;
    default:
      break;
  }
}

void CalSession::step_picking(uint64_t now_ms) {
  const std::string op = std::to_string(picker_->op());
  if (now_ms - pick_start_ms_ >= cfg_.pick_timeout_ms) {
    picker_->cancel();
    pick_note_ = "stayed on " + op +
                 (reloc_ordered_ ? ", relocate unconfirmed" : ", measure timed out");
    finish_pick("stayed", now_ms);
    return;
  }
  if (!reloc_ordered_) {
    const auto best = picker_->measured_best();
    if (!best) return;
    if (*best == picker_->op()) {
      pick_note_ = "auto: " + op + " is the best member";
      finish_pick("auto", now_ms);
      return;
    }
    if (!picker_->order_relocate(*best)) {
      picker_->cancel();
      pick_note_ = "stayed on " + op + ", relocate refused";
      finish_pick("stayed", now_ms);
      return;
    }
    reloc_ordered_ = true;
    reloc_to_ = *best;
    pick_note_ = "relocating " + op + " -> " + std::to_string(*best);
    return;
  }
  const auto settled = picker_->relocate_settled();
  if (!settled) return;
  if (*settled) {
    pick_note_ = "auto: moved to " + std::to_string(reloc_to_);
    finish_pick("auto", now_ms);
    channel_ = reloc_to_;
    return;
  }
  picker_->cancel();
  pick_note_ = "stayed on " + op + ", relocate unconfirmed";
  finish_pick("stayed", now_ms);
}

void CalSession::finish_pick(const char* pick, uint64_t now_ms) {
  pick_ = pick;
  channel_ = picker_->op();
  begin_sweep(0, now_ms);
}

void CalSession::begin_sweep(size_t i, uint64_t now_ms) {
  sweep_i_ = i;
  const cal::SweepSpec& sp = sweeps_[i];
  const uint8_t seq = static_cast<uint8_t>(i);
  cmd_ = sp.kind == cal::Kind::Power ? make_power_sweep(nonce_, seq, sp.bw)
                                     : make_capacity_sweep(nonce_, seq, sp.bw);
  // Frozen when the run first commands the drone: the drone latches the
  // pair its first SweepCmd arrived under and checks every later cal
  // message of the run against it, while the GS keeps beaconing between
  // sweeps and may adopt a fresh pair meanwhile.
  if (i == 0) tag_ctx_ = peer_ctx_;

  for (size_t r = 0; r < 8; ++r) {
    cells_[r].clear();
    rssi_[r].clear();
  }
  if (sp.kind == cal::Kind::Power) {
    const int n = cells_per_row();
    for (size_t r = 0; r < 8; ++r) {
      cells_[r].resize(static_cast<size_t>(n));
      rssi_[r].resize(static_cast<size_t>(n));
      for (int j = 0; j < n; ++j) cells_[r][static_cast<size_t>(j)].idx = cmd_.idx_lo + j;
    }
  }
  cap_ = {};
  cap_rssi_ = {};
  done_.reset();
  cell_i_ = 0;
  phase_start_ms_ = now_ms;
  sent_once_ = false;
  state_ = State::AwaitAck;
}

void CalSession::on_accepted(uint8_t flags, uint16_t start_in_ms, uint64_t now_ms) {
  ack_flags_ = flags;
  if (picker_) channel_ = picker_->op();
  if (!run_logged_) {
    // The GS has no LDPC setting of its own: the drone's stamp flags are
    // the truth about what this run flies.
    if (log_)
      log_->run(nonce_, cfg_.margin_db, channel_, pick_, (flags & cal::kFlagLdpc) != 0);
    run_logged_ = true;
  }
  if (log_)
    log_->sweep(cmd_.seq, cmd_.kind, cmd_.bw,
                cmd_.kind == cal::Kind::Capacity ? cmd_.dwell_ms : cmd_.frames_per_cell);
  sweep_end_ms_ = now_ms + start_in_ms + sweep_duration_ms(cmd_);
  window_end_ms_ = sweep_end_ms_ + cfg_.phase_slack_ms;
  state_ = State::Sweep;
}

void CalSession::tally_power(int card, const cal::CalFrameInfo& f, int rssi_dbm, bool crc_ok) {
  if (f.idx < cmd_.idx_lo || f.idx > cmd_.idx_hi) return;  // not a commanded cell
  const size_t j = static_cast<size_t>(f.idx - cmd_.idx_lo);
  CalCell& c = cells_[f.rate][j];
  cell_i_ = static_cast<uint32_t>(f.rate * cells_per_row()) + static_cast<uint32_t>(j) + 1;
  if (!crc_ok) {
    // CRC-bad frames reach the GS (keep_corrupted): loss, never delivery.
    ++c.corrupt;
    return;
  }
  ++c.received[static_cast<size_t>(card)];
  rssi_[f.rate][j][static_cast<size_t>(card)].push_back(rssi_dbm);
}

void CalSession::tally_capacity(int card, const cal::CalFrameInfo& f, int rssi_dbm, bool crc_ok,
                                size_t air_bytes) {
  if (!(cmd_.mcs_mask & (1u << f.rate))) return;
  cell_i_ = static_cast<uint32_t>(
                std::bitset<8>(cmd_.mcs_mask & ((1u << f.rate) - 1u)).count()) +
            1;
  if (!crc_ok) return;  // delivered air bits count clean frames only
  CapCell& c = cap_[f.rate];
  const size_t k = static_cast<size_t>(card);
  ++c.frames[k];
  c.air_bits[k] += static_cast<uint64_t>(air_bytes) * 8u;
  cap_rssi_[f.rate][k].push_back(rssi_dbm);
}

void CalSession::tally_verify(int card, const cal::CalFrameInfo& f, bool crc_ok) {
  // Tallied by rate, whatever index arrives: the drone parks at its own
  // margin, and an exact-index match would read a margin disagreement as
  // a total verify failure.
  size_t pos = verify_cells_.size();
  for (size_t i = 0; i < verify_cells_.size(); ++i)
    if (verify_cells_[i].first == f.rate) pos = i;
  if (pos == verify_cells_.size()) return;  // rate not parked
  cell_i_ = static_cast<uint32_t>(pos) + 1;
  if (!verify_seen_[f.rate]) {
    verify_seen_[f.rate] = true;
    verify_seen_idx_[f.rate] = f.idx;
  }
  if (!crc_ok) return;
  ++verify_recv_[f.rate][static_cast<size_t>(card)];
}

void CalSession::finish_sweep(uint64_t now_ms) {
  state_ = State::Analyze;
  const bool apply =
      cmd_.kind == cal::Kind::Power ? analyze_power() : analyze_capacity_sweep();
  if (apply)
    begin_apply(now_ms);
  else
    next_sweep(now_ms);
}

bool CalSession::analyze_power() {
  WidthResult& w = width_result();
  const int n = cells_per_row();
  apply_ = cal::ApplyCmd{};
  apply_.nonce = nonce_;
  apply_.seq = cmd_.seq;
  apply_.bw = cmd_.bw;

  std::array<std::vector<CalCell>, 8> rows;
  for (size_t r = 0; r < 8; ++r) {
    // Expected frames per cell: what the drone says it offered on this
    // row, spread over the row's equal cells; the commanded count when the
    // SweepDone was lost. A drone that died mid-row reads as loss, not as
    // a shrinking denominator, either way.
    uint32_t expected = cmd_.frames_per_cell;
    if (done_ && n > 0 && done_->offered[r] / static_cast<uint32_t>(n) > 0)
      expected = std::min<uint32_t>(done_->offered[r] / static_cast<uint32_t>(n), 0xFFFF);
    for (size_t j = 0; j < cells_[r].size(); ++j) {
      CalCell c = cells_[r][j];
      c.expected = static_cast<uint16_t>(expected);
      for (size_t k = 0; k < 2; ++k) {
        const auto& samples = rssi_[r][j][k];
        if (!samples.empty()) {
          c.rssi_dbm[k] = median(samples);
          c.have_rssi[k] = true;
        }
      }
      rows[r].push_back(c);
    }
  }
  if (log_)
    for (size_t r = 0; r < 8; ++r)
      for (const auto& c : rows[r]) log_->cell(static_cast<uint8_t>(r), c.idx, c);

  bool any = false;
  for (size_t r = 0; r < 8; ++r) {
    const RateWall rw = analyze_rate(rows[r], cfg_.th);
    w.walls[r] = rw;
    if (log_) log_->wall(static_cast<uint8_t>(r), rw);
    // An undetermined wall is left alone in the drone's config: never
    // write a fabricated number over a real measurement.
    if (rw.wall != kNoWall) {
      apply_.walls[r] = static_cast<int8_t>(rw.wall);
      any = true;
    }
  }
  w.have_walls = true;
  apply_.has_walls = any;
  // legacy_wall_rel is one number for every width; the 20 MHz table owns
  // it whenever this run measures one.
  apply_.legacy =
      (cmd_.bw == 20 || !has_power20()) ? apply_.walls[0] : cal::kWallLeave;
  apply_wrote_walls_ = any;
  return any;
}

bool CalSession::analyze_capacity_sweep() {
  WidthResult& w = width_result();
  apply_ = cal::ApplyCmd{};
  apply_.nonce = nonce_;
  apply_.seq = cmd_.seq;
  apply_.bw = cmd_.bw;
  apply_wrote_walls_ = false;
  if (!done_) {
    // Without the SweepDone neither offered nor blocked_pct is known, so a
    // tx_bound point cannot be told apart from a real one -- and a
    // tx_bound point must never be written. Nothing is.
    w.eff_unknown = true;
    w.have_eff = false;
    w.cap = {};
    w.eff_x1000 = {};
    return false;
  }
  bool any = false;
  for (size_t r = 0; r < 8; ++r) {
    if (!(cmd_.mcs_mask & (1u << r))) continue;
    CapCell c = cap_[r];
    c.offered = done_->offered[r];
    c.blocked_pct = done_->blocked_pct[r];
    for (size_t k = 0; k < 2; ++k) {
      if (!cap_rssi_[r][k].empty()) {
        c.rssi_dbm[k] = median(cap_rssi_[r][k]);
        c.have_rssi[k] = true;
      }
    }
    const CapResult res =
        analyze_capacity(c, static_cast<uint8_t>(r), cmd_.bw, cmd_.dwell_ms, cfg_.th);
    w.cap[r] = res;
    // A tx_bound point measured the drone's CPU/USB, not the air: 0 leaves
    // the config's value alone.
    const uint16_t e = (res.flags & kCalTxBound)
                           ? 0
                           : static_cast<uint16_t>(std::lround(res.eff * 1000.0));
    apply_.eff_x1000[r] = e;
    if (e) any = true;
    if (log_) log_->cap(static_cast<uint8_t>(r), c, res);
  }
  w.eff_x1000 = apply_.eff_x1000;
  w.have_eff = true;
  w.eff_unknown = false;
  apply_.has_eff = any;
  return any;
}

void CalSession::begin_apply(uint64_t now_ms) {
  phase_start_ms_ = now_ms;
  sent_once_ = false;
  state_ = State::Apply;
}

void CalSession::on_applied(const cal::Applied& a, uint64_t now_ms) {
  WidthResult& w = width_result();
  if (log_) log_->applied(a.bw, a.ok, a.reason);
  w.applied = a.ok;
  w.apply_reason = a.reason;
  if (!a.ok) {
    fail("apply refused by the drone: " + a.reason);
    return;
  }
  if (cmd_.kind == cal::Kind::Power && apply_wrote_walls_)
    begin_verify(now_ms);
  else
    next_sweep(now_ms);
}

void CalSession::begin_verify(uint64_t now_ms) {
  WidthResult& w = width_result();
  // The same rule the drone parks its verify cells by (mabur/cal_park.h),
  // so the window below covers exactly the cells it flies.
  w.park = mabur::park_rel_for(apply_.walls, cfg_.margin_db);
  verify_cells_ = verify_cells(w.park);
  verify_recv_ = {};
  verify_seen_ = {};
  verify_seen_idx_ = {};
  cell_i_ = 0;
  window_end_ms_ = now_ms + verify_duration_ms(verify_cells_.size()) + cfg_.phase_slack_ms;
  state_ = State::Verify;
}

void CalSession::finish_verify(uint64_t now_ms) {
  WidthResult& w = width_result();
  // One V row per parked rate, heard or not: a rate dead at its own parked
  // power is exactly what this pass exists to surface, and an absent row
  // would read as "never parked". Best single card, as the wall pass.
  for (const auto& [rate, idx] : verify_cells_) {
    const int best = std::max(verify_recv_[rate][0], verify_recv_[rate][1]);
    const int pct = std::min(
        100, static_cast<int>(std::lround(100.0 * best / static_cast<double>(kVerifyFrames))));
    w.verify_pct[rate] = pct;
    if (log_) log_->verify(rate, verify_seen_[rate] ? verify_seen_idx_[rate] : idx, pct);
  }
  next_sweep(now_ms);
}

void CalSession::next_sweep(uint64_t now_ms) {
  if (sweep_i_ + 1 < sweeps_.size()) {
    begin_sweep(sweep_i_ + 1, now_ms);
    return;
  }
  state_ = State::Done;
  // Closes the drone's run now -- restoring its operating power and video
  // -- instead of after its await window; the drone's close keeps the
  // config the applies wrote.
  pending_abort_ = cal::Abort{nonce_};
  // maburcal reads cal.log the moment it sees a terminal state; the
  // writer's own 1 Hz flush would race it.
  if (log_) log_->flush();
}

void CalSession::fail(const std::string& why) {
  if (state_ == State::Picking && picker_) picker_->cancel();
  // A run the drone may have opened is closed there at once, restoring
  // its power table, rather than at its own await timeout.
  if (reached_drone_) pending_abort_ = cal::Abort{nonce_};
  fail_reason_ = why;
  state_ = State::Failed;
  if (log_) log_->flush();
}

int CalSession::cells_per_row() const { return std::max(0, cmd_.idx_hi - cmd_.idx_lo + 1); }

WidthResult& CalSession::width_result() { return width_[cmd_.bw == 40 ? 1 : 0]; }

bool CalSession::has_power20() const {
  for (const auto& s : sweeps_)
    if (s.kind == cal::Kind::Power && s.bw == 20) return true;
  return false;
}

}  // namespace maburgs
