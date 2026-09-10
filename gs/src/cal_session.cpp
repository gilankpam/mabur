#include "cal_session.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "cal_log.h"

namespace maburgs {
namespace {

// How often an unacknowledged T_CAL_CMD is re-offered into the drone's
// listen window. The uplink loses 30-50% of frames (rcf-uplink-loss), so a
// single send is not enough; T_CAL_CMD is idempotent by (nonce, phase), so
// repeating it costs nothing on the drone side.
constexpr uint32_t kCmdResendIntervalMs = 200;

int median(std::vector<int> v) {
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

}  // namespace

bool CalSession::start(uint32_t vtx_id, uint32_t nonce, uint64_t now_ms,
                       std::string* err) {
  // A session already running -- AwaitAck through Verify -- blocks a new
  // start(); Idle, Done and Failed do not, so a finished or failed run can
  // be retried without a separate reset call.
  if (state_ != State::Idle && state_ != State::Done &&
      state_ != State::Failed) {
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

  vtx_id_ = vtx_id;
  nonce_ = nonce;
  fail_reason_ = "";
  result_ready_ = false;
  coarse_walls_ = {};
  final_walls_ = {};
  pending_park_ = {};

  begin_await(make_coarse_plan(vtx_id, nonce), now_ms);
  return true;
}

void CalSession::on_ack(uint32_t nonce, int base_ref_idx, uint64_t now_ms) {
  // A stale ack (wrong nonce, or arriving after the session has moved past
  // AwaitAck -- e.g. it already timed out) must not resurrect a dead
  // session or restart a phase that is already sweeping.
  if (state_ != State::AwaitAck || nonce != nonce_) return;
  base_ref_idx_ = base_ref_idx;
  phase_start_ms_ = now_ms;
  phase_end_ms_ = now_ms + plan_duration_ms(pending_cmd_);
  state_ = State::Sweep;
}

void CalSession::on_cal_frame(int card, const mabur::cal::CalFrameInfo& f,
                              int rssi_dbm, bool crc_ok, uint64_t now_ms) {
  (void)now_ms;
  if (state_ != State::Sweep && state_ != State::Verify) return;
  // Frames whose phase is not the one currently running are ignored
  // entirely -- a drone still finishing kPhaseVerify's self-check, or a
  // late straggler from a phase this session already left, must never be
  // attributed to the phase running now.
  if (f.phase != running_phase_) return;
  if (f.rate > 7 || card < 0 || card > 1) return;

  CalCell* cp = nullptr;
  if (state_ == State::Verify) {
    // Review fix (Important 4): tally by RATE alone, accepting whatever
    // index actually arrives, rather than requiring an exact match against
    // this session's OWN predicted park index. make_verify_plan seeds
    // exactly one cell per rate, so the index carries no information the
    // rate does not already -- but the drone parks its verify cells at
    // wall - lround(drone_margin_db*4), independently of this session's
    // own cfg_.margin_db (both default 1.0, nothing enforces they match).
    // An exact-index match would read ZERO delivery on all eight rates the
    // instant the two configs disagree -- a total, silent verify failure
    // sitting right next to a correct table already written to disk.
    auto& per_rate = cells_[f.rate];
    if (per_rate.empty()) return;  // this rate had nothing to verify
    cp = &per_rate.begin()->second;
  } else {
    auto it = cells_[f.rate].find(f.idx);
    if (it == cells_[f.rate].end()) return;  // not a cell this plan named
    cp = &it->second;
  }
  CalCell& c = *cp;

  if (!crc_ok) {
    // maburgs sets rx.keep_corrupted unconditionally, so CRC-bad frames do
    // arrive. Not delivered: count it, never as received.
    ++c.corrupt;
    return;
  }
  ++c.received[static_cast<size_t>(card)];
  rssi_raw_[f.rate][f.idx][static_cast<size_t>(card)].push_back(rssi_dbm);
}

std::optional<mabur::rc::CalCmd> CalSession::due_cmd(uint64_t now_ms) {
  step(now_ms);
  if (state_ != State::AwaitAck) return std::nullopt;
  if (sent_once_ && now_ms - last_sent_ms_ < kCmdResendIntervalMs)
    return std::nullopt;
  sent_once_ = true;
  last_sent_ms_ = now_ms;
  return pending_cmd_;
}

std::optional<mabur::rc::CalResult> CalSession::due_result(uint64_t now_ms) {
  step(now_ms);
  if (state_ != State::Result || !result_ready_) return std::nullopt;
  result_ready_ = false;
  // Applying the result is what arms the drone's self-initiated verify
  // sweep (spec step 9: "the drone immediately sweeps the eight park
  // indices with no further command"). There is no ack for T_CAL_RESULT,
  // so the GS has to assume delivery happens now and stay silent for the
  // verify window exactly as it did for a commanded phase -- otherwise its
  // own uplink blanks the very sweep this whole kit exists to read cleanly.
  begin_verify(now_ms);
  return pending_result_;
}

bool CalSession::radio_silent(uint64_t now_ms) const {
  if (state_ != State::Sweep && state_ != State::Verify) return false;
  return now_ms < phase_end_ms_ + cfg_.phase_slack_ms;
}

void CalSession::abort(const char* why) {
  fail_reason_ = why ? why : "aborted";
  state_ = State::Idle;
  result_ready_ = false;
  clear_cells();
}

void CalSession::set_peer(bool linked, bool cal_capable) {
  linked_ = linked;
  cal_capable_ = cal_capable;
}

uint16_t CalSession::cell_received(uint8_t rate, uint8_t idx, int card) const {
  if (rate > 7 || card < 0 || card > 1) return 0;
  auto it = cells_[rate].find(idx);
  if (it == cells_[rate].end()) return 0;
  return it->second.received[static_cast<size_t>(card)];
}

uint16_t CalSession::cell_corrupt(uint8_t rate, uint8_t idx) const {
  if (rate > 7) return 0;
  auto it = cells_[rate].find(idx);
  if (it == cells_[rate].end()) return 0;
  return it->second.corrupt;
}

std::string CalSession::progress() const {
  const char* st = "?";
  switch (state_) {
    case State::Idle: st = "idle"; break;
    case State::AwaitAck: st = "await_ack"; break;
    case State::Sweep: st = "sweep"; break;
    case State::Analyze: st = "analyze"; break;
    case State::Result: st = "result"; break;
    case State::Verify: st = "verify"; break;
    case State::Done: st = "done"; break;
    case State::Failed: st = "failed"; break;
  }
  size_t filled = 0, planned = 0;
  for (const auto& per_rate : cells_) {
    for (const auto& [idx, c] : per_rate) {
      ++planned;
      if (c.received[0] > 0 || c.received[1] > 0 || c.corrupt > 0) ++filled;
    }
  }
  char buf[160];
  std::snprintf(buf, sizeof buf,
               "state=%s phase=%d cells=%zu/%zu window=[%llu,%llu]", st,
               running_phase_, filled, planned,
               static_cast<unsigned long long>(phase_start_ms_),
               static_cast<unsigned long long>(phase_end_ms_));
  return buf;
}

// --- private ---------------------------------------------------------

void CalSession::step(uint64_t now_ms) {
  switch (state_) {
    case State::AwaitAck:
      if (now_ms - await_start_ms_ >= cfg_.ack_timeout_ms)
        fail("calibration ack timeout");
      break;
    case State::Sweep:
      if (now_ms >= phase_end_ms_ + cfg_.phase_slack_ms) finish_phase(now_ms);
      break;
    case State::Verify:
      if (now_ms >= phase_end_ms_ + cfg_.phase_slack_ms) {
        // Verify completion: one V record per rate that actually had a
        // park index to verify (make_verify_plan skips undetermined
        // rates entirely, so cells_[r] is empty for those -- nothing to
        // log, by design, not a gap). Best single card, matching
        // analyze_rate's own convention (PA compression degrades both
        // cards' waveform together; the union would only inflate the
        // number and hide a compressed rate as "verified clean").
        if (log_) {
          for (int r = 0; r < 8; ++r) {
            const auto it = cells_[static_cast<size_t>(r)].begin();
            if (it == cells_[static_cast<size_t>(r)].end()) continue;
            const CalCell& c = it->second;
            const int best =
                std::max(c.received[0], c.received[1]);
            const int pct = c.expected > 0
                ? static_cast<int>(std::lround(
                      100.0 * static_cast<double>(best) / c.expected))
                : 0;
            log_->verify(static_cast<uint8_t>(r), c.idx, pct);
          }
        }
        state_ = State::Done;
      }
      break;
    default:
      break;
  }
}

void CalSession::seed_cells(const mabur::rc::CalCmd& cmd) {
  for (const auto& w : cmd.windows) {
    if (w.idx_step == 0) continue;
    for (int i = w.idx_lo; i <= w.idx_hi; i += w.idx_step) {
      CalCell c;
      c.idx = static_cast<uint8_t>(i);
      c.expected = cmd.frames_per_cell;
      cells_[w.rate][static_cast<uint8_t>(i)] = c;
    }
  }
}

void CalSession::clear_cells() {
  for (auto& m : cells_) m.clear();
  for (auto& m : rssi_raw_) m.clear();
}

void CalSession::begin_await(const mabur::rc::CalCmd& cmd, uint64_t now_ms) {
  pending_cmd_ = cmd;
  running_phase_ = cmd.phase;
  clear_cells();
  seed_cells(cmd);
  await_start_ms_ = now_ms;
  sent_once_ = false;
  state_ = State::AwaitAck;
}

void CalSession::begin_verify(uint64_t now_ms) {
  running_phase_ = mabur::cal::kPhaseVerify;
  clear_cells();
  const auto verify_cmd = make_verify_plan(vtx_id_, nonce_, pending_park_);
  seed_cells(verify_cmd);
  phase_start_ms_ = now_ms;
  phase_end_ms_ = now_ms + plan_duration_ms(verify_cmd);
  state_ = State::Verify;
}

std::vector<CalCell> CalSession::sorted_cells(int rate) const {
  std::vector<CalCell> out;
  out.reserve(cells_[static_cast<size_t>(rate)].size());
  for (const auto& [idx, cell] : cells_[static_cast<size_t>(rate)]) {
    CalCell c = cell;
    auto rit = rssi_raw_[static_cast<size_t>(rate)].find(idx);
    if (rit != rssi_raw_[static_cast<size_t>(rate)].end()) {
      for (int card = 0; card < 2; ++card) {
        const auto& samples = rit->second[static_cast<size_t>(card)];
        if (!samples.empty()) {
          c.rssi_dbm[static_cast<size_t>(card)] = median(samples);
          c.have_rssi[static_cast<size_t>(card)] = true;
        }
      }
    }
    out.push_back(c);
  }
  // std::map<uint8_t, ...> iterates in ascending key order already, which is
  // exactly the precondition analyze_rate documents.
  return out;
}

void CalSession::finish_phase(uint64_t now_ms) {
  state_ = State::Analyze;  // transient: always advanced past within this call

  std::array<std::vector<CalCell>, 8> snapshot;
  for (int r = 0; r < 8; ++r) snapshot[static_cast<size_t>(r)] = sorted_cells(r);

  // Raw per-cell record, BOTH phases: this is the only moment the data
  // exists to log -- begin_await()/begin_verify() clear_cells() the raw
  // tally as soon as the next phase (or verify) starts, so a coarse-only
  // log call would silently lose every fine-phase cell. running_phase_
  // still names the phase that just ended (finish_phase() runs before any
  // transition touches it).
  if (log_)
    for (int r = 0; r < 8; ++r)
      for (const auto& c : snapshot[static_cast<size_t>(r)])
        log_->cell(running_phase_, static_cast<uint8_t>(r), c.idx, c);

  if (running_phase_ == mabur::cal::kPhaseCoarse) {
    for (int r = 0; r < 8; ++r)
      coarse_walls_[static_cast<size_t>(r)] =
          analyze_rate(snapshot[static_cast<size_t>(r)], cfg_.th);

    // Logged here too, not only from finalize_result(): a two-phase run
    // refines only the rows coarse found a real dip in (see the fine-phase
    // comment below), so coarse's own numbers are the only record that
    // ever exists for the rest -- and even for a refined row, seeing what
    // coarse alone concluded is exactly the kind of provenance cal.log
    // exists to keep. maburcal's reader keys W lines by rate in a dict, so
    // a later (fine-phase) W line for the same rate simply supersedes this
    // one -- harmless, not a duplicate-data bug.
    if (log_)
      for (int r = 0; r < 8; ++r)
        log_->wall(static_cast<uint8_t>(r), coarse_walls_[static_cast<size_t>(r)]);

    const auto fine_cmd = make_fine_plan(vtx_id_, nonce_, coarse_walls_);
    if (fine_cmd.windows.empty()) {
      // No row showed a real dip (or none was determinable): no fine window
      // has anything to sharpen, so the coarse pass IS the final answer.
      final_walls_ = coarse_walls_;
      finalize_result();
    } else {
      begin_await(fine_cmd, now_ms);
    }
    return;
  }

  // running_phase_ == kPhaseFine.
  //
  // Ruling: analyze_rate's "never dipped" test (wall >= last cell) is only
  // sound on a full-range sweep. The fine phase feeds a +/-kFineHalfWidth
  // window, where a rate whose true wall sits at the window's edge would be
  // misread as no-dip and fall through to a knee computed over a truncated
  // curve -- a silently wrong parked power. So the fine pass never
  // re-derives a row's classification: it only sharpens the wall index of a
  // row coarse already showed a real dip in, and any kCalNoDip/
  // kCalUndetermined verdict out of the truncated fine window is rejected
  // in favor of the coarse-resolution wall rather than trusted.
  final_walls_ = coarse_walls_;
  for (int r = 0; r < 8; ++r) {
    const auto& cw = coarse_walls_[static_cast<size_t>(r)];
    if (cw.flags & (kCalNoDip | kCalUndetermined))
      continue;  // make_fine_plan didn't sweep this row; coarse stands.
    // Seeded (and therefore non-empty) exactly when this row was in
    // fine_cmd's windows -- the same set the flag check above already
    // narrowed to.
    const RateWall fw = analyze_rate(snapshot[static_cast<size_t>(r)], cfg_.th);
    if (fw.flags & (kCalNoDip | kCalUndetermined))
      continue;  // truncated-window misread; keep the coarse wall.

    RateWall merged = fw;
    if (std::abs(fw.wall - cw.wall) > 1) merged.flags |= kCalDrift;
    final_walls_[static_cast<size_t>(r)] = merged;
  }
  if (log_)
    for (int r = 0; r < 8; ++r)
      log_->wall(static_cast<uint8_t>(r), final_walls_[static_cast<size_t>(r)]);
  finalize_result();
}

void CalSession::finalize_result() {
  // T_CAL_RESULT carries the RAW measured wall, not a park index: the
  // drone's power_plan.h is what subtracts margin_db
  // (diff[r] = walls[r] - m - base_ref_idx), and it is drone-config-owned
  // -- nothing forces this session's cfg_.margin_db to equal it. Sending a
  // pre-subtracted value here would mean the drone adds a margin back
  // (whose value it has no way to verify against what the GS actually
  // used) before power_plan.h subtracts one again: two independently
  // configured margins in one derivation, silently wrong if they ever
  // differ, in a kit whose whole purpose is getting this table right.
  // pending_park_ below is a SEPARATE, purely local concern: the GS's own
  // verify-phase tally needs a park index too, computed with this
  // session's margin_db, but that computation never leaves this process.
  const int m = static_cast<int>(std::lround(cfg_.margin_db * 4.0));

  mabur::rc::CalResult res;
  res.vtx_id = vtx_id_;
  res.nonce = nonce_;
  uint32_t flags = 0;
  for (int r = 0; r < 8; ++r) {
    const auto& w = final_walls_[static_cast<size_t>(r)];
    flags |= w.flags;
    if (w.wall < 0) {
      res.walls[static_cast<size_t>(r)] = -1;
      pending_park_[static_cast<size_t>(r)] = -1;
    } else {
      res.walls[static_cast<size_t>(r)] = static_cast<int16_t>(w.wall);
      pending_park_[static_cast<size_t>(r)] = w.wall - m;
    }
  }
  res.legacy_wall = res.walls[0];
  res.flags = flags;

  pending_result_ = res;
  result_ready_ = true;
  state_ = State::Result;
}

void CalSession::fail(const char* why) {
  fail_reason_ = why;
  state_ = State::Failed;
}

}  // namespace maburgs
