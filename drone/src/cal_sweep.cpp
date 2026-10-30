#include "cal_sweep.h"

#include <algorithm>

namespace mabur {

namespace {
// Verify has no SweepCmd of its own to copy pacing from (arm_verify is
// called directly by the owner, not over the wire) -- it runs at the same
// pacing the protocol's own SweepCmd defaults use.
constexpr uint16_t kVerifyFramesPerCell = 20;
constexpr uint32_t kVerifySettleMs = 5;
constexpr uint32_t kVerifyGapUs = 1000;
}  // namespace

uint8_t CalSweep::stamp_flags() const {
  return cal::kFlagStbc | (cfg_.ldpc ? cal::kFlagLdpc : 0);
}

void CalSweep::build_cells(const cal::SweepCmd& c) {
  cells_.clear();
  if (c.kind == cal::Kind::Capacity) {
    for (int mcs = 0; mcs < 8; ++mcs)
      if (c.mcs_mask & (1 << mcs)) cells_.push_back(Cell{static_cast<uint8_t>(mcs), 0});
    return;
  }
  for (int mcs = 0; mcs < 8; ++mcs) {
    if (!(c.mcs_mask & (1 << mcs))) continue;
    for (int idx = c.idx_lo; idx <= c.idx_hi; ++idx)
      cells_.push_back(Cell{static_cast<uint8_t>(mcs), idx});
  }
}

void CalSweep::build_verify_cells(const std::array<int, 8>& park_rel) {
  cells_.clear();
  for (int mcs = 0; mcs < 8; ++mcs)
    if (park_rel[mcs] != cal::kWallLeave)
      cells_.push_back(Cell{static_cast<uint8_t>(mcs), park_rel[mcs]});
}

uint16_t CalSweep::remaining_start_ms(uint64_t now_ms) const {
  if (sweep_start_ms_ <= now_ms) return 0;
  const uint64_t remaining = sweep_start_ms_ - now_ms;
  return static_cast<uint16_t>(remaining > 65535 ? 65535 : remaining);
}

bool CalSweep::starts_sweep(const cal::SweepCmd& c) const {
  // The same three branches on_cmd() takes, without the side effects.
  if (!has_session_ || c.nonce != nonce_)
    return !(last_nonce_valid_ && c.nonce == last_closed_nonce_);
  return static_cast<int>(c.seq) > last_started_seq_;
}

void CalSweep::on_cmd(const cal::SweepCmd& c, uint64_t now_ms, PowerCtl& pwr) {
  const bool new_session = !has_session_ || c.nonce != nonce_;
  // Any SWEEP of the open run shows the GS has moved past the last
  // SweepDone (take_done()), whatever on_cmd does with it below.
  if (!new_session) repeat_done_.reset();
  // A closed run (hard cap, abort, or await timeout) never reopens: a
  // straggler or a GS resend of that exact nonce arriving after the
  // close is ignored outright, with no ack and no PowerCtl touched, so
  // hard_cap_ms keeps bounding the run it was armed for and an abort
  // cannot be undone by a late retransmission.
  if (new_session && last_nonce_valid_ && c.nonce == last_closed_nonce_) return;
  if (new_session) {
    // A brand-new nonce arriving while a run is still live -- any second
    // `maburcal start` after an abort, after a GS restart, or within
    // await_next_ms of the previous run's last frame -- finds the TXAGC
    // override still parked at the PREVIOUS run's last swept cell (only
    // close_session() restores it). Undo that park before reading the
    // anchor, so this read sees the same clean reference a genuinely
    // first run would.
    if (has_session_) pwr.set_index_override(anchor_idx_);
    // Zero BEFORE reading, whatever the kind: with a custom diff table live
    // the chip reports reference + diff, not the reference, and every
    // Power/Verify cell of this run is placed relative to what this read
    // returns. A capacity-first run's owner programs its width's plan back
    // after accepting the command.
    pwr.zero_rate_diffs();
    const int anchor = pwr.read_anchor_idx();
    if (anchor < 0) {
      // Cannot place a relative cell without the reference: refuse this
      // command, answering with an explicit ack rather than staying
      // silent. Full teardown (not just has_session_ = false), so
      // pump()'s switch(state_) never keeps dispatching into a stale
      // Sweeping state for cells nobody owns any more.
      has_session_ = false;
      state_ = State::Idle;
      cursor_ = 0;
      cell_entered_ = false;
      cells_.clear();
      pending_done_.reset();
      repeat_done_.reset();
      pending_apply_.reset();
      pending_ack_ = cal::SweepAck{c.nonce, c.seq, cal::AckResult::AnchorUnreadable, 200,
                                   stamp_flags()};
      // zero_rate_diffs() above already flattened the table and nothing
      // here undoes it -- this class's PowerCtl has no concept of the
      // operating plan, so only the caller can put the real table back.
      pending_refusal_ = true;
      return;
    }
    anchor_idx_ = anchor;
    nonce_ = c.nonce;
    has_session_ = true;
    hard_cap_deadline_ms_ = now_ms + cfg_.hard_cap_ms;
    last_started_seq_ = -1;
    // Per-run state from any earlier run must not leak into this one.
    power_restored_ = false;
    handed_out_apply_ = false;
  } else {
    if (static_cast<int>(c.seq) == last_started_seq_) {
      // Exact repeat of the sweep already running: re-arm the ack every
      // time (the GS resends into a 30-50%-lossy uplink, and a single
      // lost ack must not cost the whole sweep), but touch nothing else
      // -- the cell cursor is left exactly as it is. start_in_ms reports
      // what's left of the ORIGINAL 200 ms start delay, not a fresh one.
      pending_ack_ = cal::SweepAck{nonce_, c.seq, cal::AckResult::Accepted,
                                   remaining_start_ms(now_ms), stamp_flags()};
      return;
    }
    if (static_cast<int>(c.seq) < last_started_seq_) {
      // Stale duplicate of an EARLIER sweep arriving late: ignored
      // outright, no ack -- the GS is not waiting on this seq any more.
      return;
    }
    // Seq strictly ahead: a new sweep within the same run, reusing the
    // anchor already captured above.
    if (c.kind == cal::Kind::Power) pwr.zero_rate_diffs();
  }

  last_started_seq_ = c.seq;
  current_kind_ = c.kind;
  current_bw_ = c.bw;
  frames_per_cell_ = c.frames_per_cell;
  settle_ms_ = c.settle_ms;
  gap_ms_ = c.gap_us / 1000;
  dwell_ms_ = c.dwell_ms;
  offered_.fill(0);
  blocked_pct_.fill(0);
  build_cells(c);
  cursor_ = 0;
  cell_entered_ = false;
  state_ = State::Sweeping;
  pending_done_.reset();
  repeat_done_.reset();
  // This sweep supersedes any ApplyCmd still queued from an earlier
  // sweep of this run -- take_pending_apply() must not hand out a result
  // for a sweep that just got superseded.
  pending_apply_.reset();
  // Capacity flies the production power plan the owner programs for
  // this width -- restore the override to the anchor in case a
  // Power/Verify sweep earlier in this run left it parked at a swept
  // cell. Power/Verify instead program the override per cell, inside
  // pump().
  if (c.kind == cal::Kind::Capacity) pwr.set_index_override(anchor_idx_);
  // The first cell does not open until 200 ms from now (pump_power_verify
  // / pump_capacity check this): the ack needs a chance to reach the GS
  // first.
  sweep_start_ms_ = now_ms + 200;
  pending_ack_ = cal::SweepAck{nonce_, c.seq, cal::AckResult::Accepted,
                               remaining_start_ms(now_ms), stamp_flags()};
}

void CalSweep::on_apply(const cal::ApplyCmd& a, uint64_t now_ms) {
  if (!has_session_ || a.nonce != nonce_) return;
  // The GS analysed the sweep: its SweepDone need not repeat.
  repeat_done_.reset();
  // A sweep actively running owns the radio right now; accepting an
  // apply here would silently abandon its progress. Verify only ever
  // follows a drained apply, never races one.
  if (state_ == State::Sweeping) return;
  // The GS resends APPLY every 200 ms until it hears APPLIED. A repeat
  // of a (nonce, seq) already handed out by take_pending_apply() must be
  // a no-op -- not a re-queue, which would restart Applying and disturb
  // a verify sweep the owner already armed from the first delivery. Keyed
  // on the sweep's seq, not the width: a run applies a power sweep and
  // then a capacity sweep at the same width, and the second is new.
  if (handed_out_apply_ && a.nonce == handed_out_nonce_ && a.seq == handed_out_seq_) return;
  pending_apply_ = a;
  state_ = State::Applying;
  // The wait-for-next-event window starts now, at acceptance -- not
  // inherited from whenever the last sweep happened to finish.
  await_deadline_ms_ = now_ms + cfg_.await_next_ms;
}

void CalSweep::on_abort(const cal::Abort& a, uint64_t now_ms, PowerCtl& pwr) {
  (void)now_ms;
  if (has_session_ && a.nonce == nonce_) close_session(pwr);
}

std::optional<cal::SweepAck> CalSweep::take_ack() {
  if (!pending_ack_.has_value()) return std::nullopt;
  auto a = *pending_ack_;
  pending_ack_.reset();
  return a;
}

std::optional<cal::SweepDone> CalSweep::take_done() {
  if (!pending_done_.has_value()) return std::nullopt;
  auto d = *pending_done_;
  pending_done_.reset();
  return d;
}

std::optional<cal::ApplyCmd> CalSweep::take_pending_apply() {
  if (!pending_apply_.has_value()) return std::nullopt;
  auto a = *pending_apply_;
  pending_apply_.reset();
  // Remember this (nonce, seq) as handed out so a GS resend of the same
  // apply -- it keeps resending until it hears APPLIED -- is ignored by
  // on_apply() rather than re-queued over the verify sweep this apply is
  // about to trigger.
  handed_out_apply_ = true;
  handed_out_nonce_ = a.nonce;
  handed_out_seq_ = a.seq;
  if (state_ == State::Applying) state_ = State::Idle;
  return a;
}

bool CalSweep::take_refused() {
  const bool v = pending_refusal_;
  pending_refusal_ = false;
  return v;
}

void CalSweep::arm_verify(const std::array<int, 8>& park_rel, uint8_t bw, uint64_t now_ms) {
  // No run to verify, or a Power/Capacity sweep is actively Sweeping and
  // owns the radio right now -- never interrupt it. Without this guard,
  // calling arm_verify() with no open run would build a cell plan and
  // flip state_ to Sweeping with has_session_ still false: pump()'s
  // hard-cap/await guards are gated on has_session_, so nothing would
  // ever close this "run" or restore power again.
  if (!has_session_ || state_ == State::Sweeping) return;
  current_kind_ = cal::Kind::Verify;
  current_bw_ = bw;
  frames_per_cell_ = kVerifyFramesPerCell;
  settle_ms_ = kVerifySettleMs;
  gap_ms_ = kVerifyGapUs / 1000;
  offered_.fill(0);
  blocked_pct_.fill(0);
  build_verify_cells(park_rel);
  cursor_ = 0;
  cell_entered_ = false;
  pending_done_.reset();
  repeat_done_.reset();
  // Same 200 ms start delay as an on_cmd sweep -- see that method.
  sweep_start_ms_ = now_ms + 200;
  state_ = State::Sweeping;
}

void CalSweep::pump(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr, CapacityCtl& cap) {
  // Constraint 2, checked before anything else: losing the link mid-run
  // is the expected case, not an error, so a GS that never sends another
  // byte must never leave the drone parked mid-sweep.
  if (has_session_ && now_ms >= hard_cap_deadline_ms_) {
    close_session(pwr);
    return;
  }

  switch (state_) {
    case State::Idle:
      if (has_session_ && now_ms >= await_deadline_ms_) {
        close_session(pwr);
        return;
      }
      if (repeat_done_ && now_ms >= next_done_ms_) {
        pending_done_ = repeat_done_;
        // Deadline-based like the frame pacing, but a late pump re-arms
        // once rather than in a burst.
        next_done_ms_ += kDoneRepeatMs;
        if (next_done_ms_ <= now_ms) next_done_ms_ = now_ms + kDoneRepeatMs;
      }
      return;
    case State::Applying:
      // Waiting on the caller to drain take_pending_apply() and either
      // call arm_verify() or let the next command land. Bounded by both
      // the shared hard cap above and this state's own await deadline --
      // a GS that stops talking mid-apply must not leave the drone
      // stuck here forever either.
      if (has_session_ && now_ms >= await_deadline_ms_) close_session(pwr);
      return;
    case State::Sweeping:
      if (current_kind_ == cal::Kind::Capacity) {
        pump_capacity(now_ms, cap);
      } else {
        pump_power_verify(now_ms, tx, pwr);
      }
      return;
  }
}

void CalSweep::pump_power_verify(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr) {
  // The ack (or, for Verify, the owner's applied config) needs a chance
  // to reach the GS before the first cell starts using up the air.
  if (now_ms < sweep_start_ms_) return;
  if (!cell_entered_) {
    if (cursor_ >= cells_.size()) {
      finish_sweep(now_ms);
      return;
    }
    if (frames_per_cell_ == 0) {
      // Nothing to attribute to this cell -- step over it rather than
      // parking the radio there to send a frame nobody asked for.
      ++cursor_;
      return;
    }
    const Cell cell = cells_[cursor_];
    active_mcs_ = cell.mcs;
    active_idx_ = cell.idx;
    // Every frame is stamped with the RELATIVE cell it's sent in, so the
    // ladder and the TXAGC index are both pinned to this cell before the
    // first frame goes out -- programmed as the anchor plus the relative
    // offset, clamped to the chip's real [0, 127] index range.
    pwr.set_index_override(std::clamp(anchor_idx_ + active_idx_, 0, 127));
    tx.set_ladder({rc::LayerTxSpec{rc::PhyMode::HT, active_mcs_, current_bw_, false, cfg_.ldpc,
                                   true},
                   rc::LayerTxSpec{rc::PhyMode::HT, active_mcs_, current_bw_, false, cfg_.ldpc,
                                   true}},
                  std::nullopt);
    cell_seq_ = 0;
    settle_deadline_ms_ = now_ms + settle_ms_;
    next_send_ms_ = settle_deadline_ms_;  // first frame may fire as soon as settled
    cell_entered_ = true;
    return;
  }

  if (now_ms < settle_deadline_ms_ || now_ms < next_send_ms_) return;

  const auto payload = cal::build_cal_payload(current_kind_, active_mcs_,
                                               static_cast<int8_t>(active_idx_), current_bw_,
                                               stamp_flags(), cell_seq_);
  tx.send_body(0, payload.data(), payload.size());
  ++cell_seq_;
  if (current_kind_ == cal::Kind::Power) ++offered_[active_mcs_];
  // DEADLINE-based, not now_ms-based: the next frame is due gap_ms_ after
  // the one that was DUE, not after the one that was SENT, so a late
  // wake absorbs itself instead of pushing every later frame later too.
  next_send_ms_ += gap_ms_;

  if (cell_seq_ >= frames_per_cell_) {
    ++cursor_;
    cell_entered_ = false;
  }
}

void CalSweep::pump_capacity(uint64_t now_ms, CapacityCtl& cap) {
  // Same start delay as Power/Verify -- see pump_power_verify().
  if (now_ms < sweep_start_ms_) return;
  if (!cell_entered_) {
    if (cursor_ >= cells_.size()) {
      finish_sweep(now_ms);
      return;
    }
    const Cell cell = cells_[cursor_];
    active_mcs_ = cell.mcs;
    cap.set_point(active_mcs_, current_bw_);
    settle_deadline_ms_ = now_ms + settle_ms_;
    cell_entered_ = true;
    return;
  }

  if (now_ms < settle_deadline_ms_) return;

  const auto payload = cal::build_cal_payload(cal::Kind::Capacity, active_mcs_, /*idx=*/0,
                                               current_bw_, stamp_flags(), /*seq=*/0,
                                               cfg_.capacity_frame_len);
  // Blocks for the dwell by design -- see the class comment.
  const BlastResult r = cap.blast(payload.data(), payload.size(), dwell_ms_);
  offered_[active_mcs_] = r.offered;
  blocked_pct_[active_mcs_] = r.blocked_pct;
  ++cursor_;
  cell_entered_ = false;
}

void CalSweep::finish_sweep(uint64_t now_ms) {
  // Verify emits no SweepDone -- the GS times its own verify window.
  if (current_kind_ != cal::Kind::Verify) {
    pending_done_ = cal::SweepDone{nonce_, static_cast<uint8_t>(last_started_seq_), offered_,
                                   blocked_pct_};
    repeat_done_ = pending_done_;
    next_done_ms_ = now_ms + kDoneRepeatMs;
  }
  // The run stays open -- Capacity can follow Power, and a result's
  // verify can follow an apply -- so this is Idle-but-active, not a full
  // close: only the await timer (or the hard cap) closes the run.
  state_ = State::Idle;
  cell_entered_ = false;
  cursor_ = 0;
  await_deadline_ms_ = now_ms + cfg_.await_next_ms;
}

void CalSweep::close_session(PowerCtl& pwr) {
  // NOTE this restores a FLAT INDEX OVERRIDE parked at the anchor, not
  // "no override at all" -- the caller's falling-edge handler is what
  // actually clears the override and re-applies the real operating plan
  // on top, because this class's narrow PowerCtl interface has no
  // concept of "the operating plan".
  if (has_session_) {
    pwr.set_index_override(anchor_idx_);
    power_restored_ = true;
    // Remember this nonce as closed so a straggler or a GS resend of it
    // can never reopen a run (see on_cmd()).
    last_closed_nonce_ = nonce_;
    last_nonce_valid_ = true;
  }
  has_session_ = false;
  state_ = State::Idle;
  cursor_ = 0;
  cell_entered_ = false;
  pending_done_.reset();
  repeat_done_.reset();
  pending_apply_.reset();
  handed_out_apply_ = false;
}

}  // namespace mabur
