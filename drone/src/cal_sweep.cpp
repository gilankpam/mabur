#include "cal_sweep.h"

#include "mabur/cal_wire.h"

namespace mabur {

void CalSweep::on_cmd(const rc::CalCmd& c, uint64_t now_ms) {
  const bool new_session = !has_session_ || c.nonce != nonce_;
  if (new_session) {
    // A brand-new nonce: reset every piece of session state, including the
    // hard cap, which is measured from THIS command, not from whenever the
    // drone happened to boot.
    nonce_ = c.nonce;
    has_session_ = true;
    zeroed_for_session_ = false;
    power_restored_ = false;
    result_accepted_ = false;
    hard_cap_deadline_ms_ = now_ms + cfg_.hard_cap_ms;
    last_started_phase_ = -1;
  } else if (static_cast<int>(c.phase) <= last_started_phase_) {
    // Constraint 3: idempotent by (nonce, phase), and MONOTONIC within a
    // session -- phases only ever advance (coarse < fine < verify), so a
    // phase at or behind the one already accepted can only be a stale
    // retransmission, never a legitimate next step. Rejecting <= rather
    // than == also catches a late duplicate of an EARLIER phase arriving
    // after the session has moved on (e.g. a delayed coarse retransmission
    // landing after fine has started, or after a result is already sitting
    // undrained in Applying) -- a plain == guard would treat that as a
    // fresh command and silently discard the in-progress phase's cursor
    // and any undrained CalResult.
    return;
  }

  last_started_phase_ = c.phase;
  current_phase_ = c.phase;
  build_cells(c.windows);
  cursor_ = 0;
  cell_entered_ = false;
  frames_per_cell_ = c.frames_per_cell;
  settle_ms_ = c.settle_ms;
  gap_ms_ = c.gap_us / 1000;
  state_ = State::Sweeping;
  pending_result_.reset();
}

void CalSweep::on_result(const rc::CalResult& r, uint64_t now_ms) {
  // Stale/foreign session: a result whose nonce doesn't match the one in
  // flight (or arriving with no session open at all) is ignored outright,
  // same as a repeated command -- see stale_nonce_result_is_ignored.
  if (!has_session_ || r.nonce != nonce_) return;
  // Idempotent by nonce, same spirit as on_cmd's (nonce, phase) guard: the
  // uplink can duplicate a result frame exactly as it duplicates commands,
  // and a second acceptance means a second trip through the apply path --
  // a second /etc/mabur.toml flash write. cal_apply.h's whole design is
  // one write per session (a past bug wore out flash writing config on
  // every bitrate change), so a repeat of the nonce already accepted here
  // must be a no-op, not a re-arm of Applying.
  if (result_accepted_) return;
  result_accepted_ = true;
  pending_result_ = r;
  state_ = State::Applying;
  // The verify window starts now, at acceptance -- not inherited from
  // whenever the last phase happened to finish. A result can legitimately
  // arrive with its own now_ms already past a stale await_deadline_ms_
  // (pump() just hasn't been called with a time that late yet), and
  // leaving the old value in place would let the very next pump() close
  // the session before Task 11 gets a chance to drain this result and
  // drive the verify phase.
  await_deadline_ms_ = now_ms + cfg_.await_next_ms;
}

std::optional<rc::CalResult> CalSweep::take_pending_result() {
  if (!pending_result_.has_value()) return std::nullopt;
  auto r = pending_result_;
  pending_result_.reset();
  // Nothing left to apply: fall back to the same "awaiting the next
  // session event" limbo a finished phase leaves behind. await_deadline_ms_
  // was already set fresh by on_result() at acceptance time, so the window
  // for Task 11 to drive the verify-phase CalCmd is well-defined regardless
  // of how long the result took to arrive.
  if (state_ == State::Applying) state_ = State::Idle;
  return r;
}

void CalSweep::build_cells(const std::vector<rc::CalWindow>& windows) {
  cells_.clear();
  for (const auto& w : windows) {
    if (w.idx_step == 0) continue;  // malformed window: never advances, skip
    for (int idx = w.idx_lo; idx <= w.idx_hi; idx += w.idx_step)
      cells_.push_back(Cell{w.rate, static_cast<uint8_t>(idx)});
  }
}

void CalSweep::pump(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr) {
  // Constraint 2, checked before anything else: losing the link mid-sweep
  // is the expected case, not an error, so a GS that never sends another
  // byte must never leave the drone parked at a swept TXAGC index.
  if (has_session_ && now_ms >= hard_cap_deadline_ms_) {
    close_session(pwr);
    return;
  }

  switch (state_) {
    case State::Idle:
      // Between phases (or between the last phase and a result, or
      // between draining a result and the verify phase it should trigger):
      // give the GS up to await_next_ms to move things along before this
      // session gives up on its own.
      if (has_session_ && now_ms >= await_deadline_ms_) close_session(pwr);
      return;
    case State::Applying:
      // Waiting on the caller to drain take_pending_result(); the shared
      // hard-cap check above is this state's deadline.
      return;
    case State::Sweeping:
      pump_sweeping(now_ms, tx, pwr);
      return;
  }
}

void CalSweep::pump_sweeping(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr) {
  // Global constraint: per-rate walls must be measured against a common
  // base, so the wall-equalized diff table comes off before the very
  // first frame of the session -- once, not once per phase, so fine and
  // verify measure against the same zeroed baseline coarse already
  // established.
  if (!zeroed_for_session_) {
    pwr.zero_rate_diffs();
    base_ref_idx_ = pwr.read_base_ref_idx();
    zeroed_for_session_ = true;
  }

  if (!cell_entered_) {
    if (cursor_ >= cells_.size()) {
      // Phase exhausted. Session stays open -- fine follows coarse, and
      // the verify pass follows a result -- so this is Idle-but-active,
      // not a full close: only the await_next timer (or the hard cap)
      // closes the session and restores power from here.
      state_ = State::Idle;
      await_deadline_ms_ = now_ms + cfg_.await_next_ms;
      return;
    }
    const Cell cell = cells_[cursor_];
    if (frames_per_cell_ == 0) {
      // Nothing to attribute to this cell -- step over it rather than
      // parking the radio there (index write + ladder swap) to send a
      // frame nobody asked for.
      ++cursor_;
      return;
    }
    active_rate_ = cell.rate;
    active_idx_ = cell.idx;
    // Every frame is stamped with the cell it's sent in (attribution IS
    // the measurement), so the ladder and the TXAGC index are both
    // pinned to this cell before the first frame goes out.
    pwr.set_index_override(cell.idx);
    tx.set_ladder({rc::LayerTxSpec{rc::PhyMode::HT, cell.rate, 20},
                   rc::LayerTxSpec{}},
                  std::nullopt);
    cell_seq_ = 0;
    settle_deadline_ms_ = now_ms + settle_ms_;
    next_send_ms_ = settle_deadline_ms_;  // first frame may fire as soon as settled
    cell_entered_ = true;
    return;
  }

  if (now_ms < settle_deadline_ms_ || now_ms < next_send_ms_) return;

  const auto payload = cal::build_cal_payload(active_rate_, active_idx_,
                                              current_phase_, cell_seq_);
  tx.send_body(0, payload.data(), payload.size());
  ++cell_seq_;
  next_send_ms_ = now_ms + gap_ms_;

  if (cell_seq_ >= frames_per_cell_) {
    ++cursor_;
    cell_entered_ = false;
  }
}

void CalSweep::close_session(PowerCtl& pwr) {
  // Only actually restore if this session ever touched the radio (zeroed
  // the diffs / wrote an index override) -- a session that hard-caps out
  // on its very first pump() call never left anything to undo.
  if (zeroed_for_session_) {
    pwr.set_index_override(base_ref_idx_);
    power_restored_ = true;
  }
  has_session_ = false;
  state_ = State::Idle;
  pending_result_.reset();
  cell_entered_ = false;
  cursor_ = 0;
}

}  // namespace mabur
