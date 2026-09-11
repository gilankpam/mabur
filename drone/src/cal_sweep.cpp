#include "cal_sweep.h"

#include "mabur/cal_wire.h"

namespace mabur {

void CalSweep::on_cmd(const rc::CalCmd& c, uint64_t now_ms, PowerCtl& pwr) {
  const bool new_session = !has_session_ || c.nonce != nonce_;
  if (new_session) {
    // A brand-new nonce ARRIVING WHILE A SESSION IS STILL LIVE is a real
    // case, not a theoretical one: new_session is `!has_session_ ||
    // nonce != nonce_`, and the second disjunct fires for any second
    // `maburcal start` after an abort, after a GS restart, or within
    // await_next_ms (15 s) of the previous run's last frame. At that
    // moment the TXAGC override is still parked at the PREVIOUS session's
    // last swept cell (pump_sweeping never restores between cells or
    // phases; only close_session() does), and devourer's GetTxPowerState
    // reports that override rather than the anchor -- so the
    // read_base_ref_idx() below would latch a swept cell's index as this
    // session's base reference. That wrong anchor rides the ack to the GS,
    // comes back inside the result, and lands in radio.base_ref_idx in
    // /etc/mabur.toml, where every subsequent boot derives
    // diff[r] = wall - m - base_ref from it: a parked cell BELOW the true
    // anchor makes every rate transmit ABOVE its measured wall, persisted
    // across reboots, from the kit that exists to prevent exactly that.
    // The same-nonce version of this hazard is guarded by reusing the
    // latched value (see take_ack_base_ref()); this is its mirror --
    // restore the anchor before reading, so the chip is in the same clean
    // state a genuinely first session would find it in.
    if (has_session_) pwr.set_index_override(base_ref_idx_);
    // A brand-new nonce: reset every piece of session state, including the
    // hard cap, which is measured from THIS command, not from whenever the
    // drone happened to boot.
    nonce_ = c.nonce;
    has_session_ = true;
    power_restored_ = false;
    result_accepted_ = false;
    hard_cap_deadline_ms_ = now_ms + cfg_.hard_cap_ms;
    last_started_phase_ = -1;
    // Global constraint: per-rate walls must be measured against a common
    // base, so the wall-equalized diff table comes off -- and this
    // session's base reference index is captured -- HERE, synchronously,
    // at acceptance, not lazily on pump()'s first call. The chip is clean
    // by the time of the read: no phase of THIS session has parked the
    // TXAGC at a swept cell's index yet, and any PREVIOUS session's park
    // was just undone above (a later phase of the same session reuses
    // base_ref_idx_ below rather than reading again for exactly that
    // reason -- see take_ack_base_ref()).
    pwr.zero_rate_diffs();
    base_ref_idx_ = pwr.read_base_ref_idx();
    zeroed_for_session_ = true;
  } else if (static_cast<int>(c.phase) == last_started_phase_) {
    // An exact repeat of the phase already running -- the GS resends its
    // CalCmd every 200 ms and gives up at 3000 ms (gs/src/cal_session.cpp),
    // over an uplink that loses 30-50% of frames. Ruling (Task 11 review):
    // re-arm the ack every time this happens, not just once per phase. The
    // ack is the only non-redundant frame in this protocol -- a single
    // lost ack Telem otherwise costs the GS the WHOLE phase (it never
    // leaves AwaitAck, so it never stops transmitting retries into the
    // live sweep, which is precisely the airtime contamination the
    // radio-silence rule exists to prevent) -- and CalSession::on_ack() is
    // a no-op outside AwaitAck (gs/src/cal_session.h), so answering a
    // retransmission a second (or fifth) time is harmless. This does NOT
    // restart the phase: cells_/cursor_/frames_per_cell_ etc. are left
    // exactly as they are, only the ack gets re-armed.
    pending_ack_base_ref_ = base_ref_idx_;
    return;
  } else if (static_cast<int>(c.phase) < last_started_phase_) {
    // Constraint 3, the other half: a phase STRICTLY BEHIND the one
    // already accepted can only be a stale retransmission of an EARLIER
    // phase arriving late (a delayed coarse repeat landing after fine has
    // started, or after a result is already sitting undrained in
    // Applying) -- ignored outright, with NO ack (unlike the exact-repeat
    // case above): the GS is not awaiting an ack for this old phase any
    // more, and re-sending one now would tell it its stale phase was just
    // accepted while the session has actually moved on. Silently
    // discarding the in-progress phase's cursor and any undrained
    // CalResult would be the failure mode a plain == guard (instead of
    // this <) would reintroduce.
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
  // Task 11: arm this phase's ack. Always base_ref_idx_ (captured once,
  // above or by an earlier phase of this same session) -- see
  // take_ack_base_ref()'s header comment for why a fresh read here would
  // be wrong for any phase after the first.
  pending_ack_base_ref_ = base_ref_idx_;
}

std::optional<int> CalSweep::take_ack_base_ref() {
  auto v = pending_ack_base_ref_;
  pending_ack_base_ref_.reset();
  return v;
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
  // Global constraint ("per-rate walls must be measured against a common
  // base"): the wall-equalized diff table comes off, and base_ref_idx_ is
  // captured, once per session -- but now inside on_cmd() at acceptance
  // (Task 11), not here. By the time this is ever reached, on_cmd() has
  // already run at least once for this session and zeroed_for_session_ is
  // therefore already true; nothing left to do here.
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
  // DEADLINE-based, not now_ms-based: the next frame is due gap_ms_ after
  // the one that was DUE, not after the one that was SENT. This matters
  // because the writer thread now SLEEPS between pumps (drone/src/main.cpp
  // stopped spinning a core for the whole session), and the phase is
  // measured against a GS listen window sized from plan_duration_ms()
  // (settle + frames*gap per cell) that leaves one gap -- 2 ms -- of slack
  // per 140 ms cell. Re-basing off the observed clock charges every late
  // wake to the phase and can only push it later; the deadline form
  // absorbs a late wake instead (it simply fires the next frame
  // immediately, catching the phase back up to its plan) and stays
  // anchored to the plan the GS is timing against no matter what the
  // caller's loop costs. tests/test_cal_e2e.cpp pins the realized
  // duration against plan_duration_ms() for both the shipped 200 us
  // sleep and a 1 ms one.
  next_send_ms_ += gap_ms_;

  if (cell_seq_ >= frames_per_cell_) {
    ++cursor_;
    cell_entered_ = false;
  }
}

void CalSweep::close_session(PowerCtl& pwr) {
  // zeroed_for_session_ is set unconditionally inside on_cmd()'s
  // new_session branch, before this class's caller could ever call
  // pump() -- and therefore this function -- for the session (Task 11
  // moved it there from a lazy once-per-session guard in pump_sweeping).
  // So by the time close_session() can run for any session that ever set
  // has_session_, this is always true; the stale comment this replaced
  // described a "hard-caps out before ever touching the radio" case that
  // predates that move and is no longer reachable. Kept as a defensive
  // check, not because the false branch still happens.
  //
  // NOTE this restores a FLAT INDEX OVERRIDE parked at the anchor, not "no
  // override at all" -- main.cpp's cal_active falling edge is what
  // actually clears the override (SetTxPowerIndexOverride(-1)) and
  // re-applies the real operating plan (rate diffs + global offset) on
  // top, because THIS class's narrow PowerCtl interface has no concept of
  // "the operating plan" (that is config/power_mode-driven, main.cpp's
  // business, not this one's).
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
