#include "cal_sweep.h"

#include <algorithm>

namespace mabur {

namespace {
bool known_kind(cal::Kind k) { return k == cal::Kind::Wall || k == cal::Kind::Capacity; }
}  // namespace

uint8_t CalSweep::stamp_flags() const {
  return cal::kFlagStbc | (cfg_.ldpc ? cal::kFlagLdpc : 0);
}

void CalSweep::build_points(const cal::SweepCmd& c) {
  points_.clear();
  if (c.kind != cal::Kind::Capacity) return;
  for (int mcs = 0; mcs < 8; ++mcs)
    if (c.mcs_mask & (1 << mcs)) points_.push_back(static_cast<uint8_t>(mcs));
}

uint16_t CalSweep::remaining_start_ms(uint64_t now_ms) const {
  if (sweep_start_ms_ <= now_ms) return 0;
  const uint64_t remaining = sweep_start_ms_ - now_ms;
  return static_cast<uint16_t>(remaining > 65535 ? 65535 : remaining);
}

bool CalSweep::starts_sweep(const cal::SweepCmd& c) const {
  // The same three branches on_cmd() takes, without the side effects.
  if (!known_kind(c.kind)) return false;
  if (!has_session_ || c.nonce != nonce_)
    return !(last_nonce_valid_ && c.nonce == last_closed_nonce_);
  return static_cast<int>(c.seq) > last_started_seq_;
}

void CalSweep::cancel_wall(WallCtl& wall) {
  if (!wall_started_) return;
  wall.cancel();
  wall_started_ = false;
}

void CalSweep::on_cmd(const cal::SweepCmd& c, uint64_t now_ms, PowerCtl& pwr, WallCtl& wall) {
  const bool new_session = !has_session_ || c.nonce != nonce_;
  // An exact repeat of a sweep that has already finished: the GS never
  // heard the ack (or the sweep outran it) and is still asking. The answer
  // it is missing is the report, not a fresh ack that would send it
  // waiting for a sweep that is over -- re-send the stored SweepDone, on
  // its repeat schedule, and touch nothing else.
  if (!new_session && state_ == State::Idle && repeat_done_ &&
      static_cast<int>(c.seq) == last_started_seq_) {
    pending_done_ = repeat_done_;
    return;
  }
  // Any other SWEEP of the open run is taken as the GS having moved past
  // the last SweepDone (take_done()) -- not proof that it heard it --
  // whatever on_cmd does with it below.
  if (!new_session) repeat_done_.reset();
  // A closed run (hard cap, abort, or await timeout) never reopens: a
  // straggler or a GS resend of that exact nonce arriving after the
  // close is ignored outright, with no ack and no PowerCtl touched, so
  // hard_cap_ms keeps bounding the run it was armed for and an abort
  // cannot be undone by a late retransmission.
  if (new_session && last_nonce_valid_ && c.nonce == last_closed_nonce_) return;
  if (!new_session) {
    if (static_cast<int>(c.seq) == last_started_seq_) {
      // Exact repeat of the sweep already running: re-arm the ack every
      // time (the GS resends into a 30-50%-lossy uplink, and a single
      // lost ack must not cost the whole sweep), but touch nothing else.
      // start_in_ms reports what's left of the ORIGINAL 200 ms start
      // delay, not a fresh one.
      pending_ack_ = cal::SweepAck{nonce_, c.seq, cal::AckResult::Accepted,
                                   remaining_start_ms(now_ms), stamp_flags()};
      return;
    }
    if (static_cast<int>(c.seq) < last_started_seq_) {
      // Stale duplicate of an EARLIER sweep arriving late: ignored
      // outright, no ack -- the GS is not waiting on this seq any more.
      return;
    }
  }
  // A kind this drone does not know (a retired one, say) is refused, never
  // run as an empty sweep that would report success.
  if (!known_kind(c.kind)) {
    pending_ack_ = cal::SweepAck{c.nonce, c.seq, cal::AckResult::Unsupported, 200, stamp_flags()};
    return;
  }
  // A Wall sweep the drone cannot run is refused before anything touches
  // the device or the run: the GS goes on with the rest of its run.
  if (c.kind == cal::Kind::Wall) {
    const cal::AckResult r = wall.check(c.channel, c.bw);
    if (r != cal::AckResult::Accepted) {
      pending_ack_ = cal::SweepAck{c.nonce, c.seq, r, 200, stamp_flags()};
      return;
    }
  }
  // Accepted from here on: a measurement still open belongs to the sweep
  // this command supersedes, and the radio goes back first.
  cancel_wall(wall);
  if (new_session) {
    // A brand-new nonce arriving while a run is still live -- any second
    // `maburcal start` after an abort, after a GS restart, or within
    // await_next_ms of the previous run's last frame -- finds the TXAGC
    // override still parked at the anchor of the PREVIOUS run (only
    // close_session() restores it). Undo that before reading, so this
    // read sees the same clean reference a genuinely first run would.
    if (has_session_) pwr.set_index_override(anchor_idx_);
    // Zero BEFORE reading, whatever the kind: with a custom diff table live
    // the chip reports reference + diff, not the reference. A capacity
    // sweep's owner programs its width's plan back after accepting the
    // command. A table that would not zero gives no clean read, and the
    // command is refused as an unreadable anchor is.
    const bool zeroed = pwr.zero_rate_diffs();
    const int anchor = zeroed ? pwr.read_anchor_idx() : -1;
    if (anchor < 0) {
      // Refuse with an explicit ack rather than staying silent. Full
      // teardown (not just has_session_ = false), so pump()'s
      // switch(state_) never keeps dispatching into a stale Sweeping
      // state nobody owns any more.
      has_session_ = false;
      state_ = State::Idle;
      cursor_ = 0;
      point_entered_ = false;
      points_.clear();
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
  }

  last_started_seq_ = c.seq;
  current_kind_ = c.kind;
  current_bw_ = c.bw;
  current_channel_ = c.channel;
  settle_ms_ = c.settle_ms;
  dwell_ms_ = c.dwell_ms;
  offered_.fill(0);
  blocked_pct_.fill(0);
  build_points(c);
  cursor_ = 0;
  point_entered_ = false;
  state_ = State::Sweeping;
  pending_done_.reset();
  repeat_done_.reset();
  // This sweep supersedes any ApplyCmd still queued from an earlier
  // sweep of this run -- take_pending_apply() must not hand out a result
  // for a sweep that just got superseded.
  pending_apply_.reset();
  // Capacity flies the production power plan the owner programs for this
  // width, from the anchor with no flat override parked elsewhere.
  if (c.kind == cal::Kind::Capacity) pwr.set_index_override(anchor_idx_);
  // The sweep does not start until 200 ms from now (pump_capacity /
  // pump_wall check this): the ack needs a chance to reach the GS first.
  sweep_start_ms_ = now_ms + 200;
  pending_ack_ = cal::SweepAck{nonce_, c.seq, cal::AckResult::Accepted,
                               remaining_start_ms(now_ms), stamp_flags()};
}

void CalSweep::on_apply(const cal::ApplyCmd& a, uint64_t now_ms) {
  if (!has_session_ || a.nonce != nonce_) return;
  // The GS analysed the sweep: its SweepDone need not repeat.
  repeat_done_.reset();
  // A sweep actively running owns the radio right now; accepting an
  // apply here would silently abandon its progress.
  if (state_ == State::Sweeping) return;
  // The GS resends APPLY every 200 ms until it hears APPLIED. A repeat
  // of a (nonce, seq) already handed out by take_pending_apply() must be
  // a no-op, not a re-queue. Keyed on the sweep's seq, not the width: a
  // run may apply twice at one width, and the second is new.
  if (handed_out_apply_ && a.nonce == handed_out_nonce_ && a.seq == handed_out_seq_) return;
  pending_apply_ = a;
  state_ = State::Applying;
  // The wait-for-next-event window starts now, at acceptance -- not
  // inherited from whenever the last sweep happened to finish.
  await_deadline_ms_ = now_ms + cfg_.await_next_ms;
}

void CalSweep::on_abort(const cal::Abort& a, uint64_t now_ms, PowerCtl& pwr, WallCtl& wall) {
  (void)now_ms;
  if (!has_session_ || a.nonce != nonce_) return;
  cancel_wall(wall);
  close_session(pwr);
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
  // on_apply() rather than re-queued.
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

void CalSweep::pump(uint64_t now_ms, PowerCtl& pwr, CapacityCtl& cap, WallCtl& wall) {
  // Constraint 2, checked before anything else: losing the link mid-run
  // is the expected case, not an error, so a GS that never sends another
  // byte must never leave the drone mid-sweep.
  if (has_session_ && now_ms >= hard_cap_deadline_ms_) {
    cancel_wall(wall);
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
        // Deadline-based, but a late pump re-arms once rather than in a
        // burst.
        next_done_ms_ += kDoneRepeatMs;
        if (next_done_ms_ <= now_ms) next_done_ms_ = now_ms + kDoneRepeatMs;
      }
      return;
    case State::Applying:
      // Waiting on the caller to drain take_pending_apply() or the next
      // command to land. Bounded by both the shared hard cap above and
      // this state's own await deadline.
      if (has_session_ && now_ms >= await_deadline_ms_) close_session(pwr);
      return;
    case State::Sweeping:
      if (current_kind_ == cal::Kind::Wall)
        pump_wall(now_ms, wall);
      else
        pump_capacity(now_ms, cap);
      return;
  }
}

void CalSweep::pump_wall(uint64_t now_ms, WallCtl& wall) {
  if (now_ms < sweep_start_ms_) return;
  if (!wall_started_) {
    wall.start(current_channel_, current_bw_, now_ms);
    wall_started_ = true;
    return;
  }
  if (wall.step(now_ms)) return;
  wall_started_ = false;
  cal::SweepDone d = wall.finish();
  d.nonce = nonce_;
  d.seq = static_cast<uint8_t>(last_started_seq_);
  finish_sweep(now_ms, d);
}

void CalSweep::pump_capacity(uint64_t now_ms, CapacityCtl& cap) {
  // The ack needs a chance to reach the GS before the first point starts
  // using up the air.
  if (now_ms < sweep_start_ms_) return;
  if (!point_entered_) {
    if (cursor_ >= points_.size()) {
      cal::SweepDone d;
      d.nonce = nonce_;
      d.seq = static_cast<uint8_t>(last_started_seq_);
      d.offered = offered_;
      d.blocked_pct = blocked_pct_;
      finish_sweep(now_ms, d);
      return;
    }
    cap.set_point(points_[cursor_], current_bw_);
    settle_deadline_ms_ = now_ms + settle_ms_;
    point_entered_ = true;
    return;
  }

  if (now_ms < settle_deadline_ms_) return;

  const uint8_t mcs = points_[cursor_];
  const auto payload = cal::build_cal_payload(cal::Kind::Capacity, mcs, /*idx=*/0, current_bw_,
                                               stamp_flags(), /*seq=*/0,
                                               cfg_.capacity_frame_len);
  // Blocks for the dwell by design -- see the class comment.
  const BlastResult r = cap.blast(payload.data(), payload.size(), dwell_ms_);
  offered_[mcs] = r.offered;
  blocked_pct_[mcs] = r.blocked_pct;
  ++cursor_;
  point_entered_ = false;
}

void CalSweep::finish_sweep(uint64_t now_ms, const cal::SweepDone& d) {
  pending_done_ = d;
  repeat_done_ = d;
  next_done_ms_ = now_ms + kDoneFirstRepeatMs;
  // The run stays open -- another sweep or an apply can follow -- so this
  // is Idle-but-active, not a full close: only the await timer (or the
  // hard cap) closes the run.
  state_ = State::Idle;
  point_entered_ = false;
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
  point_entered_ = false;
  pending_done_.reset();
  repeat_done_.reset();
  pending_apply_.reset();
  handed_out_apply_ = false;
}

}  // namespace mabur
