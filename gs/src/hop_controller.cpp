#include "hop_controller.h"

#include "hop_blank.h"

#include <cmath>

namespace maburgs {

HopController::HopController(HopCfg cfg, uint8_t home) : cfg_(cfg), home_(home) {}

HopAction HopController::tick(const HopTick& in) {
  HopAction out;
  switch (state_) {
    case HopState::Idle:
    case HopState::Hold:
      idle_tick(in, out);
      break;
    case HopState::Ordered:
      ordered_tick(in, out);
      break;
    case HopState::Verifying:
      verifying_tick(in, out);
      break;
  }
  // The kill switch: the whole machine above still ran (state, epoch,
  // backoff, events -- logged with a "would_" prefix by log_event), but
  // nothing is actually ordered while disabled.
  if (!cfg_.enable) out = HopAction{};
  return out;
}

// The session's falling edge (run_radio() stops ticking this machine while
// the link is out of SESSION). An order still waiting for its confirm must
// not survive that: ChannelPlan ignores link loss while a hop is in flight,
// so an open order parked the lead card on the target and the trailing card
// on the old op forever -- no split_home -- while the drone, which never
// confirmed, had gone home on move_confirm_ms (bench 2026-09-24, GS session
// 0207). Withdraw it exactly like a confirm_ms timeout (target backed off,
// epoch bumped so the RCF stops carrying the order), which clears the plan's
// hop and lets its ordinary link-loss path put a card on home. A confirmed
// hop has already moved the plan's op; only the stale verify is dropped.
HopAction HopController::on_session_lost(double now_ms, uint8_t cur_op) {
  HopAction out;
  if (state_ == HopState::Ordered) {
    const uint8_t failed_target = hop_ch_;
    back_off(failed_target, now_ms);
    ++epoch_;
    hop_ch_ = cur_op;
    state_ = HopState::Idle;
    out.kind = HopAction::Withdraw;
    out.target = cur_op;
    out.epoch = epoch_;
    log_event(now_ms, "session_lost", epoch_, failed_target, 0, now_ms - order_ms_);
  } else if (state_ == HopState::Verifying) {
    state_ = HopState::Idle;
    log_event(now_ms, "session_lost", epoch_, hop_ch_, 0, now_ms - verify_start_);
  }
  if (!cfg_.enable) out = HopAction{};   // same kill switch as tick()
  return out;
}

void HopController::idle_tick(const HopTick& in, HopAction& out) {
  if (!in.verdict.trigger) {
    // The reason to hold is gone. Without this a Hold entered while the
    // trigger was latched is terminal -- every path out of Hold runs
    // through order(), which needs a trigger -- so the state, and the
    // sideport's hop.state with it, would read "hold" for the rest of the
    // flight after one exhausted episode, and no hold_end would ever close
    // the episode in scan.log.
    leave_hold(in.now_ms, in.cur_op);
    return;
  }
  if (in.now_ms - last_confirm_ms_ < cfg_.cooldown_ms) return;   // still cooling down: wait, silently
  prune_hop_times(in.now_ms);
  if (static_cast<int>(hop_times_.size()) >= cfg_.max_hops_per_min) {
    enter_hold(in.now_ms, "hold_cap", in.cur_op, 0, out);
    return;
  }
  if (in.best.has_value()) {
    leave_hold(in.now_ms, in.cur_op);
    flee(in.cur_op, in.now_ms);
    order(*in.best, in.verdict.ref_rung, in.lead_card, in.best_score, in.now_ms, "order", out);
    return;
  }
  if (home_available(in.cur_op, in.now_ms)) {
    leave_hold(in.now_ms, in.cur_op);
    flee(in.cur_op, in.now_ms);
    order(home_, in.verdict.ref_rung, in.lead_card, 0, in.now_ms, "order", out);
    return;
  }
  enter_hold(in.now_ms, "hold_exhausted", in.cur_op, 0, out);
}

void HopController::ordered_tick(const HopTick& in, HopAction& out) {
  if (in.video_on_target) {
    state_ = HopState::Verifying;
    verify_start_ = in.now_ms;
    out.kind = HopAction::Confirm;
    out.target = hop_ch_;
    out.epoch = epoch_;
    log_event(in.now_ms, "lead_confirm", epoch_, hop_ch_, 0, in.now_ms - order_ms_);
    return;
  }
  if (in.n_cards == 1 && !one_card_retuned_ && in.rcf_sent_since_order >= cfg_.one_card_repeats) {
    one_card_retuned_ = true;
    out.kind = HopAction::OneCardRetune;
    out.target = hop_ch_;
    out.epoch = epoch_;
    log_event(in.now_ms, "one_card_retune", epoch_, hop_ch_, 0, in.now_ms - order_ms_);
    return;
  }
  if (in.now_ms - order_ms_ >= cfg_.confirm_ms) {
    withdraw(in.cur_op, in.now_ms, out);
    return;
  }
  // Still waiting on the lead card: no action, no event.
}

void HopController::verifying_tick(const HopTick& in, HopAction& out) {
  // A verdict window whose measurement span STARTED before the hop landed
  // describes the channel we just left, and on that channel the verdict is
  // Interfered by construction -- it is why we hopped. The caller ticks
  // this machine every control tick (~10 ms) but only recomputes a verdict
  // every hop.window_ms (150 ms), so the cached VerdictOut immediately
  // after a Confirm is ALWAYS that pre-hop one. Acting on it failed the
  // verify of a perfectly good channel ~10 ms after landing on it, backed
  // that channel off for 30 s (doubling), and marched through the
  // remaining candidates into hold_cap in about half a second.
  //
  // t_start_ms (not t_ms) is the test: a window that merely ENDS after the
  // confirm still gathered most of its counter deltas on the old channel.
  // The cost is one verdict window of detection latency inside a 1000 ms
  // verify window; the first eligible window lands ~2 * window_ms after
  // the confirm, leaving five of them.
  //
  // And not merely after the confirm: after the confirm PLUS the landing
  // settle (kHopSettleBlankMs, the span the verdict's loss window is blanked
  // for). The hop's own retune gap is still being repaired then, so a window
  // starting inside the settle carries that debris as "recovered" symbols:
  // 40 failed its verify on 85 then 32 of them with 0 % loss, in a window
  // starting 27 ms after landing (bench 2026-09-24).
  const bool measured_after_landing =
      in.verdict.t_start_ms >= verify_start_ + kHopSettleBlankMs;
  if (in.verdict.v == Verdict::Interfered && measured_after_landing) {
    const uint8_t failed_target = hop_ch_;
    back_off(failed_target, in.now_ms);
    std::optional<uint8_t> next = in.best;
    if (next.has_value() && is_backed_off(*next, in.now_ms)) next.reset();   // skip backed off
    const bool have_candidate = next.has_value() || home_available(in.cur_op, in.now_ms);
    if (have_candidate) {
      // Without the persist delay: act on a raw Interfered window, not a
      // fresh multi-window trigger -- this path is "still on a bad
      // channel", not a new decision to hop. cooldown_ms is exempt for
      // retries (spec 2026-09-14-inflight-channel-hop-design.md §5), but
      // max_hops_per_min is NOT -- it counts retries, then holds.
      prune_hop_times(in.now_ms);
      if (static_cast<int>(hop_times_.size()) >= cfg_.max_hops_per_min) {
        enter_hold(in.now_ms, "hold_cap", failed_target, in.now_ms - verify_start_, out);
        return;
      }
      if (next.has_value()) {
        order(*next, in.verdict.ref_rung, in.lead_card, in.best_score, in.now_ms, "verify_fail", out);
      } else {
        order(home_, in.verdict.ref_rung, in.lead_card, 0, in.now_ms, "verify_fail", out);
      }
      return;
    }
    enter_hold(in.now_ms, "verify_fail", failed_target, in.now_ms - verify_start_, out);
    return;
  }
  if (in.now_ms - verify_start_ >= cfg_.verify_ms) {
    const uint8_t landed = hop_ch_;
    backoff_.erase(landed);   // cleared on verify_pass for the landed channel
    state_ = HopState::Idle;
    last_confirm_ms_ = in.now_ms;
    ++hops_;
    // The caller's cue to thaw the verdict engine's frozen references
    // (spec section 2: "or after a hop's verify window ends"). Emitted
    // only here, never on verify_fail/withdraw: those RE-ORDER, and spec
    // section 5 has the retry reuse the pre-onset ref_rung. Thawing there
    // would not damage the retry's own Order (its restore_rung is already
    // built from the cached verdict) -- the cost lands one window later,
    // when the next impaired window re-freezes ref_rung at the mid-hop
    // rung instead of the pre-onset one.
    out.kind = HopAction::VerifyPass;
    out.target = landed;
    out.epoch = epoch_;
    log_event(in.now_ms, "verify_pass", epoch_, landed, 0, in.now_ms - verify_start_);
  }
}

void HopController::order(uint8_t target, int restore_rung, int lead_card, uint32_t score, double now,
                          const char* event_kind, HopAction& out) {
  ++epoch_;
  hop_ch_ = target;
  state_ = HopState::Ordered;
  order_ms_ = now;
  one_card_retuned_ = false;
  hop_times_.push_back(now);
  out.kind = HopAction::Order;
  out.target = target;
  out.epoch = epoch_;
  out.restore_rung = restore_rung;
  out.lead_card = lead_card;
  log_event(now, event_kind, epoch_, target, score, 0);
}

void HopController::withdraw(uint8_t restore_to, double now, HopAction& out) {
  const uint8_t failed_target = hop_ch_;
  back_off(failed_target, now);
  ++epoch_;
  hop_ch_ = restore_to;
  state_ = HopState::Idle;
  out.kind = HopAction::Withdraw;
  out.target = restore_to;
  out.epoch = epoch_;
  log_event(now, "withdraw", epoch_, failed_target, 0, now - order_ms_);
}

void HopController::enter_hold(double now, const char* why, uint8_t target, double elapsed_ms,
                               HopAction& out) {
  out.kind = HopAction::Hold;
  if (state_ == HopState::Hold) return;   // already holding: a state, not a fresh event
  state_ = HopState::Hold;
  hold_start_ms_ = now;
  ++holds_;
  log_event(now, why, epoch_, target, 0, elapsed_ms);
}

void HopController::leave_hold(double now, uint8_t cur_op) {
  if (state_ != HopState::Hold) return;
  state_ = HopState::Idle;
  // elapsed_ms = how long the hold lasted, which with the entry event's
  // own kind (hold_cap / hold_exhausted / verify_fail) is everything an
  // operator needs out of an episode that no longer logs per tick.
  log_event(now, "hold_end", epoch_, cur_op, 0, now - hold_start_ms_);
}

// The channel a trigger is fleeing is as bad as a target that failed its
// verify, so it gets the same backoff. Without it only failed TARGETS were
// excluded and the fled channel stayed a retry candidate -- and the in-flight
// ranker (event counts over 5 ms dwells) scores a long-frame jammer low, so
// when the first target failed its verify the retry went straight back into
// the jam (bench 2026-09-24, GS session 0207: 144 -> 128 -> 144). A withdraw
// still returns to it: that path restores the op, it does not consult the
// ranker.
void HopController::flee(uint8_t ch, double now) { back_off(ch, now); }

// Home is the fallback when nothing is ranked -- unless the link is already
// there, or home is backed off (it is the channel just fled, or it failed a
// verify): then going home is going back into the problem, and holding on
// the current channel is the better answer (bench 2026-09-24, run 1: the jam
// was on home and the fallback ordered it straight back).
bool HopController::home_available(uint8_t cur_op, double now) const {
  return cur_op != home_ && !is_backed_off(home_, now);
}

void HopController::back_off(uint8_t ch, double now) {
  auto it = backoff_.find(ch);
  const int k = (it == backoff_.end()) ? 1 : it->second.second + 1;
  double dur = static_cast<double>(cfg_.backoff_ms) * std::pow(2.0, k - 1);
  if (dur > 300000.0) dur = 300000.0;
  backoff_[ch] = {now + dur, k};
}

bool HopController::is_backed_off(uint8_t ch, double now) const {
  auto it = backoff_.find(ch);
  return it != backoff_.end() && it->second.first > now;
}

void HopController::prune_hop_times(double now) {
  while (!hop_times_.empty() && now - hop_times_.front() > 60000.0) hop_times_.pop_front();
}

void HopController::log_event(double now, const std::string& kind, uint8_t epoch, uint8_t target,
                              uint32_t score, double elapsed_ms) {
  HopEvent e;
  e.t_ms = now;
  e.kind = cfg_.enable ? kind : ("would_" + kind);
  e.epoch = epoch;
  e.target = target;
  e.score = score;
  e.elapsed_ms = elapsed_ms;
  events_.push_back(e);
}

uint8_t HopController::hop_ch() const { return cfg_.enable ? hop_ch_ : 0; }
uint8_t HopController::epoch() const { return epoch_; }
HopState HopController::state() const { return state_; }

std::vector<uint8_t> HopController::backed_off(double now_ms) const {
  std::vector<uint8_t> v;
  for (const auto& kv : backoff_)
    if (kv.second.first > now_ms) v.push_back(kv.first);
  return v;
}

std::vector<HopEvent> HopController::take_events() {
  std::vector<HopEvent> v = std::move(events_);
  events_.clear();
  return v;
}

uint32_t HopController::hops() const { return hops_; }
uint32_t HopController::holds() const { return holds_; }

}  // namespace maburgs
