#include "hop_controller.h"

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

void HopController::idle_tick(const HopTick& in, HopAction& out) {
  if (!in.verdict.trigger) return;
  if (in.now_ms - last_confirm_ms_ < cfg_.cooldown_ms) return;   // still cooling down: wait, silently
  prune_hop_times(in.now_ms);
  if (static_cast<int>(hop_times_.size()) >= cfg_.max_hops_per_min) {
    ++holds_;
    state_ = HopState::Hold;
    out.kind = HopAction::Hold;
    log_event(in.now_ms, "hold cap", epoch_, in.cur_op, 0, 0);
    return;
  }
  if (in.best.has_value()) {
    order(*in.best, in.verdict.ref_rung, in.lead_card, in.best_score, in.now_ms, "order", out);
    return;
  }
  if (in.cur_op != home_) {
    order(home_, in.verdict.ref_rung, in.lead_card, 0, in.now_ms, "order", out);
    return;
  }
  ++holds_;
  state_ = HopState::Hold;
  out.kind = HopAction::Hold;
  log_event(in.now_ms, "hold exhausted", epoch_, in.cur_op, 0, 0);
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
  if (in.verdict.v == Verdict::Interfered) {
    const uint8_t failed_target = hop_ch_;
    back_off(failed_target, in.now_ms);
    std::optional<uint8_t> next = in.best;
    if (next.has_value() && is_backed_off(*next, in.now_ms)) next.reset();   // skip backed off
    if (next.has_value()) {
      // Without the persist delay: act on a raw Interfered window, not a
      // fresh multi-window trigger, and skip cooldown/rate-cap -- this
      // path is "still on a bad channel", not a new decision to hop.
      order(*next, in.verdict.ref_rung, in.lead_card, in.best_score, in.now_ms, "verify_fail", out);
      return;
    }
    if (in.cur_op != home_) {
      order(home_, in.verdict.ref_rung, in.lead_card, 0, in.now_ms, "verify_fail", out);
      return;
    }
    ++holds_;
    state_ = HopState::Hold;
    out.kind = HopAction::Hold;
    log_event(in.now_ms, "verify_fail", epoch_, failed_target, 0, in.now_ms - verify_start_);
    return;
  }
  if (in.now_ms - verify_start_ >= cfg_.verify_ms) {
    const uint8_t landed = hop_ch_;
    backoff_.erase(landed);   // cleared on verify_pass for the landed channel
    state_ = HopState::Idle;
    last_confirm_ms_ = in.now_ms;
    ++hops_;
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
