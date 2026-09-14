#include "channel_plan.h"

namespace maburgs {

const char* to_string(MoveReason r) {
  switch (r) {
    case MoveReason::Commit: return "commit";
    case MoveReason::AckOverride: return "ack_override";
    case MoveReason::SplitHome: return "split_home";
    case MoveReason::Reunite: return "reunite";
    case MoveReason::HopLead: return "hop_lead";
    case MoveReason::HopFollow: return "hop_follow";
    case MoveReason::HopWithdraw: return "hop_withdraw";
    case MoveReason::HopOneCard: return "hop_one_card";
  }
  return "?";
}

ChannelPlan::ChannelPlan(ChannelPlanCfg cfg) : cfg_(cfg), op_(cfg.home) {}

void ChannelPlan::tick(double now_ms, bool in_session) {
  now_ms_ = now_ms;
  if (hopping_) return;
  if (in_session) {
    have_lost_since_ = false;
    if (split_) reunite_(now_ms);
    return;
  }
  if (!frozen_) return;
  if (!have_lost_since_) {
    have_lost_since_ = true;
    lost_since_ms_ = now_ms;
  }
  // op == home: the split has nothing to split. Both halves of the
  // rendezvous set are the same channel, so entering it would fan the same
  // DISC out on both cards on one channel (and log a from==to SplitHome
  // move) while changing nothing about what is on the air.
  if (op_ == cfg_.home) return;
  if (!split_ && now_ms - lost_since_ms_ >= cfg_.split_after_ms) {
    split_ = true;
    split_at_ms_ = now_ms;
    events_.push_back(MoveEvent{now_ms, 0, op_, cfg_.home, MoveReason::SplitHome});
  }
}

void ChannelPlan::on_ack(double now_ms, uint8_t agreed, uint8_t proposed) {
  now_ms_ = now_ms;
  have_lost_since_ = false;
  if (frozen_ && agreed == op_) {
    if (split_) reunite_(now_ms);
    return;
  }
  const uint8_t from = op_;
  op_ = agreed;
  frozen_ = true;
  split_ = false;
  events_.push_back(MoveEvent{now_ms, -1, from, op_,
                              agreed == proposed ? MoveReason::Commit
                                                 : MoveReason::AckOverride});
}

void ChannelPlan::hop_order(double now_ms, uint8_t target, int lead_card) {
  now_ms_ = now_ms;
  if (hopping_) {
    // A second order abandons the in-flight hop: log its withdrawal before
    // starting the new one, so the flight log carries a trace of it instead
    // of the lead card silently snapping back to op_. The exclusion window
    // for the split timer (hop_start_ms_) is NOT restarted here: however
    // many re-orders happen, the whole hopping stretch is one window.
    events_.push_back(MoveEvent{now_ms, hop_lead_, hop_target_, op_, MoveReason::HopWithdraw});
  } else {
    hop_start_ms_ = now_ms;
  }
  hopping_ = true;
  hop_target_ = target;
  hop_lead_ = lead_card;
  hop_from_ = op_;
  split_ = false;
  events_.push_back(MoveEvent{now_ms, lead_card, op_, target,
                              lead_card < 0 ? MoveReason::HopOneCard : MoveReason::HopLead});
}

void ChannelPlan::hop_confirmed(double now_ms) {
  now_ms_ = now_ms;
  op_ = hop_target_;
  frozen_ = true;
  hopping_ = false;
  // Exclude the hop window from the split timer: loss during a deliberate
  // hop is expected by construction, not evidence of a fade, but loss
  // before and after the hop is real and must keep counting.
  if (have_lost_since_) lost_since_ms_ += (now_ms - hop_start_ms_);
  events_.push_back(MoveEvent{now_ms, -1, hop_from_, op_, MoveReason::HopFollow});
}

void ChannelPlan::hop_withdraw(double now_ms) {
  now_ms_ = now_ms;
  hopping_ = false;
  // See hop_confirmed: shift, don't clear, so pre-hop loss still counts.
  if (have_lost_since_) lost_since_ms_ += (now_ms - hop_start_ms_);
  events_.push_back(MoveEvent{now_ms, hop_lead_, hop_target_, op_, MoveReason::HopWithdraw});
}

void ChannelPlan::reunite_(double now_ms) {
  split_ = false;
  events_.push_back(MoveEvent{now_ms, 0, cfg_.home, op_, MoveReason::Reunite});
}

int ChannelPlan::window_(double now_ms) const {
  const double since = now_ms - split_at_ms_;
  return static_cast<int>(since / cfg_.home_window_ms);
}

bool ChannelPlan::quiet_gap_(double now_ms) const {
  const double in_window = now_ms - split_at_ms_ - window_(now_ms) * static_cast<double>(cfg_.home_window_ms);
  return in_window >= cfg_.home_window_ms - cfg_.beacon_period_ms;
}

uint8_t ChannelPlan::desired(int card) const {
  if (hopping_) return (hop_lead_ < 0 || card == hop_lead_) ? hop_target_ : op_;
  if (!split_) return op_;
  if (cfg_.n_cards >= 2) return card == 0 ? cfg_.home : op_;
  return (window_(now_ms_) % 2 == 0) ? cfg_.home : op_;
}

std::optional<std::vector<int>> ChannelPlan::beacon_cards() const {
  if (!split_) return std::nullopt;
  std::vector<int> out;
  if (cfg_.n_cards >= 2) {
    for (int i = 0; i < cfg_.n_cards; ++i) out.push_back(i);
    return out;
  }
  if (!quiet_gap_(now_ms_)) out.push_back(0);
  return out;
}

std::vector<MoveEvent> ChannelPlan::take_events() {
  std::vector<MoveEvent> out;
  out.swap(events_);
  return out;
}

}  // namespace maburgs
