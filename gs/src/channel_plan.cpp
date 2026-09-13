#include "channel_plan.h"

namespace maburgs {

const char* to_string(MoveReason r) {
  switch (r) {
    case MoveReason::Commit: return "commit";
    case MoveReason::AckOverride: return "ack_override";
    case MoveReason::SplitHome: return "split_home";
    case MoveReason::Reunite: return "reunite";
  }
  return "?";
}

ChannelPlan::ChannelPlan(ChannelPlanCfg cfg) : cfg_(cfg), op_(cfg.home) {}

void ChannelPlan::tick(double now_ms, bool in_session) {
  now_ms_ = now_ms;
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
