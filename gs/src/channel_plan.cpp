#include "channel_plan.h"

namespace maburgs {

const char* to_string(MoveReason r) {
  switch (r) {
    case MoveReason::Commit: return "commit";
    case MoveReason::AckOverride: return "ack_override";
    case MoveReason::HopLead: return "hop_lead";
    case MoveReason::HopFollow: return "hop_follow";
    case MoveReason::HopWithdraw: return "hop_withdraw";
    case MoveReason::HopOneCard: return "hop_one_card";
  }
  return "?";
}

ChannelPlan::ChannelPlan(ChannelPlanCfg cfg) : cfg_(std::move(cfg)), op_(cfg_.start) {}

bool ChannelPlan::member(uint8_t ch) const { return mabur::channel_set_member(cfg_.channels, ch); }

void ChannelPlan::tick(double now_ms, bool in_session) {
  now_ms_ = now_ms;
  in_session_ = in_session;
  if (in_session) {
    ever_linked_ = true;
    have_lost_since_ = false;
    return;
  }
  if (hopping_) return;
  if (!have_lost_since_) {
    have_lost_since_ = true;
    lost_since_ms_ = now_ms;
  }
}

bool ChannelPlan::release_scout() const {
  if (in_session_ || hopping_) return false;
  if (!ever_linked_) return true;
  return have_lost_since_ && now_ms_ - lost_since_ms_ >= cfg_.search_after_ms;
}

void ChannelPlan::on_ack(double now_ms, uint8_t agreed, uint8_t proposed) {
  now_ms_ = now_ms;
  if (agreed == op_ || !member(agreed)) return;
  const uint8_t from = op_;
  op_ = agreed;
  events_.push_back(MoveEvent{now_ms, -1, from, op_,
                              agreed == proposed ? MoveReason::Commit : MoveReason::AckOverride});
}

void ChannelPlan::commit(double now_ms, uint8_t to) {
  now_ms_ = now_ms;
  if (to == op_ || !member(to)) return;
  events_.push_back(MoveEvent{now_ms, -1, op_, to, MoveReason::Commit});
  op_ = to;
}

void ChannelPlan::hop_order(double now_ms, uint8_t target, int lead_card) {
  now_ms_ = now_ms;
  if (hopping_) {
    // A second order abandons the in-flight hop: log its withdrawal before
    // starting the new one, so the flight log carries a trace of it instead
    // of the lead card silently snapping back to op_. The exclusion window
    // for the search timer (hop_start_ms_) is NOT restarted here: however
    // many re-orders happen, the whole hopping stretch is one window.
    events_.push_back(MoveEvent{now_ms, hop_lead_, hop_target_, op_, MoveReason::HopWithdraw});
  } else {
    hop_start_ms_ = now_ms;
  }
  hopping_ = true;
  hop_target_ = target;
  hop_lead_ = lead_card;
  hop_from_ = op_;
  events_.push_back(MoveEvent{now_ms, lead_card, op_, target,
                              lead_card < 0 ? MoveReason::HopOneCard : MoveReason::HopLead});
}

void ChannelPlan::hop_confirmed(double now_ms) {
  now_ms_ = now_ms;
  // No hop in flight: nothing to confirm. HopController and ChannelPlan do
  // not start hopping at the same instant -- a one-card Order deliberately
  // does NOT call hop_order() (the sole radio has to stay on the old
  // channel while the order rides one_card_repeats RCFs, see main.cpp), so
  // the controller can reach Confirm or, far more often, its confirm_ms
  // Withdraw while this plan has never entered hopping_. Unguarded, that
  // Withdraw ran the search-timer shift below with hop_start_ms_ still 0,
  // pushing lost_since_ms_ a whole session into the future and suppressing
  // release_scout -- a one-card GS's only convergence mechanism when the
  // two ends disagree. (hop_confirmed would additionally have moved op_ to
  // a hop_target_ of 0.)
  if (!hopping_) return;
  op_ = hop_target_;
  hopping_ = false;
  // Exclude the hop window from the search timer: loss during a deliberate
  // hop is expected by construction, not evidence of a fade, but loss
  // before and after the hop is real and must keep counting.
  if (have_lost_since_) lost_since_ms_ += (now_ms - hop_start_ms_);
  events_.push_back(MoveEvent{now_ms, -1, hop_from_, op_, MoveReason::HopFollow});
}

void ChannelPlan::hop_withdraw(double now_ms) {
  now_ms_ = now_ms;
  if (!hopping_) return;   // see hop_confirmed: safe to call with no hop in flight
  hopping_ = false;
  // See hop_confirmed: shift, don't clear, so pre-hop loss still counts.
  if (have_lost_since_) lost_since_ms_ += (now_ms - hop_start_ms_);
  events_.push_back(MoveEvent{now_ms, hop_lead_, hop_target_, op_, MoveReason::HopWithdraw});
}

uint8_t ChannelPlan::desired(int card) const {
  if (hopping_) return (hop_lead_ < 0 || card == hop_lead_) ? hop_target_ : op_;
  return op_;
}

std::vector<MoveEvent> ChannelPlan::take_events() {
  std::vector<MoveEvent> out;
  out.swap(events_);
  return out;
}

}  // namespace maburgs
