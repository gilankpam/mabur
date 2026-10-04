#include "spotter_follow.h"

namespace webgs {

const char* to_string(FollowState s) {
  switch (s) {
    case FollowState::Locked: return "locked";
    case FollowState::Following: return "following";
    default: return "sweeping";
  }
}

SpotterFollow::SpotterFollow(SpotterFollowCfg cfg) : cfg_(std::move(cfg)) {
  if (cfg_.channels.empty()) cfg_.channels = {cfg_.start};
  const std::size_t idx = member_(cfg_.start) ? index_(cfg_.start) : 0;
  state_ = FollowState::Sweeping;
  desired_ = cfg_.channels[idx];
  // t_request_ms_ is stamped by the first tick()/on_card_channel(): the
  // constructor has no clock.
}

bool SpotterFollow::member_(uint8_t ch) const {
  for (uint8_t c : cfg_.channels)
    if (c == ch) return true;
  return false;
}

std::size_t SpotterFollow::index_(uint8_t ch) const {
  for (std::size_t i = 0; i < cfg_.channels.size(); ++i)
    if (cfg_.channels[i] == ch) return i;
  return 0;
}

void SpotterFollow::lock_(uint8_t ch, double now_ms) {
  state_ = FollowState::Locked;
  desired_ = ch;
  last_heard_ms_ = now_ms;
  t_on_member_ms_.reset();
  t_request_ms_.reset();
}

void SpotterFollow::sweep_at_(std::size_t idx, double now_ms) {
  state_ = FollowState::Sweeping;
  desired_ = cfg_.channels[idx % cfg_.channels.size()];
  t_request_ms_ = now_ms;
  t_on_member_ms_.reset();
}

void SpotterFollow::follow_(uint8_t target, double now_ms) {
  if (state_ != FollowState::Following) back_ = desired_;
  state_ = FollowState::Following;
  desired_ = target;
  t_order_ms_ = now_ms;
  t_on_member_ms_.reset();
  t_request_ms_.reset();
  ++follows_;
}

void SpotterFollow::on_frame(uint8_t rx_ch, double now_ms) {
  if (rx_ch == 0) return;
  switch (state_) {
    case FollowState::Locked:
      if (rx_ch == desired_) last_heard_ms_ = now_ms;
      break;
    case FollowState::Following:
    case FollowState::Sweeping:
      if (rx_ch == desired_) lock_(rx_ch, now_ms);
      break;
  }
}

void SpotterFollow::on_rcf(uint8_t hop_ch, uint8_t hop_epoch, uint8_t rx_ch, double now_ms) {
  if (rx_ch == 0) return;
  // An RCF is a frame too: a sweep hearing the GS on the member locks first.
  on_frame(rx_ch, now_ms);
  const std::pair<uint8_t, uint8_t> pair{hop_ch, hop_epoch};
  if (last_pair_ && *last_pair_ == pair) return;
  last_pair_ = pair;
  if (hop_ch == 0 || !member_(hop_ch)) return;
  switch (state_) {
    case FollowState::Locked:
      if (rx_ch == desired_ && hop_ch != desired_) follow_(hop_ch, now_ms);
      break;
    case FollowState::Following:
      if (hop_ch == back_) {
        lock_(back_, now_ms);          // withdrawn before the card moved
      } else if (hop_ch != desired_) {
        follow_(hop_ch, now_ms);       // retargeted; back_ stays
      }
      break;
    case FollowState::Sweeping:
      break;                           // not on the member (on_frame would have locked)
  }
}

void SpotterFollow::on_card_channel(uint8_t ch, double now_ms) {
  if (state_ != FollowState::Sweeping || ch != desired_) return;
  if (!t_request_ms_) t_request_ms_ = now_ms;
  if (!t_on_member_ms_) t_on_member_ms_ = now_ms;
}

void SpotterFollow::tick(double now_ms) {
  switch (state_) {
    case FollowState::Locked:
      if (now_ms - last_heard_ms_ > cfg_.silence_ms) sweep_at_(index_(desired_) + 1, now_ms);
      break;
    case FollowState::Following:
      if (now_ms - t_order_ms_ > cfg_.confirm_ms) lock_(back_, now_ms);
      break;
    case FollowState::Sweeping:
      if (!t_request_ms_) t_request_ms_ = now_ms;
      if (t_on_member_ms_) {
        if (now_ms - *t_on_member_ms_ >= cfg_.dwell_ms) sweep_at_(index_(desired_) + 1, now_ms);
      } else if (now_ms - *t_request_ms_ >= cfg_.tune_timeout_ms) {
        sweep_at_(index_(desired_) + 1, now_ms);
      }
      break;
  }
}

}  // namespace webgs
