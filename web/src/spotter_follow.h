#pragma once
// SpotterFollow: the spotter page's channel follower (spec 2026-10-04
// web-gs-channel-core §6). Pure: injected clock, no I/O. Decides the member
// the one card should sit on; WebGs (spotter mode) feeds it and retunes.
//
//  Sweeping  round-robin over the set; a member's dwell counts from the
//            card's report (on_card_channel); a frame on the member locks.
//            No terminal state: with nothing on the air it keeps sweeping.
//            A member the card never reports is skipped after tune_timeout_ms.
//  Locked    a frame on cur refreshes last_heard; silence_ms without one ->
//            Sweeping from the next member. An RCF ordering another member
//            (new (hop_ch, hop_epoch) pair, hop_ch a member != cur) -> Following.
//  Following desired = target, back = where we were. A frame on target ->
//            Locked(target). confirm_ms with nothing on target -> back,
//            Locked(back) with last_heard = now. A new order to `back` heard
//            before the card moved (a withdrawal) returns at once; a new
//            order elsewhere retargets (the confirm clock restarts).
//
// The RCF pair is read in the clear and never verified (§6.2). The same
// pair repeated is ignored (idempotent orders); hop_ch 0 / a non-member is
// recorded as seen and ignored. rx_channel 0 (a frame delivered across a
// retune) is ignored everywhere.
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace webgs {

struct SpotterFollowCfg {
  std::vector<uint8_t> channels;   // the set (1..8 members, validated upstream)
  uint8_t start = 0;               // pin, else remembered, else channels[0]; a non-member -> channels[0]
  int silence_ms = 1000;           // locked, nothing heard this long -> sweep
  int confirm_ms = 2000;           // following: nothing on the target this long -> return
  int dwell_ms = 150;              // sweeping: per member, from the card's report
  int tune_timeout_ms = 1000;      // sweeping: no card report this long -> next member
};

enum class FollowState { Locked, Following, Sweeping };
const char* to_string(FollowState s);   // "locked" | "following" | "sweeping"

class SpotterFollow {
 public:
  explicit SpotterFollow(SpotterFollowCfg cfg);
  // Any CRC-good canonical body (video, telem, DISC, RCF) heard on rx_ch.
  void on_frame(uint8_t rx_ch, double now_ms);
  // A parsed RCF heard on rx_ch. Call on_frame for the same body first (or
  // not at all: on_rcf locks a sweep on its own when rx_ch is the member).
  void on_rcf(uint8_t hop_ch, uint8_t hop_epoch, uint8_t rx_ch, double now_ms);
  // The card reports it is ready on ch (the sweep dwell clock starts).
  void on_card_channel(uint8_t ch, double now_ms);
  void tick(double now_ms);
  uint8_t desired() const { return desired_; }
  FollowState state() const { return state_; }
  uint64_t follows() const { return follows_; }

 private:
  bool member_(uint8_t ch) const;
  std::size_t index_(uint8_t ch) const;
  void lock_(uint8_t ch, double now_ms);
  void sweep_at_(std::size_t idx, double now_ms);
  void follow_(uint8_t target, double now_ms);

  SpotterFollowCfg cfg_;
  FollowState state_ = FollowState::Sweeping;
  uint8_t desired_ = 0;
  uint8_t back_ = 0;                        // Following: where to return
  double last_heard_ms_ = 0;                // Locked
  double t_order_ms_ = 0;                   // Following: the last order
  std::optional<double> t_request_ms_;      // Sweeping: when desired_ was requested (set on the first clock seen)
  std::optional<double> t_on_member_ms_;    // Sweeping: on_card_channel(desired_)
  std::optional<std::pair<uint8_t, uint8_t>> last_pair_;   // (hop_ch, hop_epoch) last seen
  uint64_t follows_ = 0;
};

}  // namespace webgs
