#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace maburgs {

enum class MoveReason { Commit, AckOverride, SplitHome, Reunite, HopLead, HopFollow, HopWithdraw, HopOneCard };
const char* to_string(MoveReason r);

struct MoveEvent {
  double t_ms = 0;
  int card = -1;  // -1 = all cards
  uint8_t from = 0, to = 0;
  MoveReason reason = MoveReason::Commit;
};

struct ChannelPlanCfg {
  uint8_t home = 149;
  int n_cards = 2;
  int split_after_ms = 5000;
  int home_window_ms = 300;
  int beacon_period_ms = 20;
};

// Where the link lives (spec 2026-09-13-auto-channel-select §6, GS side).
// Pure: the caller passes the clock and the rendezvous state; the plan
// answers "which channel should card i be on" and "which cards carry this
// DISC", and records the moves that change where the link lives. Scout
// dwells and one-card interleave hops are not moves.
class ChannelPlan {
 public:
  explicit ChannelPlan(ChannelPlanCfg cfg);
  void tick(double now_ms, bool in_session);
  void on_ack(double now_ms, uint8_t agreed, uint8_t proposed);
  uint8_t op() const { return op_; }
  bool frozen() const { return frozen_; }
  bool split() const { return split_; }
  uint8_t desired(int card) const;
  std::optional<std::vector<int>> beacon_cards() const;
  std::vector<MoveEvent> take_events();

  // In-flight channel hop: one card leads onto target, the other keeps the
  // link alive on op_ until hop_confirmed() (video seen on target) or
  // hop_withdraw() (no video, lead card returns). op_ only moves on confirm.
  void hop_order(double now_ms, uint8_t target, int lead_card);
  // Both are no-ops when no hop is in flight: the caller's hop state
  // machine and this plan do not enter hopping_ at the same instant (a
  // one-card Order defers hop_order() until OneCardRetune), so a withdraw
  // can legitimately arrive with nothing to withdraw.
  void hop_confirmed(double now_ms);
  void hop_withdraw(double now_ms);
  // Does a video body received on rx_ch keep the session alive? Only where
  // the link lives: op, or the hop target while a hop is in flight (the lead
  // card's confirming frames). Scout dwells elsewhere must not -- bench
  // 2026-09-26: home-dwell frames held a split GS in SESSION for 60 s.
  // rx_ch 0 = mid-retune or a replay source: unknown, counts.
  bool is_link_video(uint8_t rx_ch) const {
    return rx_ch == 0 || rx_ch == op_ || (hopping_ && rx_ch == hop_target_);
  }
  bool hopping() const { return hopping_; }
  uint8_t hop_target() const { return hop_target_; }
  int hop_lead() const { return hop_lead_; }

 private:
  // One-card interleave: window index since split; even = home, odd = op.
  int window_(double now_ms) const;
  bool quiet_gap_(double now_ms) const;
  void reunite_(double now_ms);

  ChannelPlanCfg cfg_;
  uint8_t op_;
  bool frozen_ = false;
  bool split_ = false;
  bool have_lost_since_ = false;
  double lost_since_ms_ = 0;
  double split_at_ms_ = 0;
  double now_ms_ = 0;
  std::vector<MoveEvent> events_;

  bool hopping_ = false;
  uint8_t hop_target_ = 0;
  int hop_lead_ = -1;
  uint8_t hop_from_ = 0;
  double hop_start_ms_ = 0;
};

}  // namespace maburgs
