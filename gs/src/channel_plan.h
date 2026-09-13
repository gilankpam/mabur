#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace maburgs {

enum class MoveReason { Commit, AckOverride, SplitHome, Reunite };
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
};

}  // namespace maburgs
