#pragma once
#include <cstdint>
#include <vector>

namespace maburgs {

// One scout visit's worth of the scouting card's own counters. Never
// compared across cards.
struct RankSample {
  uint8_t ch = 0;
  uint32_t cca = 0, fa = 0, own = 0, foreign = 0;
  uint32_t leak = 0;  // own-TX leak estimate (frames x k), subtracted with own
  bool floor_valid = false;
  int8_t floor_dbm = 0;
  bool busy_valid = false;
  double busy_pct = 0;
};

struct RankEntry {
  uint8_t ch = 0;
  uint32_t worst_busy = 0;
  uint32_t visits = 0;
  bool floor_valid = false;
  int8_t floor_dbm = 0;
  bool busy_valid = false;
  double worst_busy_pct = 0;
};

// Pure boot-time ranking: worst visit wins-or-loses; tie-break by valid floor
// before none, lower floor_dbm first among valid, then config order. No
// "clean" bar beyond the margin below. The blocked tier sorts ahead of
// worst_busy: a channel whose worst-visit NHM busy % reaches blocked_pct
// ranks after every unblocked channel regardless of worst_busy, tiebroken
// among the blocked by lower busy -- see
// is_blocked() below.
//
// There is no fixed home: proposal(current) plays the current channel's
// row in the margin check below. The margin is a busy-%-points comparison
// (ranked()'s own ordering, mirrored in pair_pick.h's pair_proposal() and
// cal_round_pick.h): current/the best candidate's row is left only when
// the best is in a better tier, has a busy reading where current has
// none, or both read and current's busy % beats the best's by margin_pct
// points. An exact tie, or neither side reading busy, stays.
class ChannelRanker {
 public:
  // margin_pct: a candidate replaces `current` only when its worst-visit
  // busy % reads at least this many points below current's (0 = lowest
  // busy wins outright, even on an exact tie -- the original rule). Bench:
  // clean channels tie within a few points, so without it the pick among
  // clean channels is a coin toss that costs a calibrated current channel
  // for nothing.
  ChannelRanker(std::vector<uint8_t> channels, int min_rounds, double margin_pct = 0,
                double blocked_pct = 1e9);
  void add(const RankSample& s);
  std::vector<RankEntry> ranked() const { return ranked(min_rounds_); }
  // Same, with a caller-chosen visit bar (the scout's one-card deadline
  // ranks on 1 visit before min_rounds full passes have run).
  std::vector<RankEntry> ranked(int min_rounds) const;
  std::vector<RankEntry> all() const { return entries_; }
  uint8_t proposal(uint8_t current) const { return proposal(current, min_rounds_); }
  uint8_t proposal(uint8_t current, int min_rounds) const;
  static uint32_t busy(const RankSample& s);
  // A channel whose worst-visit NHM busy % has reached blocked_pct: ranks
  // last regardless of event score.
  bool is_blocked(const RankEntry& e) const { return e.busy_valid && e.worst_busy_pct >= blocked_pct_; }

 private:
  int min_rounds_;
  double margin_pct_;
  double blocked_pct_;
  std::vector<RankEntry> entries_;  // config order
};

}  // namespace maburgs
