#pragma once
#include <cstdint>
#include <vector>

namespace maburgs {

// One scout visit's worth of the scouting card's own counters (spec
// 2026-09-13-auto-channel-select §5). Never compared across cards.
struct RankSample {
  uint8_t ch = 0;
  uint32_t cca = 0, fa = 0, own = 0, foreign = 0;
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
// before none, lower floor_dbm first among valid, then config order (home first).
// No improvement margin, no "clean" bar. The blocked tier sorts ahead of
// worst_busy: a channel whose worst-visit NHM busy % reaches blocked_pct
// ranks after every unblocked channel regardless of worst_busy, tiebroken
// among the blocked by lower busy (spec 2026-09-25-nhm-airtime §7) -- see
// is_blocked() below.
class ChannelRanker {
 public:
  // home_margin: a candidate replaces home only when its worst visit is at
  // least this many busy units below home's (0 = lowest worst wins, the
  // original rule). Bench 2026-09-13: clean channels tie within ~10 units,
  // so without it the pick among clean channels is a coin toss that costs
  // a calibrated home for nothing.
  ChannelRanker(uint8_t home, const std::vector<uint8_t>& candidates, int min_rounds,
                uint32_t home_margin = 0, double blocked_pct = 1e9);
  void add(const RankSample& s);
  std::vector<RankEntry> ranked() const;
  std::vector<RankEntry> all() const { return entries_; }
  uint8_t proposal() const;
  uint8_t home() const { return home_; }
  static uint32_t busy(const RankSample& s);
  // A channel whose worst-visit NHM busy % has reached blocked_pct: ranks
  // last regardless of event score (spec 2026-09-25-nhm-airtime §7).
  bool is_blocked(const RankEntry& e) const { return e.busy_valid && e.worst_busy_pct >= blocked_pct_; }

 private:
  uint8_t home_;
  int min_rounds_;
  uint32_t home_margin_;
  double blocked_pct_;
  std::vector<RankEntry> entries_;  // config order, home at [0]
};

}  // namespace maburgs
