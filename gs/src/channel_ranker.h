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
};

struct RankEntry {
  uint8_t ch = 0;
  uint32_t worst_busy = 0;
  uint32_t visits = 0;
  bool floor_valid = false;
  int8_t floor_dbm = 0;
};

// Pure boot-time ranking: worst visit wins-or-loses, floor as tie-break,
// home first, then config order. No improvement margin, no "clean" bar.
class ChannelRanker {
 public:
  ChannelRanker(uint8_t home, const std::vector<uint8_t>& candidates, int min_rounds);
  void add(const RankSample& s);
  std::vector<RankEntry> ranked() const;
  std::vector<RankEntry> all() const { return entries_; }
  uint8_t proposal() const;
  uint8_t home() const { return home_; }
  static uint32_t busy(const RankSample& s);

 private:
  uint8_t home_;
  int min_rounds_;
  std::vector<RankEntry> entries_;  // config order, home at [0]
};

}  // namespace maburgs
