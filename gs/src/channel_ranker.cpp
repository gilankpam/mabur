#include "channel_ranker.h"

#include <algorithm>

namespace maburgs {

ChannelRanker::ChannelRanker(std::vector<uint8_t> channels, int min_rounds, uint32_t margin,
                             double blocked_pct)
    : min_rounds_(min_rounds), margin_(margin), blocked_pct_(blocked_pct) {
  for (uint8_t c : channels) {
    bool dup = false;
    for (const RankEntry& e : entries_) dup = dup || e.ch == c;
    if (!dup) entries_.push_back(RankEntry{c, 0, 0, false, 0});
  }
}

uint32_t ChannelRanker::busy(const RankSample& s) {
  const int64_t foreign_cca = static_cast<int64_t>(s.cca) - s.own - s.leak;
  return static_cast<uint32_t>(std::max<int64_t>(foreign_cca, 0)) + s.fa + s.foreign;
}

void ChannelRanker::add(const RankSample& s) {
  for (RankEntry& e : entries_) {
    if (e.ch != s.ch) continue;
    const uint32_t b = busy(s);
    if (e.visits == 0 || b > e.worst_busy) e.worst_busy = b;
    ++e.visits;
    if (s.floor_valid && (!e.floor_valid || s.floor_dbm > e.floor_dbm)) {
      e.floor_valid = true;
      e.floor_dbm = s.floor_dbm;
    }
    if (s.busy_valid && (!e.busy_valid || s.busy_pct > e.worst_busy_pct)) {
      e.busy_valid = true;
      e.worst_busy_pct = s.busy_pct;
    }
    return;
  }
}

std::vector<RankEntry> ChannelRanker::ranked(int min_rounds) const {
  std::vector<RankEntry> out;
  for (const RankEntry& e : entries_)
    if (e.visits >= static_cast<uint32_t>(min_rounds)) out.push_back(e);
  // Stable sort keeps config order as the final tie-break.
  // Tie-break: valid floor before invalid; among valid, lower floor_dbm first.
  std::stable_sort(out.begin(), out.end(), [this](const RankEntry& a, const RankEntry& b) {
    const bool ab = is_blocked(a), bb = is_blocked(b);
    if (ab != bb) return !ab;                                   // blocked tier last
    if (ab && a.worst_busy_pct != b.worst_busy_pct) return a.worst_busy_pct < b.worst_busy_pct;
    if (a.worst_busy != b.worst_busy) return a.worst_busy < b.worst_busy;
    if (a.floor_valid != b.floor_valid) return a.floor_valid;   // valid before invalid
    if (a.floor_valid && a.floor_dbm != b.floor_dbm) return a.floor_dbm < b.floor_dbm;
    return false;                                                 // stable: config order
  });
  return out;
}

uint8_t ChannelRanker::proposal(uint8_t current, int min_rounds) const {
  auto k = ranked(min_rounds);
  if (k.empty()) return current;
  const RankEntry& best = k.front();
  if (best.ch == current || margin_ == 0) return best.ch;
  for (const RankEntry& e : k)
    if (e.ch == current) {
      if (is_blocked(e) && !is_blocked(best)) return best.ch;   // margin applies within a tier
      return best.worst_busy + margin_ <= e.worst_busy ? best.ch : current;
    }
  return best.ch;  // current not ranked yet: the best ranked one
}

}  // namespace maburgs
