#include "channel_ranker.h"

#include <algorithm>

namespace maburgs {

ChannelRanker::ChannelRanker(uint8_t home, const std::vector<uint8_t>& candidates,
                             int min_rounds, uint32_t home_margin)
    : home_(home), min_rounds_(min_rounds), home_margin_(home_margin) {
  entries_.push_back(RankEntry{home, 0, 0, false, 0});
  for (uint8_t c : candidates) {
    bool dup = false;
    for (const RankEntry& e : entries_) dup = dup || e.ch == c;
    if (!dup) entries_.push_back(RankEntry{c, 0, 0, false, 0});
  }
}

uint32_t ChannelRanker::busy(const RankSample& s) {
  const uint32_t cca_foreign = s.cca > s.own ? s.cca - s.own : 0;
  return cca_foreign + s.fa + s.foreign;
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
    return;
  }
}

std::vector<RankEntry> ChannelRanker::ranked() const {
  std::vector<RankEntry> out;
  for (const RankEntry& e : entries_)
    if (e.visits >= static_cast<uint32_t>(min_rounds_)) out.push_back(e);
  // Stable sort keeps config order (home first) as the final tie-break.
  // Tie-break: valid floor before invalid; among valid, lower floor_dbm first.
  std::stable_sort(out.begin(), out.end(), [](const RankEntry& a, const RankEntry& b) {
    if (a.worst_busy != b.worst_busy) return a.worst_busy < b.worst_busy;
    if (a.floor_valid != b.floor_valid) return a.floor_valid;   // valid before invalid
    if (a.floor_valid && a.floor_dbm != b.floor_dbm) return a.floor_dbm < b.floor_dbm;
    return false;                                                 // stable: config order
  });
  return out;
}

uint8_t ChannelRanker::proposal() const {
  auto k = ranked();
  if (k.empty()) return home_;
  const RankEntry& best = k.front();
  if (best.ch == home_ || home_margin_ == 0) return best.ch;
  for (const RankEntry& e : k)
    if (e.ch == home_)
      return best.worst_busy + home_margin_ <= e.worst_busy ? best.ch : home_;
  return best.ch;  // home not ranked yet: the best ranked candidate
}

}  // namespace maburgs
