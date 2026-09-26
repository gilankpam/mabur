#include "hop_ranker.h"

#include <algorithm>

namespace maburgs {

HopRanker::HopRanker(HopCfg cfg, std::vector<uint8_t> candidates, uint8_t home, uint8_t boot_pick)
    : cfg_(cfg), candidates_(std::move(candidates)), home_(home), boot_pick_(boot_pick) {
  if (std::find(candidates_.begin(), candidates_.end(), home_) == candidates_.end())
    candidates_.push_back(home_);
  visits_.resize(candidates_.size());
}

void HopRanker::add(const HopVisit& v) {
  for (size_t i = 0; i < candidates_.size(); ++i) {
    if (candidates_[i] != v.ch) continue;
    visits_[i].push_back(v);
    while (visits_[i].size() > static_cast<size_t>(cfg_.rank_visits)) visits_[i].pop_front();
    return;
  }
}

uint32_t HopRanker::score(const HopVisit& v) {
  const int64_t busy = static_cast<int64_t>(v.cca) - static_cast<int64_t>(v.own);
  return v.fa + static_cast<uint32_t>(std::max<int64_t>(busy, 0)) + 4 * v.foreign;
}

std::vector<HopRankEntry> HopRanker::ranking(double now_ms) const {
  std::vector<HopRankEntry> entries;
  entries.reserve(candidates_.size());
  for (size_t i = 0; i < candidates_.size(); ++i) {
    HopRankEntry e;
    e.ch = candidates_[i];
    int fresh = 0, busy_n = 0;
    uint32_t sum = 0;
    double busy_sum = 0;
    for (const auto& v : visits_[i]) {
      if (now_ms - v.t_ms > cfg_.rank_max_age_ms) continue;
      ++fresh;
      sum += score(v);
      if (v.busy_valid) { ++busy_n; busy_sum += v.busy_pct; }
    }
    e.visits = fresh;
    e.score = sum;
    e.ranked = fresh >= 2;
    e.busy_pct = busy_n ? busy_sum / busy_n : 0.0;
    e.blocked = busy_n > 0 && e.busy_pct >= cfg_.verdict.blocked_pct;
    entries.push_back(e);
  }
  // Ranked first, then by score; ties among RANKED entries -> boot-time
  // pick, then home, then config order (spec §3: "ties -> boot-time pick,
  // then home"). Unranked entries skip both tiebreaks and fall straight
  // to config order, so their relative order stays pure config order
  // (stable_sort preserving entries' original, candidates_-derived
  // order).
  std::stable_sort(entries.begin(), entries.end(), [&](const HopRankEntry& a, const HopRankEntry& b) {
    if (a.ranked != b.ranked) return a.ranked;
    if (a.ranked) {
      if (a.blocked != b.blocked) return !a.blocked;   // blocked tier ranks last
      if (a.blocked && a.busy_pct != b.busy_pct) return a.busy_pct < b.busy_pct;
      if (a.score != b.score) return a.score < b.score;
      const bool a_boot = a.ch == boot_pick_, b_boot = b.ch == boot_pick_;
      if (a_boot != b_boot) return a_boot;
      const bool a_home = a.ch == home_, b_home = b.ch == home_;
      if (a_home != b_home) return a_home;
    }
    return false;
  });
  return entries;
}

std::optional<uint8_t> HopRanker::best(double now_ms, uint8_t exclude, const std::vector<uint8_t>& skip,
                                       bool require_unblocked) const {
  for (const auto& e : ranking(now_ms)) {
    if (!e.ranked || e.ch == exclude) continue;
    if (require_unblocked && e.blocked) continue;
    if (std::find(skip.begin(), skip.end(), e.ch) != skip.end()) continue;
    return e.ch;
  }
  return std::nullopt;
}

}  // namespace maburgs
