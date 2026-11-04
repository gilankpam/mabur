#include "pair_pick.h"

#include <algorithm>
#include <optional>

namespace maburgs {

namespace {
// nullopt = unranked: the primary was never visited, or is short of
// min_rounds.
std::optional<RankEntry> row_for(const std::vector<RankEntry>& all, uint8_t primary,
                                 int min_rounds) {
  for (const RankEntry& e : all) {
    if (e.ch != primary) continue;
    if (e.visits < static_cast<uint32_t>(min_rounds)) return std::nullopt;
    return e;
  }
  return std::nullopt;
}

bool blocked(const RankEntry& e, double blocked_pct) {
  return e.busy_valid && e.worst_busy_pct >= blocked_pct;
}

// Tier, then a busy reading before none, then busy %, then frame score.
// Strict: equal rows are not better, so callers keep config order on ties.
bool better(const RankEntry& a, const RankEntry& b, double blocked_pct) {
  const bool ab = blocked(a, blocked_pct), bb = blocked(b, blocked_pct);
  if (ab != bb) return !ab;
  if (a.busy_valid != b.busy_valid) return a.busy_valid;
  if (a.busy_valid && a.worst_busy_pct != b.worst_busy_pct) return a.worst_busy_pct < b.worst_busy_pct;
  return a.worst_busy < b.worst_busy;
}
}  // namespace

uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t current,
                      const std::vector<uint8_t>& set, int min_rounds,
                      double margin_pct, double blocked_pct) {
  const std::optional<RankEntry> h = row_for(all, current, min_rounds);
  std::optional<RankEntry> best;
  for (uint8_t c : set) {
    if (c == current) continue;
    const std::optional<RankEntry> r = row_for(all, c, min_rounds);
    if (r && (!best || better(*r, *best, blocked_pct))) best = r;   // strict: config order on ties
  }
  if (!best) return current;
  if (!h) return best->ch;                                   // current unranked: best ranked candidate
  if (!better(*best, *h, blocked_pct)) return current;       // ties, and a worse tier, stay
  // From here best out-orders current: a better tier or a busy reading
  // where current has none moves at once; within the same class the margin
  // applies to the term that decided the order.
  if (blocked(*best, blocked_pct) != blocked(*h, blocked_pct)) return best->ch;
  if (best->busy_valid != h->busy_valid) return best->ch;
  if (best->busy_valid && best->worst_busy_pct != h->worst_busy_pct)
    return h->worst_busy_pct - best->worst_busy_pct >= margin_pct ? best->ch : current;
  return current;   // an exact busy tie, or neither side read: stay
}

bool any_pair_ranked(const std::vector<RankEntry>& all, const std::vector<uint8_t>& set,
                     int min_rounds) {
  for (uint8_t c : set)
    if (row_for(all, c, min_rounds)) return true;
  return false;
}

bool all_pairs_ranked(const std::vector<RankEntry>& all, const std::vector<uint8_t>& set,
                      int min_rounds) {
  for (uint8_t c : set)
    if (!row_for(all, c, min_rounds)) return false;
  return !set.empty();
}

std::vector<uint8_t> pair_ranking(const std::vector<RankEntry>& all,
                                  const std::vector<uint8_t>& set, int min_rounds,
                                  double blocked_pct) {
  std::vector<RankEntry> rows;
  for (uint8_t c : set)
    if (auto r = row_for(all, c, min_rounds)) rows.push_back(*r);
  std::stable_sort(rows.begin(), rows.end(), [blocked_pct](const RankEntry& a, const RankEntry& b) {
    return better(a, b, blocked_pct);
  });
  std::vector<uint8_t> out;
  for (const RankEntry& r : rows) out.push_back(r.ch);
  return out;
}

}  // namespace maburgs
