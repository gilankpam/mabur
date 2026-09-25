#include "pair_pick.h"

#include <algorithm>
#include <optional>

#include "mabur/ht40.h"

namespace maburgs {

std::vector<uint8_t> scan_half_set(uint8_t home, const std::vector<uint8_t>& candidates) {
  std::vector<uint8_t> out;
  auto add = [&out](uint8_t ch) {
    if (ch != 0 && std::find(out.begin(), out.end(), ch) == out.end()) out.push_back(ch);
  };
  auto add_pair = [&](uint8_t primary) {
    const uint8_t other = mabur::ht40_pair_other(primary);
    if (other == 0) return;
    add(std::min(primary, other));
    add(std::max(primary, other));
  };
  add_pair(home);
  for (uint8_t c : candidates) add_pair(c);
  return out;
}

namespace {
struct PairScore {
  uint8_t primary;
  uint32_t score;
  bool blocked;
  double busy;
};

// Tier first (an unblocked pair beats any blocked one; among blocked the
// less busy), then the existing worse-half event score.
bool better(const PairScore& a, const PairScore& b) {
  if (a.blocked != b.blocked) return !a.blocked;
  if (a.blocked && a.busy != b.busy) return a.busy < b.busy;
  return a.score < b.score;
}

// nullopt = unranked: no pair, a half never visited, or a half short of
// min_rounds. Both halves must be present in `all` -- a pair on half the
// evidence never wins.
std::optional<PairScore> score_pair(const std::vector<RankEntry>& all, uint8_t primary,
                                    int min_rounds, double blocked_pct) {
  const uint8_t other = mabur::ht40_pair_other(primary);
  if (other == 0) return std::nullopt;
  uint32_t worst = 0;
  bool blocked = false;
  double busy = 0.0;
  int seen = 0;
  for (const RankEntry& e : all) {
    if (e.ch != primary && e.ch != other) continue;
    if (e.visits < static_cast<uint32_t>(min_rounds)) return std::nullopt;
    worst = std::max(worst, e.worst_busy);
    blocked = blocked || (e.busy_valid && e.worst_busy_pct >= blocked_pct);
    busy = std::max(busy, e.busy_valid ? e.worst_busy_pct : 0.0);
    ++seen;
  }
  if (seen != 2) return std::nullopt;
  return PairScore{primary, worst, blocked, busy};
}
}  // namespace

uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t home,
                      const std::vector<uint8_t>& candidates, int min_rounds,
                      uint32_t home_margin, double blocked_pct) {
  const std::optional<PairScore> h = score_pair(all, home, min_rounds, blocked_pct);
  std::optional<PairScore> best;
  for (uint8_t c : candidates) {
    if (c == home) continue;
    const std::optional<PairScore> s = score_pair(all, c, min_rounds, blocked_pct);
    if (s && (!best || better(*s, *best))) best = s;   // strict: config order on ties
  }
  if (!best) return home;
  if (!h) return best->primary;                                  // home unranked: best ranked candidate
  if (h->blocked && !best->blocked) return best->primary;         // margin applies within a tier
  if (!better(*best, *h)) return home;                            // ties -> home
  // best->blocked != h->blocked is unreachable here: the (home blocked,
  // best unblocked) case already returned above, and the reverse (home
  // unblocked, best blocked) makes better(*best, *h) false, so the
  // !better(...) check above already returned home for it. Both branches
  // below this point therefore share a tier.
  return best->score + home_margin <= h->score ? best->primary : home;
}

bool any_pair_ranked(const std::vector<RankEntry>& all, uint8_t home,
                     const std::vector<uint8_t>& candidates, int min_rounds) {
  if (score_pair(all, home, min_rounds, 1e9)) return true;
  for (uint8_t c : candidates)
    if (score_pair(all, c, min_rounds, 1e9)) return true;
  return false;
}

}  // namespace maburgs
