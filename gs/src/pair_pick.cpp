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
};

// nullopt = unranked: no pair, a half never visited, or a half short of
// min_rounds. Both halves must be present in `all` -- a pair on half the
// evidence never wins.
std::optional<PairScore> score_pair(const std::vector<RankEntry>& all, uint8_t primary,
                                    int min_rounds) {
  const uint8_t other = mabur::ht40_pair_other(primary);
  if (other == 0) return std::nullopt;
  uint32_t worst = 0;
  int seen = 0;
  for (const RankEntry& e : all) {
    if (e.ch != primary && e.ch != other) continue;
    if (e.visits < static_cast<uint32_t>(min_rounds)) return std::nullopt;
    worst = std::max(worst, e.worst_busy);
    ++seen;
  }
  if (seen != 2) return std::nullopt;
  return PairScore{primary, worst};
}
}  // namespace

uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t home,
                      const std::vector<uint8_t>& candidates, int min_rounds,
                      uint32_t home_margin) {
  const std::optional<PairScore> h = score_pair(all, home, min_rounds);
  std::optional<PairScore> best;
  for (uint8_t c : candidates) {
    if (c == home) continue;
    const std::optional<PairScore> s = score_pair(all, c, min_rounds);
    if (s && (!best || s->score < best->score)) best = s;   // strict: config order on ties
  }
  if (!best) return home;
  if (!h) return best->primary;             // home unranked: best ranked candidate
  if (best->score >= h->score) return home; // ties -> home
  return best->score + home_margin <= h->score ? best->primary : home;
}

}  // namespace maburgs
