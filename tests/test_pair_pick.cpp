// Pair pick for a 40 MHz boot scan (docs/bw40.md §3): halves are ranked
// as today's 20 MHz channels; a pair scores as its WORSE half and is only
// ranked once both halves have min_rounds visits.
#include <vector>
#include "mtest.h"
#include "pair_pick.h"
#include "mabur/ht40.h"
using namespace maburgs;

static_assert(mabur::ht40_pair_other(136) == 132 && mabur::ht40_pair_other(132) == 136, "132+136");
static_assert(mabur::ht40_pair_other(144) == 140 && mabur::ht40_pair_other(40) == 36, "");
static_assert(mabur::ht40_pair_other(128) == 124 && mabur::ht40_pair_other(165) == 0, "");

TEST(half_set_is_both_halves_of_home_and_candidates_in_config_order) {
  const std::vector<uint8_t> want = {132, 136, 140, 144, 36, 40, 124, 128};
  CHECK(scan_half_set(136, {144, 40, 128}) == want);
  CHECK(scan_half_set(136, {165}).size() == 2);   // no pair: skipped (config rejects it anyway)
  CHECK(scan_half_set(136, {132}).size() == 2);   // same pair as home: deduplicated
}

TEST(pair_score_is_the_worse_half_and_home_margin_applies_to_the_pair) {
  // Home 132+136: 136 clean, 132 dirty. Candidate 140+144: both moderate.
  std::vector<RankEntry> all = {{136, 0, 3, false, 0}, {132, 500, 3, false, 0},
                                {144, 100, 3, false, 0}, {140, 120, 3, false, 0}};
  CHECK(pair_proposal(all, 136, {144}, 3, 20) == 144);    // 120 + 20 <= 500
  CHECK(pair_proposal(all, 136, {144}, 3, 400) == 136);   // margin not met -> home
}

TEST(pair_unranked_until_both_halves_reach_min_rounds) {
  std::vector<RankEntry> all = {{136, 0, 3, false, 0}, {132, 0, 1, false, 0},
                                {144, 50, 3, false, 0}, {140, 50, 3, false, 0}};
  // Home's pair is unranked (132 has one visit): the best RANKED candidate
  // wins, the same rule ChannelRanker::proposal applies to an unranked home.
  CHECK(pair_proposal(all, 136, {144}, 3, 20) == 144);
  std::vector<RankEntry> none = {{136, 0, 3, false, 0}, {132, 0, 1, false, 0},
                                 {144, 50, 3, false, 0}, {140, 50, 1, false, 0}};
  CHECK(pair_proposal(none, 136, {144}, 3, 20) == 136);   // nothing ranked -> home
  std::vector<RankEntry> half = {{136, 0, 3, false, 0}, {132, 0, 3, false, 0},
                                 {144, 0, 3, false, 0}};   // 140 never visited
  CHECK(pair_proposal(half, 136, {144}, 3, 0) == 136);    // a pair on half the evidence never wins
}

TEST(pair_ties_go_home_then_a_strictly_cleaner_pair_wins_at_margin_zero) {
  std::vector<RankEntry> all = {{136, 10, 3, false, 0}, {132, 10, 3, false, 0},
                                {144, 10, 3, false, 0}, {140, 10, 3, false, 0},
                                {40, 10, 3, false, 0},  {36, 10, 3, false, 0}};
  CHECK(pair_proposal(all, 136, {144, 40}, 3, 0) == 136);
  all[4].worst_busy = 5; all[5].worst_busy = 5;
  CHECK(pair_proposal(all, 136, {144, 40}, 3, 0) == 40);
}

TEST(any_pair_ranked_needs_both_halves_of_some_pair) {
  // The boot-pick freeze's "was anything measured" test at radio.width 40:
  // one half of home's pair at min_rounds (the one-card home half also
  // collects home-window visits) is NOT a ranked pair.
  std::vector<RankEntry> one_half = {{136, 0, 3, false, 0}, {132, 0, 1, false, 0},
                                     {144, 0, 2, false, 0}, {140, 0, 2, false, 0}};
  CHECK(!any_pair_ranked(one_half, 136, {144}, 3));
  CHECK(!any_pair_ranked({{136, 0, 3, false, 0}}, 136, {144}, 3));   // 132 never visited
  CHECK(!any_pair_ranked({}, 136, {144}, 3));
  // Both halves of a candidate pair: ranked, home's pair still is not.
  std::vector<RankEntry> cand = {{136, 0, 3, false, 0}, {132, 0, 1, false, 0},
                                 {144, 0, 3, false, 0}, {140, 0, 3, false, 0}};
  CHECK(any_pair_ranked(cand, 136, {144}, 3));
  // Both halves of home's pair, no candidate ranked.
  std::vector<RankEntry> home = {{136, 0, 3, false, 0}, {132, 0, 4, false, 0},
                                 {144, 0, 1, false, 0}};
  CHECK(any_pair_ranked(home, 136, {144}, 3));
}

MTEST_MAIN
