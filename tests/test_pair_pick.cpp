// Pair pick for a 40 MHz boot scan (docs/bw40.md "Boot scan dwells once per
// pair"): one row per pair, keyed by its primary; NHM busy % first, the
// frame score second.
#include <vector>
#include "mtest.h"
#include "pair_pick.h"
#include "mabur/ht40.h"
using namespace maburgs;

static_assert(mabur::ht40_pair_other(136) == 132 && mabur::ht40_pair_other(132) == 136, "132+136");
static_assert(mabur::ht40_pair_other(144) == 140 && mabur::ht40_pair_other(40) == 36, "");
static_assert(mabur::ht40_pair_other(128) == 124 && mabur::ht40_pair_other(165) == 0, "");

TEST(pair_ranked_on_the_primary_row_alone) {
  std::vector<RankEntry> all = {{136, 0, 3, false, 0, true, 5}, {144, 0, 2, false, 0, true, 5}};
  CHECK(any_pair_ranked(all, {136}, 3)); CHECK(!any_pair_ranked(all, {144}, 3));
  CHECK(!all_pairs_ranked(all, {136, 144}, 3)); CHECK(all_pairs_ranked(all, {136, 144}, 2));
}
TEST(lower_busy_pct_beats_lower_frame_score) {
  std::vector<RankEntry> all = {{136, 10, 3, false, 0, true, 40}, {144, 500, 3, false, 0, true, 12}};
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 20, 10) == 144);     // 40 - 12 >= 10
  CHECK(pair_ranking(all, {136, 144}, 3)[0] == 144);
}
TEST(margin_pct_holds_current_pair) {
  std::vector<RankEntry> all = {{136, 10, 3, false, 0, true, 20}, {144, 0, 3, false, 0, true, 12}};
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 20, 10) == 136);     // 8 points short of 10
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 20, 8) == 144);
}
TEST(busy_tie_falls_to_frame_score_with_event_margin) {
  std::vector<RankEntry> all = {{136, 100, 3, false, 0, true, 15}, {144, 70, 3, false, 0, true, 15}};
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 20, 10) == 144);     // 70 + 20 <= 100
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 40, 10) == 136);
}
TEST(blocked_pair_ranks_last_and_blocked_current_loses_to_any_unblocked) {
  std::vector<RankEntry> all = {{136, 0, 3, false, 0, true, 60}, {144, 900, 3, false, 0, true, 30}};
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 1000, 100, 50) == 144);
  CHECK(pair_ranking(all, {136, 144}, 3, 50)[0] == 144);
}
TEST(pair_without_busy_reading_ranks_after_pairs_with_one) {
  std::vector<RankEntry> all = {{136, 0, 3, false, 0, false, 0}, {144, 300, 3, false, 0, true, 45}};
  CHECK(pair_ranking(all, {136, 144}, 3)[0] == 144);
  CHECK(pair_proposal(all, 136, {136, 144}, 3, 20, 10) == 144);
}
TEST(unranked_current_yields_to_best_ranked_and_nothing_ranked_stays) {
  std::vector<RankEntry> a = {{136, 0, 1, false, 0, true, 0}, {144, 50, 3, false, 0, true, 20}};
  CHECK(pair_proposal(a, 136, {136, 144}, 3, 20, 10) == 144);
  std::vector<RankEntry> b = {{136, 0, 1, false, 0, true, 0}, {144, 50, 1, false, 0, true, 20}};
  CHECK(pair_proposal(b, 136, {136, 144}, 3, 20, 10) == 136);
}
TEST(config_order_breaks_full_ties) {
  std::vector<RankEntry> all = {{144, 10, 3, false, 0, true, 10}, {136, 10, 3, false, 0, true, 10}};
  CHECK(pair_ranking(all, {136, 144}, 3)[0] == 136);
}

MTEST_MAIN
