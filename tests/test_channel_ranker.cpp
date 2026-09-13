#include "mtest.h"
#include "channel_ranker.h"
using namespace maburgs;

static RankSample S(uint8_t ch, uint32_t cca, uint32_t fa, uint32_t own, uint32_t foreign,
                    bool fv = false, int8_t f = 0) {
  RankSample s; s.ch = ch; s.cca = cca; s.fa = fa; s.own = own; s.foreign = foreign;
  s.floor_valid = fv; s.floor_dbm = f; return s;
}

TEST(busy_subtracts_own_frames_and_never_underflows) {
  CHECK(ChannelRanker::busy(S(1, 100, 5, 40, 2)) == 67);   // (100-40)+5+2
  CHECK(ChannelRanker::busy(S(1, 10, 0, 40, 0)) == 0);     // own > cca clamps
}

TEST(unranked_until_min_rounds) {
  ChannelRanker r(136, {149, 153}, 3);
  r.add(S(149, 0, 0, 0, 0)); r.add(S(149, 0, 0, 0, 0));
  r.add(S(136, 500, 0, 0, 0)); r.add(S(136, 500, 0, 0, 0)); r.add(S(136, 500, 0, 0, 0));
  CHECK(r.ranked().size() == 1);          // only home has 3 visits
  CHECK(r.proposal() == 136);
  r.add(S(149, 0, 0, 0, 0));
  CHECK(r.ranked().size() == 2);
  CHECK(r.proposal() == 149);
}

TEST(worst_visit_beats_mean) {
  ChannelRanker r(136, {149}, 2);
  r.add(S(136, 5, 0, 0, 0)); r.add(S(136, 5, 0, 0, 0));      // steady 5
  r.add(S(149, 0, 0, 0, 0)); r.add(S(149, 900, 0, 0, 0));    // mean 450, worst 900
  auto k = r.ranked();
  REQUIRE(k.size() == 2);
  CHECK(k[0].ch == 136); CHECK(k[0].worst_busy == 5);
  CHECK(k[1].ch == 149); CHECK(k[1].worst_busy == 900);
}

TEST(floor_tiebreak_only_when_both_valid_and_home_first_otherwise) {
  ChannelRanker r(136, {149, 153}, 1);
  r.add(S(136, 3, 0, 0, 0, true, -90));
  r.add(S(149, 3, 0, 0, 0, true, -96));
  r.add(S(153, 3, 0, 0, 0));                 // no floor
  auto k = r.ranked();
  REQUIRE(k.size() == 3);
  CHECK(k[0].ch == 149);                     // lower floor wins vs 136
  CHECK(k[1].ch == 136);                     // 136 vs 153: no common floor -> home first
  CHECK(k[2].ch == 153);
}

TEST(config_order_final_tiebreak_and_home_dedup) {
  ChannelRanker r(136, {153, 136, 149}, 1);
  r.add(S(153, 1, 0, 0, 0)); r.add(S(149, 1, 0, 0, 0)); r.add(S(136, 7, 0, 0, 0));
  auto a = r.all();
  REQUIRE(a.size() == 3);                    // home listed once
  CHECK(a[0].ch == 136); CHECK(a[1].ch == 153); CHECK(a[2].ch == 149);
  auto k = r.ranked();
  CHECK(k[0].ch == 153); CHECK(k[1].ch == 149); CHECK(k[2].ch == 136);
}

TEST(unknown_channel_ignored) {
  ChannelRanker r(136, {149}, 1);
  r.add(S(44, 0, 0, 0, 0));
  CHECK(r.ranked().empty());
  CHECK(r.proposal() == 136);
}

TEST(floor_tiebreak_is_transitive_with_a_floorless_entry_between) {
  ChannelRanker r(1, {2, 3, 4}, 1);
  r.add(S(1, 0, 0, 0, 0));
  r.add(S(2, 0, 0, 0, 0, true, -70));
  r.add(S(3, 0, 0, 0, 0));
  r.add(S(4, 0, 0, 0, 0, true, -95));
  auto k = r.ranked();
  REQUIRE(k.size() == 4);
  CHECK(k[0].ch == 4); CHECK(k[1].ch == 2); CHECK(k[2].ch == 1); CHECK(k[3].ch == 3);
}
MTEST_MAIN
