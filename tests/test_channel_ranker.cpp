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
// home_margin: a candidate replaces home only when its worst visit is at
// least `margin` busy units below home's. 0 = plain lowest-worst wins.
TEST(home_margin_keeps_home_unless_a_candidate_is_clearly_cleaner) {
  ChannelRanker r(136, {149, 165}, 1, /*home_margin=*/20);
  r.add(S(136, 6, 0, 0, 0)); r.add(S(149, 2, 0, 0, 0)); r.add(S(165, 0, 0, 0, 0));
  CHECK(r.proposal() == 136);                 // 6 vs 0: within the margin
  ChannelRanker r2(136, {149, 165}, 1, 20);
  r2.add(S(136, 30, 0, 0, 0)); r2.add(S(149, 8, 0, 0, 0)); r2.add(S(165, 12, 0, 0, 0));
  CHECK(r2.proposal() == 149);                // 30 - 8 >= 20: leave, to the best candidate
  ChannelRanker r3(136, {149}, 1, 20);
  r3.add(S(136, 25, 0, 0, 0)); r3.add(S(149, 5, 0, 0, 0));
  CHECK(r3.proposal() == 149);                // exactly 20 counts
  ChannelRanker r4(136, {149}, 1, 20);
  r4.add(S(136, 24, 0, 0, 0)); r4.add(S(149, 5, 0, 0, 0));
  CHECK(r4.proposal() == 136);                // 19 does not
  ChannelRanker r5(136, {149}, 2, 20);
  r5.add(S(149, 0, 0, 0, 0)); r5.add(S(149, 0, 0, 0, 0)); r5.add(S(136, 300, 0, 0, 0));
  CHECK(r5.proposal() == 149);                // home unranked (1 visit): best ranked wins
  ChannelRanker r6(136, {149}, 1, 0);
  r6.add(S(136, 6, 0, 0, 0)); r6.add(S(149, 2, 0, 0, 0));
  CHECK(r6.proposal() == 149);                // margin 0 = old rule
}

static maburgs::RankSample rsb(uint8_t ch, uint32_t fa, double busy) {
  maburgs::RankSample s; s.ch = ch; s.fa = fa; s.busy_valid = true; s.busy_pct = busy; return s;
}
TEST(worst_busy_visit_blocks_the_channel) {
  maburgs::ChannelRanker r(136, {144}, 1, 0, 30.0);
  r.add(rsb(144, 0, 5)); r.add(rsb(144, 0, 80));
  for (const auto& e : r.all()) if (e.ch == 144) CHECK(r.is_blocked(e));
}
TEST(blocked_home_loses_despite_margin) {
  maburgs::ChannelRanker r(136, {144}, 1, /*home_margin=*/20, 30.0);
  r.add(rsb(136, 0, 100));   // analog on home: FA 0, 100 % busy
  r.add(rsb(144, 15, 0));    // 15 events, unblocked
  CHECK(r.proposal() == 144);
}
TEST(unblocked_candidate_still_needs_the_margin) {
  maburgs::ChannelRanker r(136, {144}, 1, 20, 30.0);
  r.add(rsb(136, 10, 0)); r.add(rsb(144, 0, 0));
  CHECK(r.proposal() == 136);
}
MTEST_MAIN
