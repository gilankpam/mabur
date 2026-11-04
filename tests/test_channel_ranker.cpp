#include "mtest.h"
#include "channel_ranker.h"
using namespace maburgs;

static RankSample S(uint8_t ch, uint32_t cca, uint32_t fa, uint32_t own, uint32_t foreign,
                    bool fv = false, int8_t f = 0, bool bv = false, double bp = 0) {
  RankSample s; s.ch = ch; s.cca = cca; s.fa = fa; s.own = own; s.foreign = foreign;
  s.floor_valid = fv; s.floor_dbm = f; s.busy_valid = bv; s.busy_pct = bp; return s;
}

TEST(busy_subtracts_own_frames_and_never_underflows) {
  CHECK(ChannelRanker::busy(S(1, 100, 5, 40, 2)) == 67);   // (100-40)+5+2
  CHECK(ChannelRanker::busy(S(1, 10, 0, 40, 0)) == 0);     // own > cca clamps
}

TEST(unranked_until_min_rounds) {
  ChannelRanker r({136, 149, 153}, 3);
  r.add(S(149, 0, 0, 0, 0)); r.add(S(149, 0, 0, 0, 0));
  r.add(S(136, 500, 0, 0, 0)); r.add(S(136, 500, 0, 0, 0)); r.add(S(136, 500, 0, 0, 0));
  CHECK(r.ranked().size() == 1);          // only home has 3 visits
  CHECK(r.proposal(136) == 136);
  r.add(S(149, 0, 0, 0, 0));
  CHECK(r.ranked().size() == 2);
  CHECK(r.proposal(136) == 149);
}

TEST(worst_visit_beats_mean) {
  ChannelRanker r({136, 149}, 2);
  r.add(S(136, 5, 0, 0, 0)); r.add(S(136, 5, 0, 0, 0));      // steady 5
  r.add(S(149, 0, 0, 0, 0)); r.add(S(149, 900, 0, 0, 0));    // mean 450, worst 900
  auto k = r.ranked();
  REQUIRE(k.size() == 2);
  CHECK(k[0].ch == 136); CHECK(k[0].worst_busy == 5);
  CHECK(k[1].ch == 149); CHECK(k[1].worst_busy == 900);
}

TEST(floor_tiebreak_only_when_both_valid_and_home_first_otherwise) {
  ChannelRanker r({136, 149, 153}, 1);
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
  ChannelRanker r({136, 153, 136, 149}, 1);
  r.add(S(153, 1, 0, 0, 0)); r.add(S(149, 1, 0, 0, 0)); r.add(S(136, 7, 0, 0, 0));
  auto a = r.all();
  REQUIRE(a.size() == 3);                    // home listed once
  CHECK(a[0].ch == 136); CHECK(a[1].ch == 153); CHECK(a[2].ch == 149);
  auto k = r.ranked();
  CHECK(k[0].ch == 153); CHECK(k[1].ch == 149); CHECK(k[2].ch == 136);
}

TEST(unknown_channel_ignored) {
  ChannelRanker r({136, 149}, 1);
  r.add(S(44, 0, 0, 0, 0));
  CHECK(r.ranked().empty());
  CHECK(r.proposal(136) == 136);
}

TEST(floor_tiebreak_is_transitive_with_a_floorless_entry_between) {
  ChannelRanker r({1, 2, 3, 4}, 1);
  r.add(S(1, 0, 0, 0, 0));
  r.add(S(2, 0, 0, 0, 0, true, -70));
  r.add(S(3, 0, 0, 0, 0));
  r.add(S(4, 0, 0, 0, 0, true, -95));
  auto k = r.ranked();
  REQUIRE(k.size() == 4);
  CHECK(k[0].ch == 4); CHECK(k[1].ch == 2); CHECK(k[2].ch == 1); CHECK(k[3].ch == 3);
}
// margin_pct: a candidate replaces current only when its busy % reads at
// least `margin_pct` points below current's. 0 = plain lowest-busy wins.
TEST(busy_margin_keeps_home_unless_a_candidate_is_clearly_cleaner) {
  ChannelRanker r({136, 149, 165}, 1, /*margin_pct=*/10);
  r.add(S(136, 0, 0, 0, 0, false, 0, true, 20));
  r.add(S(149, 0, 0, 0, 0, false, 0, true, 11));
  r.add(S(165, 0, 0, 0, 0, false, 0, true, 30));
  CHECK(r.proposal(136) == 136);                 // 20 - 11 = 9 < 10: stays
  ChannelRanker r2({136, 149, 165}, 1, 10);
  r2.add(S(136, 0, 0, 0, 0, false, 0, true, 20));
  r2.add(S(149, 0, 0, 0, 0, false, 0, true, 10));
  r2.add(S(165, 0, 0, 0, 0, false, 0, true, 30));
  CHECK(r2.proposal(136) == 149);                // 20 - 10 = 10 >= 10: moves
  ChannelRanker r3({136, 149}, 1, 0);
  r3.add(S(136, 0, 0, 0, 0, false, 0, true, 20));
  r3.add(S(149, 0, 0, 0, 0, false, 0, true, 19));
  CHECK(r3.proposal(136) == 149);                // margin_pct 0: lowest busy wins
  ChannelRanker r4({136, 149}, 2, 10);
  r4.add(S(149, 0, 0, 0, 0, false, 0, true, 5)); r4.add(S(149, 0, 0, 0, 0, false, 0, true, 5));
  r4.add(S(136, 0, 0, 0, 0, false, 0, true, 80));
  CHECK(r4.proposal(136) == 149);                // home unranked (1 visit): best ranked wins
}
// Ruling: ranked() sorts the unblocked tier by busy % before frame score,
// so proposal()'s margin check always compares against the lowest-busy
// candidate, not the lowest-frame-score one.
TEST(ranked_sorts_by_busy_before_frame_score_in_the_unblocked_tier) {
  ChannelRanker r({136, 149}, 1);
  r.add(S(136, 500, 0, 0, 0, false, 0, true, 5));    // high frame score, low busy %
  r.add(S(149, 0, 0, 0, 0, false, 0, true, 20));     // low frame score, high busy %
  auto k = r.ranked();
  REQUIRE(k.size() == 2);
  CHECK(k[0].ch == 136);                             // lower busy % ranks first
  CHECK(k[1].ch == 149);
}

static maburgs::RankSample rsb(uint8_t ch, uint32_t fa, double busy) {
  maburgs::RankSample s; s.ch = ch; s.fa = fa; s.busy_valid = true; s.busy_pct = busy; return s;
}
TEST(worst_busy_visit_blocks_the_channel) {
  maburgs::ChannelRanker r({136, 144}, 1, 0, 30.0);
  r.add(rsb(144, 0, 5)); r.add(rsb(144, 0, 80));
  for (const auto& e : r.all()) if (e.ch == 144) CHECK(r.is_blocked(e));
}
TEST(blocked_home_loses_despite_margin) {
  maburgs::ChannelRanker r({136, 144}, 1, /*margin_pct=*/20, 30.0);
  r.add(rsb(136, 0, 100));   // analog on home: FA 0, 100 % busy
  r.add(rsb(144, 15, 0));    // 15 events, unblocked
  CHECK(r.proposal(136) == 144);
}
TEST(unblocked_candidate_still_needs_the_margin) {
  maburgs::ChannelRanker r({136, 144}, 1, 20, 30.0);
  r.add(rsb(136, 10, 0)); r.add(rsb(144, 0, 0));
  CHECK(r.proposal(136) == 136);
}

TEST(leak_is_subtracted_before_the_clamp) {
  RankSample s = S(1, 100, 5, 40, 2); s.leak = 50;
  CHECK(ChannelRanker::busy(s) == 17);          // (100-40-50)+5+2
  s.leak = 500;
  CHECK(ChannelRanker::busy(s) == 7);           // clamps at 0, fa+foreign stay
}
TEST(margin_is_against_the_current_channel_not_a_fixed_home) {
  ChannelRanker r({136, 149}, 1, /*margin_pct=*/20);
  r.add(S(136, 0, 0, 0, 0, false, 0, true, 30)); r.add(S(149, 0, 0, 0, 0, false, 0, true, 15));
  CHECK(r.proposal(136) == 136);   // 30 - 15 = 15 < 20: stay
  CHECK(r.proposal(149) == 149);   // from 149's point of view it is already best
  r.add(S(136, 0, 0, 0, 0, false, 0, true, 60));   // worst-visit busy now 60
  CHECK(r.proposal(136) == 149);   // 60 - 15 = 45 >= 20: move
}

TEST(ranked_with_an_explicit_visit_bar_overrides_min_rounds) {
  ChannelRanker r({136, 149}, 3);
  r.add(S(136, 100, 0, 0, 0)); r.add(S(149, 0, 0, 0, 0));
  CHECK(r.ranked().empty());               // 1 visit < min_rounds 3
  auto k = r.ranked(1);
  REQUIRE(k.size() == 2);
  CHECK(k[0].ch == 149 && k[1].ch == 136);
  CHECK(r.ranked(2).empty());
}

TEST(proposal_with_an_explicit_visit_bar_overrides_min_rounds) {
  ChannelRanker r({136, 149}, 3, 20);
  r.add(S(136, 0, 0, 0, 0, false, 0, true, 100)); r.add(S(149, 0, 0, 0, 0, false, 0, true, 0));
  CHECK(r.proposal(136) == 136);           // nothing ranked at 3
  CHECK(r.proposal(136, 1) == 149);        // 100 - 0 clears the margin of 20
  CHECK(r.proposal(136, 2) == 136);
}

MTEST_MAIN
