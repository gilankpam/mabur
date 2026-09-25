#include "hop_ranker.h"
#include "mtest.h"
using namespace maburgs;
static HopCfg cfg() { HopCfg c; c.rank_visits = 5; c.rank_max_age_ms = 10000; return c; }
static HopVisit V(uint8_t ch, double t, uint32_t fa, uint32_t cca = 0, uint32_t own = 0, uint32_t foreign = 0) {
  HopVisit v; v.ch = ch; v.t_ms = t; v.fa = fa; v.cca = cca; v.own = own; v.foreign = foreign; return v;
}
TEST(score_formula) {
  CHECK(HopRanker::score(V(1, 0, 3, 10, 12, 0)) == 3);        // cca - own clamps to 0
  CHECK(HopRanker::score(V(1, 0, 3, 10, 2, 2)) == 3 + 8 + 8);
}
TEST(unranked_below_two_fresh_visits) {
  HopRanker r(cfg(), {120, 149, 165}, 136, 136);
  r.add(V(120, 0, 0));
  auto k = r.ranking(100);
  CHECK(!k[0].ranked);
  CHECK(!r.best(100, 136, {}).has_value());
}
TEST(sum_of_last_five_visits_lowest_wins) {
  HopRanker r(cfg(), {120, 149, 165}, 136, 136);
  for (int i = 0; i < 8; ++i) { r.add(V(120, i * 300, i < 3 ? 30 : 0)); r.add(V(149, i * 300, 4)); r.add(V(165, i * 300, 13)); }
  auto k = r.ranking(2500);
  CHECK(k[0].ch == 120 && k[0].score == 0);     // the three old busy visits fell out of the window of 5
  CHECK(k[1].ch == 149 && k[1].score == 20);
  CHECK(k[2].ch == 165 && k[2].score == 65);
  CHECK(*r.best(2500, 136, {}) == 120);
}
TEST(stale_visits_are_dropped) {
  HopRanker r(cfg(), {120}, 136, 136);
  r.add(V(120, 0, 0)); r.add(V(120, 100, 0));
  CHECK(r.best(5000, 136, {}).has_value());
  CHECK(!r.best(20000, 136, {}).has_value());
}
TEST(tie_break_prefers_home_over_config_order) {
  // Spec 2026-09-14-inflight-channel-hop-design.md §3: "ties -> boot-time
  // pick, then home". None of the three tied candidates below is the boot
  // pick (200), but 165 is home -- home must win over plain config order.
  HopRanker r(cfg(), {120, 149, 165}, 165, 200);
  for (int i = 0; i < 3; ++i) { r.add(V(120, i * 100, 2)); r.add(V(149, i * 100, 2)); r.add(V(165, i * 100, 2)); }
  CHECK(*r.best(1000, 255, {}) == 165);
}
TEST(exclude_and_skip_lists_and_tiebreak) {
  HopRanker r(cfg(), {120, 149, 165}, 136, 149);
  for (int i = 0; i < 3; ++i) { r.add(V(120, i * 100, 2)); r.add(V(149, i * 100, 2)); r.add(V(165, i * 100, 2)); }
  CHECK(*r.best(1000, 136, {}) == 149);            // tie -> boot pick
  CHECK(*r.best(1000, 149, {}) == 120);            // exclude current; tie -> config order
  CHECK(*r.best(1000, 149, {120}) == 165);         // skip backed-off
  CHECK(!r.best(1000, 149, {120, 165}).has_value());
}
// I3: the boot-time pick is not known when the ranker is constructed (the
// boot scan has not resolved yet), so main.cpp used to pass the configured
// home as BOTH home and boot_pick -- collapsing the spec's "ties ->
// boot-time pick, then home" into "ties -> home" and making the first
// tiebreak term dead. The real pick arrives at the first DiscAck.
TEST(boot_pick_published_after_construction_wins_the_tiebreak) {
  HopRanker r(cfg(), {120, 149, 165}, 136, /*boot_pick=*/0);   // 0 = not known yet
  for (int i = 0; i < 3; ++i) { r.add(V(120, i * 100, 2)); r.add(V(149, i * 100, 2)); r.add(V(165, i * 100, 2)); }
  CHECK(*r.best(1000, 136, {}) == 120);   // no boot pick, home not a candidate: config order
  r.set_boot_pick(165);
  CHECK(*r.best(1000, 136, {}) == 165);
}
// No boot pick ever (radio.scan.enable off, or the drone appeared before
// any channel reached min_rounds): 0 is never a real channel, so the
// tiebreak falls through to home exactly as before.
TEST(no_boot_pick_falls_through_to_home) {
  HopRanker r(cfg(), {120, 149, 165}, 165, /*boot_pick=*/0);
  for (int i = 0; i < 3; ++i) { r.add(V(120, i * 100, 2)); r.add(V(149, i * 100, 2)); r.add(V(165, i * 100, 2)); }
  CHECK(*r.best(1000, 255, {}) == 165);
}

static maburgs::HopVisit bv(uint8_t ch, double t, uint32_t fa, double busy) {
  maburgs::HopVisit v; v.ch = ch; v.t_ms = t; v.fa = fa; v.busy_valid = true; v.busy_pct = busy; return v;
}
TEST(blocked_channel_loses_to_a_busier_by_events_unblocked_one) {
  maburgs::HopCfg c;   // blocked_pct 50
  maburgs::HopRanker r(c, {144, 64}, 136, 0);
  for (int i = 0; i < 3; ++i) { r.add(bv(144, i, 0, 90)); r.add(bv(64, i, 40, 0)); }
  auto b = r.best(10, 136, {});
  REQUIRE(b.has_value());
  CHECK(*b == 64);   // today's score alone would pick 144 (0 events)
}
TEST(busy_is_the_mean_over_fresh_visits) {
  maburgs::HopCfg c;
  maburgs::HopRanker r(c, {144}, 136, 0);
  r.add(bv(144, 1, 0, 100)); r.add(bv(144, 2, 0, 0)); r.add(bv(144, 3, 0, 0)); r.add(bv(144, 4, 0, 0));
  for (const auto& e : r.ranking(10)) if (e.ch == 144) { CHECK(!e.blocked); CHECK(e.busy_pct == 25.0); }
}
TEST(all_blocked_least_busy_wins) {
  maburgs::HopCfg c;
  maburgs::HopRanker r(c, {144, 64}, 136, 0);
  for (int i = 0; i < 3; ++i) { r.add(bv(144, i, 0, 95)); r.add(bv(64, i, 0, 60)); }
  CHECK(*r.best(10, 136, {}) == 64);
}
TEST(visits_without_busy_rank_as_today) {
  maburgs::HopCfg c;
  maburgs::HopRanker r(c, {144, 64}, 136, 0);
  for (int i = 0; i < 3; ++i) {
    maburgs::HopVisit a; a.ch = 144; a.t_ms = i; a.fa = 5; r.add(a);
    maburgs::HopVisit b; b.ch = 64; b.t_ms = i; b.fa = 1; r.add(b);
  }
  CHECK(*r.best(10, 136, {}) == 64);
}
MTEST_MAIN
