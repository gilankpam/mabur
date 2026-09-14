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
MTEST_MAIN
