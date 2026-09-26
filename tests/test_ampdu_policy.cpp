// Per-rung, per-width A-MPDU policy. 20 MHz: agg6 loses 6-13 points vs
// singles at mcs0-3 (docs/bandwidth-sweep-findings-2026-09-17.md), so the
// threshold is 4. 40 MHz: frames are half as long on air, the fixed
// per-PPDU cost doubles its share, and aggregation pays from mcs2
// (docs/bw40-sweep-findings-2026-09-23.md "Findings" 3) -- a separate
// threshold per width.
#include "mtest.h"
#include "ampdu_policy.h"
using namespace mabur;

TEST(ampdu_policy_off_when_max_num_zero) {
  AmpduCfg c; c.max_num = 0; c.min_mcs_20 = 0; c.min_mcs_40 = 0;
  for (uint8_t m = 0; m <= 7; ++m) {
    CHECK(!ampdu_mode_for(c, m, 20).enabled);
    CHECK(!ampdu_mode_for(c, m, 40).enabled);
  }
}

TEST(ampdu_policy_threshold_is_inclusive_and_per_width) {
  AmpduCfg c; c.max_num = 6; c.max_time = 32; c.min_mcs_20 = 4; c.min_mcs_40 = 2;
  CHECK(!ampdu_mode_for(c, 3, 20).enabled);
  CHECK(ampdu_mode_for(c, 4, 20).enabled);
  CHECK(ampdu_mode_for(c, 7, 20).enabled);
  CHECK(!ampdu_mode_for(c, 0, 20).enabled);
  // The same MCS aggregates at 40 MHz but not at 20: the 40/3 rung
  // aggregates, the 20/3 rung flies singles.
  CHECK(!ampdu_mode_for(c, 1, 40).enabled);
  CHECK(ampdu_mode_for(c, 2, 40).enabled);
  CHECK(ampdu_mode_for(c, 3, 40).enabled);
  CHECK(!ampdu_mode_for(c, 3, 20).enabled);
}

TEST(ampdu_policy_min_mcs_zero_aggregates_every_rung) {
  AmpduCfg c; c.max_num = 6; c.min_mcs_20 = 0; c.min_mcs_40 = 0;
  for (uint8_t m = 0; m <= 7; ++m) {
    CHECK(ampdu_mode_for(c, m, 20).enabled);
    CHECK(ampdu_mode_for(c, m, 40).enabled);
  }
}

TEST(ampdu_policy_enabled_mode_is_maburd_recipe) {
  AmpduCfg c; c.max_num = 6; c.max_time = 32; c.min_mcs_20 = 4; c.min_mcs_40 = 2;
  const devourer::AmpduMode m = ampdu_mode_for(c, 5, 20);
  CHECK(m.enabled);
  CHECK(m.tid == 0);
  CHECK(m.max_num == 6);
  CHECK(m.density == 7);
  CHECK(m.no_ack);
  CHECK(m.max_time == 32);
}

TEST(ampdu_policy_same_mode_compares_equal) {
  AmpduCfg c; c.max_num = 6; c.max_time = 32; c.min_mcs_20 = 4; c.min_mcs_40 = 2;
  CHECK(ampdu_mode_same(ampdu_mode_for(c, 4, 20), ampdu_mode_for(c, 7, 20)));
  CHECK(ampdu_mode_same(ampdu_mode_for(c, 0, 20), ampdu_mode_for(c, 3, 20)));
  CHECK(!ampdu_mode_same(ampdu_mode_for(c, 3, 20), ampdu_mode_for(c, 4, 20)));
  // Crossing from 20/4 to 40/3 keeps aggregation on: no chip write.
  CHECK(ampdu_mode_same(ampdu_mode_for(c, 4, 20), ampdu_mode_for(c, 3, 40)));
}

MTEST_MAIN
