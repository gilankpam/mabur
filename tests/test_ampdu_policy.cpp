// Per-rung A-MPDU policy (docs/bandwidth-sweep-findings-2026-09-17.md): the
// saturation sweep measured agg6 LOSING 6/10/13/4 points of delivered
// capacity at mcs0-3 versus QoS-Data singles and gaining 8-13 points at
// mcs5/7, crossover near mcs4; the rung-pinned A/B (sessions 0121-0128)
// showed singles never worse on latency at rungs 0-3. So the aggregation
// mode follows the op's MCS: below ampdu.min_mcs the chip flies singles.
#include "mtest.h"
#include "ampdu_policy.h"
using namespace mabur;

TEST(ampdu_policy_off_when_max_num_zero) {
  AmpduCfg c; c.max_num = 0; c.min_mcs = 0;
  for (uint8_t m = 0; m <= 7; ++m) CHECK(!ampdu_mode_for(c, m).enabled);
}

TEST(ampdu_policy_threshold_is_inclusive) {
  AmpduCfg c; c.max_num = 6; c.max_time = 32; c.min_mcs = 4;
  CHECK(!ampdu_mode_for(c, 3).enabled);
  CHECK(ampdu_mode_for(c, 4).enabled);
  CHECK(ampdu_mode_for(c, 7).enabled);
  CHECK(!ampdu_mode_for(c, 0).enabled);
}

TEST(ampdu_policy_min_mcs_zero_aggregates_every_rung) {
  // min_mcs 0 is the pre-2026-09-17 behaviour: aggregate everywhere.
  AmpduCfg c; c.max_num = 6; c.min_mcs = 0;
  for (uint8_t m = 0; m <= 7; ++m) CHECK(ampdu_mode_for(c, m).enabled);
}

TEST(ampdu_policy_enabled_mode_is_maburd_recipe) {
  // The exact recipe drone/src/main.cpp programmed at bring-up before the
  // policy moved into the actuator: tid 0, density 7, no-ack, config
  // max_num/max_time. A different descriptor half is a different
  // measurement, not the one the sweep validated.
  AmpduCfg c; c.max_num = 6; c.max_time = 32; c.min_mcs = 4;
  const devourer::AmpduMode m = ampdu_mode_for(c, 5);
  CHECK(m.enabled);
  CHECK(m.tid == 0);
  CHECK(m.max_num == 6);
  CHECK(m.density == 7);
  CHECK(m.no_ack);
  CHECK(m.max_time == 32);
}

TEST(ampdu_policy_same_mode_compares_equal) {
  // The actuator only writes the chip when the mode CHANGES (a register
  // write per RCF would be pointless USB traffic on the agent thread), so
  // two modes derived for rungs on the same side of the threshold must
  // compare equal, and modes across it must not.
  AmpduCfg c; c.max_num = 6; c.max_time = 32; c.min_mcs = 4;
  CHECK(ampdu_mode_same(ampdu_mode_for(c, 4), ampdu_mode_for(c, 7)));
  CHECK(ampdu_mode_same(ampdu_mode_for(c, 0), ampdu_mode_for(c, 3)));
  CHECK(!ampdu_mode_same(ampdu_mode_for(c, 3), ampdu_mode_for(c, 4)));
}

MTEST_MAIN
