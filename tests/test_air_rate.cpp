// delivered_mbps: nominal PHY rate × air_clock.efficiency_<bw>[mcs].
#include <cmath>
#include "mtest.h"
#include "air_rate.h"
using namespace mabur;

TEST(delivered_rate_is_nominal_times_mcs_efficiency_at_20) {
  AirClockCfg c;
  c.efficiency_20 = {0.93, 0.88, 0.84, 0.80, 0.76, 0.76, 0.78, 0.76};
  const auto l2 = rc::ladder_from(rc::PhyMode::HT, 2, 20);
  const auto l5 = rc::ladder_from(rc::PhyMode::HT, 5, 20);
  const double n2 = rc::phy_rate_mbps(l2[0]);
  const double n5 = rc::phy_rate_mbps(l5[1]);
  CHECK(std::abs(delivered_mbps(l2[0], c) - n2 * 0.84) < 1e-9);
  CHECK(std::abs(delivered_mbps(l5[1], c) - n5 * 0.76) < 1e-9);
}

TEST(delivered_rate_picks_the_40_table_by_spec_width) {
  AirClockCfg c;
  c.efficiency_20 = {0.93, 0.88, 0.84, 0.80, 0.76, 0.76, 0.78, 0.76};
  c.efficiency_40 = {0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75};
  const auto l3_40 = rc::ladder_from(rc::PhyMode::HT, 3, 40);
  const auto l3_20 = rc::ladder_from(rc::PhyMode::HT, 3, 20);
  // HT40 mcs3 nominal 54 x 0.75 = 40.5; HT20 mcs3 nominal 26 x 0.80 = 20.8.
  CHECK(std::abs(delivered_mbps(l3_40[0], c) - 54.0 * 0.75) < 1e-9);
  CHECK(std::abs(delivered_mbps(l3_20[0], c) - 26.0 * 0.80) < 1e-9);
}

TEST(delivered_rate_default_tables_are_nominal) {
  AirClockCfg c;
  for (uint8_t m = 0; m <= 7; ++m) {
    const auto l20 = rc::ladder_from(rc::PhyMode::HT, m, 20);
    const auto l40 = rc::ladder_from(rc::PhyMode::HT, m, 40);
    CHECK(delivered_mbps(l20[0], c) == rc::phy_rate_mbps(l20[0]));
    CHECK(delivered_mbps(l40[0], c) == rc::phy_rate_mbps(l40[0]));
  }
}

MTEST_MAIN
