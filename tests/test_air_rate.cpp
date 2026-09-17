// delivered_mbps: nominal PHY rate × air_clock.efficiency[mcs].
#include "mtest.h"
#include "air_rate.h"
using namespace mabur;

TEST(delivered_rate_is_nominal_times_mcs_efficiency) {
  AirClockCfg c;
  c.efficiency = {0.93, 0.88, 0.84, 0.80, 0.76, 0.76, 0.78, 0.76};
  const auto l2 = rc::ladder_from(rc::PhyMode::HT, 2, 20);
  const auto l5 = rc::ladder_from(rc::PhyMode::HT, 5, 20);
  const double n2 = rc::phy_rate_mbps(l2[0]);
  const double n5 = rc::phy_rate_mbps(l5[1]);
  CHECK(std::abs(delivered_mbps(l2[0], c) - n2 * 0.84) < 1e-9);
  CHECK(std::abs(delivered_mbps(l5[1], c) - n5 * 0.76) < 1e-9);
}

TEST(delivered_rate_default_table_is_nominal) {
  // The struct default (no [air_clock] section) is all-ones = nominal
  // pricing, the pre-2026-09-17 bitrate policy exactly.
  AirClockCfg c;
  for (uint8_t m = 0; m <= 7; ++m) {
    const auto l = rc::ladder_from(rc::PhyMode::HT, m, 20);
    CHECK(delivered_mbps(l[0], c) == rc::phy_rate_mbps(l[0]));
  }
}

MTEST_MAIN
