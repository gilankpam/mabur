#include <cmath>
#include "mtest.h"
#include "own_air.h"
using namespace maburgs;

static bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }

TEST(ht_rates_match_the_standard_table) {
  CHECK(near(ht_rate_mbps(0, 20, false), 6.5));
  CHECK(near(ht_rate_mbps(7, 20, false), 65.0));
  CHECK(near(ht_rate_mbps(3, 40, false), 54.0));
  CHECK(near(ht_rate_mbps(4, 40, false), 81.0));
  CHECK(near(ht_rate_mbps(7, 20, true), 65.0 * 10.0 / 9.0));
  CHECK(ht_rate_mbps(8, 20, false) == 0.0);
}
TEST(single_frames_pay_a_preamble_each) {
  OwnAirAcc a;
  // 650 B at MCS0/20 = 800 us payload + 40 us preamble, twice.
  a.on_frame(650, 0, /*physt=*/true, 20, false, false);
  a.on_frame(650, 0, true, 20, false, false);
  CHECK(a.total_us() == 2 * (800 + 40));
}
TEST(aggregate_pays_one_preamble_and_later_subframes_inherit_width) {
  OwnAirAcc a;
  // First subframe carries the PHY status: 40 MHz MCS3 (54 Mb/s), STBC.
  // 1350 B = 200 us. Later subframes arrive with bw=20/stbc=0 (no PHY
  // status) and MUST be timed at the PPDU's 40 MHz, not 20.
  a.on_frame(1350, 3, true, 40, true, false);
  a.on_frame(1350, 3, false, 20, false, false);
  a.on_frame(1350, 3, false, 20, false, false);
  CHECK(a.total_us() == 3 * 200 + 40 + 4);
}
TEST(unknown_rate_frames_add_nothing) {
  OwnAirAcc a;
  a.on_frame(1000, 255, true, 20, false, false);
  CHECK(a.total_us() == 0);
}

MTEST_MAIN
