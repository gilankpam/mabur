#include <cmath>
#include "mtest.h"
#include "nhm_busy.h"
using namespace maburgs;

static NhmBusyRead r(std::initializer_list<int> b) {
  NhmBusyRead x; x.valid = true; int i = 0;
  for (int v : b) x.buckets[i++] = static_cast<uint8_t>(v);
  return x;
}
TEST(edges_are_the_absolute_table) {
  CHECK(busy_dbm_is_edge(-83) && busy_dbm_is_edge(-104) && busy_dbm_is_edge(-70));
  CHECK(!busy_dbm_is_edge(-82) && !busy_dbm_is_edge(-60));
}
TEST(busy_is_the_share_at_or_above_the_edge) {
  // -83 dBm is edge index 7 -> buckets 8..11 are "above".
  auto x = r({100, 0, 0, 0, 0, 0, 0, 0, 50, 50, 25, 30});
  CHECK(std::fabs(*nhm_busy_pct(x, -83) - 100.0 * 155 / 255) < 1e-9);
  // -70 (top edge, index 10) -> bucket 11 only.
  CHECK(std::fabs(*nhm_busy_pct(x, -70) - 100.0 * 30 / 255) < 1e-9);
}
TEST(invalid_or_empty_window_is_no_reading) {
  NhmBusyRead x;  // valid=false
  CHECK(!nhm_busy_pct(x, -83).has_value());
  auto z = r({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  CHECK(!nhm_busy_pct(z, -83).has_value());   // "ready but all zero" is NOT 0 % busy
}
TEST(period_clamps_to_16_bits) {
  CHECK(nhm_period_4us(140) == 35000);
  CHECK(nhm_period_4us(5) == 1250);
  CHECK(nhm_period_4us(1990) == 65535);
  CHECK(nhm_period_4us(0) == 1);
}

MTEST_MAIN
