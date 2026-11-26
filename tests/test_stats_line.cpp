#include "../drone/src/stats_line.h"
#include "mtest.h"

using mabur::mav_stats_suffix;

TEST(all_zero_first_line_prints_nothing) {
  const uint64_t now[8] = {};
  uint64_t last[8] = {};
  CHECK(mav_stats_suffix(now, last).empty());
}

TEST(change_prints_exact_suffix) {
  const uint64_t now[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint64_t last[8] = {};
  CHECK(mav_stats_suffix(now, last) ==
        " mav rx=1/2 tx=3/4 drop=5 reject=6 cal_late=7 update_dropped=8");
  for (int i = 0; i < 8; ++i) CHECK(last[i] == now[i]);
}

TEST(repeat_prints_nothing) {
  const uint64_t now[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint64_t last[8] = {};
  mav_stats_suffix(now, last);
  CHECK(mav_stats_suffix(now, last).empty());
}

TEST(any_counter_change_prints) {
  for (int i = 0; i < 8; ++i) {
    uint64_t now[8] = {};
    uint64_t last[8] = {};
    now[i] = 1;
    CHECK(!mav_stats_suffix(now, last).empty());
  }
}

MTEST_MAIN
