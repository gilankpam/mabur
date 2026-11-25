#include "../drone/src/window_stat.h"
#include "mtest.h"

using mabur::WindowStat;

TEST(empty_mean_is_zero) {
  WindowStat w;
  CHECK(w.n == 0 && w.mean() == 0 && w.max == 0);
}

TEST(add_tracks_n_sum_max) {
  WindowStat w;
  w.add(3); w.add(7); w.add(5);
  CHECK(w.n == 3);
  CHECK(w.sum == 15);
  CHECK(w.max == 7);
  CHECK(w.mean() == 5);
}

TEST(mean_truncates) {
  WindowStat w;
  w.add(1); w.add(2);
  CHECK(w.mean() == 1);
}

TEST(reset_zeroes) {
  WindowStat w;
  w.add(9);
  w.reset();
  CHECK(w.n == 0 && w.sum == 0 && w.max == 0 && w.mean() == 0);
}

MTEST_MAIN
