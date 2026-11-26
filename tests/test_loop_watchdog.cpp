#include "../drone/src/loop_watchdog.h"
#include "mtest.h"

using mabur::LoopWatchdog;
using Stall = LoopWatchdog::Stall;

TEST(no_stall_inside_grace) {
  LoopWatchdog w(1000);
  CHECK(w.check(1000 + 9999, 0, 0, true, false) == Stall::None);
  CHECK(w.check(1000 + 10000, 0, 0, true, false) == Stall::None);  // grace is exclusive
}

TEST(static_hot_beat_stalls_after_grace) {
  LoopWatchdog w(0);
  CHECK(w.check(10001, 0, 1, false, false) == Stall::Hot);
}

TEST(moving_hot_quiet_rx_unlinked_is_fine) {
  LoopWatchdog w(0);
  uint64_t hb = 0;
  for (uint64_t t = 0; t <= 30000; t += 100)
    CHECK(w.check(t, ++hb, 0, false, false) == Stall::None);
}

TEST(quiet_rx_while_linked_stalls) {
  LoopWatchdog w(0);
  uint64_t hb = 0;
  Stall s = Stall::None;
  uint64_t t = 0;
  for (; t <= 12000 && s == Stall::None; t += 100) s = w.check(t, ++hb, 0, true, false);
  CHECK(s == Stall::Rx);
  CHECK(t - 100 == 10100);  // first tick past grace with 3000 ms of silence
}

TEST(stale_boundary_is_exclusive) {
  LoopWatchdog w(0);
  CHECK(w.stale_ms() == 3000);
  CHECK(w.check(10001, 1, 0, true, false) == Stall::Rx);
  LoopWatchdog v(0);
  v.check(7000, 1, 1, true, false);
  CHECK(v.check(10000, 2, 1, true, false) == Stall::None);  // inside grace
  CHECK(v.check(10001, 3, 1, true, false) == Stall::Rx);   // 3001 ms since rx beat
}

TEST(cal_active_pins_rx) {
  LoopWatchdog w(0);
  uint64_t hb = 0;
  for (uint64_t t = 0; t <= 60000; t += 100)
    CHECK(w.check(t, ++hb, 0, true, true) == Stall::None);
}

TEST(cal_end_gives_fresh_rx_window) {
  LoopWatchdog w(0);
  uint64_t hb = 0;
  for (uint64_t t = 0; t <= 20000; t += 100) w.check(t, ++hb, 0, true, true);
  CHECK(w.check(20000 + 3000, ++hb, 0, true, false) == Stall::None);
  CHECK(w.check(20000 + 3001, ++hb, 0, true, false) == Stall::Rx);
}

TEST(hot_reported_before_rx) {
  LoopWatchdog w(0);
  CHECK(w.check(20000, 0, 0, true, false) == Stall::Hot);
}

MTEST_MAIN
