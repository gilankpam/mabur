#include "mtest.h"
#include "nhm_window.h"
using namespace maburgs;

static NhmBusyRead rd(uint16_t period, bool valid = true) {
  NhmBusyRead r; r.valid = valid; r.period = period; r.buckets[0] = 255; return r;
}
TEST(own_window_on_same_channel_is_usable) {
  NhmWindowTracker t; t.armed(136, 35000);
  CHECK(t.usable(rd(35000), 136));
}
// Review focus 1: a scout dwell re-armed the engine (5 ms = 1250) between
// two verdict windows -- the result is the dwell's, not ours.
TEST(someone_elses_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000);
  CHECK(!t.usable(rd(1250), 136));
}
TEST(channel_change_since_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000);
  CHECK(!t.usable(rd(35000), 144));
}
TEST(never_armed_or_not_ready_is_not_usable) {
  NhmWindowTracker t;
  CHECK(!t.usable(rd(35000), 136));
  t.armed(136, 35000);
  CHECK(!t.usable(rd(35000, false), 136));
  t.invalidate();
  CHECK(!t.usable(rd(35000), 136));
}
// Review focus 5: hop.window_ms up to 2000 -> the period clamps and the
// reading is still used.
TEST(clamped_period_still_matches) {
  NhmWindowTracker t; t.armed(136, nhm_period_4us(1990));
  CHECK(t.usable(rd(65535), 136));
}
MTEST_MAIN
