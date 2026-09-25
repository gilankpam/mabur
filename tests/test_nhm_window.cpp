#include "mtest.h"
#include "nhm_window.h"
using namespace maburgs;

static NhmBusyRead rd(uint16_t period, bool valid = true) {
  NhmBusyRead r; r.valid = valid; r.period = period; r.buckets[0] = 255; return r;
}
TEST(own_window_on_same_channel_is_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7);
  CHECK(t.usable(rd(35000), 136, 7));
}
// Review focus 1: a scout dwell re-armed the engine (5 ms = 1250) between
// two verdict windows -- the result is the dwell's, not ours.
TEST(someone_elses_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7);
  CHECK(!t.usable(rd(1250), 136, 7));
}
TEST(channel_change_since_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7);
  CHECK(!t.usable(rd(35000), 144, 7));
}
TEST(never_armed_or_not_ready_is_not_usable) {
  NhmWindowTracker t;
  CHECK(!t.usable(rd(35000), 136, 7));
  t.armed(136, 35000, 7);
  CHECK(!t.usable(rd(35000, false), 136, 7));
  t.invalidate();
  CHECK(!t.usable(rd(35000), 136, 7));
}
// Review focus 5: hop.window_ms up to 2000 -> the period clamps and the
// reading is still used.
TEST(clamped_period_still_matches) {
  NhmWindowTracker t; t.armed(136, nhm_period_4us(1990), 7);
  CHECK(t.usable(rd(65535), 136, 7));
}
// Fix round 1 (task-6 review): InflightScout::dwell() retunes via
// FastRetune, which does not clear devourer's NHM-ready state and lands
// the card back on its own channel by the next tick -- period and channel
// both still match, so the generation bump main.cpp does after every
// completed dwell (success or fail) is the only thing that catches a
// dwell landing mid-window. Reverting the gen check in usable() makes
// this pass as usable -- that's the bug this test pins.
TEST(a_dwell_since_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7);
  CHECK(!t.usable(rd(35000), 136, 8));
}
MTEST_MAIN
