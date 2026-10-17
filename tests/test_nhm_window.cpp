#include "mtest.h"
#include "nhm_window.h"
using namespace maburgs;

static NhmBusyRead rd(uint16_t period, bool valid = true) {
  NhmBusyRead r; r.valid = valid; r.period = period; r.buckets[0] = 255; return r;
}
TEST(own_window_on_same_channel_is_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7, 3);
  CHECK(t.usable(rd(35000), 136, 7, 3));
}
// A scout dwell re-armed the engine (5 ms = 1250) between two verdict
// windows -- the result is the dwell's, not ours.
TEST(someone_elses_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7, 3);
  CHECK(!t.usable(rd(1250), 136, 7, 3));
}
TEST(channel_change_since_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7, 3);
  CHECK(!t.usable(rd(35000), 144, 7, 3));
}
TEST(never_armed_or_not_ready_is_not_usable) {
  NhmWindowTracker t;
  CHECK(!t.usable(rd(35000), 136, 7, 3));
  t.armed(136, 35000, 7, 3);
  CHECK(!t.usable(rd(35000, false), 136, 7, 3));
  t.invalidate();
  CHECK(!t.usable(rd(35000), 136, 7, 3));
}
// hop.window_ms up to 2000 -> the period clamps and the reading is still used.
TEST(clamped_period_still_matches) {
  NhmWindowTracker t; t.armed(136, nhm_period_4us(1990), 7, 3);
  CHECK(t.usable(rd(65535), 136, 7, 3));
}
// InflightScout::dwell() retunes via FastRetune, which does not clear
// devourer's NHM-ready state and lands the card back on its own channel by
// the next tick -- period and channel both still match, so the per-card
// dwell generation is the only thing that catches a dwell landing
// mid-window.
TEST(a_dwell_since_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7, 3);
  CHECK(!t.usable(rd(35000), 136, 8, 3));
}
// The core's own hop retune (or width resync) that went out and came back
// inside one window leaves the channel equal to the armed one; the tune
// generation the core bumps after each retune is what catches it.
TEST(a_retune_since_arm_is_not_usable) {
  NhmWindowTracker t; t.armed(136, 35000, 7, 3);
  CHECK(!t.usable(rd(35000), 136, 7, 4));
}
MTEST_MAIN
