// Host test for gs/player/src/display_follow.h: whether, and to which mode,
// the display follows the camera's frame rate.
#include <vector>

#include "display_follow.h"
#include "mtest.h"

using maburplay::follow_display;
using maburplay::FollowIn;
using maburplay::FollowOut;
using maburplay::ModeCand;
using maburplay::parse_screen_want;

namespace {
// CEA 1080p60 (preferred), 720p60, 1080p100, 720p100: a goggle with no 90 Hz.
const ModeCand k1080p60{1920, 1080, 148500, 2200, 1125, false, true};
const ModeCand k720p60{1280, 720, 74250, 1650, 750, false, false};
const ModeCand k1080p100{1920, 1080, 297000, 2640, 1125, false, false};
const ModeCand k720p100{1280, 720, 148500, 1980, 750, false, false};
// A second 1080p mode within reach of 100 Hz (an off-CEA timing).
const ModeCand k1080p99_8{1920, 1080, 296406, 2640, 1125, false, false};

FollowIn auto_in(double camera_hz, int running) {
  FollowIn in;
  in.want = parse_screen_want("auto");
  in.camera_hz = camera_hz;
  in.modes = {k1080p60, k720p60, k1080p100, k720p100};
  in.running = running;
  return in;
}
}  // namespace

TEST(no_modes_means_keep_running_and_no_rebuild) {
  FollowIn in = auto_in(100.0, 0);
  in.modes.clear();  // the EDID read failed: the connector lists nothing
  const FollowOut out = follow_display(in);
  CHECK(!out.rebuild);
  CHECK(!out.mismatch);
  // ... and the same with no running index either.
  in.running = -1;
  CHECK(!follow_display(in).rebuild);
}

TEST(full_mode_never_follows) {
  FollowIn in = auto_in(100.0, 0);
  in.want = parse_screen_want("1920x1080@60");
  const FollowOut out = follow_display(in);
  CHECK(!out.rebuild);
  CHECK(!out.mismatch);
}

TEST(low_power_never_follows) {
  FollowIn in = auto_in(30.0, 2);
  in.low_power = true;
  const FollowOut out = follow_display(in);
  CHECK(!out.rebuild);
  CHECK(!out.mismatch);
}

TEST(unknown_camera_rate_never_follows) {
  const FollowOut out = follow_display(auto_in(0.0, 2));
  CHECK(!out.rebuild);
  CHECK(!out.mismatch);
}

TEST(a_camera_rate_change_rebuilds_to_the_largest_mode_at_that_rate) {
  const FollowOut out = follow_display(auto_in(100.0, 0));  // running 1080p60
  CHECK(out.rebuild);
  CHECK(out.mode == 2);  // 1920x1080@100
  CHECK(!out.mismatch);
  CHECK(out.shown_hz > 99.99 && out.shown_hz < 100.01);
}

TEST(a_size_want_follows_at_that_size) {
  FollowIn in = auto_in(100.0, 1);  // running 720p60
  in.want = parse_screen_want("1280x720");
  const FollowOut out = follow_display(in);
  CHECK(out.rebuild);
  CHECK(out.mode == 3);  // 1280x720@100
}

TEST(already_running_the_pick_means_no_rebuild_even_when_mismatched) {
  const FollowOut out = follow_display(auto_in(90.0, 2));  // running 1080p100
  CHECK(!out.rebuild);
  CHECK(out.mode == 2);
  CHECK(out.mismatch);
  CHECK(out.shown_hz > 99.99 && out.shown_hz < 100.01);
}

TEST(a_running_mode_of_the_picked_size_within_reach_is_kept) {
  // The camera at 99.85 Hz picks 99.8 by closest rate, but the running
  // 1080p100 is in the phase lock's reach too: a rebuild between two modes
  // the camera can be steered onto would only blank the screen.
  FollowIn in = auto_in(99.85, 2);
  in.modes.push_back(k1080p99_8);  // index 4
  const FollowOut out = follow_display(in);
  CHECK(!out.rebuild);
  CHECK(out.mode == 2);
  CHECK(!out.mismatch);
  // Running a different size: the rule still moves to the largest.
  FollowIn small = in;
  small.running = 3;  // 720p100
  CHECK(follow_display(small).rebuild);
}

MTEST_MAIN
