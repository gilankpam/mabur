// Host test for gs/player/src/display_follow.h: whether, and to which mode,
// the display follows the camera's frame rate.
#include <vector>

#include "display_follow.h"
#include "mtest.h"

using maburplay::DisplayFollower;
using maburplay::follow_display;
using maburplay::FollowIn;
using maburplay::FollowOut;
using maburplay::ModeCand;
using maburplay::parse_screen_want;
using maburplay::RateMatch;
using maburplay::ScreenWant;

namespace {
// CEA 1080p60 (preferred), 720p60, 1080p100, 720p100: a goggle with no 90 Hz.
const ModeCand k1080p60{1920, 1080, 148500, 2200, 1125, false, true};
const ModeCand k720p60{1280, 720, 74250, 1650, 750, false, false};
const ModeCand k1080p100{1920, 1080, 297000, 2640, 1125, false, false};
const ModeCand k720p100{1280, 720, 148500, 1980, 750, false, false};
// CEA 1080p120 and 1440p60.
const ModeCand k1080p120{1920, 1080, 297000, 2200, 1125, false, false};
const ModeCand k1440p60{2560, 1440, 241500, 2720, 1481, false, false};
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

TEST(a_camera_midway_between_two_rates_does_not_flap) {
  // A 60/120 Hz monitor and a 90 fps drone: the measured median crosses the
  // midpoint window to window. From either running mode, neither side of it
  // rebuilds.
  FollowIn in;
  in.want = parse_screen_want("auto");
  in.modes = {k1080p60, k1080p120};
  for (int running : {0, 1}) {
    for (double hz : {90.0009, 89.9928}) {
      in.running = running;
      in.camera_hz = hz;
      const FollowOut out = follow_display(in);
      CHECK(!out.rebuild);
      CHECK(out.mode == running);
      CHECK(out.mismatch);
    }
  }
  // ... but a larger mode at the nearest rate still wins over a smaller one.
  in.modes = {k720p60, k1080p120};
  in.running = 0;
  in.camera_hz = 90.0009;
  CHECK(follow_display(in).rebuild);
}

// --- DisplayFollower: the follow over 1 Hz windows ---

namespace {
const std::vector<ModeCand> kMonitor{k1080p60, k1080p100, k1080p120, k1440p60};
const ScreenWant kAuto = parse_screen_want("auto");
constexpr auto kNo = RateMatch::LowPower::kNo;
constexpr auto kYes = RateMatch::LowPower::kYes;
constexpr auto kUnknown = RateMatch::LowPower::kUnknown;
}  // namespace

TEST(three_agreeing_windows_rebuild) {
  DisplayFollower f;
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  const FollowOut out = f.on_window(kAuto, 100.0, kNo, kMonitor, 0);
  CHECK(out.rebuild);
  CHECK(out.mode == 1);  // 1920x1080@100
  // Landed: the next windows see it running and stay put.
  for (int i = 0; i < 5; ++i) CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 1).rebuild);
}

TEST(a_matched_rate_still_moves_up_to_the_largest_mode) {
  // No rate mismatch at all (camera 60, panel 60): "auto" still takes the
  // larger 60 Hz mode.
  DisplayFollower f;
  FollowOut out;
  for (int i = 0; i < DisplayFollower::kAgreeWindows; ++i)
    out = f.on_window(kAuto, 60.0, kNo, kMonitor, 0);
  CHECK(out.rebuild);
  CHECK(out.mode == 3);  // 2560x1440@60
}

TEST(a_fade_window_does_not_rebuild) {
  // Running 1080p100 with a 100 fps camera; one window loses over half its
  // frames and reads 50 Hz (its pick: a 60 Hz mode).
  DisplayFollower f;
  for (double hz : {100.0, 100.0, 50.0, 100.0, 100.0, 50.0, 50.0, 100.0})
    CHECK(!f.on_window(kAuto, hz, kNo, kMonitor, 1).rebuild);
}

TEST(a_change_mid_sequence_restarts_the_count) {
  DisplayFollower f;
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 120.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 120.0, kNo, kMonitor, 0).rebuild);
  const FollowOut out = f.on_window(kAuto, 120.0, kNo, kMonitor, 0);
  CHECK(out.rebuild);
  CHECK(out.mode == 2);  // 1920x1080@120
}

TEST(low_power_or_a_quiet_report_never_follows_and_restarts_the_count) {
  DisplayFollower f;
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  for (int i = 0; i < 5; ++i) CHECK(!f.on_window(kAuto, 30.0, kYes, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kUnknown, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  CHECK(f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
}

TEST(a_pinned_mode_or_no_modes_never_follows) {
  DisplayFollower f;
  const ScreenWant full = parse_screen_want("1920x1080@60");
  for (int i = 0; i < 5; ++i) {
    CHECK(!f.on_window(full, 100.0, kNo, kMonitor, 0).rebuild);
    CHECK(!f.on_window(kAuto, 100.0, kNo, {}, -1).rebuild);
  }
}

TEST(resolve_holds_the_last_rate_seen_out_of_low_power) {
  DisplayFollower f;
  CHECK(f.resolve_hz(100.0) == 0.0);  // no window yet
  f.on_window(kAuto, 100.0, kNo, kMonitor, 1);
  CHECK(f.resolve_hz(99.9) == 99.9);  // out of low power: the current rate
  f.on_window(kAuto, 30.0, kYes, kMonitor, 1);
  CHECK(f.resolve_hz(30.0) == 100.0);
  f.on_window(kAuto, 30.0, kUnknown, kMonitor, 1);
  CHECK(f.resolve_hz(30.0) == 100.0);
  f.on_window(kAuto, 60.0, kNo, kMonitor, 1);
  CHECK(f.resolve_hz(60.0) == 60.0);
}

TEST(a_rebuild_that_lands_elsewhere_is_not_reissued) {
  DisplayFollower f;
  for (int i = 0; i < DisplayFollower::kAgreeWindows - 1; ++i)
    f.on_window(kAuto, 100.0, kNo, kMonitor, 0);
  CHECK(f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  // The new presenter came up on 1080p60 again (init fell back).
  for (int i = 0; i < 10; ++i) CHECK(!f.on_window(kAuto, 100.0, kNo, kMonitor, 0).rebuild);
  // Another target is still followed.
  FollowOut out;
  for (int i = 0; i < DisplayFollower::kAgreeWindows; ++i)
    out = f.on_window(kAuto, 120.0, kNo, kMonitor, 0);
  CHECK(out.rebuild);
  CHECK(out.mode == 2);
}

TEST(a_settled_unmatched_pick_reports_the_mismatch) {
  const std::vector<ModeCand> goggle{k1080p60, k720p60, k1080p100, k720p100};
  DisplayFollower f;
  const FollowOut out = f.on_window(kAuto, 90.0, kNo, goggle, 2);
  CHECK(!out.rebuild);
  CHECK(out.mismatch);
  CHECK(out.shown_hz > 99.99 && out.shown_hz < 100.01);
  // Still pending a rebuild: nothing to report yet.
  DisplayFollower g;
  CHECK(!g.on_window(kAuto, 90.0, kNo, goggle, 0).mismatch);
}

MTEST_MAIN
