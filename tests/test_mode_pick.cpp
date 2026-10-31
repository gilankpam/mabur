#include <cmath>
#include <string>
#include <vector>

#include "mode_pick.h"
#include "mtest.h"

using maburplay::ModeCand;
using maburplay::mode_hz;
using maburplay::mode_string;
using maburplay::parse_screen_want;
using maburplay::pick_display_mode;
using maburplay::pick_mode;
using maburplay::ScreenWant;

namespace {
// CEA timings: 1080p60.000 (148.5 MHz), 1080p59.94 (148.352 MHz), 1080i60,
// 720p60.
const ModeCand k1080p5994{1920, 1080, 148352, 2200, 1125, false, false};
const ModeCand k1080p60{1920, 1080, 148500, 2200, 1125, false, true};
const ModeCand k1080i60{1920, 1080, 74250, 2200, 1125, true, false};
const ModeCand k720p60{1280, 720, 74250, 1650, 750, false, false};
// CEA 1080p100 and 720p100.
const ModeCand k1080p100{1920, 1080, 297000, 2640, 1125, false, false};
const ModeCand k720p100{1280, 720, 148500, 1980, 750, false, false};

// A goggle with 60 and 100 Hz at two sizes and no 90 Hz mode at all; 1080p60
// is the preferred one.
const std::vector<ModeCand> kAvatar{k1080p60, k720p60, k1080p100, k720p100};
}  // namespace

TEST(parse_accepts_integer_and_decimal_rates) {
  auto a = parse_screen_want("1920x1080@60");
  CHECK(a.ok && a.w == 1920 && a.h == 1080 && a.hz == 60.0);
  auto b = parse_screen_want("1920x1080@59.94");
  CHECK(b.ok && b.hz > 59.93 && b.hz < 59.95);
  CHECK(!parse_screen_want("1920x1080@60mhz").ok);
  CHECK(!parse_screen_want("1080p").ok);
  CHECK(!parse_screen_want("1920x1080@0").ok);
}

TEST(at60_prefers_60000_over_5994) {
  const std::vector<ModeCand> modes{k1080p5994, k1080i60, k1080p60, k720p60};
  bool matched = false;
  CHECK(pick_mode(modes, parse_screen_want("1920x1080@60"), &matched) == 2);
  CHECK(matched);
}

TEST(at5994_forces_5994) {
  const std::vector<ModeCand> modes{k1080p60, k1080p5994};
  bool matched = false;
  CHECK(pick_mode(modes, parse_screen_want("1920x1080@59.94"), &matched) == 1);
  CHECK(matched);
}

TEST(a_5994_only_sink_still_matches_at60) {
  const std::vector<ModeCand> modes{k1080i60, k1080p5994};
  bool matched = false;
  CHECK(pick_mode(modes, parse_screen_want("1920x1080@60"), &matched) == 1);
  CHECK(matched);
}

TEST(no_match_falls_back_to_preferred_then_first) {
  bool matched = true;
  const std::vector<ModeCand> modes{k720p60, k1080p60};
  CHECK(pick_mode(modes, parse_screen_want("1920x1080@50"), &matched) == 1);
  CHECK(!matched);
  const std::vector<ModeCand> no_pref{k720p60, k1080p5994};
  CHECK(pick_mode(no_pref, parse_screen_want("3840x2160@60"), &matched) == 0);
  CHECK(pick_mode({}, parse_screen_want("1920x1080@60"), &matched) == -1);
}

TEST(interlaced_never_matches) {
  const std::vector<ModeCand> modes{k1080i60};
  bool matched = true;
  pick_mode(modes, parse_screen_want("1920x1080@60"), &matched);
  CHECK(!matched);
}

TEST(parse_accepts_auto_size_and_full_forms) {
  const ScreenWant a = parse_screen_want("auto");
  CHECK(a.ok && a.kind == ScreenWant::Kind::kAuto);
  const ScreenWant s = parse_screen_want("1920x1080");
  CHECK(s.ok && s.kind == ScreenWant::Kind::kSize && s.w == 1920 && s.h == 1080);
  CHECK(s.hz == 0.0);
  const ScreenWant f = parse_screen_want("1920x1080@100");
  CHECK(f.ok && f.kind == ScreenWant::Kind::kFull && f.w == 1920 && f.h == 1080);
  CHECK(f.hz == 100.0);
  CHECK(!parse_screen_want("Auto").ok);
  CHECK(!parse_screen_want("1920x").ok);
  CHECK(!parse_screen_want("1920x1080@").ok);
  CHECK(!parse_screen_want("0x1080").ok);
  CHECK(!parse_screen_want("").ok);
}

TEST(auto_picks_the_largest_size_at_the_camera_rate) {
  bool matched = false;
  const int i = pick_display_mode(kAvatar, parse_screen_want("auto"), 100.0, &matched);
  CHECK(i == 2);  // 1920x1080@100
  CHECK(matched);
  // A camera rate a hair off the panel's still lands on it.
  CHECK(pick_display_mode(kAvatar, parse_screen_want("auto"), 99.7, &matched) == 2);
  CHECK(matched);
  CHECK(pick_display_mode(kAvatar, parse_screen_want("auto"), 60.0, &matched) == 0);
  CHECK(matched);
}

TEST(auto_ties_on_size_go_to_the_closest_rate) {
  const std::vector<ModeCand> modes{k1080p5994, k1080p60};
  bool matched = false;
  CHECK(pick_display_mode(modes, parse_screen_want("auto"), 60.0, &matched) == 1);
  CHECK(pick_display_mode(modes, parse_screen_want("auto"), 59.94, &matched) == 0);
  CHECK(matched);
}

TEST(size_picks_that_size_at_the_camera_rate) {
  bool matched = false;
  const int i = pick_display_mode(kAvatar, parse_screen_want("1280x720"), 100.0, &matched);
  CHECK(i == 3);  // 1280x720@100
  CHECK(matched);
}

TEST(auto_with_an_unshowable_rate_picks_the_nearest_rate) {
  bool matched = true;
  const int i = pick_display_mode(kAvatar, parse_screen_want("auto"), 90.0, &matched);
  CHECK(i == 2);  // 1920x1080@100: 10 Hz off against 60 Hz's 30
  CHECK(!matched);
  // A size with no mode at the camera's rate takes that size at the nearest.
  CHECK(pick_display_mode(kAvatar, parse_screen_want("1280x720"), 90.0, &matched) == 3);
  CHECK(!matched);
}

TEST(size_absent_at_the_camera_rate_is_unmatched) {
  bool matched = true;
  const int i = pick_display_mode(kAvatar, parse_screen_want("2560x1440"), 100.0, &matched);
  CHECK(i == 0);  // the preferred mode
  CHECK(!matched);
}

TEST(interlaced_modes_are_never_followed) {
  const std::vector<ModeCand> modes{k1080i60, k720p60};
  bool matched = false;
  CHECK(pick_display_mode(modes, parse_screen_want("auto"), 60.0, &matched) == 1);
  CHECK(matched);
}

TEST(unknown_camera_rate_takes_the_preferred_mode) {
  bool matched = false;
  CHECK(pick_display_mode(kAvatar, parse_screen_want("auto"), 0.0, &matched) == 0);
  // No preferred non-interlaced mode: the first.
  const std::vector<ModeCand> no_pref{k720p100, k1080p100};
  CHECK(pick_display_mode(no_pref, parse_screen_want("auto"), 0.0, &matched) == 0);
  // A size: that size at the rate closest to the preferred mode's.
  CHECK(pick_display_mode(kAvatar, parse_screen_want("1280x720"), 0.0, &matched) == 1);
  CHECK(matched);
  // A size the display lacks: the preferred mode.
  CHECK(pick_display_mode(kAvatar, parse_screen_want("2560x1440"), 0.0, &matched) == 0);
  CHECK(!matched);
  CHECK(pick_display_mode({}, parse_screen_want("auto"), 100.0, &matched) == -1);
  CHECK(pick_display_mode({}, parse_screen_want("auto"), 0.0, &matched) == -1);
}

TEST(a_full_want_keeps_the_exact_rate_pick) {
  bool matched = false;
  CHECK(pick_display_mode(kAvatar, parse_screen_want("1280x720@60"), 100.0, &matched) == 1);
  CHECK(matched);
}

TEST(mode_string_round_trips_through_parse_within_half_a_hz) {
  for (const ModeCand& m : {k1080p60, k1080p5994, k720p100, k1080p100}) {
    const std::string s = mode_string(m);
    const ScreenWant w = parse_screen_want(s);
    CHECK(w.ok && w.kind == ScreenWant::Kind::kFull);
    CHECK(w.w == m.w && w.h == m.h);
    CHECK(std::fabs(w.hz - mode_hz(m)) < 0.5);
    // ... and the exact-rate pick finds that very mode again.
    const std::vector<ModeCand> both{k1080p60, k1080p5994, k720p100, k1080p100};
    bool matched = false;
    const int i = pick_mode(both, w, &matched);
    CHECK(matched && both[static_cast<size_t>(i)].clock_khz == m.clock_khz &&
          both[static_cast<size_t>(i)].w == m.w);
  }
  CHECK(mode_string(k1080p100) == "1920x1080@100.000");
}

TEST(want_offered_refuses_a_size_the_camera_rate_lacks) {
  using maburplay::want_offered;
  CHECK(want_offered(kAvatar, parse_screen_want("auto"), 90.0));
  CHECK(want_offered(kAvatar, parse_screen_want("1280x720"), 100.0));
  CHECK(!want_offered(kAvatar, parse_screen_want("1280x720"), 90.0));
  CHECK(!want_offered(kAvatar, parse_screen_want("2560x1440"), 100.0));
  // Camera rate not known yet: any size the display has at all.
  CHECK(want_offered(kAvatar, parse_screen_want("1280x720"), 0.0));
  CHECK(!want_offered(kAvatar, parse_screen_want("2560x1440"), 0.0));
  // A full mode: the exact-rate rule, camera aside.
  CHECK(want_offered(kAvatar, parse_screen_want("1280x720@60"), 100.0));
  CHECK(!want_offered(kAvatar, parse_screen_want("1280x720@90"), 90.0));
}

MTEST_MAIN
