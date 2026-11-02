#include "mtest.h"
#include "mabur/camera_modes.h"
using namespace mabur::cammode;

TEST(mode_for_picks_by_rate_and_readout) {
  CHECK(mode_for(60, true)->max_fps == 90);
  CHECK(mode_for(91, true)->max_fps == 100);
  CHECK(mode_for(120, true)->max_fps == 120);
  CHECK(mode_for(60, false)->max_fps == 60);
  CHECK(mode_for(90, false)->max_fps == 100);      // native 61-100 is one sensor mode
  CHECK(mode_for(101, false) == nullptr);
  CHECK(mode_for(121, true) == nullptr);
  CHECK(mode_for(0, true) == nullptr);
}

TEST(sizes_match_the_web_table) {
  const std::vector<SensorMode>& m = modes();
  REQUIRE(m.size() == 5);

  // binned <=90: 1920x1080, 1280x720
  CHECK(m[0].binned && m[0].max_fps == 90);
  REQUIRE(m[0].sizes.size() == 2);
  CHECK(m[0].sizes[0].w == 1920 && m[0].sizes[0].h == 1080);
  CHECK(m[0].sizes[1].w == 1280 && m[0].sizes[1].h == 720);

  // binned <=100: 1728x972, 1280x720
  CHECK(m[1].binned && m[1].max_fps == 100);
  REQUIRE(m[1].sizes.size() == 2);
  CHECK(m[1].sizes[0].w == 1728 && m[1].sizes[0].h == 972);
  CHECK(m[1].sizes[1].w == 1280 && m[1].sizes[1].h == 720);

  // binned <=120: 1728x816, 1152x544, 864x408
  CHECK(m[2].binned && m[2].max_fps == 120);
  REQUIRE(m[2].sizes.size() == 3);
  CHECK(m[2].sizes[0].w == 1728 && m[2].sizes[0].h == 816);
  CHECK(m[2].sizes[1].w == 1152 && m[2].sizes[1].h == 544);
  CHECK(m[2].sizes[2].w == 864 && m[2].sizes[2].h == 408);

  // native <=60: 1920x1080, 1600x900, 1280x720
  CHECK(!m[3].binned && m[3].max_fps == 60);
  REQUIRE(m[3].sizes.size() == 3);
  CHECK(m[3].sizes[0].w == 1920 && m[3].sizes[0].h == 1080);
  CHECK(m[3].sizes[1].w == 1600 && m[3].sizes[1].h == 900);
  CHECK(m[3].sizes[2].w == 1280 && m[3].sizes[2].h == 720);

  // native <=100: 1920x1080, 1600x900, 1280x720
  CHECK(!m[4].binned && m[4].max_fps == 100);
  REQUIRE(m[4].sizes.size() == 3);
  CHECK(m[4].sizes[0].w == 1920 && m[4].sizes[0].h == 1080);
  CHECK(m[4].sizes[1].w == 1600 && m[4].sizes[1].h == 900);
  CHECK(m[4].sizes[2].w == 1280 && m[4].sizes[2].h == 720);
}

TEST(choice_labels_carry_the_zoom_and_bin_tag) {
  CHECK(choice_label(60, 1920, 1080, true) == "60 fps · 1920×1080 · 1.0× bin");
  CHECK(choice_label(60, 1920, 1080, false) == "60 fps · 1920×1080 · 1.3×");
  CHECK(choice_label(100, 1728, 972, true) == "100 fps · 1728×972 · 1.1× bin");
  CHECK(choice_label(100, 1920, 1080, false) == "100 fps · 1920×1080 · 1.7×");
  CHECK(choice_label(120, 1728, 816, true) == "120 fps · 1728×816 · 1.1× bin");
  CHECK(choice_label(90, 1280, 720, false) == "90 fps · 1280×720 · 1.7×");
}

MTEST_MAIN
