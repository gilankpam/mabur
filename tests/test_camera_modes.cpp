// Host test for drone/venc/camera_modes.c: which sensor mode serves which
// venc.fps, how the pipeline finds it in the driver's list, and which
// encode sizes a mode can serve.
#include "camera_modes.h"
#include "mtest.h"

namespace {

bool same(CameraMode a, CameraMode b) {
  return a.w == b.w && a.h == b.h && a.max_fps == b.max_fps;
}

const CameraMode kWaybeam[] = {
    {3840, 2160, 33}, {2944, 1656, 60}, {1920, 1080, 90},
    {2272, 1278, 100}, {1728, 972, 100}, {1728, 816, 120}};
const CameraMode kStock[] = {
    {3840, 2160, 30}, {2560, 1440, 60}, {1920, 1080, 90}, {1472, 816, 120}};

}  // namespace

TEST(mode_for_binned_picks_by_fps) {
  CHECK(same(camera_mode_for(1, 60, 1920, 1080), CameraMode{1920, 1080, 90}));
  CHECK(same(camera_mode_for(1, 90, 1920, 1080), CameraMode{1920, 1080, 90}));
  CHECK(same(camera_mode_for(1, 91, 1920, 1080), CameraMode{1728, 972, 100}));
  CHECK(same(camera_mode_for(1, 100, 1920, 1080), CameraMode{1728, 972, 100}));
  CHECK(same(camera_mode_for(1, 101, 1920, 1080), CameraMode{1728, 816, 120}));
  CHECK(same(camera_mode_for(1, 120, 1920, 1080), CameraMode{1728, 816, 120}));
}

TEST(mode_for_native_picks_by_fps) {
  CHECK(same(camera_mode_for(0, 1, 0, 0), CameraMode{2944, 1656, 60}));
  CHECK(same(camera_mode_for(0, 60, 0, 0), CameraMode{2944, 1656, 60}));
  CHECK(same(camera_mode_for(0, 61, 0, 0), CameraMode{2272, 1278, 100}));
  CHECK(same(camera_mode_for(0, 100, 0, 0), CameraMode{2272, 1278, 100}));
  CHECK(camera_mode_for(0, 101, 0, 0).w == 0);   // no native mode above 100
  CHECK(camera_mode_for(0, 120, 0, 0).w == 0);
}

TEST(mode_for_4k_record_needs_above_1080p_and_above_the_mode) {
  CHECK(same(camera_mode_for(1, 30, 3840, 2160), CameraMode{3840, 2160, 33}));
  CHECK(same(camera_mode_for(0, 30, 3840, 2160), CameraMode{3840, 2160, 33}));
  CHECK(same(camera_mode_for(1, 60, 2560, 1440), CameraMode{3840, 2160, 33}));   // above the 1920 mode
  CHECK(same(camera_mode_for(0, 60, 2560, 1440), CameraMode{2944, 1656, 60}));   // fits the 2944 mode
  CHECK(same(camera_mode_for(0, 100, 2560, 1440), CameraMode{3840, 2160, 33}));  // above the 2272 mode
  CHECK(same(camera_mode_for(1, 100, 1920, 1080), CameraMode{1728, 972, 100}));  // the legal upscale
}

TEST(find_matches_geometry_and_max_fps) {
  CHECK(camera_mode_find(kWaybeam, 6, CameraMode{1728, 972, 100}) == 4);
  CHECK(camera_mode_find(kWaybeam, 6, CameraMode{1920, 1080, 90}) == 2);
  CHECK(camera_mode_find(kWaybeam, 6, CameraMode{2272, 1278, 100}) == 3);
  CHECK(camera_mode_find(kStock, 4, CameraMode{1728, 972, 100}) == -1);
  // Same size, different ceiling: not the mode the table means.
  CHECK(camera_mode_find(kStock, 4, CameraMode{3840, 2160, 33}) == -1);
}

TEST(size_ok_needs_aspect_and_fit) {
  const CameraMode m100{1728, 972, 100}, m120{1728, 816, 120};
  CHECK(camera_mode_size_ok(m100, 1280, 720) == 1);
  CHECK(camera_mode_size_ok(m100, 1728, 972) == 1);
  CHECK(camera_mode_size_ok(m100, 1920, 1080) == 0);  // larger than the mode
  CHECK(camera_mode_size_ok(m100, 1472, 816) == 0);   // other aspect
  CHECK(camera_mode_size_ok(m120, 1728, 816) == 1);
  CHECK(camera_mode_size_ok(m120, 1152, 544) == 1);
  CHECK(camera_mode_size_ok(m120, 1280, 720) == 0);
}

TEST(size_ok_for_the_native_modes) {
  const CameraMode m60{2944, 1656, 60}, m100{2272, 1278, 100};
  CHECK(camera_mode_size_ok(m60, 2944, 1656) == 1);
  CHECK(camera_mode_size_ok(m60, 2560, 1440) == 1);
  CHECK(camera_mode_size_ok(m60, 1920, 1080) == 1);
  CHECK(camera_mode_size_ok(m100, 2272, 1278) == 1);
  CHECK(camera_mode_size_ok(m100, 1600, 900) == 1);
  CHECK(camera_mode_size_ok(m100, 2560, 1440) == 0);  // larger than the mode
}

MTEST_MAIN
