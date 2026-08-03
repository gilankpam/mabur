#include "mtest.h"
#include "osd_layout.h"
using namespace maburplay;

// The shipping case: Betaflight HD canvas, 1080p, 36x54 atlas -> 1:1.
TEST(hd_50x18_on_1080p_is_one_to_one) {
  const OsdLayout l = compute_layout(1920, 1080, 50, 18, 36, 54, ScaleMode::kSharp);
  CHECK(l.draw_w == 36);
  CHECK(l.draw_h == 54);
  CHECK(l.origin_x == (1920 - 36 * 50) / 2);  // 60
  CHECK(l.origin_y == (1080 - 54 * 18) / 2);  // 54
  CHECK(l.cols == 50);
  CHECK(l.rows == 18);
}

// SD canvas: 2x would be 2160x1728 (too tall), so it stays 1:1 and centred.
TEST(sd_30x16_on_1080p_stays_one_to_one) {
  const OsdLayout l = compute_layout(1920, 1080, 30, 16, 36, 54, ScaleMode::kSharp);
  CHECK(l.draw_w == 36);
  CHECK(l.draw_h == 54);
  CHECK(l.origin_x == (1920 - 36 * 30) / 2);  // 420
  CHECK(l.origin_y == (1080 - 54 * 16) / 2);  // 108
}

// 60x22 does not fit at native size -> aspect-preserving downscale (width-limited).
TEST(dense_60x22_downscales_preserving_aspect) {
  const OsdLayout l = compute_layout(1920, 1080, 60, 22, 36, 54, ScaleMode::kSharp);
  // avail 32x49; width-limited (32*54 = 1728 <= 49*36 = 1764).
  CHECK(l.draw_w == 32);
  CHECK(l.draw_h == 48);
  CHECK(l.origin_x == 0);
  CHECK(l.origin_y == (1080 - 48 * 22) / 2);  // 12
}

// Tall cells downscale height-limited (the other aspect case).
TEST(tall_cells_downscale_height_limited) {
  const OsdLayout l = compute_layout(1920, 1080, 10, 54, 54, 36, ScaleMode::kSharp);
  // avail 192x20; height-limited (192*36 = 6912 > 20*54 = 1080).
  CHECK(l.draw_w == 30);
  CHECK(l.draw_h == 20);
  CHECK(l.origin_x == 810);
  CHECK(l.origin_y == 0);
  CHECK(l.cols == 10);
  CHECK(l.rows == 54);
}

// A small atlas on a big screen scales by a whole number, never 1.6x.
TEST(small_atlas_uses_integer_multiple_only) {
  const OsdLayout l = compute_layout(1920, 1080, 50, 18, 24, 36, ScaleMode::kSharp);
  // avail 38x60 -> k = min(38/24, 60/36) = 1. Not 1.58x.
  CHECK(l.draw_w == 24);
  CHECK(l.draw_h == 36);

  const OsdLayout l2 = compute_layout(1920, 1080, 20, 9, 24, 36, ScaleMode::kSharp);
  // avail 96x120 -> k = min(4, 3) = 3.
  CHECK(l2.draw_w == 72);
  CHECK(l2.draw_h == 108);
}

TEST(fill_mode_uses_the_whole_cell) {
  const OsdLayout l = compute_layout(1920, 1080, 50, 18, 36, 54, ScaleMode::kFill);
  CHECK(l.draw_w == 1920 / 50);  // 38
  CHECK(l.draw_h == 1080 / 18);  // 60
  CHECK(l.origin_x == (1920 - 38 * 50) / 2);
}

TEST(degenerate_inputs_yield_an_empty_layout) {
  CHECK(compute_layout(0, 1080, 50, 18, 36, 54, ScaleMode::kSharp).draw_w == 0);
  CHECK(compute_layout(1920, 1080, 0, 18, 36, 54, ScaleMode::kSharp).draw_w == 0);
  CHECK(compute_layout(1920, 1080, 50, 18, 0, 54, ScaleMode::kSharp).draw_w == 0);
  CHECK(compute_layout(10, 10, 50, 18, 36, 54, ScaleMode::kSharp).draw_w == 0);
}

TEST(layouts_compare_equal_by_value) {
  const OsdLayout a = compute_layout(1920, 1080, 50, 18, 36, 54, ScaleMode::kSharp);
  const OsdLayout b = compute_layout(1920, 1080, 50, 18, 36, 54, ScaleMode::kSharp);
  const OsdLayout c = compute_layout(1920, 1080, 30, 16, 36, 54, ScaleMode::kSharp);
  CHECK(a == b);
  CHECK(!(a == c));
}

MTEST_MAIN
