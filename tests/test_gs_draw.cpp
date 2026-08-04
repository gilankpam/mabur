#include "mtest.h"
#include "gs_draw.h"
#include "gs_font.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace maburplay;

static std::string make_font(const char* sizes) {
  std::string path = std::string(std::tmpnam(nullptr)) + ".gfont";
  const std::string cmd = std::string("python3 ") + GEN_GSFONT + " --synthetic " +
                          path + " --sizes " + sizes + " >/dev/null 2>&1";
  REQUIRE(std::system(cmd.c_str()) == 0);
  return path;
}

struct Canvas {
  std::vector<uint32_t> px;
  Surface s;
  Canvas(int w, int h) : px((size_t)w * h, 0u) {
    s.pixels = px.data(); s.width = w; s.height = h; s.stride_px = w;
  }
  uint32_t at(int x, int y) const { return px[(size_t)y * s.stride_px + x]; }
  int nonzero() const {
    int n = 0;
    for (uint32_t v : px) if (v) ++n;
    return n;
  }
};

TEST(premul_scales_channels_and_sets_alpha) {
  CHECK(premul(0xFF8000u, 255) == 0xFFFF8000u);
  CHECK(premul(0xFFFFFFu, 0) == 0x00000000u);
  const uint32_t half = premul(0xFFFFFFu, 128);
  CHECK((half >> 24) == 128u);
  CHECK(((half >> 16) & 0xFF) == 128u);  // premultiplied, not 255
}

TEST(fill_rect_writes_exactly_the_rect) {
  Canvas c(32, 16);
  fill_rect(c.s, 4, 2, 8, 3, 0x3FC99Au);
  CHECK(c.nonzero() == 8 * 3);
  CHECK(c.at(4, 2) == premul(0x3FC99Au, 255));
  CHECK(c.at(11, 4) == premul(0x3FC99Au, 255));
  CHECK(c.at(3, 2) == 0u);
  CHECK(c.at(12, 2) == 0u);
  CHECK(c.at(4, 5) == 0u);
}

TEST(fill_rect_clips_to_the_surface) {
  Canvas c(8, 8);
  fill_rect(c.s, -4, -4, 8, 8, 0xFFFFFFu);   // top-left quadrant only
  CHECK(c.nonzero() == 4 * 4);
  Canvas d(8, 8);
  fill_rect(d.s, 6, 6, 8, 8, 0xFFFFFFu);     // bottom-right corner only
  CHECK(d.nonzero() == 2 * 2);
  Canvas e(8, 8);
  fill_rect(e.s, 100, 100, 4, 4, 0xFFFFFFu); // fully outside
  CHECK(e.nonzero() == 0);
}

TEST(clear_region_zeroes_only_its_rect) {
  Canvas c(16, 16);
  fill_rect(c.s, 0, 0, 16, 16, 0xFFFFFFu);
  clear_region(c.s, DirtyRect{4, 4, 4, 4});
  CHECK(c.nonzero() == 16 * 16 - 16);
  CHECK(c.at(4, 4) == 0u);
  CHECK(c.at(3, 4) != 0u);
}

TEST(text_width_is_advance_times_glyph_count) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  CHECK(text_width(*a, "") == 0);
  CHECK(text_width(*a, "ABC") == 3 * a->advance_x);
  // Multi-byte codepoints advance ONCE, not once per byte -- this is the
  // whole reason text_width decodes UTF-8 rather than counting chars.
  CHECK(text_width(*a, "\xE2\x88\x92") == a->advance_x);         // U+2212
  CHECK(text_width(*a, "\xE2\x88\x92" "58") == 3 * a->advance_x);
  std::remove(p.c_str());
}

TEST(draw_text_lands_pixels_and_returns_advance) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  Canvas c(200, 60);
  const int adv = draw_text(c.s, *a, 10, a->baseline, "0", 0xF2F3F5u);
  CHECK(adv == a->advance_x);
  CHECK(c.nonzero() > 0);
}

TEST(draw_text_colours_coverage_and_blackens_shadow) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  Canvas c(200, 60);
  draw_text(c.s, *a, 10, a->baseline, "0", 0x3FC99Au);
  // Fully covered pixels are the token colour at full alpha; shadow-only
  // pixels are opaque-ish black. Neither may be the other's colour.
  bool saw_token = false, saw_shadow = false;
  for (uint32_t v : c.px) {
    if (!v) continue;
    const uint32_t rgb = v & 0x00FFFFFFu;
    if (v == premul(0x3FC99Au, 255)) saw_token = true;
    if (rgb == 0 && (v >> 24) > 0) saw_shadow = true;
  }
  CHECK(saw_token);
  CHECK(saw_shadow);
  std::remove(p.c_str());
}

TEST(draw_text_clips_at_every_edge_without_crashing) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  Canvas c(40, 40);
  draw_text(c.s, *a, -100, a->baseline, "000", 0xFFFFFFu);
  draw_text(c.s, *a, 200, a->baseline, "000", 0xFFFFFFu);
  draw_text(c.s, *a, 10, -100, "000", 0xFFFFFFu);
  draw_text(c.s, *a, 10, 500, "000", 0xFFFFFFu);
  CHECK(true);  // survived
  std::remove(p.c_str());
}

// A codepoint the atlas lacks still advances the pen, so a missing glyph
// leaves a gap rather than shifting the whole rest of the line.
TEST(missing_glyph_advances_but_draws_nothing) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  Canvas c(200, 60);
  const int adv = draw_text(c.s, *a, 10, a->baseline, "\xE4\xB8\x80", 0xFFFFFFu);  // U+4E00
  CHECK(adv == a->advance_x);
  CHECK(c.nonzero() == 0);
  std::remove(p.c_str());
}

TEST(null_surface_is_a_safe_no_op) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  Surface null_s;  // pixels == nullptr: the presenter's OSD-disabled state
  CHECK(draw_text(null_s, *a, 0, 0, "ABC", 0xFFFFFFu) == 3 * a->advance_x);
  fill_rect(null_s, 0, 0, 4, 4, 0xFFFFFFu);
  clear_region(null_s, DirtyRect{0, 0, 4, 4});
  CHECK(true);  // survived
  std::remove(p.c_str());
}

MTEST_MAIN
