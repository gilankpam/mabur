#include "mtest.h"
#include "msp_font.h"
using namespace maburgs;

TEST(font_table_shape) {
  CHECK(kMspFontBtfl.glyph_w == 24);
  CHECK(kMspFontBtfl.glyph_h == 36);
  CHECK(kMspFontBtfl.n_glyphs == 1024);
  REQUIRE(kMspFontBtfl.pixels != nullptr);
}

static int opaque_px(int gi) {
  const auto& f = kMspFontBtfl;
  const uint32_t* g = f.pixels + (size_t)gi * f.glyph_w * f.glyph_h;
  int n = 0;
  for (int i = 0; i < f.glyph_w * f.glyph_h; ++i)
    if (g[i] >> 24) ++n;  // alpha > 0
  return n;
}

TEST(known_glyphs_present) {
  // Char '1' (0x31) and 'A' (0x41) on page 0 have ink; the null char is empty.
  CHECK(opaque_px(0x31) > 0);
  CHECK(opaque_px(0x41) > 0);
  CHECK(opaque_px(0x00) == 0);
}

MTEST_MAIN;
