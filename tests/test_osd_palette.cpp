#include "mtest.h"
#include "osd_palette.h"
#include "osd_font.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace maburplay;

static uint8_t Y(uint32_t e) { return e & 0xFF; }
static uint8_t U(uint32_t e) { return (e >> 8) & 0xFF; }
static uint8_t V(uint32_t e) { return (e >> 16) & 0xFF; }
static uint8_t A(uint32_t e) { return (e >> 24) & 0xFF; }

// Surface helper: w x h, all transparent unless set.
struct Canvas {
  std::vector<uint32_t> px;
  Surface s;
  Canvas(int w, int h) : px((size_t)w * h, 0u) { s = Surface{px.data(), w, h, w}; }
  void set(int x, int y, uint32_t argb) { px[(size_t)y * s.stride_px + x] = argb; }
};

// A palette containing exactly transparent + opaque white + opaque black.
static OsdPalette tiny_palette() {
  std::vector<uint32_t> pix = {0x00000000u, 0xFFFFFFFFu, 0xFF000000u};
  GlyphAtlas a{1, 3, 1, pix.data()};
  return build_palette(a);
}

TEST(entry0_is_always_fully_transparent) {
  const OsdPalette p = tiny_palette();
  CHECK(A(p.entry[0]) == 0);
}

// The trap that rendered every white glyph pink on hardware: MPP wants
// Y in the LOW byte, despite the header's bitfield names.
TEST(white_is_bt601_limited_with_luma_in_the_low_byte) {
  const OsdPalette p = tiny_palette();
  int wi = -1;
  for (int i = 0; i < p.n; ++i)
    if (A(p.entry[i]) == 255 && Y(p.entry[i]) > 200) wi = i;
  REQUIRE(wi >= 0);
  CHECK(Y(p.entry[wi]) >= 233 && Y(p.entry[wi]) <= 237);  // 235
  CHECK(U(p.entry[wi]) >= 126 && U(p.entry[wi]) <= 130);  // 128
  CHECK(V(p.entry[wi]) >= 126 && V(p.entry[wi]) <= 130);  // 128
}

TEST(black_is_bt601_limited_luma_16) {
  const OsdPalette p = tiny_palette();
  int bi = -1;
  for (int i = 0; i < p.n; ++i)
    if (A(p.entry[i]) == 255 && Y(p.entry[i]) < 40) bi = i;
  REQUIRE(bi >= 0);
  CHECK(Y(p.entry[bi]) >= 14 && Y(p.entry[bi]) <= 18);  // 16
}

// Premultiplied input must be un-premultiplied before the RGB->YUV step,
// or half-alpha white converts as half-grey.
TEST(premultiplied_input_is_unpremultiplied_before_conversion) {
  std::vector<uint32_t> pix = {0x80808080u};  // 50% alpha white, premultiplied
  GlyphAtlas a{1, 1, 1, pix.data()};
  const OsdPalette p = build_palette(a);
  int wi = -1;
  for (int i = 0; i < p.n; ++i) if (A(p.entry[i]) > 0) wi = i;
  REQUIRE(wi >= 0);
  CHECK(A(p.entry[wi]) >= 126 && A(p.entry[wi]) <= 130);  // alpha preserved
  CHECK(Y(p.entry[wi]) >= 230);  // WHITE, not mid-grey (~125)
}

// The other trap: raster layout, stride = mb_w*16 -- NOT 16x16 tiles.
TEST(index_map_is_raster_with_mb_stride) {
  const OsdPalette p = tiny_palette();
  Canvas c(48, 32);
  c.set(17, 3, 0xFFFFFFFFu);  // second macroblock column, 4th row
  OsdIndexMap m;
  quantize(c.s, p, &m);
  CHECK(m.mb_w == 3);
  CHECK(m.mb_h == 2);
  CHECK(m.stride() == 48);
  CHECK(m.px.size() == (size_t)48 * 32);
  const uint8_t idx = m.px[(size_t)3 * m.stride() + 17];
  CHECK(idx != 0);
  CHECK(A(p.entry[idx]) == 255);
  // A macroblock-tiled writer would have put it at byte 256+... instead:
  CHECK(m.px[256] == 0);
}

TEST(macroblock_dims_round_up_and_1080_needs_68_rows) {
  const OsdPalette p = tiny_palette();
  Canvas c(1920, 1080);
  OsdIndexMap m;
  quantize(c.s, p, &m);
  CHECK(m.mb_w == 120);
  CHECK(m.mb_h == 68);              // NOT 67 -- the bottom 8 lines matter
  CHECK(m.px.size() == (size_t)1920 * 1088);
}

TEST(transparent_pixels_map_to_index_zero) {
  const OsdPalette p = tiny_palette();
  Canvas c(32, 16);
  OsdIndexMap m;
  quantize(c.s, p, &m);
  for (uint8_t v : m.px) CHECK(v == 0);
}

TEST(padding_beyond_the_surface_is_transparent) {
  const OsdPalette p = tiny_palette();
  Canvas c(20, 20);          // pads to 32x32
  c.set(0, 0, 0xFFFFFFFFu);
  OsdIndexMap m;
  quantize(c.s, p, &m);
  CHECK(m.mb_w == 2);
  CHECK(m.mb_h == 2);
  CHECK(m.px[0] != 0);
  CHECK(m.px[25] == 0);                              // x=25, past width 20
  CHECK(m.px[(size_t)25 * m.stride() + 5] == 0);     // y=25, past height 20
}

// The incremental path exists so a full-screen quantize does not run on
// maburplay's 2 ms main loop. It is only worth anything if it lands on the
// same bytes a full pass would.
TEST(incremental_quantize_matches_a_full_pass) {
  const OsdPalette p = tiny_palette();
  Canvas c(64, 48);
  for (int x = 0; x < 30; ++x) c.set(x, 5, 0xFFFFFFFFu);
  OsdIndexMap inc, full;
  quantize(c.s, p, &inc);
  quantize(c.s, p, &full);
  REQUIRE(inc.px == full.px);

  // Change one band; re-quantize only it, and compare against a from-
  // scratch pass over the whole changed surface.
  for (int x = 10; x < 20; ++x) {
    c.set(x, 5, 0xFF000000u);
    c.set(x, 6, 0xFFFFFFFFu);
  }
  const DirtyRect r{8, 4, 16, 4};
  QuantizeCache cache;
  CHECK(quantize_rects(c.s, p, &r, 1, &inc, &cache));
  quantize(c.s, p, &full);
  CHECK(inc.px == full.px);
  CHECK(inc.mb_w == full.mb_w && inc.mb_h == full.mb_h);
}

TEST(incremental_quantize_clips_rects_to_the_surface) {
  const OsdPalette p = tiny_palette();
  Canvas c(20, 20);  // pads to 32x32
  c.set(19, 19, 0xFFFFFFFFu);
  OsdIndexMap m;
  quantize(c.s, p, &m);
  const DirtyRect r{-4, -4, 100, 100};  // overhangs on every side
  CHECK(quantize_rects(c.s, p, &r, 1, &m, nullptr));
  CHECK(m.px[(size_t)19 * m.stride() + 19] != 0);
  CHECK(m.px[(size_t)25 * m.stride() + 25] == 0);  // padding untouched
}

// A map that was never sized for this surface must be refused, not written
// through: that is the caller's signal to run a full quantize().
TEST(incremental_quantize_refuses_an_unsized_map) {
  const OsdPalette p = tiny_palette();
  Canvas c(64, 48);
  OsdIndexMap empty;
  const DirtyRect r{0, 0, 64, 48};
  CHECK(!quantize_rects(c.s, p, &r, 1, &empty, nullptr));
  CHECK(empty.px.empty());

  OsdIndexMap wrong;
  Canvas other(32, 32);
  quantize(other.s, p, &wrong);
  CHECK(!quantize_rects(c.s, p, &r, 1, &wrong, nullptr));
}

// The cache is a pure memo: it must not change any answer.
TEST(quantize_cache_reuse_does_not_change_the_result) {
  const OsdPalette p = tiny_palette();
  Canvas a(48, 32), b(48, 32);
  a.set(3, 3, 0xFFFFFFFFu);
  a.set(4, 3, 0xFF000000u);
  b.set(9, 9, 0xFF000000u);
  QuantizeCache cache;
  OsdIndexMap ma, mb, ra, rb;
  quantize(a.s, p, &ma, &cache);
  quantize(b.s, p, &mb, &cache);  // second use: warm cache
  quantize(a.s, p, &ra);
  quantize(b.s, p, &rb);
  CHECK(ma.px == ra.px);
  CHECK(mb.px == rb.px);
}

TEST(real_atlas_quantizes_within_a_sane_error_bound) {
  OsdFont font;
  std::string err;
  REQUIRE(font.load(MABUR_PLAY_BUNDLE_DIR "/font_btfl.mfont", &err));
  const OsdPalette p = build_palette(font.native());
  CHECK(p.n > 200);       // a real atlas should fill most of the table
  CHECK(p.n <= 256);
  CHECK(A(p.entry[0]) == 0);
}

MTEST_MAIN
