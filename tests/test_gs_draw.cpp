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

// Canvas carries GUARD sentinel-filled rows immediately before and after
// its visible s.height rows, in the SAME allocation, so s.pixels is not
// the allocation base. A right/bottom overflow of a few rows, or a
// left/top underflow of a few rows, lands inside these guard bands rather
// than truly outside the allocation -- so guard_intact() can catch it
// deterministically by inspecting memory the test owns, with no sanitizer
// needed. (An underflow that lands OUTSIDE the whole allocation -- e.g.
// py landing before the guard band too -- is a bug an order of magnitude
// worse than an off-by-one and is out of scope for this fixture; ASan
// still catches that class.) A right-edge/bottom-edge overflow that lands
// inside the visible width/height (the padding-column and off-by-one
// cases the other assertions cover) is unaffected by the guard band.
struct Canvas {
  static constexpr int GUARD = 4;
  static constexpr uint32_t SENTINEL = 0xDEADBEEFu;
  std::vector<uint32_t> buf;  // width * (height + 2*GUARD); NOT what s.pixels points at
  Surface s;
  int width, height;
  Canvas(int w, int h)
      : buf((size_t)w * (h + 2 * GUARD), 0u), width(w), height(h) {
    s.pixels = buf.data() + (size_t)GUARD * w;  // offset past the leading guard band
    s.width = w; s.height = h; s.stride_px = w;
    // Visible pixels start at 0 (matching every existing test's
    // expectations); only the guard bands before/after get the sentinel.
    const size_t lead = (size_t)GUARD * w;
    const size_t trail = lead + (size_t)h * w;
    for (size_t i = 0; i < lead; ++i) buf[i] = SENTINEL;
    for (size_t i = trail; i < buf.size(); ++i) buf[i] = SENTINEL;
  }
  uint32_t at(int x, int y) const { return s.pixels[(size_t)y * s.stride_px + x]; }
  int nonzero() const {
    int n = 0;
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        if (at(x, y)) ++n;
    return n;
  }
  // True iff every guard-band pixel still holds SENTINEL -- an
  // out-of-frame write that a canvas-pixel check can never observe
  // (because it lands outside the visible width/height entirely, e.g. a
  // negative-row underflow) corrupts one of these instead.
  bool guard_intact() const {
    const size_t lead = (size_t)GUARD * width;
    for (size_t i = 0; i < lead; ++i)
      if (buf[i] != SENTINEL) return false;
    const size_t trail = lead + (size_t)height * width;
    for (size_t i = trail; i < buf.size(); ++i)
      if (buf[i] != SENTINEL) return false;
    return true;
  }
};

// --- Glyph-geometry helpers for the edge-straddle tests below ------------
//
// The synthetic glyph's shadow is a 2D blur, so it recedes at the extreme
// corners of the padded cell -- the very last row/column of a glyph is
// NOT reliably nonzero. A straddle test that blindly pushes the cell's
// outermost row or column out of frame can therefore "pass" for the wrong
// reason: the pixel it pushed out was already blank, so no write was ever
// attempted and no mutant gets caught. These helpers find an ACTUAL
// nonzero (coverage or shadow) pixel to push across the boundary instead.
static bool glyph_nonzero_at(const MaskAtlas& a, int gi, int x, int y) {
  const uint8_t* px = a.glyph(gi) + (size_t)(y * a.glyph_w + x) * 2;
  return px[0] != 0 || px[1] != 0;
}

// First nonzero pixel in row-major scan order (topmost, then leftmost).
static bool first_nonzero_pixel(const MaskAtlas& a, uint32_t cp, int* x, int* y) {
  const int gi = a.index_of(cp);
  REQUIRE(gi >= 0);
  for (int yy = 0; yy < a.glyph_h; ++yy)
    for (int xx = 0; xx < a.glyph_w; ++xx)
      if (glyph_nonzero_at(a, gi, xx, yy)) { *x = xx; *y = yy; return true; }
  return false;
}

// Rightmost column with any nonzero pixel, and a row where that column is
// actually nonzero (the two are found independently -- the corner where
// both are simultaneously extremal need not exist).
static bool rightmost_content_column(const MaskAtlas& a, uint32_t cp, int* x, int* y) {
  const int gi = a.index_of(cp);
  REQUIRE(gi >= 0);
  int max_x = -1;
  for (int yy = 0; yy < a.glyph_h; ++yy)
    for (int xx = 0; xx < a.glyph_w; ++xx)
      if (glyph_nonzero_at(a, gi, xx, yy) && xx > max_x) max_x = xx;
  if (max_x < 0) return false;
  for (int yy = 0; yy < a.glyph_h; ++yy)
    if (glyph_nonzero_at(a, gi, max_x, yy)) { *x = max_x; *y = yy; return true; }
  return false;
}

// Bottommost row with any nonzero pixel.
static bool bottommost_content_row(const MaskAtlas& a, uint32_t cp, int* y) {
  const int gi = a.index_of(cp);
  REQUIRE(gi >= 0);
  int max_y = -1;
  for (int yy = 0; yy < a.glyph_h; ++yy)
    for (int xx = 0; xx < a.glyph_w; ++xx)
      if (glyph_nonzero_at(a, gi, xx, yy) && yy > max_y) max_y = yy;
  if (max_y < 0) return false;
  *y = max_y;
  return true;
}

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
  CHECK(c.guard_intact());
  Canvas d(8, 8);
  fill_rect(d.s, 6, 6, 8, 8, 0xFFFFFFu);     // bottom-right corner only
  CHECK(d.nonzero() == 2 * 2);
  CHECK(d.guard_intact());
  Canvas e(8, 8);
  fill_rect(e.s, 100, 100, 4, 4, 0xFFFFFFu); // fully outside
  CHECK(e.nonzero() == 0);
  CHECK(e.guard_intact());
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
  for (int y = 0; y < c.height; ++y) {
    for (int x = 0; x < c.width; ++x) {
      const uint32_t v = c.at(x, y);
      if (!v) continue;
      const uint32_t rgb = v & 0x00FFFFFFu;
      if (v == premul(0x3FC99Au, 255)) saw_token = true;
      if (rgb == 0 && (v >> 24) > 0) saw_shadow = true;
    }
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
  CHECK(c.guard_intact());
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

// --- Review round F1: edge-straddle + stride-pitch + utf8_next coverage --
//
// draw_text_clips_at_every_edge_without_crashing (above) only ever places
// the glyph WHOLLY off-canvas, which returns via the cheap "is this glyph
// cell anywhere near the surface" arithmetic and never reaches the
// per-pixel `px`/`py` bounds check inside the blit loop. These tests place
// a REAL nonzero pixel (found via the helpers above) so it straddles
// exactly one edge, with the canvas dimensioned so that pixel's
// out-of-frame copy lands exactly one element past (or before) the pixel
// buffer. That makes an `px >= s.width` -> `px > s.width` class mutant a
// heap-buffer-overflow or -underflow under ASan, not merely a silently
// wrong (or never-attempted) pixel.

TEST(draw_text_straddles_right_edge_without_overflow) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  int content_x = 0, content_y = 0;
  REQUIRE(rightmost_content_column(*a, '0', &content_x, &content_y));
  const int w = 50;
  // height == content_y + 1: the row that actually has ink at the
  // rightmost content column becomes the canvas's LAST row, so pushing
  // that column one past width lands exactly one element past the whole
  // buffer -- not just past the row, which a plain contiguous canvas
  // would silently alias into the next row.
  Canvas c(w, content_y + 1);
  const int ox = w - content_x;  // that column lands at px == w
  const int pen_x = ox + (a->glyph_w - a->advance_x) / 2;
  draw_text(c.s, *a, pen_x, a->baseline, "0", 0xFFFFFFu);
  CHECK(c.nonzero() > 0);  // the in-bounds columns still drew
  CHECK(c.guard_intact());
  std::remove(p.c_str());
}

TEST(draw_text_straddles_left_edge_without_underflow) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  int content_x = 0, content_y = 0;
  REQUIRE(first_nonzero_pixel(*a, '0', &content_x, &content_y));
  const int w = 50, h = 40;
  Canvas c(w, h);
  // px == -1 is only a genuine one-before-the-start underflow on row 0 --
  // on any later row it aliases into the previous row's last column,
  // which a plain contiguous canvas would not fault on. So the content
  // pixel we push out must land on row 0.
  const int oy = -content_y;         // content row lands at py == 0
  const int ox = -1 - content_x;     // content column lands at px == -1
  const int baseline_y = a->baseline + oy;
  const int pen_x = ox + (a->glyph_w - a->advance_x) / 2;
  draw_text(c.s, *a, pen_x, baseline_y, "0", 0xFFFFFFu);
  CHECK(c.nonzero() > 0);
  CHECK(c.guard_intact());
  std::remove(p.c_str());
}

TEST(draw_text_straddles_top_edge_without_underflow) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  int content_x = 0, content_y = 0;
  REQUIRE(first_nonzero_pixel(*a, '0', &content_x, &content_y));
  const int w = 50, h = 40;
  Canvas c(w, h);
  // py == -1 is an absolute buffer-start underflow for ANY in-range
  // column (index = -stride + px < 0 whenever px < stride), so no column
  // alignment is needed here -- only the row has to land at py == -1.
  const int oy = -1 - content_y;
  const int baseline_y = a->baseline + oy;
  draw_text(c.s, *a, 10, baseline_y, "0", 0xFFFFFFu);
  CHECK(c.nonzero() > 0);
  // py == -1 computes `s.pixels + (size_t)(-1) * s.stride_px`: the
  // unsigned cast makes the intermediate offset huge, but two's-complement
  // wraparound means the resulting pointer is -- in practice, not by
  // contract -- exactly one row before s.pixels. On a bare (non-guarded)
  // buffer that lands before the allocation entirely: invisible to any
  // canvas-pixel check, and only reliably caught by ASan/a sanitizer,
  // which is exactly the gap this Canvas's guard band exists to close.
  CHECK(c.guard_intact());
  std::remove(p.c_str());
}

TEST(draw_text_straddles_bottom_edge_without_overflow) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  int content_y = 0;
  REQUIRE(bottommost_content_row(*a, '0', &content_y));
  const int w = 50;
  // height == content_y: the row that actually has ink becomes py ==
  // height, one past the last valid row, for EVERY nonzero column in it
  // -- so, unlike the right-edge case, no column alignment is needed.
  Canvas c(w, content_y);
  const int pen_x = (a->glyph_w - a->advance_x) / 2;  // ox == 0
  draw_text(c.s, *a, pen_x, a->baseline, "0", 0xFFFFFFu);
  CHECK(c.nonzero() > 0);
  CHECK(c.guard_intact());
  std::remove(p.c_str());
}

// A Surface whose stride_px is wider than its width, like a real DRM dumb
// buffer's padded row pitch. Canvas above always sets stride_px == width,
// so it can never catch a bound check that confuses "column index" with
// "pixel offset into the row" -- writing into the padding wouldn't show up
// as a wrong on-screen pixel, and without a genuinely out-of-bounds row it
// wouldn't necessarily fault under ASan either.
struct PaddedCanvas {
  std::vector<uint32_t> px;
  Surface s;
  int width;
  PaddedCanvas(int w, int h, int extra_pad)
      : px((size_t)(w + extra_pad) * h, 0u), width(w) {
    s.pixels = px.data();
    s.width = w;
    s.height = h;
    s.stride_px = w + extra_pad;
  }
  // True iff every padding column (x in [width, stride_px) on every row)
  // is still zero -- nothing in the API contract permits writing there.
  bool padding_untouched() const {
    for (int y = 0; y < s.height; ++y)
      for (int x = width; x < s.stride_px; ++x)
        if (px[(size_t)y * s.stride_px + x] != 0) return false;
    return true;
  }
};

TEST(fill_rect_never_writes_the_row_padding) {
  PaddedCanvas c(16, 8, 7);  // stride_px = 23; columns 16..22 are padding
  fill_rect(c.s, 0, 0, 16, 8, 0xFFFFFFu);  // exactly full width, every row
  CHECK(c.padding_untouched());
}

TEST(clear_region_never_writes_the_row_padding) {
  PaddedCanvas c(16, 8, 7);
  fill_rect(c.s, 0, 0, 16, 8, 0xFFFFFFu);
  clear_region(c.s, DirtyRect{0, 0, 16, 8});
  CHECK(c.padding_untouched());
}

TEST(draw_text_never_writes_the_row_padding) {
  const std::string p = make_font("20");
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  int content_x = 0, content_y = 0;
  REQUIRE(rightmost_content_column(*a, '0', &content_x, &content_y));
  // Surface width == glyph_w, so a straight (ox == 0) placement never
  // reaches the padding at all -- the rightmost content column must be
  // pushed past width, exactly as in the right-edge straddle test above,
  // but here on a padded pitch instead of a tightly-packed buffer.
  PaddedCanvas c(a->glyph_w, a->glyph_h, 7);
  const int ox = a->glyph_w - content_x;  // that column lands at px == width
  const int pen_x = ox + (a->glyph_w - a->advance_x) / 2;
  draw_text(c.s, *a, pen_x, a->baseline, "0", 0xFFFFFFu);
  CHECK(c.padding_untouched());
  std::remove(p.c_str());
}

// utf8_next is exposed "for testing" but had no direct test: draw_text and
// text_width only ever fed it well-formed ASCII/UTF-8, so a decoder that
// spun in place (never advancing) or read past the terminator on a
// truncated sequence would have sailed through undetected.
TEST(utf8_next_truncated_and_malformed_sequences_advance_and_never_overrun) {
  // Each truncated case ends with the string's own NUL terminator exactly
  // where a continuation byte would need to be. The short-circuit &&
  // chain in utf8_next is supposed to stop at the first byte that fails
  // the continuation check, and 0x00 never passes that check -- so if any
  // of these ever read past the terminator, ASan would catch it.
  {
    const char s[] = "\xC2";  // truncated 2-byte lead
    const char* p = s;
    CHECK(utf8_next(&p) == 0xFFFDu);
    CHECK(p == s + 1);  // did not consume a nonexistent 2nd byte
  }
  {
    const char s[] = "\xE0\x80";  // truncated 3-byte, one continuation
    const char* p = s;
    CHECK(utf8_next(&p) == 0xFFFDu);
    CHECK(p == s + 1);
  }
  {
    const char s[] = "\xF0\x80\x80";  // truncated 4-byte, two continuations
    const char* p = s;
    CHECK(utf8_next(&p) == 0xFFFDu);
    CHECK(p == s + 1);
  }
  {
    const char s[] = "\x80";  // lone continuation byte, no lead at all
    const char* p = s;
    CHECK(utf8_next(&p) == 0xFFFDu);
    CHECK(p == s + 1);
  }
  {
    const char s[] = "\xFF";  // not a valid lead byte under any width
    const char* p = s;
    CHECK(utf8_next(&p) == 0xFFFDu);
    CHECK(p == s + 1);
  }
  {
    // Overlong 2-byte encoding of NUL (0xC0 0x80): structurally
    // well-formed, so it is accepted and both bytes are consumed --
    // utf8_next is not required to reject overlong forms, only to never
    // spin and never read out of bounds, which the other cases cover.
    const char s[] = "\xC0\x80";
    const char* p = s;
    utf8_next(&p);
    CHECK(p == s + 2);
  }
}

MTEST_MAIN
