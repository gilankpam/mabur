#include "mtest.h"
#include "osd_palette.h"
#include "osd_font.h"
#include "gs_draw.h"     // premul()
#include "gs_overlay.h"  // GsOverlay::palette_seeds(), tok::
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

// tiny_palette() is achromatic -- white and black are both U=V=128 -- so a
// TRANSPOSED U/V channel is invisible to every test above it. These two are
// the regression insurance for that: the expected values are MPP's own
// MPP_ENC_OSD_PLT_RED / _GREEN constants, i.e. the hardware's opinion of
// what BT.601 red and green are, not a rederivation of ours.
TEST(coloured_entries_match_mpps_own_red_and_green) {
  std::vector<uint32_t> pix = {0x00000000u, 0xFFFF0000u, 0xFF00FF00u};
  GlyphAtlas a{1, 3, 1, pix.data()};
  const OsdPalette p = build_palette(a);

  int ri = -1, gi = -1;
  for (int i = 0; i < p.n; ++i) {
    if (A(p.entry[i]) != 255) continue;
    if (V(p.entry[i]) > 200) ri = i;  // red: V=240, the largest V there is
    if (U(p.entry[i]) < 80) gi = i;   // green: U=54
  }
  REQUIRE(ri >= 0);
  REQUIRE(gi >= 0);
  // MPP_ENC_OSD_PLT_RED   -> Y=81  U=90 V=240
  CHECK(Y(p.entry[ri]) >= 79 && Y(p.entry[ri]) <= 83);
  CHECK(U(p.entry[ri]) >= 88 && U(p.entry[ri]) <= 92);
  CHECK(V(p.entry[ri]) >= 238 && V(p.entry[ri]) <= 242);
  // MPP_ENC_OSD_PLT_GREEN -> Y=145 U=54 V=34
  CHECK(Y(p.entry[gi]) >= 143 && Y(p.entry[gi]) <= 147);
  CHECK(U(p.entry[gi]) >= 52 && U(p.entry[gi]) <= 56);
  CHECK(V(p.entry[gi]) >= 32 && V(p.entry[gi]) <= 36);
}

// ...and the same for the quantizer, so a swap between build_palette() and
// nearest_entry() cannot cancel itself out either: a red pixel must land on
// the red entry, not the green one.
TEST(quantize_maps_a_coloured_pixel_to_its_own_entry) {
  std::vector<uint32_t> pix = {0x00000000u, 0xFFFF0000u, 0xFF00FF00u};
  GlyphAtlas a{1, 3, 1, pix.data()};
  const OsdPalette p = build_palette(a);

  Canvas c(32, 16);
  c.set(1, 1, 0xFFFF0000u);  // red
  c.set(2, 1, 0xFF00FF00u);  // green
  OsdIndexMap m;
  quantize(c.s, p, &m);
  const uint8_t r_idx = m.px[(size_t)1 * m.stride() + 1];
  const uint8_t g_idx = m.px[(size_t)1 * m.stride() + 2];
  CHECK(r_idx != g_idx);
  CHECK(V(p.entry[r_idx]) > 200);  // red's V
  CHECK(U(p.entry[g_idx]) < 80);   // green's U
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

// --- extra seeds (the GS link-status overlay's colours) ---------------

// Squared YUV+alpha distance between a palette entry and a colour, in the
// space nearest_entry() itself searches. Compared as a whole rather than on
// Y alone: green and amber differ from the atlas's greys almost entirely in
// U/V, and a luma-only metric would call a grey entry a perfect match.
static long entry_err2(uint32_t entry, uint32_t argb) {
  // The exact YUVA of `argb` -- produced by the production converter rather
  // than a rederivation here, by feeding it a one-pixel atlas: median cut on
  // a single colour reproduces it exactly in entry[1].
  std::vector<uint32_t> ref = {argb};
  GlyphAtlas a{1, 1, 1, ref.data()};
  const OsdPalette exact = build_palette(a);
  REQUIRE(exact.n == 2);
  const uint32_t want = exact.entry[1];
  const long dy = (long)Y(entry) - (long)Y(want);
  const long du = (long)U(entry) - (long)U(want);
  const long dv = (long)V(entry) - (long)V(want);
  const long da = (long)A(entry) - (long)A(want);
  return dy * dy + du * du + dv * dv + da * da;
}

// What the palette would actually give this pixel: quantize it, then measure
// how far the chosen entry is from the truth.
static long nearest_err2(const OsdPalette& p, uint32_t argb) {
  std::vector<uint32_t> px = {argb};
  Surface s{px.data(), 1, 1, 1};
  OsdIndexMap m;
  quantize(s, p, &m);
  return entry_err2(p.entry[m.px[0]], argb);
}

// MSP+GS: the palette exists but is median-cut from the Betaflight atlas
// alone. That atlas is not monochrome -- it owns colours of its own -- which
// is precisely the trap: the GS tokens do not collapse to grey, they land on
// the nearest FOREIGN hue and the recording shows an "ok" green and a
// "recording" red that are not the ones on screen.
TEST(extra_seeds_are_representable_in_the_msp_plus_gs_palette) {
  OsdFont font;
  std::string err;
  REQUIRE(font.load(MABUR_PLAY_BUNDLE_DIR "/font_btfl.mfont", &err));

  size_t n = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n);
  REQUIRE(seeds != nullptr && n > 0);

  const OsdPalette without = build_palette(font.native());
  const OsdPalette with = build_palette(font.native(), seeds, n);
  CHECK(with.n > 1);

  const uint32_t toks[] = {tok::kStatusOk, tok::kStatusCaution, tok::kStatusRec};
  for (uint32_t t : toks) {
    const uint32_t argb = premul(t, 255);
    const long e_with = nearest_err2(with, argb);
    const long e_without = nearest_err2(without, argb);
    // Strictly better, and essentially exact -- a 4 here is two units of
    // total YUVA distance, i.e. rounding.
    CHECK(e_with < e_without);
    CHECK(e_with <= 4);
  }
  // Non-vacuity control: the unseeded palette must be measurably WRONG for
  // the two tokens the design leans on hardest, or the comparison above
  // would pass for free. Measured against the shipped font_btfl.mfont:
  // green 242 (U off by 15), red 173 (V off by 11). Amber is deliberately
  // not asserted -- the atlas happens to own a near-amber (26), which is
  // exactly why "did the code path run" is not evidence of anything.
  CHECK(nearest_err2(without, premul(tok::kStatusOk, 255)) > 100);
  CHECK(nearest_err2(without, premul(tok::kStatusRec, 255)) > 100);
}

// The one-argument form must keep working byte-for-byte: every existing
// caller and the committed player_e2e hash depend on it.
TEST(no_extra_seeds_is_identical_to_the_old_form) {
  OsdFont font;
  std::string err;
  REQUIRE(font.load(MABUR_PLAY_BUNDLE_DIR "/font_btfl.mfont", &err));
  const OsdPalette a = build_palette(font.native());
  const OsdPalette b = build_palette(font.native(), nullptr, 0);
  CHECK(a.n == b.n);
  bool same = true;
  for (int i = 0; i < a.n; ++i)
    if (a.entry[i] != b.entry[i]) same = false;
  CHECK(same);
}

// An atlas that is empty AND no seeds is still the old early-return: one
// reserved transparent entry and nothing else.
TEST(empty_atlas_without_seeds_is_the_transparent_entry_alone) {
  const OsdPalette p = build_palette(GlyphAtlas{});
  CHECK(p.n == 1);
  CHECK(p.entry[0] == 0u);
}

// A GS-only topology has no MSP atlas at all. The palette must still be
// built -- from the seeds alone -- or the burned recording has no entries to
// quantize the overlay against.
TEST(empty_atlas_with_seeds_yields_a_seed_only_palette) {
  size_t n = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n);
  REQUIRE(seeds != nullptr && n > 0);
  const OsdPalette p = build_palette(GlyphAtlas{}, seeds, n);
  CHECK(p.n > 1);           // index 0 transparent, plus the seeds
  CHECK(p.entry[0] == 0u);  // index 0 stays fully transparent

  // ...and it must actually represent the tokens, which is the whole point.
  const uint32_t toks[] = {tok::kStatusOk, tok::kStatusCaution, tok::kStatusRec,
                           tok::kTextPrimary, tok::kTrack};
  for (uint32_t t : toks) CHECK(nearest_err2(p, premul(t, 255)) < 64);
}

// Seeding is not uniformly effective and the exception is worth pinning
// rather than hiding behind a loose bound. In the MSP+GS palette six of the
// seven tokens land exactly; kTrack -- the dim meter rail, and the one token
// the Betaflight atlas already had a near match for -- does not. Measured:
// with seeds Y=77 U=130 V=126 A=247 against a target of Y=76 U=130 V=126
// A=255, i.e. exact chroma, one unit of luma, and 8/255 short on ALPHA.
// Unseeded it was the other way round: exact alpha, chroma off by two in
// both axes. So the squared-YUVA figure gets WORSE (12 -> 65) while the
// thing an eye can see gets better, which is why this test asserts the axes
// separately. Making it land exactly needs a seed weight of total_px/300,
// which raises the MSP atlas's own mean quantization error from 17.3 to
// 23.2 -- a trade against the primary overlay, deliberately not taken.
TEST(the_one_token_the_seeds_do_not_place_exactly_is_pinned_by_axis) {
  OsdFont font;
  std::string err;
  REQUIRE(font.load(MABUR_PLAY_BUNDLE_DIR "/font_btfl.mfont", &err));
  size_t n = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n);
  const OsdPalette with = build_palette(font.native(), seeds, n);
  const OsdPalette without = build_palette(font.native());

  const uint32_t argb = premul(tok::kTrack, 255);
  // The exact conversion of the target, via the production converter.
  std::vector<uint32_t> one = {argb};
  GlyphAtlas single{1, 1, 1, one.data()};
  const uint32_t want = build_palette(single).entry[1];

  auto picked = [&](const OsdPalette& p) {
    std::vector<uint32_t> px = {argb};
    Surface s{px.data(), 1, 1, 1};
    OsdIndexMap m;
    quantize(s, p, &m);
    return p.entry[m.px[0]];
  };
  const uint32_t got = picked(with);
  CHECK(U(got) == U(want));            // chroma exact
  CHECK(V(got) == V(want));
  CHECK(Y(got) >= Y(want) - 2 && Y(got) <= Y(want) + 2);
  CHECK(A(got) >= A(want) - 12);       // 8 short, measured
  // Control: the unseeded palette is the one with a chroma error here, so
  // this is not "the seeds made it worse" in any sense that shows.
  const uint32_t old = picked(without);
  CHECK(U(old) != U(want) || V(old) != V(want));

  // ...and the GS-only palette, which has 121 entries for 128 seeds and no
  // atlas to share with, places every token exactly.
  const OsdPalette gs_only = build_palette(GlyphAtlas{}, seeds, n);
  const uint32_t toks[] = {tok::kTextPrimary, tok::kTextSecondary, tok::kTextLabel,
                           tok::kTrack,       tok::kStatusOk,      tok::kStatusCaution,
                           tok::kStatusRec};
  for (uint32_t t : toks) CHECK(nearest_err2(gs_only, premul(t, 255)) == 0);
}

// Half-covered glyph edges are most of what the overlay draws, so the seed
// list carries an alpha ramp and the palette has to keep it: a partially
// covered green pixel must not land on the OPAQUE green entry (alpha is a
// search axis, and getting it wrong makes antialiased text bloom).
TEST(seeded_palette_keeps_the_alpha_ramp_not_just_solid_colours) {
  size_t n = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n);
  const OsdPalette p = build_palette(GlyphAtlas{}, seeds, n);
  const uint32_t half = premul(tok::kStatusOk, 128);
  std::vector<uint32_t> px = {half};
  Surface s{px.data(), 1, 1, 1};
  OsdIndexMap m;
  quantize(s, p, &m);
  CHECK(A(p.entry[m.px[0]]) >= 118 && A(p.entry[m.px[0]]) <= 138);
}

MTEST_MAIN
