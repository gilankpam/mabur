#include "mtest.h"
#include "osd_font.h"
#include "scratch.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace maburplay;

// Every font here is one TEST's own shape -- and two of them are shapes no
// loader should accept -- so each is written into a caller-owned
// ScratchFile, which unlinks it on the way out of the TEST.

// Writes a .mfont with `n` glyphs of gw x gh; glyph gi is filled with
// `0xFF000000 | gi` in its top-left pixel and opaque white elsewhere.
static void write_font(const ScratchFile& sf, int gw, int gh, int n,
                       uint32_t magic = 0x544E464DU) {
  std::FILE* f = std::fopen(sf.c_str(), "wb");
  REQUIRE(f != nullptr);
  uint32_t hdr[8] = {magic, 1, (uint32_t)gw, (uint32_t)gh, (uint32_t)n, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::vector<uint32_t> g((size_t)gw * gh);
  for (int gi = 0; gi < n; ++gi) {
    for (auto& px : g) px = 0xFFFFFFFFu;
    g[0] = 0xFF000000u | (uint32_t)gi;
    std::fwrite(g.data(), 4, g.size(), f);
  }
  std::fclose(f);
}

// Writes a .mfont with a single glyph whose gw*gh pixels are given
// explicitly (row-major), for tests that need a specific spatial pattern
// rather than write_font()'s single-corner-marker convention.
static void write_font_pixels(const ScratchFile& sf, int gw, int gh,
                              const std::vector<uint32_t>& px) {
  std::FILE* f = std::fopen(sf.c_str(), "wb");
  REQUIRE(f != nullptr);
  uint32_t hdr[8] = {0x544E464DU, 1, (uint32_t)gw, (uint32_t)gh, 1, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::fwrite(px.data(), 4, px.size(), f);
  std::fclose(f);
}

TEST(load_reports_native_geometry) {
  const ScratchFile p("osd_font", ".mfont");
  write_font(p, 4, 6, 1024);
  OsdFont font;
  std::string err;
  REQUIRE(font.load(p.path, &err));
  CHECK(font.native().glyph_w == 4);
  CHECK(font.native().glyph_h == 6);
  CHECK(font.native().n_glyphs == 1024);
  CHECK(font.native().pixels[0] == 0xFF000000u);
}

TEST(load_rejects_bad_magic_and_truncation) {
  const ScratchFile bad("osd_font", ".mfont");
  write_font(bad, 4, 6, 4, 0xDEADBEEFu);
  OsdFont f1;
  std::string err;
  CHECK(!f1.load(bad.path, &err));
  CHECK(!err.empty());

  const ScratchFile trunc("osd_font", ".mfont");
  write_font(trunc, 4, 6, 4);
  // Claim 1024 glyphs in the header but keep the 4-glyph payload.
  std::FILE* f = std::fopen(trunc.c_str(), "r+b");
  uint32_t n = 1024;
  std::fseek(f, 16, SEEK_SET);
  std::fwrite(&n, 4, 1, f);
  std::fclose(f);
  OsdFont f2;
  CHECK(!f2.load(trunc.path, &err));

  OsdFont f3;
  CHECK(!f3.load("/nonexistent/nope.mfont", &err));
}

TEST(load_rejects_overflowing_header) {
  // glyph_w = glyph_h = n_glyphs = 2^22: the naive 64-bit product
  // (glyph_w * glyph_h * n_glyphs * 4) wraps around to a tiny value, so a
  // loader that multiplies before range-checking would treat this
  // header-only file as a valid, absurdly-dimensioned font. It must be
  // rejected on the field bounds instead.
  const ScratchFile path("osd_font", ".mfont");
  std::FILE* f = std::fopen(path.c_str(), "wb");
  REQUIRE(f != nullptr);
  uint32_t hdr[8] = {0x544E464DU, 1, 4194304u, 4194304u, 4194304u, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::fclose(f);

  OsdFont font;
  std::string err;
  CHECK(!font.load(path.path, &err));
  CHECK(!err.empty());
}

TEST(atlas_at_rejects_oversized_request) {
  const ScratchFile p("osd_font", ".mfont");
  write_font(p, 4, 6, 4);
  OsdFont font;
  REQUIRE(font.load(p.path, nullptr));
  CHECK(font.atlas_at(5000, 6, ScaleMode::kSharp) == nullptr);
  CHECK(font.atlas_at(4, 5000, ScaleMode::kSharp) == nullptr);
}

TEST(atlas_at_native_size_is_the_mapping_itself) {
  const ScratchFile p("osd_font", ".mfont");
  write_font(p, 4, 6, 1024);
  OsdFont font;
  REQUIRE(font.load(p.path, nullptr));
  const GlyphAtlas* a = font.atlas_at(4, 6, ScaleMode::kSharp);
  REQUIRE(a != nullptr);
  CHECK(a->pixels == font.native().pixels);
}

TEST(atlas_at_integer_multiple_replicates) {
  const ScratchFile p("osd_font", ".mfont");
  write_font(p, 2, 3, 1024);
  OsdFont font;
  REQUIRE(font.load(p.path, nullptr));
  const GlyphAtlas* a = font.atlas_at(4, 6, ScaleMode::kSharp);
  REQUIRE(a != nullptr);
  CHECK(a->glyph_w == 4);
  CHECK(a->glyph_h == 6);
  CHECK(a->n_glyphs == 1024);
  // Glyph 7's top-left source pixel (0xFF000007) covers a 2x2 block.
  const uint32_t* g = a->pixels + (size_t)7 * 4 * 6;
  CHECK(g[0] == 0xFF000007u);
  CHECK(g[1] == 0xFF000007u);
  CHECK(g[4] == 0xFF000007u);
  CHECK(g[5] == 0xFF000007u);
  CHECK(g[2] == 0xFFFFFFFFu);
}

TEST(atlas_at_smaller_size_box_averages) {
  const ScratchFile p("osd_font", ".mfont");
  write_font(p, 2, 2, 1024);
  OsdFont font;
  REQUIRE(font.load(p.path, nullptr));
  // Glyph 0: top-left 0xFF000000, other three 0xFFFFFFFF -> average of the
  // 2x2 box is alpha 0xFF, rgb (0+255+255+255)/4 = 191 = 0xBF.
  const GlyphAtlas* a = font.atlas_at(1, 1, ScaleMode::kSharp);
  REQUIRE(a != nullptr);
  CHECK(a->glyph_w == 1);
  CHECK(a->pixels[0] == 0xFFBFBFBFu);
}

TEST(atlas_at_box_downscale_averages_each_quadrant) {
  // 4x4 glyph split into four constant-valued 2x2 quadrants. A correct box
  // downscale to 2x2 must average strictly within each quadrant (the
  // average of a constant region is itself), so each output pixel should
  // equal its own quadrant's value exactly. An off-by-one in the box()
  // sub-region bounds would blend across the midline and this would fail,
  // unlike the degenerate 2x2->1x1 case above where the whole source is
  // always the single region regardless of the boundary math.
  const uint32_t tl = 0xFF100000u, tr = 0xFF200000u, bl = 0xFF300000u, br = 0xFF400000u;
  const std::vector<uint32_t> px = {
      tl, tl, tr, tr,
      tl, tl, tr, tr,
      bl, bl, br, br,
      bl, bl, br, br,
  };
  const ScratchFile p("osd_font", ".mfont");
  write_font_pixels(p, 4, 4, px);
  OsdFont font;
  REQUIRE(font.load(p.path, nullptr));
  const GlyphAtlas* a = font.atlas_at(2, 2, ScaleMode::kSharp);
  REQUIRE(a != nullptr);
  CHECK(a->pixels[0] == tl);
  CHECK(a->pixels[1] == tr);
  CHECK(a->pixels[2] == bl);
  CHECK(a->pixels[3] == br);
}

TEST(atlas_at_caches_one_size) {
  const ScratchFile p("osd_font", ".mfont");
  write_font(p, 2, 3, 1024);
  OsdFont font;
  REQUIRE(font.load(p.path, nullptr));
  const GlyphAtlas* a = font.atlas_at(4, 6, ScaleMode::kSharp);
  CHECK(font.builds() == 1);
  const GlyphAtlas* b = font.atlas_at(4, 6, ScaleMode::kSharp);
  CHECK(font.builds() == 1);
  CHECK(a == b);
  CHECK(a->pixels == b->pixels);
  // A different requested size must evict the cache and build again.
  const GlyphAtlas* c = font.atlas_at(2, 2, ScaleMode::kSharp);
  REQUIRE(c != nullptr);
  CHECK(font.builds() == 2);
}

MTEST_MAIN
