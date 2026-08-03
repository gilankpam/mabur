#include "mtest.h"
#include "osd_font.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace maburplay;

// Writes a .mfont with `n` glyphs of gw x gh; glyph gi is filled with
// `0xFF000000 | gi` in its top-left pixel and opaque white elsewhere.
static std::string write_font(int gw, int gh, int n, uint32_t magic = 0x544E464DU) {
  std::string path = std::string(std::tmpnam(nullptr)) + ".mfont";
  std::FILE* f = std::fopen(path.c_str(), "wb");
  uint32_t hdr[8] = {magic, 1, (uint32_t)gw, (uint32_t)gh, (uint32_t)n, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::vector<uint32_t> g((size_t)gw * gh);
  for (int gi = 0; gi < n; ++gi) {
    for (auto& px : g) px = 0xFFFFFFFFu;
    g[0] = 0xFF000000u | (uint32_t)gi;
    std::fwrite(g.data(), 4, g.size(), f);
  }
  std::fclose(f);
  return path;
}

TEST(load_reports_native_geometry) {
  const std::string p = write_font(4, 6, 1024);
  OsdFont font;
  std::string err;
  REQUIRE(font.load(p, &err));
  CHECK(font.native().glyph_w == 4);
  CHECK(font.native().glyph_h == 6);
  CHECK(font.native().n_glyphs == 1024);
  CHECK(font.native().pixels[0] == 0xFF000000u);
  std::remove(p.c_str());
}

TEST(load_rejects_bad_magic_and_truncation) {
  const std::string bad = write_font(4, 6, 4, 0xDEADBEEFu);
  OsdFont f1;
  std::string err;
  CHECK(!f1.load(bad, &err));
  CHECK(!err.empty());
  std::remove(bad.c_str());

  const std::string trunc = write_font(4, 6, 4);
  // Claim 1024 glyphs in the header but keep the 4-glyph payload.
  std::FILE* f = std::fopen(trunc.c_str(), "r+b");
  uint32_t n = 1024;
  std::fseek(f, 16, SEEK_SET);
  std::fwrite(&n, 4, 1, f);
  std::fclose(f);
  OsdFont f2;
  CHECK(!f2.load(trunc, &err));
  std::remove(trunc.c_str());

  OsdFont f3;
  CHECK(!f3.load("/nonexistent/nope.mfont", &err));
}

TEST(atlas_at_native_size_is_the_mapping_itself) {
  const std::string p = write_font(4, 6, 1024);
  OsdFont font;
  REQUIRE(font.load(p, nullptr));
  const GlyphAtlas* a = font.atlas_at(4, 6, ScaleMode::kSharp);
  REQUIRE(a != nullptr);
  CHECK(a->pixels == font.native().pixels);
  std::remove(p.c_str());
}

TEST(atlas_at_integer_multiple_replicates) {
  const std::string p = write_font(2, 3, 1024);
  OsdFont font;
  REQUIRE(font.load(p, nullptr));
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
  std::remove(p.c_str());
}

TEST(atlas_at_smaller_size_box_averages) {
  const std::string p = write_font(2, 2, 1024);
  OsdFont font;
  REQUIRE(font.load(p, nullptr));
  // Glyph 0: top-left 0xFF000000, other three 0xFFFFFFFF -> average of the
  // 2x2 box is alpha 0xFF, rgb (0+255+255+255)/4 = 191 = 0xBF.
  const GlyphAtlas* a = font.atlas_at(1, 1, ScaleMode::kSharp);
  REQUIRE(a != nullptr);
  CHECK(a->glyph_w == 1);
  CHECK(a->pixels[0] == 0xFFBFBFBFu);
  std::remove(p.c_str());
}

TEST(atlas_at_caches_one_size) {
  const std::string p = write_font(2, 3, 1024);
  OsdFont font;
  REQUIRE(font.load(p, nullptr));
  const GlyphAtlas* a = font.atlas_at(4, 6, ScaleMode::kSharp);
  const GlyphAtlas* b = font.atlas_at(4, 6, ScaleMode::kSharp);
  CHECK(a == b);
  CHECK(a->pixels == b->pixels);
  std::remove(p.c_str());
}

MTEST_MAIN
