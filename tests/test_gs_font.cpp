#include "mtest.h"
#include "gs_font.h"
#include "scratch.h"
#include <cstdio>
#include <cstdlib>
#include <string>
using namespace maburplay;

// GSFONT_8_12_16 / GSFONT_12 / GSFONT_20 are absolute paths to synthetic
// .gfont fixtures the build generated once (see tests/CMakeLists.txt).
// Read-only: they are shared with the other .gfont tests, so nothing here
// may remove or rewrite one.

TEST(load_exposes_every_size_in_the_directory) {
  const std::string p = GSFONT_8_12_16;
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  CHECK(f.ok());
  CHECK(f.n_sizes() == 3);
  CHECK(f.atlas(8) != nullptr);
  CHECK(f.atlas(12) != nullptr);
  CHECK(f.atlas(16) != nullptr);
  CHECK(f.atlas(10) == nullptr);  // absent size: nullptr, never a fallback

  // Exercise a real lookup + glyph fetch on a NON-first atlas. The first
  // atlas's codepoint block always lands 4-byte aligned (it starts right
  // after the header + directory), which would hide a misaligned-read bug
  // that only shows up once an odd-sized earlier glyph block pushes a later
  // one off that boundary -- exactly what px=16 does in this fixture.
  const MaskAtlas* a16 = f.atlas(16);
  REQUIRE(a16 != nullptr);
  const int gi = a16->index_of('0');
  CHECK(gi >= 0);
  const uint8_t* g = a16->glyph(gi);
  REQUIRE(g != nullptr);
  int cov = 0;
  for (int i = 0; i < a16->glyph_w * a16->glyph_h; ++i) cov += g[i * 2];
  CHECK(cov > 0);

}

TEST(atlas_geometry_matches_the_generator) {
  const std::string p = GSFONT_20;
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(20);
  REQUIRE(a != nullptr);
  CHECK(a->px == 20);
  CHECK(a->advance_x == 12);            // max(2, 20*3/5)
  CHECK(a->glyph_w == 12 + 8);          // advance + 2*PAD
  CHECK(a->glyph_h == 20 + 5 + 8);      // ascender + descender + 2*PAD
  CHECK(a->baseline == 4 + 20);         // PAD + ascender
  CHECK(a->glyph_w > a->advance_x);
}

TEST(index_of_finds_ascii_and_the_four_extras) {
  const std::string p = GSFONT_12;
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(12);
  REQUIRE(a != nullptr);
  CHECK(a->index_of(' ') >= 0);
  CHECK(a->index_of('0') >= 0);
  CHECK(a->index_of('Z') >= 0);
  CHECK(a->index_of('%') >= 0);
  CHECK(a->index_of(0x2014) >= 0);  // EM DASH (kEmDashPair)
  CHECK(a->index_of(0x2212) >= 0);  // MINUS SIGN
  CHECK(a->index_of(0x2192) >= 0);  // RIGHTWARDS ARROW
  CHECK(a->index_of(0x25CF) >= 0);  // BLACK CIRCLE
  CHECK(a->index_of(0x25CB) >= 0);  // WHITE CIRCLE
  CHECK(a->index_of(0x4E00) < 0);   // absent: negative, never index 0
}

TEST(glyph_pixels_carry_coverage_and_shadow) {
  const std::string p = GSFONT_12;
  GsFont f;
  std::string err;
  REQUIRE(f.load(p, &err));
  const MaskAtlas* a = f.atlas(12);
  REQUIRE(a != nullptr);
  const int gi = a->index_of('0');
  REQUIRE(gi >= 0);
  const uint8_t* g = a->glyph(gi);
  REQUIRE(g != nullptr);
  int cov = 0, sha = 0;
  for (int i = 0; i < a->glyph_w * a->glyph_h; ++i) {
    cov += g[i * 2];
    sha += g[i * 2 + 1];
  }
  CHECK(cov > 0);
  CHECK(sha > 0);  // the shadow channel was baked, not left zero
}

// Failure is a reason, never a crash: the overlay disables itself on any
// of these and the rest of the player runs on.
TEST(bad_files_fail_with_a_reason) {
  GsFont f;
  std::string err;
  CHECK(!f.load("/nonexistent/nope.gfont", &err));
  CHECK(!err.empty());
  CHECK(!f.ok());

  const ScratchFile junk_file("gs_font", ".gfont");
  std::FILE* fp = std::fopen(junk_file.c_str(), "wb");
  REQUIRE(fp != nullptr);
  const char junk[64] = {0};
  std::fwrite(junk, 1, sizeof(junk), fp);
  std::fclose(fp);
  GsFont f2;
  err.clear();
  CHECK(!f2.load(junk_file.path, &err));
  CHECK(!err.empty());
}

// A truncated file must be rejected at load, not read past at draw time.
TEST(truncated_file_is_rejected) {
  const std::string p = GSFONT_12;
  std::FILE* fp = std::fopen(p.c_str(), "rb");
  REQUIRE(fp != nullptr);
  std::fseek(fp, 0, SEEK_END);
  const long n = std::ftell(fp);
  std::fseek(fp, 0, SEEK_SET);
  std::string buf((size_t)n, '\0');
  REQUIRE(std::fread(&buf[0], 1, (size_t)n, fp) == (size_t)n);
  std::fclose(fp);

  const ScratchFile half("gs_font", ".gfont");
  std::FILE* tf = std::fopen(half.c_str(), "wb");
  REQUIRE(tf != nullptr);
  std::fwrite(buf.data(), 1, (size_t)n / 2, tf);  // half the file
  std::fclose(tf);

  GsFont f;
  std::string err;
  CHECK(!f.load(half.path, &err));
  CHECK(!err.empty());
}

MTEST_MAIN
