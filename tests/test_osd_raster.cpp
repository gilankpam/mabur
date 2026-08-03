#include "mtest.h"
#include "osd_raster.h"
#include "mabur/msp_dp.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace maburplay;

// 2x2 glyphs, 1024 entries: glyph gi is fully opaque 0xFF000000|gi.
static std::string write_font(int gw, int gh) {
  std::string path = std::string(std::tmpnam(nullptr)) + ".mfont";
  std::FILE* f = std::fopen(path.c_str(), "wb");
  uint32_t hdr[8] = {0x544E464DU, 1, (uint32_t)gw, (uint32_t)gh, 1024, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::vector<uint32_t> g((size_t)gw * gh);
  for (int gi = 0; gi < 1024; ++gi) {
    for (auto& px : g) px = 0xFF000000u | (uint32_t)gi;
    std::fwrite(g.data(), 4, g.size(), f);
  }
  std::fclose(f);
  return path;
}

// One self-contained snapshot: CLEAR, DRAW_STRING(row,col,text), DRAW_SCREEN.
static std::vector<uint8_t> snapshot(int row, int col, const std::string& text) {
  std::vector<uint8_t> s;
  std::vector<uint8_t> clr = {2};
  mabur::msp_append_message(s, 182, clr.data(), clr.size());
  std::vector<uint8_t> ds = {3, (uint8_t)row, (uint8_t)col, 0};
  for (char c : text) ds.push_back((uint8_t)c);
  mabur::msp_append_message(s, 182, ds.data(), ds.size());
  std::vector<uint8_t> scr = {4};
  mabur::msp_append_message(s, 182, scr.data(), scr.size());
  return s;
}

static void feed(mabur::MspParser& p, mabur::MspScreen& s, const std::vector<uint8_t>& b) {
  for (const auto& m : p.feed(b.data(), b.size())) s.apply(m);
}

struct Canvas {
  std::vector<uint32_t> px;
  Surface s;
  Canvas(int w, int h) : px((size_t)w * h, 0xDEADBEEFu) {
    s = Surface{px.data(), w, h, w};
  }
};

TEST(draws_glyphs_at_the_layout_position) {
  const std::string fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);

  mabur::MspParser parser;
  mabur::MspScreen screen;
  feed(parser, screen, snapshot(0, 0, "A"));  // 'A' == 0x41, page 0

  Canvas c(100, 54);  // 50x18 grid, avail 2x3 -> draw 2x3 exactly
  const int drawn = raster.draw(screen, c.s, nullptr);
  CHECK(drawn == screen.rows() * screen.cols());  // no shadow -> full redraw
  const OsdLayout& l = raster.layout();
  CHECK(l.draw_w == 2);
  CHECK(l.draw_h == 3);
  // Cell (0,0) carries glyph 0x41; every pixel of it is 0xFF000041.
  CHECK(c.px[(size_t)l.origin_y * 100 + l.origin_x] == 0xFF000041u);
  CHECK(c.px[(size_t)(l.origin_y + 2) * 100 + l.origin_x + 1] == 0xFF000041u);
  // A blank neighbouring cell was cleared to transparent, not left as fill.
  CHECK(c.px[(size_t)l.origin_y * 100 + l.origin_x + 2] == 0x00000000u);
  std::remove(fp.c_str());
}

TEST(second_draw_of_the_same_screen_touches_nothing) {
  const std::string fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;
  feed(parser, screen, snapshot(1, 2, "HELLO"));

  Canvas c(100, 54);
  ShadowGrid shadow;
  const int first = raster.draw(screen, c.s, &shadow);
  CHECK(first == screen.rows() * screen.cols());
  const int second = raster.draw(screen, c.s, &shadow);
  CHECK(second == 0);
}

TEST(diff_render_matches_full_render_pixel_for_pixel) {
  const std::string fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;

  Canvas diff(100, 54), full(100, 54);
  ShadowGrid shadow;
  feed(parser, screen, snapshot(1, 2, "HELLO"));
  raster.draw(screen, diff.s, &shadow);
  raster.draw(screen, full.s, nullptr);
  CHECK(diff.px == full.px);

  // Change a few cells: the incremental path must land on the same pixels.
  feed(parser, screen, snapshot(1, 2, "HELLP"));
  const int drawn = raster.draw(screen, diff.s, &shadow);
  CHECK(drawn == 1);
  raster.draw(screen, full.s, nullptr);
  CHECK(diff.px == full.px);
}

TEST(canvas_change_forces_a_full_redraw) {
  const std::string fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;
  feed(parser, screen, snapshot(0, 0, "A"));

  Canvas c(100, 54);
  ShadowGrid shadow;
  raster.draw(screen, c.s, &shadow);

  // SET_OPTIONS with hd_option 0 switches the canvas to SD 30x16.
  std::vector<uint8_t> opt = {5, 0, 0};
  std::vector<uint8_t> msg;
  mabur::msp_append_message(msg, 182, opt.data(), opt.size());
  feed(parser, screen, msg);
  CHECK(screen.cols() == 30);
  const int drawn = raster.draw(screen, c.s, &shadow);
  CHECK(drawn == screen.rows() * screen.cols());
}

TEST(clear_blanks_the_surface_and_invalidates_the_shadow) {
  const std::string fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;
  feed(parser, screen, snapshot(0, 0, "A"));

  Canvas c(100, 54);
  ShadowGrid shadow;
  raster.draw(screen, c.s, &shadow);
  raster.clear(c.s, &shadow);
  for (uint32_t v : c.px) CHECK(v == 0u);
  CHECK(raster.draw(screen, c.s, &shadow) == screen.rows() * screen.cols());
}

TEST(null_surface_is_a_no_op) {
  const std::string fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspScreen screen;
  Surface none{};
  CHECK(raster.draw(screen, none, nullptr) == 0);
  std::remove(fp.c_str());
}

MTEST_MAIN
