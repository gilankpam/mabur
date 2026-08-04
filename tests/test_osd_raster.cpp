#include "mtest.h"
#include "osd_raster.h"
#include "mabur/msp_dp.h"
#include "scratch.h"
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
using namespace maburplay;

// A gw x gh, 1024-entry .mfont: glyph gi is fully opaque 0xFF000000|gi.
//
// One file per shape, written on first use into the build tree's scratch
// dir and unlinked when the process exits -- every TEST here wants one of
// two shapes, so writing a fresh one per TEST was 13 identical files' worth
// of churn for nothing. OsdFont mmaps the path, so a shape that has already
// been loaded keeps working from the same file.
static const std::string& write_font(int gw, int gh) {
  static std::map<std::pair<int, int>, std::unique_ptr<ScratchFile>> cache;
  std::unique_ptr<ScratchFile>& e = cache[std::make_pair(gw, gh)];
  if (e) return e->path;
  e.reset(new ScratchFile("osd_raster", ".mfont"));
  std::FILE* f = std::fopen(e->c_str(), "wb");
  REQUIRE(f != nullptr);
  uint32_t hdr[8] = {0x544E464DU, 1, (uint32_t)gw, (uint32_t)gh, 1024, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::vector<uint32_t> g((size_t)gw * gh);
  for (int gi = 0; gi < 1024; ++gi) {
    for (auto& px : g) px = 0xFF000000u | (uint32_t)gi;
    std::fwrite(g.data(), 4, g.size(), f);
  }
  std::fclose(f);
  return e->path;
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
  const std::string& fp = write_font(2, 3);
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
}

TEST(second_draw_of_the_same_screen_touches_nothing) {
  const std::string& fp = write_font(2, 3);
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
  const std::string& fp = write_font(2, 3);
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

  // Glyph -> BLANK through the incremental path (the other diff direction:
  // a cell that had a glyph and now has none must be erased, not left
  // behind). The last 'P' at (1,6) disappears; one cell redrawn, and the
  // result still matches a from-scratch full render pixel for pixel.
  feed(parser, screen, snapshot(1, 2, "HELL"));
  const int erased = raster.draw(screen, diff.s, &shadow);
  CHECK(erased == 1);
  raster.draw(screen, full.s, nullptr);
  CHECK(diff.px == full.px);
  // ...and the vacated cell really is transparent, not a stale glyph.
  const OsdLayout& l = raster.layout();
  const size_t x = (size_t)(l.origin_x + 6 * l.draw_w), y = (size_t)(l.origin_y + 1 * l.draw_h);
  CHECK(diff.px[y * 100 + x] == 0x00000000u);
}

TEST(canvas_change_forces_a_full_redraw) {
  const std::string& fp = write_font(2, 3);
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
  const std::string& fp = write_font(2, 3);
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

// diff() is the burned-DVR path's change detector: same cell comparison as
// draw(), but against ITS OWN shadow and reporting pixel rects instead of
// painting. Its first call must report the whole surface (nothing is known
// about the consumer's state yet).
TEST(diff_reports_the_whole_surface_on_the_first_call) {
  const std::string& fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;
  feed(parser, screen, snapshot(1, 2, "HELLO"));

  Canvas c(100, 54);
  ShadowGrid shadow;
  std::vector<DirtyRect> rects;
  const int cells = raster.diff(screen, c.s, &shadow, &rects);
  CHECK(cells == screen.rows() * screen.cols());
  REQUIRE(rects.size() == 1);
  CHECK(rects[0].x == 0 && rects[0].y == 0);
  CHECK(rects[0].w == 100 && rects[0].h == 54);
  // Same screen again: nothing changed, nothing reported.
  CHECK(raster.diff(screen, c.s, &shadow, &rects) == 0);
  CHECK(rects.empty());
}

TEST(diff_merges_a_run_of_changed_cells_into_one_rect) {
  const std::string& fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;
  feed(parser, screen, snapshot(1, 2, "HELLO"));

  Canvas c(100, 54);
  ShadowGrid shadow;
  std::vector<DirtyRect> rects;
  raster.draw(screen, c.s, nullptr);          // real usage: draw, then diff
  raster.diff(screen, c.s, &shadow, &rects);  // prime

  feed(parser, screen, snapshot(1, 2, "ABCDE"));  // 5 adjacent cells, all differ
  raster.draw(screen, c.s, nullptr);
  const int cells = raster.diff(screen, c.s, &shadow, &rects);
  CHECK(cells == 5);
  REQUIRE(rects.size() == 1);  // one contiguous run -> one rect
  const OsdLayout& l = raster.layout();
  CHECK(rects[0].x == l.origin_x + 2 * l.draw_w);
  CHECK(rects[0].y == l.origin_y + 1 * l.draw_h);
  CHECK(rects[0].w == 5 * l.draw_w);
  CHECK(rects[0].h == l.draw_h);
}

// The lineage bug this method exists to avoid: a cell going X -> Y -> X is
// invisible to a shadow that only sees every OTHER render, so a consumer
// refreshed on EVERY render must keep its own.
TEST(diff_sees_a_cell_that_flips_back_between_renders) {
  const std::string& fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspParser parser;
  mabur::MspScreen screen;

  Canvas c(100, 54);
  ShadowGrid every_other, every_one;
  std::vector<DirtyRect> rects;
  feed(parser, screen, snapshot(0, 0, "X"));
  raster.diff(screen, c.s, &every_other, &rects);  // prime both at "X"
  raster.diff(screen, c.s, &every_one, &rects);

  feed(parser, screen, snapshot(0, 0, "Y"));
  CHECK(raster.diff(screen, c.s, &every_one, &rects) == 1);  // per-render: sees Y

  feed(parser, screen, snapshot(0, 0, "X"));
  CHECK(raster.diff(screen, c.s, &every_one, &rects) == 1);  // ...and back to X
  // The every-other-render shadow saw X then X: it reports nothing, which is
  // right for ITS buffer and wrong for anyone updated every render.
  CHECK(raster.diff(screen, c.s, &every_other, &rects) == 0);
}

TEST(null_surface_is_a_no_op) {
  const std::string& fp = write_font(2, 3);
  OsdFont font;
  REQUIRE(font.load(fp, nullptr));
  OsdRaster raster(font, ScaleMode::kSharp);
  mabur::MspScreen screen;
  Surface none{};
  CHECK(raster.draw(screen, none, nullptr) == 0);
}

// --- Task 1: region-scoped clears -------------------------------------
// A sentinel pixel OUTSIDE the MSP grid stands in for a second layer.
// Nothing OsdRaster does may touch it.

static constexpr uint32_t kSentinel = 0xFF00FF00u;

// The 50x18 HD canvas (MspScreen's default -- snapshot() below never calls
// SET_OPTIONS) on a 320x180 surface with 4x6 glyphs: the grid is 200x108 at
// origin (60,36). (0,0) is well outside it.
TEST(draw_full_redraw_does_not_clear_outside_the_grid) {
  const std::string& fp = write_font(4, 6);
  OsdFont font;
  std::string err;
  REQUIRE(font.load(fp, &err));
  Canvas c(320, 180);
  c.s.pixels[0] = kSentinel;

  mabur::MspParser p;
  mabur::MspScreen scr;
  feed(p, scr, snapshot(1, 2, "HI"));

  OsdRaster r(font, ScaleMode::kSharp);
  ShadowGrid sh;
  r.draw(scr, c.s, &sh);  // first draw == full redraw

  CHECK(c.s.pixels[0] == kSentinel);
}

// clear() is the stale-blank path. It must erase the grid and nothing else.
TEST(clear_erases_only_the_previously_drawn_grid) {
  const std::string& fp = write_font(4, 6);
  OsdFont font;
  std::string err;
  REQUIRE(font.load(fp, &err));
  Canvas c(320, 180);

  mabur::MspParser p;
  mabur::MspScreen scr;
  feed(p, scr, snapshot(1, 2, "HI"));

  OsdRaster r(font, ScaleMode::kSharp);
  ShadowGrid sh;
  r.draw(scr, c.s, &sh);
  const OsdLayout lay = r.layout();
  REQUIRE(lay.draw_w > 0);

  c.s.pixels[0] = kSentinel;
  r.clear(c.s, &sh);

  CHECK(c.s.pixels[0] == kSentinel);
  // A pixel inside the grid is now transparent.
  const size_t inside = (size_t)(lay.origin_y + 1) * c.s.stride_px + lay.origin_x + 1;
  CHECK(c.s.pixels[inside] == 0u);
}

// clear() with a never-drawn shadow has no grid to erase and must be a no-op.
TEST(clear_with_fresh_shadow_touches_nothing) {
  const std::string& fp = write_font(4, 6);
  OsdFont font;
  std::string err;
  REQUIRE(font.load(fp, &err));
  Canvas c(320, 180);
  for (int i = 0; i < 320 * 180; ++i) c.s.pixels[i] = kSentinel;

  OsdRaster r(font, ScaleMode::kSharp);
  ShadowGrid fresh;
  r.clear(c.s, &fresh);

  int untouched = 0;
  for (int i = 0; i < 320 * 180; ++i)
    if (c.s.pixels[i] == kSentinel) ++untouched;
  CHECK(untouched == 320 * 180);
}

// A canvas change moves the grid. Stale pixels from the OLD grid must go,
// so the clear covers the union -- but still nothing beyond it.
TEST(layout_change_clears_union_of_old_and_new_grids) {
  const std::string& fp = write_font(4, 6);
  OsdFont font;
  std::string err;
  REQUIRE(font.load(fp, &err));
  Canvas c(320, 180);

  OsdRaster r(font, ScaleMode::kSharp);
  ShadowGrid sh;

  // HD 50x18 first. Explicit SET_OPTIONS(hd_option=1) even though it's the
  // default: relying on the implicit default was the F1 bug -- and it's
  // not just belt-and-suspenders here, HD has to come FIRST. Both grids
  // are centred on the same screen centre, so whichever grid is smaller in
  // both dimensions is a strict subset of the other. SD (30x16) is smaller
  // than HD (50x18) in both, so an SD -> HD transition leaves the old (SD)
  // grid entirely inside the new (HD) one -- union_rect() would pass with
  // the old-grid contribution deleted outright. HD -> SD is the shrink
  // direction: the old HD grid has pixels the new SD grid does NOT cover,
  // which is exactly the case union_rect() exists to handle.
  mabur::MspParser p1;
  mabur::MspScreen hd;
  {
    std::vector<uint8_t> s;
    std::vector<uint8_t> clr = {2};
    mabur::msp_append_message(s, 182, clr.data(), clr.size());
    std::vector<uint8_t> opt = {5, 0, 1};
    mabur::msp_append_message(s, 182, opt.data(), opt.size());
    std::vector<uint8_t> ds = {3, 0, 0, 0, 'A', 'A', 'A', 'A'};
    mabur::msp_append_message(s, 182, ds.data(), ds.size());
    std::vector<uint8_t> scr1 = {4};
    mabur::msp_append_message(s, 182, scr1.data(), scr1.size());
    feed(p1, hd, s);
  }
  r.draw(hd, c.s, &sh);
  const OsdLayout old_lay = r.layout();

  // Mark a pixel inside the OLD (HD) grid that the NEW (SD) grid does not
  // cover -- the pixel the union logic exists to reach.
  const size_t old_px = (size_t)old_lay.origin_y * c.s.stride_px + old_lay.origin_x;
  c.s.pixels[old_px] = kSentinel;
  c.s.pixels[0] = kSentinel;  // outside both

  // Now SD 30x16 (SET_OPTIONS payload {5, 0, 0}).
  mabur::MspParser p2;
  mabur::MspScreen sd;
  {
    std::vector<uint8_t> s;
    std::vector<uint8_t> clr = {2};
    mabur::msp_append_message(s, 182, clr.data(), clr.size());
    std::vector<uint8_t> opt = {5, 0, 0};
    mabur::msp_append_message(s, 182, opt.data(), opt.size());
    std::vector<uint8_t> ds = {3, 0, 0, 0, 'B', 'B'};
    mabur::msp_append_message(s, 182, ds.data(), ds.size());
    std::vector<uint8_t> scr2 = {4};
    mabur::msp_append_message(s, 182, scr2.data(), scr2.size());
    feed(p2, sd, s);
  }
  r.draw(sd, c.s, &sh);

  CHECK(c.s.pixels[old_px] != kSentinel);  // old grid was cleared
  CHECK(c.s.pixels[0] == kSentinel);       // outside both: untouched
}

MTEST_MAIN
