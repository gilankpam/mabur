#include "mtest.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gs_draw.h"
#include "gs_font.h"
#include "gs_layer.h"
#include "menu_layer.h"
#include "menu_model.h"

using namespace maburplay;

// The settings menu painted into the OSD surface, against the synthetic
// scaled .gfont fixture (the same one test_gs_compact uses).

namespace {

struct Canvas {
  std::vector<uint32_t> px;
  Surface s;
  Canvas(int w, int h) : px((size_t)w * h, 0u) {
    s.pixels = px.data(); s.width = w; s.height = h; s.stride_px = w;
  }
  uint32_t at(int x, int y) const { return px[(size_t)y * s.stride_px + x]; }
  bool all_zero() const {
    for (uint32_t p : px)
      if (p) return false;
    return true;
  }
  bool any_in(const DirtyRect& r) const {
    for (int y = r.y; y < r.y + r.h; ++y)
      for (int x = r.x; x < r.x + r.w; ++x)
        if (at(x, y)) return true;
    return false;
  }
};

GsFont& font() {
  static GsFont f;
  static bool loaded = false;
  if (!loaded) {
    std::string err;
    REQUIRE(f.load(GSFONT_SCALED, &err));
    loaded = true;
  }
  return f;
}

MenuView sample() {
  MenuView v;
  v.open = true;
  v.crumb = "Video";
  v.linked = true;
  ViewRow a;
  a.id = "video.mode"; a.label = "Camera mode"; a.value = "60 fps"; a.vtx = true; a.selected = true;
  ViewRow b;
  b.id = "video.slices"; b.label = "Slices"; b.value = "On"; b.adv = true;
  ViewRow c;
  c.id = "video.colour"; c.label = "Colour"; c.value = "Vivid"; c.warn = "judder";
  v.rows = {a, b, c};
  v.help = "Pick the camera mode.";
  v.keys = "▶ change · ◀ back";
  return v;
}

bool intersects(const DirtyRect& a, const DirtyRect& b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

bool inside(const DirtyRect& in, const DirtyRect& out) {
  return in.x >= out.x && in.y >= out.y && in.x + in.w <= out.x + out.w &&
         in.y + in.h <= out.y + out.h;
}

}  // namespace

TEST(a_closed_view_paints_nothing_and_has_empty_bounds) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  m.set_view(MenuView{});
  std::vector<DirtyRect> out;
  CHECK(m.paint(c.s, &out) == 0);
  CHECK(out.empty());
  CHECK(c.all_zero());
  const DirtyRect b = m.bounds();
  CHECK(b.w == 0 || b.h == 0);
  CHECK(m.debug_texts().empty());
}

TEST(an_open_view_draws_crumb_rows_help_and_keys_in_order) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  m.set_view(sample());
  CHECK(m.dirty());
  std::vector<DirtyRect> out;
  CHECK(m.paint(c.s, &out) > 0);
  CHECK(!m.dirty());
  const std::vector<std::string> want = {
      "Video", "● drone linked",
      "Camera mode", "VTX", "60 fps",
      "Slices", "ADV", "On",
      "Colour", "⚠ judder", "Vivid",
      "Pick the camera mode.",
      "▶ change · ◀ back"};
  CHECK(m.debug_texts() == want);

  MenuView off = sample();
  off.linked = false;
  m.set_view(off);
  m.paint(c.s, &out);
  CHECK(m.debug_texts()[1] == "○ no drone");
}

TEST(the_panel_is_45_percent_wide_and_inset) {
  for (auto [w, h] : {std::pair{1920, 1080}, std::pair{1280, 720}, std::pair{3840, 2160}}) {
    Canvas c(w, h);
    MenuLayer m(font());
    std::string err;
    REQUIRE(m.layout(w, h, &err));
    m.set_view(sample());
    std::vector<DirtyRect> out;
    m.paint(c.s, &out);
    const DirtyRect p = m.debug_panel_box();
    const int inset = (int)std::lround(24.0 * h / 1080);
    CHECK(p.x == inset);
    CHECK(p.y == inset);
    CHECK(p.w == (int)std::lround(0.45 * w));
    CHECK(p.h > 0 && p.y + p.h < h);
    // The fill is premultiplied like every other OSD pixel: the bottom-right
    // corner of the panel sits right of the keys text, on bare panel.
    CHECK(c.at(p.x + p.w - 1, p.y + p.h - 1) == premul(tok::kMenuPanel, 232));
    CHECK(out.size() == 1);
    CHECK(inside(p, m.bounds()));
  }
}

TEST(a_picker_draws_to_the_right_of_the_panel_with_no_title) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  MenuView v = sample();
  m.set_view(v);
  m.paint(c.s, nullptr);
  const std::vector<std::string> panel_texts = m.debug_texts();
  CHECK(m.debug_picker_box().w == 0);

  v.picker = MenuView::Picker::kChoices;
  v.choices = {{"60", "60 fps", "", true}, {"90", "90 fps", "judder", true}};
  v.picker_cursor = 1;
  v.picker_current = 0;
  m.set_view(v);
  m.paint(c.s, nullptr);
  const std::vector<std::string> all = m.debug_texts();
  REQUIRE(all.size() > panel_texts.size());
  const std::vector<std::string> picker(all.begin() + (long)panel_texts.size(), all.end());
  CHECK(picker.front() == "60 fps ·");
  CHECK((picker == std::vector<std::string>{"60 fps ·", "90 fps", "judder"}));

  const DirtyRect p = m.debug_panel_box();
  const DirtyRect k = m.debug_picker_box();
  CHECK(k.x == p.x + p.w + 12);
  CHECK(k.y == p.y);
  CHECK(k.w > 0 && k.h > 0);
  CHECK(c.any_in(k));
  CHECK(inside(k, m.bounds()));

  // A number picker: arrows, the value, the range.
  v.picker = MenuView::Picker::kNumber;
  v.choices.clear();
  v.number_text = "24000";
  v.number_range = "4000 – 40000";
  m.set_view(v);
  m.paint(c.s, nullptr);
  const std::vector<std::string> num = m.debug_texts();
  CHECK((std::vector<std::string>(num.begin() + (long)panel_texts.size(), num.end()) ==
         std::vector<std::string>{"▲", "24000", "▼", "4000 – 40000"}));

  // A confirm: the sentence and the two answers, no title.
  v.picker = MenuView::Picker::kConfirm;
  v.confirm_text = "Restart the encoder?";
  m.set_view(v);
  m.paint(c.s, nullptr);
  const std::vector<std::string> conf = m.debug_texts();
  CHECK((std::vector<std::string>(conf.begin() + (long)panel_texts.size(), conf.end()) ==
         std::vector<std::string>{"Restart the encoder?", "▶ Go ahead", "◀ Cancel"}));
}

TEST(a_changed_row_dirties_only_its_rect) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  MenuView v = sample();
  m.set_view(v);
  m.paint(c.s, nullptr);
  const DirtyRect p = m.debug_panel_box();

  v.rows[1].value = "Off";
  m.set_view(v);
  std::vector<DirtyRect> out;
  CHECK(m.paint(c.s, &out) == 1);
  REQUIRE(out.size() == 1);
  CHECK(inside(out[0], p));
  CHECK(out[0].w == p.w);
  CHECK(out[0].h * 5 < p.h);

  // Nothing changed: nothing drawn.
  m.set_view(v);
  out.clear();
  CHECK(!m.dirty());
  CHECK(m.paint(c.s, &out) == 0);
  CHECK(out.empty());

  // invalidate() forces the whole panel.
  m.invalidate();
  CHECK(m.dirty());
  CHECK(m.paint(c.s, &out) >= 1);
}

TEST(closing_erases_every_box_it_drew) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  MenuView v = sample();
  v.picker = MenuView::Picker::kChoices;
  v.choices = {{"60", "60 fps", "", true}, {"90", "90 fps", "", false}};
  v.picker_cursor = 0;
  v.picker_current = 0;
  v.toast = "Saved";
  m.set_view(v);
  m.paint(c.s, nullptr);
  const DirtyRect b = m.bounds();
  CHECK(c.any_in(b));

  m.set_view(MenuView{});
  std::vector<DirtyRect> out;
  CHECK(m.paint(c.s, &out) > 0);
  CHECK(c.all_zero());
  for (const DirtyRect& r : out) CHECK(inside(r, b));
  CHECK(m.debug_texts().empty());
}

TEST(a_busy_row_animates_on_tick) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  MenuView v = sample();
  v.rows[1].busy = true;
  v.rows[1].status = "applying";
  m.set_tick(1000);
  m.set_view(v);
  m.paint(c.s, nullptr);
  // 1000 ms is the fourth 300 ms step: three dots.
  const auto t0 = m.debug_texts();
  CHECK(t0[8] == "applying...");
  const std::vector<uint32_t> before = c.px;

  m.set_tick(1400);
  CHECK(m.dirty());
  std::vector<DirtyRect> out;
  CHECK(m.paint(c.s, &out) > 0);
  CHECK(m.debug_texts()[8] == "applying");
  // The sweep bar: a 2 px strip along the busy row's bottom edge whose
  // pixels moved.
  bool strip_moved = false;
  for (const DirtyRect& r : out) {
    if (r.h != 2) continue;
    for (int y = r.y; y < r.y + r.h; ++y)
      for (int x = r.x; x < r.x + r.w; ++x)
        if (c.px[(size_t)y * 1920 + x] != before[(size_t)y * 1920 + x]) strip_moved = true;
  }
  CHECK(strip_moved);

  // A tick inside the same sweep step and dot step changes nothing.
  m.set_tick(1401);
  CHECK(!m.dirty());

  // An idle view ignores ticks.
  MenuView idle = sample();
  m.set_view(idle);
  m.paint(c.s, nullptr);
  m.set_tick(5000);
  CHECK(!m.dirty());
}

TEST(a_menu_view_with_no_surface_is_a_no_op) {
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  m.set_view(sample());
  std::vector<DirtyRect> out;
  CHECK(m.paint(Surface{}, &out) == 0);
  CHECK(out.empty());
  DirtyRect r{0, 0, 1920, 1080};
  CHECK(m.repaint_intersecting(&r, 1, Surface{}, &out) == 0);
  CHECK(out.empty());
  // Still owed: the next real surface gets the whole view.
  CHECK(m.dirty());
  Canvas c(1920, 1080);
  CHECK(m.paint(c.s, &out) > 0);
  CHECK(!c.all_zero());

  // Before layout() there is nothing to draw either.
  MenuLayer fresh(font());
  fresh.set_view(sample());
  CHECK(fresh.paint(c.s, &out) == 0);
}

TEST(banner_and_toast_draw_with_the_menu_closed) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  MenuView v;
  v.banner = "⚠ Camera 90 fps: this display has no 90 Hz mode";
  v.toast = "Hold ▶ to open the menu";
  m.set_view(v);
  std::vector<DirtyRect> out;
  CHECK(m.paint(c.s, &out) == 2);
  CHECK((m.debug_texts() == std::vector<std::string>{v.toast, v.banner}));
  REQUIRE(out.size() == 2);
  const DirtyRect toast = out[0], banner = out[1];
  CHECK(c.any_in(toast));
  CHECK(c.any_in(banner));
  // Both centred; the banner at 12 % down, the toast above the GS bar.
  CHECK(std::abs((toast.x + toast.w / 2) - 960) <= 1);
  CHECK(std::abs((banner.x + banner.w / 2) - 960) <= 1);
  CHECK(banner.y == (int)std::lround(0.12 * 1080));
  CHECK(toast.y + toast.h == 1080 - 90);
  CHECK(m.debug_panel_box().w == 0);

  // The toast expires: only its box is erased, the banner stays.
  v.toast.clear();
  m.set_view(v);
  out.clear();
  CHECK(m.paint(c.s, &out) == 1);
  CHECK(!c.any_in(toast));
  CHECK(c.any_in(banner));
}

TEST(an_open_menu_moves_the_banner_off_the_panel_and_picker) {
  Canvas c(1920, 1080);
  MenuLayer m(font());
  std::string err;
  REQUIRE(m.layout(1920, 1080, &err));
  const std::string banner =
      "⚠ Camera 90 fps — this display has no 90 Hz mode. Showing at 60 Hz (judder).";
  auto banner_box = [](const std::vector<DirtyRect>& out) { return out.back(); };

  // Closed: at 12 % down.
  MenuView closed;
  closed.banner = banner;
  m.set_view(closed);
  std::vector<DirtyRect> out;
  REQUIRE(m.paint(c.s, &out) == 1);
  const DirtyRect at_top = banner_box(out);
  CHECK(at_top.y == (int)std::lround(0.12 * 1080));

  // Open with a picker up: below both, clear of each.
  MenuView open = sample();
  open.banner = banner;
  open.picker = MenuView::Picker::kChoices;
  open.choices = {{"60", "60 fps", "", true}, {"90", "90 fps", "judder", true}};
  open.picker_cursor = 0;
  open.picker_current = 1;
  m.set_view(open);
  out.clear();
  m.paint(c.s, &out);
  const DirtyRect p = m.debug_panel_box(), k = m.debug_picker_box();
  const DirtyRect b = banner_box(out);
  CHECK(b.w > 0 && b.h > 0);
  CHECK(!intersects(b, p));
  CHECK(!intersects(b, k));
  CHECK(std::abs((b.x + b.w / 2) - 960) <= 1);
  CHECK(b.y + b.h <= 1080 - 90);
  // The old banner box was erased where nothing else now draws: its top
  // rows sit inside neither the panel, the picker nor the new banner.
  bool erased = false;
  for (const DirtyRect& r : out) erased |= r.x == at_top.x && r.y == at_top.y &&
                                           r.w == at_top.w && r.h == at_top.h;
  CHECK(erased);
  const int probe_x = std::max(k.x + k.w, at_top.x) + 1;
  if (probe_x < at_top.x + at_top.w) CHECK(c.at(probe_x, at_top.y) == 0u);

  // Closed again: back at 12 %, the open-menu position erased.
  m.set_view(closed);
  out.clear();
  m.paint(c.s, &out);
  CHECK(banner_box(out).y == at_top.y);
  CHECK(!intersects(b, at_top));
  CHECK(!c.any_in(b));
  const int below = at_top.y + at_top.h;
  CHECK(!c.any_in(DirtyRect{p.x, below, p.w, p.y + p.h - below}));
}

TEST(clean_air_bars_follow_the_bands) {
  CHECK(MenuLayer::clean_air_bars(0.0) == 5);
  CHECK(MenuLayer::clean_air_bars(4.9) == 5);
  CHECK(MenuLayer::clean_air_bars(5.0) == 4);
  CHECK(MenuLayer::clean_air_bars(11.9) == 4);
  CHECK(MenuLayer::clean_air_bars(12.0) == 3);
  CHECK(MenuLayer::clean_air_bars(24.9) == 3);
  CHECK(MenuLayer::clean_air_bars(25.0) == 2);
  CHECK(MenuLayer::clean_air_bars(49.9) == 2);
  CHECK(MenuLayer::clean_air_bars(50.0) == 1);
  CHECK(MenuLayer::clean_air_bars(100.0) == 1);
}

MTEST_MAIN
