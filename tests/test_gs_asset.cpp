// The shipped-asset gate.
//
// Every geometry number in test_gs_compact is measured against the
// SYNTHETIC generator (`advance = 3px/5`, `asc = px`, `desc = px/4`, PAD 4).
// Real JetBrains Mono metrics are not those numbers -- its advance is 0.6 em,
// its ascender 1.02 em, its descender 0.30 em -- so every box size, the type
// size the bar picks and the row pitch differ from what those tests proved.
// A layout that fits and is collision-free with the synthetic font is NOT
// thereby so with the real one.
//
// So this file re-runs the load-bearing layout invariants against the
// COMMITTED asset (gs/player/bundle/gs_osd.gfont), at all four supported
// resolutions. It is the only test that touches the real .gfont; everything
// else stays on the synthetic generator deliberately, so that regenerating
// the asset cannot read as a rendering regression.
#include "mtest.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "gs_compact.h"
#include "gs_draw.h"
#include "gs_font.h"
#include "gs_scaled_sizes.h"

using namespace maburplay;

namespace {

const char* kAssetPath = MABUR_PLAY_BUNDLE_DIR "/gs_osd.gfont";

std::vector<int> scaled_sizes() {
  std::vector<int> v;
  const std::string s = kScaledSizes;
  size_t i = 0;
  while (i < s.size()) {
    size_t j = s.find(',', i);
    if (j == std::string::npos) j = s.size();
    v.push_back(std::stoi(s.substr(i, j - i)));
    i = j + 1;
  }
  return v;
}

// Loaded once: the asset is ~15 MB and every test below wants the same
// mapping. GsFont is non-copyable and owns an mmap, so a function-local
// static is both the cheapest and the only well-behaved way to share it.
GsFont& asset() {
  static GsFont f;
  static bool once = [] {
    std::string err;
    if (!f.load(kAssetPath, &err))
      std::printf("FATAL: cannot load %s: %s\n", kAssetPath, err.c_str());
    return true;
  }();
  (void)once;
  return f;
}

struct FourReso { int w, h; };
constexpr FourReso kFourResolutions[] = {
    {1280, 720}, {1920, 1080}, {2560, 1440}, {3840, 2160}};

struct Canvas {
  std::vector<uint32_t> px;
  Surface s;
  Canvas(int w, int h) : px((size_t)w * h, 0u) {
    s.pixels = px.data(); s.width = w; s.height = h; s.stride_px = w;
  }
};

// Four cards: the worst case the bar sizes its type for, so the boxes here
// are the widest the rssi/snr lists ever get.
GsSnapshot four_cards() {
  GsSnapshot s;
  s.channel = 149;
  s.mcs = 5;
  s.bw = 20;
  s.air_pct = 61.0;
  s.pre_loss_pct = 2.1;
  s.post_loss_pct = 0.0;
  s.soc_temp_c = 41;
  s.cards = {GsCard{0, true, -58.0, 18.0}, GsCard{1, true, -71.0, 9.0},
             GsCard{2, true, -60.0, 15.0}, GsCard{3, true, -65.0, 12.0}};
  return s;
}

GsPlayerState player_nominal() {
  GsPlayerState p;
  p.fps = 60.0; p.jitter_ms = 3.0; p.mbps = 24.6;
  p.vid_w = 1280; p.vid_h = 720;
  p.lat_valid = true;
  p.lat_p50_e2e_ms = 21;
  p.lat_e2e_ms = 34;
  p.rec.kind = RecState::Kind::kRecording;
  p.rec.elapsed_s = 767;
  return p;
}

bool overlaps(const DirtyRect& a, const DirtyRect& b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

size_t ink_in(const Canvas& c, const DirtyRect& b) {
  size_t lit = 0;
  for (int y = b.y; y < b.y + b.h; ++y)
    for (int x = b.x; x < b.x + b.w; ++x)
      if (c.px[(size_t)y * c.s.stride_px + x]) ++lit;
  return lit;
}

}  // namespace

// The asset must bake EXACTLY the shared bake set -- no missing size (which
// would make pick()'s 15% tolerance load-bearing at some resolution) and no
// extra one (which would mean the asset and the fixture were generated from
// different lists, so the synthetic tests would no longer speak for it).
TEST(asset_bakes_exactly_the_shared_size_set) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  const std::vector<int> want = scaled_sizes();
  CHECK(f.n_sizes() == (int)want.size());
  for (int px : want) {
    const MaskAtlas* a = f.atlas(px);
    CHECK(a != nullptr);
    if (!a) std::printf("  missing atlas for %d px\n", px);
  }
  // No extras, by PIGEONHOLE -- not by trusting the loader. GsFont::load
  // validates each entry's geometry and bounds but does NOT check that the
  // px values are unique or ascending (gs_font.cpp:107-140), and atlas() is
  // a linear first-match scan, so a duplicated size would resolve happily.
  // The argument that survives that: there are exactly n_sizes entries, the
  // 30 wanted sizes are pairwise distinct, and every one of them resolves.
  // 30 distinct values found among 30 entries leaves no entry over for
  // anything else, and no room for a duplicate either -- a duplicate would
  // have to displace one of the 30, and the loop above would have caught it.
  std::printf("asset: %s, %d sizes\n", kAssetPath, f.n_sizes());
}

// Real metrics, recorded so a regenerated asset that silently changed
// weight/family is visible in the log rather than only in a pixel diff.
// Asserts only what layout() depends on: a positive advance and a baseline
// strictly inside the cell.
TEST(asset_metrics_are_sane_at_every_baked_size) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  for (int px : scaled_sizes()) {
    const MaskAtlas* a = f.atlas(px);
    REQUIRE(a != nullptr);
    CHECK(a->advance_x > 0);
    CHECK(a->glyph_w > a->advance_x);       // the 4 px pad on each side
    CHECK(a->baseline > 0 && a->baseline < a->glyph_h);
    CHECK(a->n_glyphs == 127);              // ASCII + gen_gsfont's 5 + 14 menu glyphs + 13 icons
    std::printf("  px=%3d adv=%3d cell=%3dx%-3d baseline=%3d glyphs=%d\n", a->px,
                a->advance_x, a->glyph_w, a->glyph_h, a->baseline, a->n_glyphs);
  }
}

// Every glyph the bar can draw -- the item icons, the recording dot, the
// ASCII figures -- must exist at EVERY size. A missing one renders as a
// hole (index_of returns -1 and the blitter draws nothing) that every
// string-level test passes. Walked off the bar's own worst-case strings, so
// a new icon that never reached gen_gsfont.py's BAR_ICONS fails here.
TEST(asset_carries_every_glyph_the_bar_draws_at_every_size) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::vector<uint32_t> cps;
  for (int i = 0; i < (int)GsBarField::kCount; ++i)
    for (RecTarget t : {RecTarget::kGs, RecTarget::kVtx, RecTarget::kBoth}) {
      const std::string s = GsCompactBar::worst_case((GsBarField)i, kMaxCards, t);
      const char* p = s.c_str();
      while (*p) cps.push_back(utf8_next(&p));
    }
  for (const char* extra : {"KEY?--(h)0123456789"}) {
    const char* p = extra;
    while (*p) cps.push_back(utf8_next(&p));
  }
  for (int px : scaled_sizes()) {
    const MaskAtlas* a = f.atlas(px);
    REQUIRE(a != nullptr);
    for (uint32_t cp : cps) {
      const int gi = a->index_of(cp);
      if (gi < 0) std::printf("  px=%d missing U+%04X\n", px, cp);
      CHECK(gi >= 0);
    }
  }
}

// Every glyph the settings menu draws (menu_model.cpp, menu_layer.cpp) must
// also exist at EVERY size, for the same reason the bar's icons are pinned
// above: index_of returns -1 for a missing codepoint and draw_text advances
// the pen anyway, so a hole renders as correctly-sized empty space that no
// string-level test would ever catch.
TEST(asset_carries_every_menu_glyph_at_every_size) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  const uint32_t cps[] = {
      0x00B7,  // ·  picker separator
      0x00D7,  // ×  "WxH" resolution strings (size_label(): Display, rec.burn.size)
      0x2013,  // –  number-range dash
      0x203A,  // ›  breadcrumb separator
      0x25B2,  // ▲  picker up
      0x25B6,  // ▶  forward/open/apply/confirm
      0x25B8,  // ▸  unset group value
      0x25BC,  // ▼  picker down
      0x25C0,  // ◀  back/cancel/close
      0x26A0,  // ⚠  row warning
      0x2713,  // ✓  saved/confirmed
      0x2717,  // ✗  failed
      0x2026,  // …  drawn by the "Reading GS settings…" toast
      0x2260,  // ≠  Display row's "≠ camera" warning
  };
  for (int px : scaled_sizes()) {
    const MaskAtlas* a = f.atlas(px);
    REQUIRE(a != nullptr);
    for (uint32_t cp : cps) {
      const int gi = a->index_of(cp);
      if (gi < 0) std::printf("  px=%d missing U+%04X\n", px, cp);
      CHECK(gi >= 0);
    }
  }
}

// The bar must fit with REAL metrics at every supported resolution, and the
// size it lands on is recorded so a regenerated asset that shrank the bar
// is visible in the log. The floor is the two-row split's reason to exist:
// a single line capped the type at 22 px on a 1080p panel, too small to
// read on the GS screen.
TEST(asset_layout_succeeds_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  for (const FourReso& r : kFourResolutions) {
    GsCompactBar bar(f);
    std::string err;
    const bool ok = bar.layout(r.w, r.h, &err);
    if (!ok) std::printf("  %dx%d: %s\n", r.w, r.h, err.c_str());
    CHECK(ok);
    std::printf("  %dx%d: bar %d px, REC %d px\n", r.w, r.h, bar.debug_atlas_px(),
                bar.debug_rec_atlas_px());
    CHECK(bar.debug_atlas_px() * 1080 >= 33 * r.h);  // >= 33 px at 1080p, scaled
  }
}

// The sizes the operator signed off on, pinned. The bar's size comes from
// the sizing reference row (gs_compact.h), not from its own rows, so
// shortening an item must not move these -- and a regenerated asset whose
// metrics shifted shows up as a changed size here rather than as a bar that
// quietly grew on the glass.
TEST(asset_type_size_is_the_signed_off_size_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  const struct { int w, h, px, rec; } want[] = {
      {1280, 720, 25, 13}, {1920, 1080, 37, 19}, {2560, 1440, 48, 24}, {3840, 2160, 75, 37}};
  for (const auto& w : want) {
    GsCompactBar bar(f);
    std::string err;
    REQUIRE(bar.layout(w.w, w.h, &err));
    CHECK(bar.debug_atlas_px() == w.px);
    CHECK(bar.debug_rec_atlas_px() == w.rec);
  }
}

// Real glyph cells are wider and taller than the synthetic ones, so this is
// the invariant most at risk from the asset swap: a field box grown by real
// metrics can collide with its neighbour where the synthetic box did not.
// Scoped to ACTIVE fields: an inactive field keeps whatever box it last had,
// and nothing draws or clears through it.
TEST(asset_no_two_active_field_boxes_overlap_at_any_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  for (const FourReso& r : kFourResolutions) {
    GsCompactBar bar(f);
    std::string err;
    REQUIRE(bar.layout(r.w, r.h, &err));
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    bar.update(four_cards(), false, player_nominal(), c.s, &rects);
    for (int i = 0; i < (int)GsBarField::kCount; ++i)
      for (int j = i + 1; j < (int)GsBarField::kCount; ++j) {
        if (!bar.debug_field_active((GsBarField)i) ||
            !bar.debug_field_active((GsBarField)j))
          continue;
        const DirtyRect a = bar.debug_field_box((GsBarField)i);
        const DirtyRect b = bar.debug_field_box((GsBarField)j);
        if (overlaps(a, b))
          std::printf("  %dx%d: fields %d and %d overlap\n", r.w, r.h, i, j);
        CHECK(!overlaps(a, b));
      }
  }
}

// Every box inside the surface, the two rows in the bottom half, REC in the
// top-right clear of them, and the centre of frame untouched -- against the
// real asset's box sizes.
TEST(asset_bar_stays_on_the_edges_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  for (const FourReso& r : kFourResolutions) {
    GsCompactBar bar(f);
    std::string err;
    REQUIRE(bar.layout(r.w, r.h, &err));
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    bar.update(four_cards(), false, player_nominal(), c.s, &rects);
    const DirtyRect centre{r.w / 4, r.h / 4, r.w / 2, r.h / 2};
    int row_y0 = 1 << 30;
    for (int i = 0; i < (int)GsBarField::kCount; ++i) {
      const GsBarField id = (GsBarField)i;
      if (!bar.debug_field_active(id)) continue;
      const DirtyRect b = bar.debug_field_box(id);
      CHECK(b.x >= 0 && b.y >= 0);
      CHECK(b.x + b.w <= r.w && b.y + b.h <= r.h);
      CHECK(!overlaps(b, centre));
      if (GsCompactBar::row_of(id) == GsCompactBar::kCorner) continue;
      CHECK(b.y > r.h / 2);
      row_y0 = std::min(row_y0, b.y);
    }
    const DirtyRect rec = bar.debug_field_box(GsBarField::kRec);
    CHECK(rec.y + rec.h < row_y0);
    CHECK(rec.x + rec.w > r.w / 2);
  }
}

// Real ink, not just boxes: with the real asset an update must actually put
// pixels on the surface and emit rects for them. A .gfont whose glyphs are
// all blank would satisfy every geometry assertion above.
TEST(asset_update_draws_real_ink_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  for (const FourReso& r : kFourResolutions) {
    GsCompactBar bar(f);
    REQUIRE(bar.layout(r.w, r.h, &err));
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    const int n = bar.update(four_cards(), false, player_nominal(), c.s, &rects);
    CHECK(n == (int)GsBarField::kCount);
    CHECK(!rects.empty());
    size_t lit = 0;
    for (int i = 0; i < (int)GsBarField::kCount; ++i) {
      const size_t in = ink_in(c, bar.debug_field_box((GsBarField)i));
      if (in == 0) std::printf("  %dx%d: field %d drew no ink\n", r.w, r.h, i);
      CHECK(in > 0);
      lit += in;
    }
    std::printf("  %dx%d: %d fields, %zu rects, %zu lit px\n", r.w, r.h, n, rects.size(), lit);
  }
}

// The recording dot in pixels rather than in glyph indices: draw_text
// advances the pen for a codepoint the atlas lacks, so a missing glyph is a
// correctly-sized blank that every string-level test passes. The dot is
// painted tok::kStatusRec, so its colour is what proves it drew.
TEST(asset_renders_the_recording_dot_in_its_own_colour) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  bar.update(four_cards(), false, player_nominal(), c.s, &rects);
  const DirtyRect rb = bar.debug_field_box(GsBarField::kRec);
  const uint32_t dot = premul(tok::kStatusRec, 255);
  size_t red = 0;
  for (int y = rb.y; y < rb.y + rb.h; ++y)
    for (int x = rb.x; x < rb.x + rb.w; ++x)
      if (c.px[(size_t)y * 1920 + x] == dot) ++red;
  std::printf("  REC dot: %zu px at full kStatusRec\n", red);
  CHECK(red > 0);

  // ...and armed draws nothing at all in that box.
  GsCompactBar armed(f);
  REQUIRE(armed.layout(1920, 1080, &err));
  Canvas c2(1920, 1080);
  GsPlayerState p = player_nominal();
  p.rec.kind = RecState::Kind::kArmed;
  rects.clear();
  armed.update(four_cards(), false, p, c2.s, &rects);
  CHECK(ink_in(c2, armed.debug_field_box(GsBarField::kRec)) == 0);
}

MTEST_MAIN
