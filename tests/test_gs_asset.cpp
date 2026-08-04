// The shipped-asset gate.
//
// Every geometry number in the GsOverlay tests is measured against the
// SYNTHETIC generator (`advance = 3px/5`, `asc = px`, `desc = px/4`, PAD 4).
// Real JetBrains Mono metrics are not those numbers -- its advance is 0.6 em,
// its ascender 1.02 em, its descender 0.30 em -- so every box size, every row
// pitch and every anchor differs from what those tests proved. A layout that
// is collision-free and inside the safe inset with the synthetic font is NOT
// thereby collision-free with the real one.
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

#include "gs_draw.h"
#include "gs_font.h"
#include "gs_overlay.h"
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

// Loaded once: the asset is ~13 MB and every test below wants the same
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

GsSnapshot nominal() {
  GsSnapshot s;
  s.mcs = 5;
  s.fec_pct = 25.0;
  s.air_pct = 61.0;
  s.pre_loss_pct = 2.1;
  s.post_loss_pct = 0.0;
  s.cards = {GsCard{0, true, -58.0, 18.0}, GsCard{1, true, -71.0, 9.0}};
  return s;
}

GsSnapshot four_cards() {
  GsSnapshot s = nominal();
  s.cards.push_back(GsCard{2, true, -60.0, 15.0});
  s.cards.push_back(GsCard{3, true, -65.0, 12.0});
  return s;
}

GsPlayerState player_nominal() {
  GsPlayerState p;
  p.fps = 60.0; p.jitter_ms = 3.0; p.mbps = 24.6;
  p.rec.kind = RecState::Kind::kRecording;
  p.rec.elapsed_s = 767;
  return p;
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
    CHECK(a->n_glyphs == 100);              // printable ASCII + the design's 5
    std::printf("  px=%3d adv=%3d cell=%3dx%-3d baseline=%3d glyphs=%d\n", a->px,
                a->advance_x, a->glyph_w, a->glyph_h, a->baseline, a->n_glyphs);
  }
}

// The four non-ASCII glyphs the design names must exist at EVERY size. A
// real TTF missing one would render as a hole (index_of returns -1 and the
// blitter draws nothing), and the value most likely to hit it is RSSI's
// true minus -- the single most-read number on the screen.
TEST(asset_carries_every_non_ascii_glyph_the_design_names) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  const uint32_t want[] = {0x2212 /* MINUS */, 0x2192 /* ARROW */,
                           0x25CF /* FILLED DOT */, 0x25CB /* HOLLOW DOT */,
                           0x2014 /* EM DASH, kEmDashPair */};
  for (int px : scaled_sizes()) {
    const MaskAtlas* a = f.atlas(px);
    REQUIRE(a != nullptr);
    for (uint32_t cp : want) {
      const int gi = a->index_of(cp);
      CHECK(gi >= 0);
      if (gi < 0) std::printf("  px=%d missing U+%04X\n", px, cp);
    }
  }
}

TEST(asset_layout_succeeds_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    CHECK(ov.layout(r.w, r.h, &err));
    CHECK(err.empty());
    if (!err.empty()) std::printf("  %dx%d: %s\n", r.w, r.h, err.c_str());
  }
}

// The whole point of baking 30 sizes: with the real asset, every one of
// layout()'s named roles must resolve EXACTLY, so pick()'s 15% tolerance
// stays what it is meant to be -- insurance that is never actually used.
TEST(asset_sizes_resolve_exactly_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  struct Role { int design_px; GsFieldId id; };
  const Role roles[] = {
      {56, GsFieldId::kFpsValue},   {38, GsFieldId::kCard0Rssi},
      {26, GsFieldId::kLossArrow},  {24, GsFieldId::kJit},
      {22, GsFieldId::kCard0Id},    {19, GsFieldId::kLossLabel},
  };
  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    const double scale = r.h / 1080.0;
    for (const Role& role : roles) {
      const int want = (int)(role.design_px * scale + 0.5);
      const int got = ov.debug_field_atlas_px(role.id);
      REQUIRE(got > 0);
      CHECK(got == want);
      if (got != want)
        std::printf("  %dx%d: design %d -> want %d, got %d\n", r.w, r.h, role.design_px,
                    want, got);
    }
    // kRung is not placed at 720p at all -- drop_top fires there, asserted
    // separately below. Not a resolution failure.
    if (r.h != 720) {
      const int want = (int)(34 * scale + 0.5);
      const int got = ov.debug_field_atlas_px(GsFieldId::kRung);
      REQUIRE(got > 0);
      CHECK(got == want);
    }
  }
}

// Real glyph cells are wider and taller than the synthetic ones, so this is
// the invariant most at risk from the asset swap: a field box grown by real
// metrics can collide with its neighbour where the synthetic box did not.
//
// Scoped to ACTIVE fields, deliberately. An inactive slot (past the card
// count, or dropped by drop_top) keeps whatever box it was last assigned;
// draw_field_ and repaint_intersecting are both gated on `active`, so a
// stale inactive box coinciding with an active one is inert, not a
// collision. Do not "fix" such a pair -- it is not a bug.
TEST(asset_no_two_active_field_boxes_overlap_at_any_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  auto overlaps = [](const DirtyRect& a, const DirtyRect& b) {
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
  };
  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    const GsSnapshot s = four_cards();
    REQUIRE((int)s.cards.size() == kMaxCards);
    ov.update(s, false, player_nominal(), c.s, &rects);

    std::vector<std::pair<GsFieldId, DirtyRect>> boxes;
    for (int i = 0; i < ov.field_count(); ++i) {
      const GsFieldId id = (GsFieldId)i;
      if (ov.debug_field_active(id)) boxes.emplace_back(id, ov.debug_field_box(id));
    }
    int bad_pairs = 0;
    for (size_t i = 0; i < boxes.size(); ++i)
      for (size_t j = i + 1; j < boxes.size(); ++j)
        if (overlaps(boxes[i].second, boxes[j].second)) {
          ++bad_pairs;
          const DirtyRect& a = boxes[i].second;
          const DirtyRect& b = boxes[j].second;
          std::printf("  %dx%d OVERLAP field %d (%d,%d %dx%d) vs field %d (%d,%d %dx%d)\n", r.w,
                      r.h, (int)boxes[i].first, a.x, a.y, a.w, a.h, (int)boxes[j].first, b.x,
                      b.y, b.w, b.h);
        }
    CHECK(bad_pairs == 0);
  }
}

// The 5% title-safe inset and the empty centre band, both against the real
// asset's box sizes. bounds() is the union of every field box, so a single
// oversized real-metric box pushing past the inset shows up here.
TEST(asset_safe_inset_and_centre_of_frame_hold_at_every_resolution) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    const double scale = r.h / 1080.0;
    const int inset_x = (int)(96 * scale + 0.5);
    const int inset_y = (int)(54 * scale + 0.5);
    const DirtyRect b = ov.bounds();
    CHECK(b.x >= inset_x);
    CHECK(b.y >= inset_y);
    CHECK(b.x + b.w <= r.w - inset_x);
    CHECK(b.y + b.h <= r.h - inset_y);
    std::printf("  %dx%d bounds (%d,%d %dx%d), inset %d/%d\n", r.w, r.h, b.x, b.y, b.w, b.h,
                inset_x, inset_y);

    // Centre of frame: the same fractional band as the synthetic test.
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    ov.update(four_cards(), false, player_nominal(), c.s, &rects);
    const int x0 = (int)(r.w * (400.0 / 1920.0));
    const int x1 = (int)(r.w * (1520.0 / 1920.0));
    const int y0 = (int)(r.h * (300.0 / 1080.0));
    const int y1 = (int)(r.h * (780.0 / 1080.0));
    int lit = 0;
    for (int y = y0; y < y1; ++y)
      for (int x = x0; x < x1; ++x)
        if (c.px[(size_t)y * r.w + x]) ++lit;
    CHECK(lit == 0);
    if (lit) std::printf("  %dx%d: %d lit px in the centre band\n", r.w, r.h, lit);
  }
}

// The responsive floor, against the real asset: at 720p the label role
// resolves to 13 px rendered, below the design's 18 px readability floor, so
// the two TOP blocks are dropped entirely. At 1080p and above they render.
TEST(asset_drop_top_fires_at_720p_and_not_above) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;

  GsOverlay ov720(f);
  REQUIRE(ov720.layout(1280, 720, &err));
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kRung) == 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kAirValue) == 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kRec) == 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kLossLabel) > 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kCard0Id) > 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kFpsValue) > 0);

  for (const FourReso& r : kFourResolutions) {
    if (r.h == 720) continue;
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    CHECK(ov.debug_field_atlas_px(GsFieldId::kRung) > 0);
    CHECK(ov.debug_field_atlas_px(GsFieldId::kAirValue) > 0);
    CHECK(ov.debug_field_atlas_px(GsFieldId::kRec) > 0);
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
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    const int n = ov.update(four_cards(), false, player_nominal(), c.s, &rects);
    CHECK(n > 0);
    CHECK(!rects.empty());
    size_t lit = 0;
    for (uint32_t v : c.px) if (v) ++lit;
    CHECK(lit > 0);
    std::printf("  %dx%d: %d fields, %zu rects, %zu lit px\n", r.w, r.h, n, rects.size(), lit);
  }
}

// The regression that motivated asset_carries_every_non_ascii_glyph, stated
// in pixels rather than in glyph indices: a field whose whole content is a
// non-ASCII glyph must put ink on the surface.
//
// Every existing test of these renderings asserts the STRING a field
// formats (debug_field_text), which is true whether or not the atlas can
// draw it -- draw_text advances the pen for a codepoint the atlas lacks,
// so a missing glyph is a correctly-sized blank. U+2014 really was missing
// from the generator's subset until this task, and nothing caught it.
TEST(asset_renders_ink_for_every_non_ascii_field) {
  GsFont& f = asset();
  REQUIRE(f.ok());
  std::string err;
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  std::vector<DirtyRect> rects;

  // Everything absent: kRung, kAirValue, kLossPre and kLossPost all fall to
  // the em-dash pair. kRec is armed, which is the hollow dot.
  GsSnapshot empty;
  empty.cards = {GsCard{0, false, 0.0, 0.0}};
  GsPlayerState p = player_nominal();
  p.rec.kind = RecState::Kind::kArmed;
  ov.update(empty, false, p, c.s, &rects);

  auto ink_in = [&](GsFieldId id) {
    const DirtyRect b = ov.debug_field_box(id);
    size_t lit = 0;
    for (int y = b.y; y < b.y + b.h; ++y)
      for (int x = b.x; x < b.x + b.w; ++x)
        if (c.px[(size_t)y * 1920 + x]) ++lit;
    return lit;
  };
  struct Case { GsFieldId id; const char* what; };
  const Case cases[] = {
      {GsFieldId::kRung, "MCS/FEC em-dash pair"},
      {GsFieldId::kAirValue, "AIR em-dash pair"},
      {GsFieldId::kLossPre, "pre-loss em-dash pair"},
      {GsFieldId::kLossPost, "post-loss em-dash pair"},
      {GsFieldId::kLossArrow, "U+2192 arrow"},
      {GsFieldId::kRec, "U+25CB hollow REC dot"},
  };
  for (const Case& cs : cases) {
    const size_t lit = ink_in(cs.id);
    CHECK(lit > 0);
    std::printf("  %-24s %zu lit px\n", cs.what, lit);
  }

  // And the filled dot, which only the recording state draws.
  GsOverlay ov2(f);
  REQUIRE(ov2.layout(1920, 1080, &err));
  Canvas c2(1920, 1080);
  rects.clear();
  ov2.update(nominal(), false, player_nominal(), c2.s, &rects);
  const DirtyRect rb = ov2.debug_field_box(GsFieldId::kRec);
  size_t lit = 0;
  for (int y = rb.y; y < rb.y + rb.h; ++y)
    for (int x = rb.x; x < rb.x + rb.w; ++x)
      if (c2.px[(size_t)y * 1920 + x]) ++lit;
  CHECK(lit > 0);
}

MTEST_MAIN
