#include "mtest.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "gs_compact.h"
#include "gs_draw.h"
#include "gs_font.h"
#include "gs_layer.h"

using namespace maburplay;

// The compact bar (osd.gs.style = "compact"): one plain-text line along the
// bottom edge. Everything here runs against the SYNTHETIC .gfont fixtures
// (GSFONT_SCALED, the full 30-size bake set), same as test_gs_overlay --
// the real asset is re-proved exactly once, in test_gs_asset.

namespace {

struct Canvas {
  std::vector<uint32_t> px;
  Surface s;
  Canvas(int w, int h) : px((size_t)w * h, 0u) {
    s.pixels = px.data(); s.width = w; s.height = h; s.stride_px = w;
  }
};

GsSnapshot nominal() {
  GsSnapshot s;
  s.channel = 149;
  s.mcs = 5;
  s.air_pct = 62.0;
  s.pre_loss_pct = 0.3;
  s.post_loss_pct = 0.0;
  GsCard a; a.id = 0; a.heard = true; a.rssi_dbm = -70.0; a.snr_db = 22.0;
  GsCard b; b.id = 1; b.heard = true; b.rssi_dbm = -72.0; b.snr_db = 20.0;
  s.cards = {a, b};
  return s;
}

GsPlayerState player_nominal() {
  GsPlayerState p;
  p.fps = 60.0;
  p.jitter_ms = 5.2;
  p.mbps = 8.1;
  p.vid_w = 1280;
  p.vid_h = 720;
  p.lat_valid = true;
  p.lat_p50_e2e_ms = 45;
  p.lat_e2e_ms = 78;
  return p;
}

// The line as one string, in draw order -- what the pilot actually reads.
std::string line_of(const GsCompactBar& bar, const GsSnapshot& snap, bool stale,
                    const GsPlayerState& ps) {
  std::string out;
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    if (i) out += " ";
    out += bar.debug_field_text(snap, stale, ps, (GsBarField)i);
  }
  return out;
}

bool overlaps(const DirtyRect& a, const DirtyRect& b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

struct Reso { int w, h; };
constexpr Reso kFourResolutions[] = {
    {1280, 720}, {1920, 1080}, {2560, 1440}, {3840, 2160}};

}  // namespace

// --- the line ---------------------------------------------------------

// The format the operator asked for, field by field. Pinned as one string
// because the ORDER is as much a part of the request as the labels.
TEST(the_line_reads_exactly_as_specified) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  CHECK(line_of(bar, nominal(), false, player_nominal()) ==
        "ch:149 mcs:5 air:62% rssi:-70/-72 snr:22/20 bitrate:8.1 "
        "res:1280x720 fps:60 jit:5.2 lat:45/78 loss:0.3/0.0");
}

// A card count of four widens exactly two items and nothing else.
TEST(four_cards_extend_the_rssi_and_snr_lists) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  GsSnapshot s = nominal();
  GsCard c; c.id = 2; c.heard = true; c.rssi_dbm = -68.0; c.snr_db = 19.0;
  GsCard d; d.id = 3; d.heard = false;  // present but silent
  s.cards = {s.cards[0], s.cards[1], c, d};
  const GsPlayerState ps = player_nominal();
  CHECK(bar.debug_field_text(s, false, ps, GsBarField::kRssi) ==
        "rssi:-70/-72/-68/--");
  CHECK(bar.debug_field_text(s, false, ps, GsBarField::kSnr) ==
        "snr:22/20/19/--");
}

// Nothing received is "--", never a fabricated zero. This is the whole
// reason GsSnapshot holds optionals rather than sentinels.
TEST(missing_values_render_as_double_dash_never_zero) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  const GsSnapshot empty;          // nothing ever received, no cards
  const GsPlayerState cold;        // nothing ever decoded
  CHECK(line_of(bar, empty, false, cold) ==
        "ch:-- mcs:-- air:-- rssi:-- snr:-- bitrate:0.0 res:-- fps:0 "
        "jit:0.0 lat:--/-- loss:--/--");
}

// A heard card with no SNR yet still shows its RSSI: the two are separate
// optionals on the wire and must stay separate on the glass.
TEST(a_heard_card_missing_one_figure_keeps_the_other) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  GsSnapshot s;
  GsCard a; a.id = 0; a.heard = true; a.rssi_dbm = -64.0;  // no snr_db
  s.cards = {a};
  const GsPlayerState ps;
  CHECK(bar.debug_field_text(s, false, ps, GsBarField::kRssi) == "rssi:-64");
  CHECK(bar.debug_field_text(s, false, ps, GsBarField::kSnr) == "snr:--");
}

// lat_valid false is the anchor being cold, not a latency of zero.
TEST(invalid_latency_renders_dashes_not_zero) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  GsPlayerState ps = player_nominal();
  ps.lat_valid = false;
  CHECK(bar.debug_field_text(nominal(), false, ps, GsBarField::kLat) ==
        "lat:--/--");
}

// --- staleness --------------------------------------------------------

// The bar's ONLY colour rule, and the reason it exists: a quiet sideport
// must not leave a line of plausible frozen numbers reading as live.
TEST(stale_dims_the_link_items_and_leaves_the_player_ones_lit) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  const GsSnapshot s = nominal();
  const GsPlayerState ps = player_nominal();
  std::vector<DirtyRect> rects;
  bar.update(s, false, ps, c.s, &rects);
  // Text is unchanged by staleness -- the value is HELD, only dimmed.
  CHECK(line_of(bar, s, true, ps) == line_of(bar, s, false, ps));
  // Every field still redraws on the transition (the colour moved), and
  // exactly the six link items are the ones that changed colour.
  rects.clear();
  const int drawn = bar.update(s, true, ps, c.s, &rects);
  CHECK(drawn == 6);
}

// --- geometry ---------------------------------------------------------

TEST(the_bar_sits_along_the_bottom_edge) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  for (const Reso& r : kFourResolutions) {
    GsCompactBar bar(f);
    REQUIRE(bar.layout(r.w, r.h, &err));
    const DirtyRect b = bar.bounds();
    CHECK(b.y > r.h / 2);            // bottom half, not floating mid-screen
    CHECK(b.y + b.h <= r.h);         // and inside the surface
    CHECK(b.x >= 0);
    CHECK(b.x + b.w <= r.w);
  }
}

// One type size for the whole line, and the biggest one that fits. The
// bar has no design-size table -- "the largest baked size whose worst-case
// line fits between the insets" IS the rule, which is what lets it render
// readably on a 720p panel and larger on a 4K one.
TEST(layout_picks_the_largest_baked_size_that_fits) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  for (const Reso& r : kFourResolutions) {
    GsCompactBar bar(f);
    REQUIRE(bar.layout(r.w, r.h, &err));
    const int px = bar.debug_atlas_px();
    CHECK(px > 0);
    const MaskAtlas* chosen = f.atlas(px);
    REQUIRE(chosen != nullptr);
    // The inset the bar reserves, recomputed the way layout() does.
    const double scale = (double)r.h / 1080.0;
    const int avail = r.w - 2 * (int)(32 * scale + 0.5);
    CHECK(GsCompactBar::worst_line_width(*chosen, kMaxCards) <= avail);
    for (int bigger = px + 1; bigger <= 512; ++bigger) {
      const MaskAtlas* a = f.atlas(bigger);
      if (!a) continue;
      const bool fits = GsCompactBar::worst_line_width(*a, kMaxCards) <= avail &&
                        a->glyph_h + (int)(24 * scale + 0.5) <= r.h;
      if (fits) std::printf("  %dx%d: chose %d px but %d px also fits\n", r.w,
                            r.h, px, bigger);
      CHECK(!fits);
    }
  }
}

// Nothing fits on a postage stamp, and the bar says so rather than
// rendering a line that runs off both edges.
TEST(layout_fails_when_the_line_cannot_fit) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  err.clear();
  CHECK(!bar.layout(160, 120, &err));
  CHECK(!err.empty());
  // A failed layout draws nothing at all rather than drawing at some
  // stale earlier geometry.
  Canvas c(160, 120);
  std::vector<DirtyRect> rects;
  CHECK(bar.update(nominal(), false, player_nominal(), c.s, &rects) == 0);
}

TEST(no_two_field_boxes_overlap_at_any_resolution) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  for (const Reso& r : kFourResolutions) {
    GsCompactBar bar(f);
    REQUIRE(bar.layout(r.w, r.h, &err));
    Canvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    // Reconcile to a real card count first: the boxes move when it lands.
    bar.update(nominal(), false, player_nominal(), c.s, &rects);
    for (int i = 0; i < (int)GsBarField::kCount; ++i)
      for (int j = i + 1; j < (int)GsBarField::kCount; ++j) {
        const DirtyRect a = bar.debug_field_box((GsBarField)i);
        const DirtyRect b = bar.debug_field_box((GsBarField)j);
        if (overlaps(a, b))
          std::printf("  %dx%d: fields %d and %d overlap\n", r.w, r.h, i, j);
        CHECK(!overlaps(a, b));
      }
  }
}

// --- the clamp invariant ----------------------------------------------

// The one that matters: draw_text clips to the SURFACE, clear_region only
// to the box, so a value wider than its box draws once and is never erased
// again. gs_snapshot.cpp bounds no magnitude, so the clamps in state_of_
// are the only thing between a corrupt datagram and permanent garbage.
TEST(absurd_values_never_draw_outside_their_field_boxes) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);

  GsSnapshot s;
  s.channel = 2000000;
  s.mcs = 99999;
  s.air_pct = 1e300;
  s.pre_loss_pct = 1e300;
  s.post_loss_pct = -1e300;
  GsCard a; a.id = 0; a.heard = true; a.rssi_dbm = -1e300; a.snr_db = 1e300;
  GsCard b; b.id = 1; b.heard = true; b.rssi_dbm = 1e300; b.snr_db = -1e300;
  GsCard d; d.id = 2; d.heard = true; d.rssi_dbm = -12345.0; d.snr_db = -9999.0;
  GsCard e; e.id = 3; e.heard = true; e.rssi_dbm = 98765.0; e.snr_db = 4321.0;
  s.cards = {a, b, d, e};

  GsPlayerState ps;
  ps.fps = 1e300;
  ps.jitter_ms = 1e300;
  ps.mbps = 1e300;
  ps.vid_w = 1 << 30;
  ps.vid_h = -5;
  ps.lat_valid = true;
  ps.lat_p50_e2e_ms = 1 << 30;
  ps.lat_e2e_ms = -1 << 30;

  std::vector<DirtyRect> rects;
  bar.update(s, false, ps, c.s, &rects);

  // Every rendered string fits the box its worst case sized.
  const MaskAtlas* atlas = f.atlas(bar.debug_atlas_px());
  REQUIRE(atlas != nullptr);
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    const std::string t = bar.debug_field_text(s, false, ps, (GsBarField)i);
    const DirtyRect box = bar.debug_field_box((GsBarField)i);
    const int w = text_width(*atlas, t.c_str());
    if (w > box.w) std::printf("  field %d: \"%s\" %d px in a %d px box\n", i,
                               t.c_str(), w, box.w);
    CHECK(w <= box.w);
  }
  // And no pixel landed outside the union of the boxes.
  const DirtyRect bounds = bar.bounds();
  for (int y = 0; y < 1080; ++y)
    for (int x = 0; x < 1920; ++x) {
      if (!c.px[(size_t)y * 1920 + x]) continue;
      const bool inside = x >= bounds.x && x < bounds.x + bounds.w &&
                          y >= bounds.y && y < bounds.y + bounds.h;
      if (!inside) std::printf("  stray pixel at %d,%d\n", x, y);
      REQUIRE(inside);
    }
}

// --- dirty tracking ---------------------------------------------------

// The budget rule from gs_overlay.h applies here too: a typical second
// must redraw the handful of items that moved, never the whole line.
TEST(update_redraws_only_the_items_that_changed) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  const GsSnapshot s = nominal();
  GsPlayerState ps = player_nominal();
  std::vector<DirtyRect> rects;

  const int first = bar.update(s, false, ps, c.s, &rects);
  CHECK(first == (int)GsBarField::kCount);  // cold: everything
  rects.clear();
  CHECK(bar.update(s, false, ps, c.s, &rects) == 0);  // nothing moved
  rects.clear();
  ps.fps = 59.0;
  CHECK(bar.update(s, false, ps, c.s, &rects) == 1);
  CHECK(rects.size() == 1);
}

// invalidate() is what a buffer swap into an unseen slot needs: the next
// update restates every item even though nothing about the data moved.
TEST(invalidate_forces_a_full_restate) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  bar.update(nominal(), false, player_nominal(), c.s, &rects);
  bar.invalidate();
  rects.clear();
  CHECK(bar.update(nominal(), false, player_nominal(), c.s, &rects) ==
        (int)GsBarField::kCount);
}

// A collision with the MSP grid repaints whatever it touched, unchanged
// data or not -- that is what makes GS pixels win.
TEST(repaint_intersecting_restates_only_the_boxes_it_hits) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  bar.update(nominal(), false, player_nominal(), c.s, &rects);

  const DirtyRect box = bar.debug_field_box(GsBarField::kFps);
  const DirtyRect hit{box.x + 1, box.y + 1, 2, 2};
  rects.clear();
  CHECK(bar.repaint_intersecting(&hit, 1, c.s, &rects) == 1);
  // Nowhere near the line: nothing to reclaim.
  const DirtyRect miss{0, 0, 4, 4};
  rects.clear();
  CHECK(bar.repaint_intersecting(&miss, 1, c.s, &rects) == 0);
}

// The card count is the only thing that reflows the line. When it moves,
// the OLD boxes must be erased -- the line is centred, so every item
// shifts, and a field's own draw only ever clears its NEW box.
TEST(a_changed_card_count_reflows_and_leaves_no_orphan_pixels) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  Canvas c(1920, 1080);
  const GsPlayerState ps = player_nominal();

  GsSnapshot four = nominal();
  GsCard x; x.id = 2; x.heard = true; x.rssi_dbm = -60.0; x.snr_db = 15.0;
  GsCard y; y.id = 3; y.heard = true; y.rssi_dbm = -61.0; y.snr_db = 16.0;
  four.cards = {four.cards[0], four.cards[1], x, y};

  std::vector<DirtyRect> rects;
  bar.update(four, false, ps, c.s, &rects);
  CHECK(bar.debug_cards() == 4);

  // Down to one card: shorter line, re-centred.
  GsSnapshot one = nominal();
  one.cards = {one.cards[0]};
  rects.clear();
  bar.update(one, false, ps, c.s, &rects);
  CHECK(bar.debug_cards() == 1);

  // Nothing may remain outside the boxes the bar now owns.
  for (int yy = 0; yy < 1080; ++yy)
    for (int xx = 0; xx < 1920; ++xx) {
      if (!c.px[(size_t)yy * 1920 + xx]) continue;
      bool owned = false;
      for (int i = 0; i < (int)GsBarField::kCount && !owned; ++i) {
        const DirtyRect b = bar.debug_field_box((GsBarField)i);
        owned = xx >= b.x && xx < b.x + b.w && yy >= b.y && yy < b.y + b.h;
      }
      if (!owned) std::printf("  orphan pixel at %d,%d\n", xx, yy);
      REQUIRE(owned);
    }
}

// A snapshot reporting more cards than either style renders is truncated,
// not overrun.
TEST(more_cards_than_kMaxCards_are_truncated) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  GsCompactBar bar(f);
  REQUIRE(bar.layout(1920, 1080, &err));
  GsSnapshot s;
  for (int i = 0; i < 7; ++i) {
    GsCard c; c.id = i; c.heard = true; c.rssi_dbm = -70.0; c.snr_db = 20.0;
    s.cards.push_back(c);
  }
  const std::string rssi = bar.debug_field_text(s, false, GsPlayerState(),
                                                GsBarField::kRssi);
  CHECK(rssi == "rssi:-70/-70/-70/-70");
  Canvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  bar.update(s, false, GsPlayerState(), c.s, &rects);
  CHECK(bar.debug_cards() == kMaxCards);
}

// --- style selection --------------------------------------------------

TEST(parse_gs_style_accepts_exactly_two_names) {
  GsStyle st = GsStyle::kEssential;
  CHECK(parse_gs_style("compact", &st));
  CHECK(st == GsStyle::kCompact);
  CHECK(parse_gs_style("essential", &st));
  CHECK(st == GsStyle::kEssential);
  CHECK(!parse_gs_style("", &st));
  CHECK(!parse_gs_style("Compact", &st));
  CHECK(!parse_gs_style("bar", &st));
}

// The factory is what osd.gs.style actually reaches. Both styles must lay
// out at the design resolution through the same handle.
TEST(make_gs_layer_builds_a_layer_that_lays_out_for_either_style) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  for (GsStyle st : {GsStyle::kCompact, GsStyle::kEssential}) {
    std::unique_ptr<GsLayer> l = make_gs_layer(st, f);
    REQUIRE(l != nullptr);
    err.clear();
    CHECK(l->layout(1920, 1080, &err));
    CHECK(err.empty());
    Canvas c(1920, 1080);
    std::vector<DirtyRect> rects;
    CHECK(l->update(nominal(), false, player_nominal(), c.s, &rects) > 0);
  }
}

MTEST_MAIN
