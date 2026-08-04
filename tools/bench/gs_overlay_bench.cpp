// gs_overlay_bench -- the measured gate on GS link-status overlay cost.
//
// GsOverlay::update() runs on maburplay's main loop: the same loop that pumps
// the AU ring and reaps DRM flip events every 2 ms. Whatever it draws is then
// re-quantized for the burned DVR by quantize_rects(), also inline. So the
// number that matters is not "how fast does the overlay draw" but
// draw + quantize together, per update, on the pump loop.
//
// This exists because of exactly one prior mistake: the first cut of the MSP
// OSD path landed a full-screen quantize() -- 3.4 ms on this host, ~25 ms on
// the RK3566's A55 -- inside that 2 ms loop, and nobody measured until a
// review did. quantize_bench.cpp is the artifact of that lesson for the MSP
// half; this is the GS half. Budget: the worst case must stay well inside
// 2 ms on the A55, and anything projecting past ~1.5 ms is a finding.
//
// The host->A55 factor is 7.4x, measured in Phase 1 for OsdRaster::draw and
// reused by quantize_bench; it is an estimate, not a measurement on the
// target, and is labelled as such in the output.
//
// Deliberately NOT a ctest: it is a wall-clock measurement, so it is neither
// fast nor deterministic. Build and run it by hand:
//   nix-shell -p pkg-config libusb1 --run \
//     "cmake --build build -j$(nproc) --target gs_overlay_bench"
//   ./build/tests/gs_overlay_bench [--screen 1920x1080]
//
// Runs against the SHIPPED asset (gs/player/bundle/gs_osd.gfont), not a
// synthetic one: real JetBrains Mono glyph boxes are what the target pays
// for, and they are larger than the synthetic generator's.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "gs_font.h"
#include "gs_overlay.h"
#include "gs_snapshot.h"
#include "osd_font.h"
#include "osd_palette.h"
#include "osd_raster.h"

using namespace maburplay;

namespace {

GsSnapshot nominal() {
  GsSnapshot s;
  s.mcs = 5;
  s.fec_pct = 25.0;
  s.air_pct = 61.0;
  s.pre_loss_pct = 2.1;
  s.post_loss_pct = 0.0;
  // Four cards: the worst case the card block can be asked to render.
  s.cards = {GsCard{0, true, -58.0, 18.0}, GsCard{1, true, -71.0, 9.0},
             GsCard{2, true, -60.0, 15.0}, GsCard{3, true, -65.0, 12.0}};
  return s;
}

GsPlayerState player_nominal() {
  GsPlayerState p;
  p.fps = 60.0;
  p.jitter_ms = 3.0;
  p.mbps = 24.6;
  p.rec.kind = RecState::Kind::kRecording;
  p.rec.elapsed_s = 767;
  return p;
}

double ms_per_call(const char* label, int iters, const std::function<void()>& fn) {
  fn();  // warm
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < iters; ++i) fn();
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
      iters;
  std::printf("  %-46s %9.4f ms/call  (A55 est %7.3f ms)\n", label, ms, ms * 7.4);
  return ms;
}

size_t area(const std::vector<DirtyRect>& v) {
  size_t n = 0;
  for (const DirtyRect& r : v) n += (size_t)r.w * (size_t)r.h;
  return n;
}

}  // namespace

int measure(GsFont& gf, const OsdPalette& pal, int w, int h, int msp_glyph_w,
            int msp_glyph_h) {
  std::string err;
  std::vector<uint32_t> px((size_t)w * (size_t)h, 0u);
  Surface s{px.data(), w, h, w};

  GsOverlay ov(gf);
  if (!ov.layout(w, h, &err)) {
    std::fprintf(stderr, "gs_overlay_bench: layout failed at %dx%d: %s\n", w, h, err.c_str());
    return 2;
  }

  const DirtyRect b = ov.bounds();
  std::printf(
      "\n================ %dx%d ================\n"
      "  overlay bounds %dx%d at (%d,%d) = %.2f%% of the surface\n",
      w, h, b.w, b.h, b.x, b.y, 100.0 * ((double)b.w * b.h) / ((double)w * h));

  // First update: every field drawn. Also sizes the index map for
  // quantize_rects(), which refuses to run against an unsized map.
  std::vector<DirtyRect> rects;
  const int n_first = ov.update(nominal(), false, player_nominal(), s, &rects);
  OsdIndexMap map;
  QuantizeCache qcache;
  quantize(s, pal, &map, &qcache);
  std::printf("  first update: %d fields, %zu rects, %zu px\n", n_first, rects.size(),
              area(rects));

  // --- Case 1: the typical tick. One player-measured field changes (the
  // jitter EMA moves on essentially every snapshot), everything else holds.
  // Alternated between two values so each iteration does real work rather
  // than hitting the unchanged-state early-out.
  {
    GsPlayerState pa = player_nominal(), pb = player_nominal();
    pa.jitter_ms = 3.0;
    pb.jitter_ms = 7.0;
    std::vector<DirtyRect> r;
    r.clear();
    ov.update(nominal(), false, pa, s, &r);
    r.clear();
    const int nf = ov.update(nominal(), false, pb, s, &r);
    std::printf("\nsingle-field change (%d field, %zu rects, %zu px = %.3f%% of surface):\n", nf,
                r.size(), area(r), 100.0 * area(r) / ((double)w * h));
    bool flip = false;
    ms_per_call("draw + quantize_rects  [per update]", 20000, [&] {
      std::vector<DirtyRect> out;
      ov.update(nominal(), false, flip ? pa : pb, s, &out);
      quantize_rects(s, pal, out.data(), out.size(), &map, &qcache);
      flip = !flip;
    });
  }

  // --- Case 2: the worst case. invalidate() forces the next update to
  // redraw every active field -- what happens after an MSP full-surface
  // clear, a buffer swap into a slot this overlay has never drawn, or a
  // resolution change. This is the number that must stay inside the loop.
  {
    ov.invalidate();
    std::vector<DirtyRect> r;
    const int nf = ov.update(nominal(), false, player_nominal(), s, &r);
    std::printf("\nfull repaint (%d fields, %zu rects, %zu px = %.2f%% of surface):\n", nf,
                r.size(), area(r), 100.0 * area(r) / ((double)w * h));
    ms_per_call("draw + quantize_rects  [per update]", 2000, [&] {
      ov.invalidate();
      std::vector<DirtyRect> out;
      ov.update(nominal(), false, player_nominal(), s, &out);
      quantize_rects(s, pal, out.data(), out.size(), &map, &qcache);
    });
    // Split, so a regression can be attributed to the right half.
    ms_per_call("  draw only", 2000, [&] {
      ov.invalidate();
      std::vector<DirtyRect> out;
      ov.update(nominal(), false, player_nominal(), s, &out);
    });
    ov.invalidate();
    std::vector<DirtyRect> fixed;
    ov.update(nominal(), false, player_nominal(), s, &fixed);
    ms_per_call("  quantize_rects only", 2000, [&] {
      quantize_rects(s, pal, fixed.data(), fixed.size(), &map, &qcache);
    });
  }

  // --- Case 3: the recurring shape of the worst case. The MSP rasterizer
  // repaints over the GS corners, and every GS field its rects touch has to
  // be redrawn so GS pixels win the collision. An MSP full-surface rect (a
  // first draw, a stale blank, a screen-size change) reclaims EVERY field --
  // which is the same work as case 2, but triggered by the MSP layer's
  // cadence rather than by a one-off invalidate. This is the number to
  // watch if the two layers are both enabled.
  {
    const DirtyRect full{0, 0, w, h};
    std::vector<DirtyRect> r;
    const int nf = ov.repaint_intersecting(&full, 1, s, &r);
    std::printf("\nMSP full-surface repaint (%d fields reclaimed, %zu px):\n", nf, area(r));
    ms_per_call("draw + quantize_rects  [per update]", 2000, [&] {
      std::vector<DirtyRect> out;
      ov.repaint_intersecting(&full, 1, s, &out);
      quantize_rects(s, pal, out.data(), out.size(), &map, &qcache);
    });
  }

  // --- Case 4: what the burn feed ACTUALLY pays per MSP tick, now that the
  // restate is scoped to the reclaimed fields rather than invalidate()-ing
  // the whole burn overlay. One changed MSP cell, landing on a GS box: with
  // an HD 50x18 sharp grid ~15% of cells do, so this is the steady-state
  // path whenever both overlays are on, not an edge case. Before scoping,
  // ANY such cell cost the case-2 figure above.
  {
    const DirtyRect box = ov.debug_field_box(GsFieldId::kCard0Rssi);
    const int cw = msp_glyph_w > 0 ? msp_glyph_w : 36;
    const int ch = msp_glyph_h > 0 ? msp_glyph_h : 54;
    const DirtyRect cell{box.x + 2, box.y + 2, cw, ch};
    std::vector<DirtyRect> r;
    const int nf = ov.repaint_intersecting(&cell, 1, s, &r);
    std::printf("\nMSP one-cell collision (%dx%d cell, %d fields reclaimed, %zu px = %.3f%%):\n",
                cw, ch, nf, area(r), 100.0 * area(r) / ((double)w * h));
    ms_per_call("draw + quantize_rects  [per update]", 20000, [&] {
      std::vector<DirtyRect> out;
      ov.repaint_intersecting(&cell, 1, s, &out);
      quantize_rects(s, pal, out.data(), out.size(), &map, &qcache);
    });
  }

  // --- Reference point: the full-screen quantize the overlay must never
  // trigger on this loop, for scale.
  std::printf("\nreference (NOT on the update path):\n");
  ms_per_call("full-surface quantize()", 20, [&] { quantize(s, pal, &map, &qcache); });

  return 0;
}

int main(int argc, char** argv) {
  std::string gsfont = MABUR_PLAY_BUNDLE_DIR "/gs_osd.gfont";
  std::string mspfont = MABUR_PLAY_BUNDLE_DIR "/font_btfl.mfont";
  int w = 0, h = 0;  // 0 == run the whole supported range
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--gs-font" && i + 1 < argc) gsfont = argv[++i];
    else if (a == "--font" && i + 1 < argc) mspfont = argv[++i];
    else if (a == "--screen" && i + 1 < argc) std::sscanf(argv[++i], "%dx%d", &w, &h);
    else {
      std::fprintf(stderr, "usage: gs_overlay_bench [--gs-font F] [--font F] [--screen WxH]\n");
      return 2;
    }
  }

  GsFont gf;
  std::string err;
  if (!gf.load(gsfont, &err)) {
    std::fprintf(stderr, "gs_overlay_bench: %s\n", err.c_str());
    return 2;
  }

  // The palette maburplay actually burns with: the MSP atlas's colours plus
  // the GS token seeds. A GS-only palette would quantize differently and
  // understate (or overstate) the per-pixel nearest-entry cost.
  OsdFont mf;
  GlyphAtlas empty;
  size_t n_seeds = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n_seeds);
  const bool have_msp = mf.load(mspfont, &err);
  const OsdPalette pal = build_palette(have_msp ? mf.native() : empty, seeds, n_seeds);

  const int gw = have_msp ? mf.native().glyph_w : 0;
  const int gh = have_msp ? mf.native().glyph_h : 0;

  std::printf("gs_overlay_bench: real JetBrains Mono asset %s (%d sizes), palette n=%d%s\n",
              gsfont.c_str(), gf.n_sizes(), pal.n,
              have_msp ? "" : " (NO MSP atlas -- seeds only)");

  // 2160p is not a curiosity: screen_mode is config, layout() supports it,
  // and everything here scales with PIXELS, so it is where the budget is
  // actually decided. Benching 1080p alone was how the 2160p figure went
  // unnoticed.
  if (w > 0 && h > 0) {
    if (measure(gf, pal, w, h, gw, gh) != 0) return 2;
  } else {
    if (measure(gf, pal, 1920, 1080, gw, gh) != 0) return 2;
    if (measure(gf, pal, 3840, 2160, gw, gh) != 0) return 2;
  }

  std::printf(
      "\nA55 estimates use Phase 1's measured 7.4x host->target factor -- an\n"
      "estimate carried from a blitting workload, not a target measurement.\n"
      "Budget: maburplay's pump loop is 2 ms; anything past ~1.5 ms projected\n"
      "is a finding, not a footnote. The quantize half is burned-DVR-only;\n"
      "in raw or no-DVR mode the cost is the draw column alone.\n");
  return 0;
}
