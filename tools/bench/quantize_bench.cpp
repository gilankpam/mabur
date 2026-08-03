// quantize_bench -- the measured gate on OSD quantization cost.
//
// BurnRecorder::set_osd() runs quantize() INLINE on maburplay's main loop,
// the loop that pumps the AU ring and reaps DRM flip events every 2 ms. A
// full-screen quantize that takes longer than a small fraction of that
// period stalls the display path at the MSP update rate (~4 Hz), which is
// exactly what a review measured on the first cut of this path: 3.45 ms on
// an x86 host, ~25 ms on the RK3566's A55 after the 7.4x host->target
// factor Phase 1 measured for OsdRaster::draw.
//
// This bench carries a FROZEN COPY of that pre-fix implementation
// (namespace legacy below) so the before/after is one run, on one machine,
// against one surface -- not a claim reconstructed from two builds.
//
// Not a ctest: it is a wall-clock measurement, so it is neither fast nor
// deterministic. Build it and run it by hand:
//   nix-shell -p pkg-config libusb1 --run "cmake --build build -j$(nproc) --target quantize_bench"
//   ./build/tests/quantize_bench
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "mabur/msp_dp.h"
#include "osd_font.h"
#include "osd_palette.h"
#include "osd_raster.h"

using namespace maburplay;

// ---------------------------------------------------------------------------
// Frozen baseline: osd_palette.cpp's quantize() as it stood before the fix,
// verbatim (the ARGB->index cache with no MRU, a full px.assign() every
// call, no dirty-region tracking). Do not "improve" it -- its whole job is
// to keep reproducing the number the fix is measured against.
namespace legacy {

uint32_t argb_to_yuva(uint32_t premul_argb) {
  const uint8_t a = (premul_argb >> 24) & 0xFF;
  if (a == 0) return 0;
  const uint8_t pr = (premul_argb >> 16) & 0xFF;
  const uint8_t pg = (premul_argb >> 8) & 0xFF;
  const uint8_t pb = premul_argb & 0xFF;
  int r = (int)pr * 255 / a;
  int g = (int)pg * 255 / a;
  int b = (int)pb * 255 / a;
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;
  auto clamp8 = [](double v) -> uint8_t {
    if (v < 0.0) v = 0.0;
    if (v > 255.0) v = 255.0;
    return (uint8_t)(v + 0.5);
  };
  const double yf = 16.0 + (65.481 * r + 128.553 * g + 24.966 * b) / 255.0;
  const double uf = 128.0 + (-37.797 * r - 74.203 * g + 112.0 * b) / 255.0;
  const double vf = 128.0 + (112.0 * r - 93.786 * g - 18.214 * b) / 255.0;
  return (uint32_t)clamp8(yf) | ((uint32_t)clamp8(uf) << 8) | ((uint32_t)clamp8(vf) << 16) |
         ((uint32_t)a << 24);
}

int nearest_entry(const OsdPalette& pal, uint32_t yuva) {
  const int y = yuva & 0xFF, u = (yuva >> 8) & 0xFF, v = (yuva >> 16) & 0xFF,
            a = (yuva >> 24) & 0xFF;
  int best = 0;
  long best_d = -1;
  for (int i = 0; i < pal.n; ++i) {
    const uint32_t e = pal.entry[i];
    const long dy = y - (long)(e & 0xFF);
    const long du = u - (long)((e >> 8) & 0xFF);
    const long dv = v - (long)((e >> 16) & 0xFF);
    const long da = a - (long)((e >> 24) & 0xFF);
    const long d = dy * dy + du * du + dv * dv + da * da;
    if (best_d < 0 || d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

void quantize(const Surface& s, const OsdPalette& pal, OsdIndexMap* out) {
  const int mb_w = (s.width + 15) / 16;
  const int mb_h = (s.height + 15) / 16;
  const int stride = mb_w * 16;
  const int height_px = mb_h * 16;
  out->mb_w = mb_w;
  out->mb_h = mb_h;
  out->px.assign((size_t)stride * (size_t)height_px, 0);
  std::unordered_map<uint32_t, uint8_t> cache;
  for (int y = 0; y < s.height; ++y) {
    const uint32_t* row = s.pixels + (size_t)y * (size_t)s.stride_px;
    uint8_t* orow = out->px.data() + (size_t)y * (size_t)stride;
    for (int x = 0; x < s.width; ++x) {
      const uint32_t argb = row[x];
      uint8_t idx;
      auto it = cache.find(argb);
      if (it != cache.end()) {
        idx = it->second;
      } else {
        if ((argb >> 24) == 0) {
          idx = 0;
        } else {
          idx = (uint8_t)nearest_entry(pal, argb_to_yuva(argb));
        }
        cache.emplace(argb, idx);
      }
      orow[x] = idx;
    }
  }
}

}  // namespace legacy
// ---------------------------------------------------------------------------

namespace {

// One MSP DisplayPort snapshot with `rows_used` rows of realistic-looking
// telemetry text, fed through the real parser into a real MspScreen.
mabur::MspScreen make_screen(int rows_used, int gen) {
  std::vector<uint8_t> bytes;
  const std::vector<uint8_t> clr = {2};
  mabur::msp_append_message(bytes, 182, clr.data(), clr.size());
  static const char* kRows[] = {"BATT 16.2V  45.1A  1240mAh", "ALT 132m  SPD 84kmh  DST 1.2km",
                                "RSSI 78  LQ 99  SNR 21dB", "GPS 12 SATS  HOME 271deg",
                                "MODE ANGLE   ARM   OSD 4.5"};
  for (int r = 0; r < rows_used; ++r) {
    std::string t = kRows[r % 5];
    t[t.size() - 1] = (char)('0' + ((gen + r) % 10));  // one cell changes per gen
    std::vector<uint8_t> ds = {3, (uint8_t)r, 2, 0};
    for (char c : t) ds.push_back((uint8_t)c);
    mabur::msp_append_message(bytes, 182, ds.data(), ds.size());
  }
  const std::vector<uint8_t> scr = {4};
  mabur::msp_append_message(bytes, 182, scr.data(), scr.size());

  mabur::MspParser parser;
  mabur::MspScreen screen;
  for (const auto& m : parser.feed(bytes.data(), bytes.size())) screen.apply(m);
  return screen;
}

double ms_per_call(const char* label, int iters, const std::function<void()>& fn) {
  fn();  // warm
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < iters; ++i) fn();
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
      iters;
  std::printf("  %-46s %8.3f ms/call\n", label, ms);
  return ms;
}

}  // namespace

int main(int argc, char** argv) {
  std::string font = MABUR_PLAY_BUNDLE_DIR "/font_btfl.mfont";
  int w = 1920, h = 1080;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--font" && i + 1 < argc) font = argv[++i];
    else if (a == "--screen" && i + 1 < argc) std::sscanf(argv[++i], "%dx%d", &w, &h);
  }

  OsdFont f;
  std::string err;
  if (!f.load(font, &err)) {
    std::fprintf(stderr, "quantize_bench: %s\n", err.c_str());
    return 2;
  }
  const OsdPalette pal = build_palette(f.native());

  std::vector<uint32_t> px((size_t)w * (size_t)h, 0u);
  Surface s{px.data(), w, h, w};
  OsdRaster raster(f, ScaleMode::kSharp);
  ShadowGrid draw_shadow, diff_shadow;
  const mabur::MspScreen screen = make_screen(5, 0);
  raster.draw(screen, s, &draw_shadow);

  std::printf("quantize_bench: %dx%d surface, palette n=%d, real atlas %dx%d\n", w, h, pal.n,
              f.native().glyph_w, f.native().glyph_h);

  OsdIndexMap m_old, m_new;
  QuantizeCache qcache;  // what BurnRecorder keeps for the session
  std::printf("full screen (typical OSD content):\n");
  const double t_legacy = ms_per_call("legacy quantize()  [pre-fix baseline]", 20,
                                      [&] { legacy::quantize(s, pal, &m_old); });
  const double t_full = ms_per_call("quantize()         [full, MRU + memo]", 20,
                                    [&] { quantize(s, pal, &m_new, &qcache); });
  if (m_old.px != m_new.px || m_old.mb_w != m_new.mb_w || m_old.mb_h != m_new.mb_h) {
    std::fprintf(stderr, "quantize_bench: FAIL -- new full quantize differs from the baseline\n");
    return 1;
  }

  // Steady state: one MSP snapshot arrives, a handful of cells changed.
  // diff() reports them; quantize_rects() touches only those pixels.
  std::vector<DirtyRect> rects;
  raster.diff(screen, s, &diff_shadow, &rects);  // first call: full
  const mabur::MspScreen screen2 = make_screen(5, 1);
  raster.draw(screen2, s, &draw_shadow);
  const int cells = raster.diff(screen2, s, &diff_shadow, &rects);
  size_t dirty_px = 0;
  for (const DirtyRect& r : rects) dirty_px += (size_t)r.w * (size_t)r.h;
  std::printf("steady state (%d cells changed, %zu rects, %zu px = %.2f%% of the surface):\n",
              cells, rects.size(), dirty_px, 100.0 * dirty_px / ((double)w * h));
  const double t_inc = ms_per_call("quantize_rects()   [incremental]", 2000, [&] {
    quantize_rects(s, pal, rects.data(), rects.size(), &m_new, &qcache);
  });
  // Same work with a COLD memo, i.e. what an incremental update would cost
  // if the ARGB->index memo did not survive the call.
  ms_per_call("quantize_rects()   [incremental, cold memo]", 200, [&] {
    quantize_rects(s, pal, rects.data(), rects.size(), &m_new);
  });

  // The other extreme the main loop hits: the stale-OSD blank.
  std::vector<uint32_t> blank((size_t)w * (size_t)h, 0u);
  Surface bs{blank.data(), w, h, w};
  std::printf("fully blank surface:\n");
  const double t_blank_legacy = ms_per_call("legacy quantize()  [pre-fix baseline]", 20,
                                            [&] { legacy::quantize(bs, pal, &m_old); });
  const double t_blank = ms_per_call("quantize()         [full, MRU + memo]", 20,
                                     [&] { quantize(bs, pal, &m_new, &qcache); });

  std::printf(
      "\nspeedup: full %.1fx, blank %.1fx, steady state %.0fx (%.3f -> %.3f ms)\n"
      "target estimate at Phase 1's measured 7.4x host->A55 factor:\n"
      "  legacy full %.1f ms  ->  steady state %.3f ms  (2 ms main-loop pump period)\n",
      t_legacy / t_full, t_blank_legacy / t_blank, t_legacy / t_inc, t_legacy, t_inc,
      t_legacy * 7.4, t_inc * 7.4);
  return 0;
}
