#include "osd_palette.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace maburplay {

namespace {

// Un-premultiplies `c' = c * a / 255` back to `c`, then converts to BT.601
// limited-range YUV. Packs byte0=Y, byte1=U, byte2=V, byte3=alpha -- the
// exact layout MppEncOSDPltVal::val wants (NOT the {v,u,y,alpha} bitfield
// order rk_venc_cmd.h declares, which is wrong on little-endian).
//
// A fully transparent pixel returns 0 regardless of its colour bytes: the
// colour is meaningless once alpha collapses to zero, and callers rely on
// 0 to mean "reserved transparent entry".
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

  const uint8_t y8 = clamp8(yf);
  const uint8_t u8 = clamp8(uf);
  const uint8_t v8 = clamp8(vf);

  return (uint32_t)y8 | ((uint32_t)u8 << 8) | ((uint32_t)v8 << 16) |
         ((uint32_t)a << 24);
}

// A distinct (already-converted) colour observed in the atlas, plus how
// many pixels had it.
struct HistPoint {
  uint8_t y = 0, u = 0, v = 0, a = 0;
  uint64_t w = 0;
};

uint8_t axis_val(const HistPoint& p, int axis) {
  switch (axis) {
    case 0: return p.y;
    case 1: return p.u;
    case 2: return p.v;
    default: return p.a;
  }
}

int nearest_entry(const OsdPalette& pal, uint32_t yuva) {
  const int y = yuva & 0xFF;
  const int u = (yuva >> 8) & 0xFF;
  const int v = (yuva >> 16) & 0xFF;
  const int a = (yuva >> 24) & 0xFF;
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

}  // namespace

OsdPalette build_palette(const GlyphAtlas& atlas, const uint32_t* extra,
                         size_t n_extra) {
  OsdPalette pal;
  pal.entry[0] = 0;  // always fully transparent
  pal.n = 1;

  const bool have_atlas = atlas.pixels && atlas.glyph_w > 0 &&
                          atlas.glyph_h > 0 && atlas.n_glyphs > 0;
  const size_t total_px =
      have_atlas
          ? (size_t)atlas.glyph_w * (size_t)atlas.glyph_h * (size_t)atlas.n_glyphs
          : 0;

  // Histogram distinct converted colours by frequency. Fully transparent
  // pixels all collapse to yuva==0 (handled by argb_to_yuva) and are dropped
  // below -- they belong to the reserved index 0, not a median-cut box.
  std::unordered_map<uint32_t, uint64_t> hist;
  if (have_atlas) {
    hist.reserve(total_px / 4 + 1);
    for (size_t i = 0; i < total_px; ++i) {
      const uint32_t yuva = argb_to_yuva(atlas.pixels[i]);
      hist[yuva]++;
    }
  }

  // Extra seeds join the histogram as ordinary colours. The weight is a
  // deliberate choice: too low and median cut discards them, too high and
  // they steal boxes from the glyph body colours that dominate the screen.
  // One per mille of the atlas pixel count puts a seed on par with a
  // moderately common atlas colour. With no atlas at all the weight is 1
  // and every seed is equally weighted, which is the right answer when the
  // seeds are the only colours there are.
  if (extra && n_extra) {
    const uint64_t w = total_px / 1000 ? (uint64_t)(total_px / 1000) : 1u;
    for (size_t i = 0; i < n_extra; ++i) {
      const uint32_t yuva = argb_to_yuva(extra[i]);
      if (yuva == 0) continue;  // transparent is always index 0
      hist[yuva] += w;
    }
  }

  std::vector<HistPoint> points;
  points.reserve(hist.size());
  for (const auto& kv : hist) {
    if (kv.first == 0) continue;  // reserved transparent entry
    HistPoint p;
    p.y = kv.first & 0xFF;
    p.u = (kv.first >> 8) & 0xFF;
    p.v = (kv.first >> 16) & 0xFF;
    p.a = (kv.first >> 24) & 0xFF;
    p.w = kv.second;
    points.push_back(p);
  }
  if (points.empty()) return pal;

  // Median-cut: start with one box holding every distinct colour, and
  // repeatedly split the box with the largest weighted extent (channel
  // range along its longest axis, weighted by the box's total pixel
  // count) at the weighted median along that axis. Index 0 is already
  // spoken for, so we cap at 255 boxes -> at most 256 entries total.
  constexpr int kMaxBoxes = 255;
  using Box = std::vector<int>;  // indices into `points`
  std::vector<Box> boxes;
  {
    Box init;
    init.reserve(points.size());
    for (int i = 0; i < (int)points.size(); ++i) init.push_back(i);
    boxes.push_back(std::move(init));
  }

  while ((int)boxes.size() < kMaxBoxes) {
    int best_box = -1;
    int best_axis = 0;
    double best_score = -1.0;
    for (int bi = 0; bi < (int)boxes.size(); ++bi) {
      const Box& b = boxes[bi];
      if (b.size() <= 1) continue;
      uint8_t mn[4] = {255, 255, 255, 255};
      uint8_t mx[4] = {0, 0, 0, 0};
      uint64_t total_w = 0;
      for (int pi : b) {
        const HistPoint& p = points[pi];
        const uint8_t vals[4] = {p.y, p.u, p.v, p.a};
        for (int k = 0; k < 4; ++k) {
          if (vals[k] < mn[k]) mn[k] = vals[k];
          if (vals[k] > mx[k]) mx[k] = vals[k];
        }
        total_w += p.w;
      }
      int axis = 0;
      int range = mx[0] - mn[0];
      for (int k = 1; k < 4; ++k) {
        const int r = mx[k] - mn[k];
        if (r > range) {
          range = r;
          axis = k;
        }
      }
      const double score = (double)range * (double)total_w;
      if (score > best_score) {
        best_score = score;
        best_box = bi;
        best_axis = axis;
      }
    }
    if (best_box < 0) break;  // nothing left worth splitting

    Box& b = boxes[best_box];
    std::sort(b.begin(), b.end(), [&](int lhs, int rhs) {
      return axis_val(points[lhs], best_axis) <
             axis_val(points[rhs], best_axis);
    });
    uint64_t total_w = 0;
    for (int pi : b) total_w += points[pi].w;
    const uint64_t half = (total_w + 1) / 2;
    uint64_t acc = 0;
    size_t split_at = b.size();
    for (size_t k = 0; k < b.size(); ++k) {
      acc += points[b[k]].w;
      if (acc >= half) {
        split_at = k + 1;
        break;
      }
    }
    if (split_at == 0) split_at = 1;
    if (split_at >= b.size()) split_at = b.size() - 1;

    Box left(b.begin(), b.begin() + split_at);
    Box right(b.begin() + split_at, b.end());
    boxes[best_box] = std::move(left);
    boxes.push_back(std::move(right));
  }

  int idx = 1;
  for (const Box& b : boxes) {
    if (idx > 255) break;  // entry[] holds 256 slots, 0 already used
    uint64_t wsum = 0;
    double sy = 0, su = 0, sv = 0, sa = 0;
    for (int pi : b) {
      const HistPoint& p = points[pi];
      wsum += p.w;
      sy += (double)p.y * (double)p.w;
      su += (double)p.u * (double)p.w;
      sv += (double)p.v * (double)p.w;
      sa += (double)p.a * (double)p.w;
    }
    const uint8_t ey = (uint8_t)std::lround(sy / (double)wsum);
    const uint8_t eu = (uint8_t)std::lround(su / (double)wsum);
    const uint8_t ev = (uint8_t)std::lround(sv / (double)wsum);
    const uint8_t ea = (uint8_t)std::lround(sa / (double)wsum);
    pal.entry[idx] = (uint32_t)ey | ((uint32_t)eu << 8) |
                     ((uint32_t)ev << 16) | ((uint32_t)ea << 24);
    ++idx;
  }
  pal.n = idx;
  return pal;
}

namespace {

// ARGB word -> palette index, with the two-level lookup the pixel loop
// needs to stay off the 256-entry nearest-neighbour scan:
//
//  1. An MRU of ONE. An OSD surface is long runs of the same word --
//     transparent background, then a glyph's fill -- so the previous answer
//     is right for most pixels. Measured on the real atlas at 1080p: 3.45 ms
//     -> 1.04 ms per full screen, purely from these two lines
//     (tools/bench/quantize_bench.cpp).
//  2. A hash map for everything else, so a distinct colour costs the scan
//     once per call rather than once per pixel.
//  3. Both of them SURVIVE the call when the caller supplies a
//     QuantizeCache, which is what makes an incremental update cost the
//     pixels it writes rather than a fresh 256-entry scan per distinct
//     antialiased glyph colour.
class Quantizer {
 public:
  Quantizer(const OsdPalette& pal, QuantizeCache* cache)
      : pal_(pal), c_(cache ? *cache : own_) {}

  uint8_t index(uint32_t argb) {
    if (argb == c_.last_argb) return c_.last_idx;
    uint8_t idx;
    auto it = c_.map.find(argb);
    if (it != c_.map.end()) {
      idx = it->second;
    } else {
      if ((argb >> 24) == 0) {
        idx = 0;  // fully transparent -> reserved index, skip the search
      } else {
        idx = (uint8_t)nearest_entry(pal_, argb_to_yuva(argb));
      }
      c_.map.emplace(argb, idx);
    }
    c_.last_argb = argb;
    c_.last_idx = idx;
    return idx;
  }

 private:
  const OsdPalette& pal_;
  QuantizeCache own_;  // used only when the caller supplied none
  QuantizeCache& c_;
};

// Quantizes the [x0,x1) x [y0,y1) pixel window; the caller has already
// clipped it to the surface and sized `out`.
//
// Run-oriented on purpose: an OSD row is long spans of one word (transparent
// background, then a glyph's fill), so finding the span and memset()ing it
// replaces a per-pixel lookup with a compare the compiler vectorizes.
void quantize_window(const Surface& s, Quantizer& q, OsdIndexMap* out, int x0, int y0, int x1,
                     int y1) {
  const int stride = out->stride();
  for (int y = y0; y < y1; ++y) {
    const uint32_t* row = s.pixels + (size_t)y * (size_t)s.stride_px;
    uint8_t* orow = out->px.data() + (size_t)y * (size_t)stride;
    for (int x = x0; x < x1;) {
      const uint32_t argb = row[x];
      int e = x + 1;
      while (e < x1 && row[e] == argb) ++e;
      std::memset(orow + x, q.index(argb), (size_t)(e - x));
      x = e;
    }
  }
}

}  // namespace

void quantize(const Surface& s, const OsdPalette& pal, OsdIndexMap* out, QuantizeCache* cache) {
  const int mb_w = (s.width + 15) / 16;
  const int mb_h = (s.height + 15) / 16;

  out->mb_w = mb_w;
  out->mb_h = mb_h;
  out->px.assign((size_t)(mb_w * 16) * (size_t)(mb_h * 16), 0);
  if (!s.pixels) return;

  Quantizer q(pal, cache);
  quantize_window(s, q, out, 0, 0, s.width, s.height);
  // Rows/columns beyond the surface (padding to the macroblock grid) stay
  // at index 0 from the assign() above.
}

bool quantize_rects(const Surface& s, const OsdPalette& pal, const DirtyRect* rects,
                    size_t n_rects, OsdIndexMap* out, QuantizeCache* cache) {
  const int mb_w = (s.width + 15) / 16;
  const int mb_h = (s.height + 15) / 16;
  if (out->mb_w != mb_w || out->mb_h != mb_h ||
      out->px.size() != (size_t)(mb_w * 16) * (size_t)(mb_h * 16)) {
    return false;  // never sized for this surface: caller must do a full pass
  }
  if (!s.pixels) return false;

  Quantizer q(pal, cache);
  for (size_t i = 0; i < n_rects; ++i) {
    const DirtyRect& r = rects[i];
    const int x0 = r.x < 0 ? 0 : r.x;
    const int y0 = r.y < 0 ? 0 : r.y;
    const int x1 = r.x + r.w > s.width ? s.width : r.x + r.w;
    const int y1 = r.y + r.h > s.height ? s.height : r.y + r.h;
    if (x1 <= x0 || y1 <= y0) continue;
    quantize_window(s, q, out, x0, y0, x1, y1);
  }
  return true;
}

}  // namespace maburplay
