#include "osd_palette.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace encosd {
namespace {

inline uint8_t clamp8(int v) { return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

struct Bucket {
  int begin = 0, end = 0;  // half-open range into the unique-colour vector
  double extent = 0.0;     // longest weighted axis extent, the split priority
  int axis = 0;
};

}  // namespace

Yuva argb_premul_to_yuva(uint32_t argb) {
  const int a = (int)((argb >> 24) & 0xFF);
  Yuva c;
  c.a = (uint8_t)a;
  if (a == 0) {
    // Fully transparent: colour is meaningless (premultiplied), park it on
    // neutral black so it never drags a median-cut box around.
    c.y = 16;
    c.u = 128;
    c.v = 128;
    return c;
  }
  // Un-premultiply. Rounding up (+a/2) keeps a 1/255 alpha edge from
  // collapsing its colour to black.
  const int pr = (int)((argb >> 16) & 0xFF);
  const int pg = (int)((argb >> 8) & 0xFF);
  const int pb = (int)(argb & 0xFF);
  const int r = clamp8((pr * 255 + a / 2) / a);
  const int g = clamp8((pg * 255 + a / 2) / a);
  const int b = clamp8((pb * 255 + a / 2) / a);
  // BT.601 limited range, the swing MPP's own palette constants use.
  const int y = (257 * r + 504 * g + 98 * b + 16000) / 1000;
  const int u = (-148 * r - 291 * g + 439 * b + 128000) / 1000;
  const int v = (439 * r - 368 * g - 71 * b + 128000) / 1000;
  c.y = clamp8(y);
  c.u = clamp8(u);
  c.v = clamp8(v);
  return c;
}

uint32_t yuva_to_argb_premul(const Yuva& c) {
  const int y = (int)c.y - 16;
  const int u = (int)c.u - 128;
  const int v = (int)c.v - 128;
  const int r = clamp8((1164 * y + 1596 * v) / 1000);
  const int g = clamp8((1164 * y - 391 * u - 813 * v) / 1000);
  const int b = clamp8((1164 * y + 2018 * u) / 1000);
  const int a = (int)c.a;
  return ((uint32_t)a << 24) | ((uint32_t)((r * a + 127) / 255) << 16) |
         ((uint32_t)((g * a + 127) / 255) << 8) | (uint32_t)((b * a + 127) / 255);
}

QuantResult quantize(const uint32_t* argb, int w, int h, int stride_px, int max_colors,
                     double alpha_weight) {
  QuantResult out;
  out.index.assign((size_t)w * h, 0);
  out.plt.entries.assign(256, pack_yuva(16, 128, 128, 0));
  if (max_colors < 1) max_colors = 1;
  if (max_colors > 255) max_colors = 255;

  // Pass 1: histogram of distinct YUVA values. Fully transparent pixels go
  // straight to the reserved index 0 and never enter the quantizer.
  std::unordered_map<uint32_t, uint32_t> hist;
  hist.reserve(8192);
  for (int y = 0; y < h; ++y) {
    const uint32_t* row = argb + (size_t)y * stride_px;
    for (int x = 0; x < w; ++x) {
      const Yuva c = argb_premul_to_yuva(row[x]);
      if (c.a == 0) continue;
      ++hist[pack_yuva(c.y, c.u, c.v, c.a)];
    }
  }
  out.distinct_colors = (int)hist.size();
  if (hist.empty()) {
    out.plt.used = 1;
    return out;
  }

  struct Col {
    uint8_t ch[4];  // y u v a
    uint32_t count;
    uint32_t key;
  };
  std::vector<Col> cols;
  cols.reserve(hist.size());
  for (const auto& kv : hist) {
    const Yuva c = unpack_yuva(kv.first);
    cols.push_back(Col{{c.y, c.u, c.v, c.a}, kv.second, kv.first});
  }

  const double axis_w[4] = {1.0, 0.5, 0.5, alpha_weight};

  auto measure = [&](Bucket& b) {
    int lo[4] = {255, 255, 255, 255}, hi[4] = {0, 0, 0, 0};
    for (int i = b.begin; i < b.end; ++i) {
      for (int k = 0; k < 4; ++k) {
        const int v = cols[i].ch[k];
        if (v < lo[k]) lo[k] = v;
        if (v > hi[k]) hi[k] = v;
      }
    }
    b.extent = -1.0;
    b.axis = 0;
    for (int k = 0; k < 4; ++k) {
      const double e = (hi[k] - lo[k]) * axis_w[k];
      if (e > b.extent) {
        b.extent = e;
        b.axis = k;
      }
    }
    // A single-colour box is unsplittable; extent 0 already says so.
  };

  std::vector<Bucket> boxes;
  boxes.push_back(Bucket{0, (int)cols.size(), 0.0, 0});
  measure(boxes[0]);

  while ((int)boxes.size() < max_colors) {
    // Split the box with the largest weighted extent.
    int best = -1;
    double best_e = 0.0;
    for (size_t i = 0; i < boxes.size(); ++i) {
      if (boxes[i].end - boxes[i].begin < 2) continue;
      if (boxes[i].extent > best_e) {
        best_e = boxes[i].extent;
        best = (int)i;
      }
    }
    if (best < 0 || best_e <= 0.0) break;  // every box is a single colour

    Bucket& b = boxes[best];
    const int ax = b.axis;
    std::sort(cols.begin() + b.begin, cols.begin() + b.end,
              [ax](const Col& p, const Col& q) { return p.ch[ax] < q.ch[ax]; });
    // Median by pixel count, not by distinct-colour count: otherwise one
    // rare edge shade weighs as much as the whole glyph interior.
    uint64_t total = 0;
    for (int i = b.begin; i < b.end; ++i) total += cols[i].count;
    uint64_t acc = 0;
    int cut = b.begin;
    for (int i = b.begin; i < b.end - 1; ++i) {
      acc += cols[i].count;
      cut = i + 1;
      if (acc * 2 >= total) break;
    }
    Bucket lo{b.begin, cut, 0.0, 0}, hi{cut, b.end, 0.0, 0};
    measure(lo);
    measure(hi);
    boxes[best] = lo;
    boxes.push_back(hi);
  }

  // Representative colour of each box: pixel-count-weighted mean.
  std::vector<Yuva> reps(boxes.size());
  for (size_t i = 0; i < boxes.size(); ++i) {
    double s[4] = {0, 0, 0, 0};
    double n = 0;
    for (int j = boxes[i].begin; j < boxes[i].end; ++j) {
      const double c = (double)cols[j].count;
      n += c;
      for (int k = 0; k < 4; ++k) s[k] += c * cols[j].ch[k];
    }
    Yuva r;
    r.y = clamp8((int)std::lround(s[0] / n));
    r.u = clamp8((int)std::lround(s[1] / n));
    r.v = clamp8((int)std::lround(s[2] / n));
    r.a = clamp8((int)std::lround(s[3] / n));
    reps[i] = r;
  }

  // Palette: index 0 reserved transparent, boxes from 1 up.
  for (size_t i = 0; i < boxes.size(); ++i)
    out.plt.entries[i + 1] = pack_yuva(reps[i].y, reps[i].u, reps[i].v, reps[i].a);
  out.plt.used = (int)boxes.size() + 1;

  // Colour -> index map. Assign every colour to its NEAREST palette entry
  // rather than to the box it happened to land in: after the weighted mean a
  // neighbouring box is sometimes closer, and this is cheap (unique colours,
  // not pixels).
  std::unordered_map<uint32_t, uint8_t> lut;
  lut.reserve(cols.size() * 2);
  double err_sum = 0.0, err_max = 0.0;
  uint64_t err_n = 0;
  for (const auto& c : cols) {
    double best_d = 1e30;
    int best_i = 1;
    for (size_t i = 0; i < boxes.size(); ++i) {
      const Yuva& r = reps[i];
      const double dy = ((double)c.ch[0] - r.y) * axis_w[0];
      const double du = ((double)c.ch[1] - r.u) * axis_w[1];
      const double dv = ((double)c.ch[2] - r.v) * axis_w[2];
      const double da = ((double)c.ch[3] - r.a) * axis_w[3];
      const double d = dy * dy + du * du + dv * dv + da * da;
      if (d < best_d) {
        best_d = d;
        best_i = (int)i + 1;
      }
    }
    lut[c.key] = (uint8_t)best_i;
    const double d = std::sqrt(best_d);
    err_sum += d * c.count;
    err_n += c.count;
    if (d > err_max) err_max = d;
  }
  out.mean_err = err_n ? err_sum / (double)err_n : 0.0;
  out.max_err = err_max;

  // Pass 2: write the index bitmap.
  for (int y = 0; y < h; ++y) {
    const uint32_t* row = argb + (size_t)y * stride_px;
    uint8_t* dst = out.index.data() + (size_t)y * w;
    for (int x = 0; x < w; ++x) {
      const Yuva c = argb_premul_to_yuva(row[x]);
      if (c.a == 0) {
        dst[x] = 0;
        continue;
      }
      const auto it = lut.find(pack_yuva(c.y, c.u, c.v, c.a));
      dst[x] = (it == lut.end()) ? 0 : it->second;
    }
  }
  return out;
}

QuantResult map_to_palette(const uint32_t* argb, int w, int h, int stride_px, const Palette& plt,
                           double alpha_weight) {
  QuantResult out;
  out.plt = plt;
  out.index.assign((size_t)w * h, 0);
  const double axis_w[4] = {1.0, 0.5, 0.5, alpha_weight};
  const int n = plt.used < 1 ? 1 : plt.used;

  std::unordered_map<uint32_t, uint8_t> lut;
  lut.reserve(8192);
  double err_sum = 0.0, err_max = 0.0;
  uint64_t err_n = 0;
  std::unordered_map<uint32_t, uint32_t> seen;

  for (int y = 0; y < h; ++y) {
    const uint32_t* row = argb + (size_t)y * stride_px;
    uint8_t* dst = out.index.data() + (size_t)y * w;
    for (int x = 0; x < w; ++x) {
      const Yuva c = argb_premul_to_yuva(row[x]);
      if (c.a == 0) {
        dst[x] = 0;
        continue;
      }
      const uint32_t key = pack_yuva(c.y, c.u, c.v, c.a);
      auto it = lut.find(key);
      if (it == lut.end()) {
        double best_d = 1e30;
        int best_i = 0;
        for (int i = 0; i < n; ++i) {
          const Yuva r = unpack_yuva(plt.entries[i]);
          const double dy = ((double)c.y - r.y) * axis_w[0];
          const double du = ((double)c.u - r.u) * axis_w[1];
          const double dv = ((double)c.v - r.v) * axis_w[2];
          const double da = ((double)c.a - r.a) * axis_w[3];
          const double d = dy * dy + du * du + dv * dv + da * da;
          if (d < best_d) {
            best_d = d;
            best_i = i;
          }
        }
        it = lut.emplace(key, (uint8_t)best_i).first;
        seen[key] = (uint32_t)std::lround(std::sqrt(best_d) * 1000.0);
      }
      dst[x] = it->second;
      const double d = (double)seen[key] / 1000.0;
      err_sum += d;
      ++err_n;
      if (d > err_max) err_max = d;
    }
  }
  out.distinct_colors = (int)lut.size();
  out.mean_err = err_n ? err_sum / (double)err_n : 0.0;
  out.max_err = err_max;
  return out;
}

}  // namespace encosd
