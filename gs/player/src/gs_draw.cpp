#include "gs_draw.h"

#include <cstring>

namespace maburplay {

uint32_t premul(uint32_t rgb, uint8_t a) {
  if (a == 0) return 0u;
  const uint32_t r = ((rgb >> 16) & 0xFF) * a / 255;
  const uint32_t g = ((rgb >> 8) & 0xFF) * a / 255;
  const uint32_t b = (rgb & 0xFF) * a / 255;
  return ((uint32_t)a << 24) | (r << 16) | (g << 8) | b;
}

uint32_t utf8_next(const char** p) {
  const uint8_t* s = (const uint8_t*)*p;
  const uint32_t c0 = s[0];
  if (c0 < 0x80) { *p += 1; return c0; }
  if ((c0 & 0xE0) == 0xC0 && (s[1] & 0xC0) == 0x80) {
    *p += 2;
    return ((c0 & 0x1Fu) << 6) | (s[1] & 0x3Fu);
  }
  if ((c0 & 0xF0) == 0xE0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80) {
    *p += 3;
    return ((c0 & 0x0Fu) << 12) | ((s[1] & 0x3Fu) << 6) | (s[2] & 0x3Fu);
  }
  if ((c0 & 0xF8) == 0xF0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80 &&
      (s[3] & 0xC0) == 0x80) {
    *p += 4;
    return ((c0 & 0x07u) << 18) | ((s[1] & 0x3Fu) << 12) | ((s[2] & 0x3Fu) << 6) |
           (s[3] & 0x3Fu);
  }
  *p += 1;  // malformed: consume one byte so no caller can spin
  return 0xFFFDu;
}

void fill_rect(const Surface& s, int x, int y, int w, int h, uint32_t rgb,
               uint8_t alpha) {
  if (!s.pixels || w <= 0 || h <= 0) return;
  const int x0 = x < 0 ? 0 : x;
  const int y0 = y < 0 ? 0 : y;
  const int x1 = (x + w) > s.width ? s.width : (x + w);
  const int y1 = (y + h) > s.height ? s.height : (y + h);
  if (x1 <= x0 || y1 <= y0) return;
  const uint32_t v = premul(rgb, alpha);
  for (int yy = y0; yy < y1; ++yy) {
    uint32_t* row = s.pixels + (size_t)yy * s.stride_px;
    for (int xx = x0; xx < x1; ++xx) row[xx] = v;
  }
}

void clear_region(const Surface& s, const DirtyRect& r) {
  if (!s.pixels || r.w <= 0 || r.h <= 0) return;
  const int x0 = r.x < 0 ? 0 : r.x;
  const int y0 = r.y < 0 ? 0 : r.y;
  const int x1 = (r.x + r.w) > s.width ? s.width : (r.x + r.w);
  const int y1 = (r.y + r.h) > s.height ? s.height : (r.y + r.h);
  if (x1 <= x0 || y1 <= y0) return;
  for (int yy = y0; yy < y1; ++yy)
    std::memset(s.pixels + (size_t)yy * s.stride_px + x0, 0,
                (size_t)(x1 - x0) * 4);
}

int text_width(const MaskAtlas& a, const char* utf8) {
  if (!utf8) return 0;
  int w = 0;
  const char* p = utf8;
  while (*p) {
    utf8_next(&p);
    w += a.advance_x;  // monospace, and missing glyphs still advance
  }
  return w;
}

int draw_text(const Surface& s, const MaskAtlas& a, int pen_x, int baseline_y,
              const char* utf8, uint32_t rgb) {
  if (!utf8) return 0;
  int advanced = 0;
  const char* p = utf8;
  while (*p) {
    const uint32_t cp = utf8_next(&p);
    const int pen = pen_x + advanced;
    advanced += a.advance_x;
    if (!s.pixels) continue;  // measure-only: still advance, draw nothing
    const int gi = a.index_of(cp);
    if (gi < 0) continue;     // missing glyph: a gap, not a substitution
    const uint8_t* g = a.glyph(gi);
    if (!g) continue;

    // The glyph cell is padded, so it starts PAD px left of and above the
    // pen/baseline. baseline tells us how far down the cell the baseline
    // sits; the horizontal pad is (glyph_w - advance_x) / 2.
    const int ox = pen - (a.glyph_w - a.advance_x) / 2;
    const int oy = baseline_y - a.baseline;

    for (int gy = 0; gy < a.glyph_h; ++gy) {
      const int py = oy + gy;
      if (py < 0 || py >= s.height) continue;
      uint32_t* row = s.pixels + (size_t)py * s.stride_px;
      const uint8_t* src = g + (size_t)gy * a.glyph_w * 2;
      for (int gx = 0; gx < a.glyph_w; ++gx) {
        const int px = ox + gx;
        if (px < 0 || px >= s.width) continue;
        const uint32_t cov = src[gx * 2];
        const uint32_t sha = src[gx * 2 + 1];
        if (!cov && !sha) continue;
        // alpha = cov over shadow; colour is premultiplied by cov alone
        // because the shadow is black and contributes no chroma.
        const uint32_t alpha = cov + sha * (255 - cov) / 255;
        const uint32_t r = ((rgb >> 16) & 0xFF) * cov / 255;
        const uint32_t gg = ((rgb >> 8) & 0xFF) * cov / 255;
        const uint32_t b = (rgb & 0xFF) * cov / 255;
        row[px] = (alpha << 24) | (r << 16) | (gg << 8) | b;
      }
    }
  }
  return advanced;
}

}  // namespace maburplay
