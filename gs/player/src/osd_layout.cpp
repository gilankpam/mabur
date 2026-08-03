#include "osd_layout.h"

#include <algorithm>

namespace maburplay {

OsdLayout compute_layout(int screen_w, int screen_h, int cols, int rows,
                         int glyph_w, int glyph_h, ScaleMode mode) {
  OsdLayout l;
  if (screen_w <= 0 || screen_h <= 0 || cols <= 0 || rows <= 0 || glyph_w <= 0 || glyph_h <= 0)
    return l;

  const int avail_w = screen_w / cols, avail_h = screen_h / rows;
  if (avail_w <= 0 || avail_h <= 0) return l;

  if (mode == ScaleMode::kFill) {
    l.draw_w = avail_w;
    l.draw_h = avail_h;
  } else {
    const int k = std::min(avail_w / glyph_w, avail_h / glyph_h);
    if (k >= 1) {
      l.draw_w = glyph_w * k;
      l.draw_h = glyph_h * k;
    } else if (avail_w * glyph_h <= avail_h * glyph_w) {  // width-limited
      l.draw_w = avail_w;
      l.draw_h = std::max(1, glyph_h * avail_w / glyph_w);
    } else {
      l.draw_h = avail_h;
      l.draw_w = std::max(1, glyph_w * avail_h / glyph_h);
    }
  }
  l.cols = cols;
  l.rows = rows;
  l.origin_x = (screen_w - l.draw_w * cols) / 2;
  l.origin_y = (screen_h - l.draw_h * rows) / 2;
  return l;
}

}  // namespace maburplay
