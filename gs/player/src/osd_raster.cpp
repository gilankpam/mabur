#include "osd_raster.h"

#include <cstring>

namespace maburplay {
namespace {

void clear_rect(const Surface& s, int x0, int y0, int w, int h) {
  for (int y = y0; y < y0 + h; ++y) {
    if (y < 0 || y >= s.height) continue;
    const int xs = x0 < 0 ? 0 : x0;
    const int xe = x0 + w > s.width ? s.width : x0 + w;
    if (xe > xs) std::memset(s.pixels + (size_t)y * s.stride_px + xs, 0, (size_t)(xe - xs) * 4);
  }
}

// 1:1 copy, transparent source pixels left untouched. The cell was cleared
// first, so "untouched" means transparent, and the plane's premultiplied
// blend does the compositing against video.
void blit(const GlyphAtlas& a, int gi, const Surface& s, int ox, int oy) {
  const uint32_t* g = a.pixels + (size_t)gi * a.glyph_w * a.glyph_h;
  for (int y = 0; y < a.glyph_h; ++y) {
    const int py = oy + y;
    if (py < 0 || py >= s.height) continue;
    uint32_t* dst = s.pixels + (size_t)py * s.stride_px;
    const uint32_t* src = g + (size_t)y * a.glyph_w;
    for (int x = 0; x < a.glyph_w; ++x) {
      const int px = ox + x;
      if (px < 0 || px >= s.width) continue;
      if (src[x] >> 24) dst[px] = src[x];
    }
  }
}

}  // namespace

void OsdRaster::clear(const Surface& s, ShadowGrid* shadow) {
  if (!s.pixels) return;
  clear_rect(s, 0, 0, s.width, s.height);
  if (shadow) {
    shadow->layout = OsdLayout{};
    shadow->cells.clear();
  }
}

int OsdRaster::diff(const mabur::MspScreen& screen, const Surface& s, ShadowGrid* shadow,
                    std::vector<DirtyRect>* out) {
  if (out) out->clear();
  if (!s.pixels) return 0;
  const GlyphAtlas& nat = font_.native();
  if (!nat.pixels) return 0;

  // Layout recomputed rather than read from layout_: diff() must be callable
  // independently of draw() and compute_layout() is pure arithmetic. layout_
  // stays draw()'s to own.
  const OsdLayout lay = compute_layout(s.width, s.height, screen.cols(), screen.rows(),
                                       nat.glyph_w, nat.glyph_h, mode_);
  if (lay.draw_w <= 0 || lay.draw_h <= 0) return 0;

  const size_t n_cells = (size_t)screen.rows() * screen.cols();
  const bool full = !shadow || shadow->layout != lay || shadow->cells.size() != n_cells;
  if (full) {
    if (shadow) {
      shadow->layout = lay;
      shadow->cells.resize(n_cells);
      for (int r = 0; r < screen.rows(); ++r)
        for (int c = 0; c < screen.cols(); ++c)
          shadow->cells[(size_t)r * screen.cols() + c] = screen.cell(r, c);
    }
    if (out) out->push_back(DirtyRect{0, 0, s.width, s.height});
    return (int)n_cells;
  }

  int changed = 0;
  for (int r = 0; r < screen.rows(); ++r) {
    int run_start = -1;  // first column of the current contiguous changed run
    for (int c = 0; c <= screen.cols(); ++c) {
      const bool dirty = c < screen.cols() &&
                         shadow->cells[(size_t)r * screen.cols() + c] != screen.cell(r, c);
      if (dirty) {
        shadow->cells[(size_t)r * screen.cols() + c] = screen.cell(r, c);
        ++changed;
        if (run_start < 0) run_start = c;
        continue;
      }
      if (run_start < 0) continue;
      // Run ended: one rect for the whole span, which is what keeps the rect
      // count near the number of updated FIELDS rather than characters.
      if (out)
        out->push_back(DirtyRect{lay.origin_x + run_start * lay.draw_w,
                                 lay.origin_y + r * lay.draw_h, (c - run_start) * lay.draw_w,
                                 lay.draw_h});
      run_start = -1;
    }
  }
  return changed;
}

int OsdRaster::draw(const mabur::MspScreen& screen, const Surface& s, ShadowGrid* shadow) {
  if (!s.pixels) return 0;
  const GlyphAtlas& nat = font_.native();
  if (!nat.pixels) return 0;

  layout_ = compute_layout(s.width, s.height, screen.cols(), screen.rows(), nat.glyph_w,
                           nat.glyph_h, mode_);
  if (layout_.draw_w <= 0 || layout_.draw_h <= 0) return 0;
  const GlyphAtlas* atlas = font_.atlas_at(layout_.draw_w, layout_.draw_h, mode_);
  if (!atlas) return 0;

  const size_t n_cells = (size_t)screen.rows() * screen.cols();
  const bool full = !shadow || shadow->layout != layout_ || shadow->cells.size() != n_cells;
  if (full) {
    clear_rect(s, 0, 0, s.width, s.height);
    if (shadow) {
      shadow->layout = layout_;
      shadow->cells.assign(n_cells, 0u);
    }
  }

  int drawn = 0;
  for (int r = 0; r < screen.rows(); ++r) {
    for (int c = 0; c < screen.cols(); ++c) {
      const uint16_t cell = screen.cell(r, c);
      const size_t idx = (size_t)r * screen.cols() + c;
      if (!full && shadow->cells[idx] == cell) continue;
      if (shadow) shadow->cells[idx] = cell;
      ++drawn;

      const int ox = layout_.origin_x + c * layout_.draw_w;
      const int oy = layout_.origin_y + r * layout_.draw_h;
      if (!full) clear_rect(s, ox, oy, layout_.draw_w, layout_.draw_h);

      const uint8_t chr = (uint8_t)(cell & 0xFF);
      if (chr == 0 || chr == 0x20) continue;  // blank: cleared, nothing to draw
      const int gi = chr | (((cell >> 8) & 0x3) << 8);
      if (gi >= atlas->n_glyphs) continue;
      blit(*atlas, gi, s, ox, oy);
    }
  }
  return drawn;
}

}  // namespace maburplay
