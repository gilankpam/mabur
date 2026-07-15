#include "msp_renderer.h"

#include <cstdio>
#include <cstring>

namespace maburgs {

void MspRenderer::on_snapshot(const uint8_t* bytes, size_t len) {
  bool complete = false;
  for (auto& m : parser_.feed(bytes, len)) complete |= screen_.apply(m);
  if (complete) render();
}

void MspRenderer::render() {
  if (!shm_.acquire()) {
    if (!warned_) {
      std::fprintf(stderr,
                   "maburgs msp: shm region '%s' not present "
                   "(PixelPilot ExternalSurfaceWidget not up?) — retrying\n",
                   cfg_.shm_name.c_str());
      warned_ = true;
    }
    ++dropped_;
    return;
  }
  warned_ = false;

  const int W = shm_.width(), H = shm_.height();
  auto* buf = reinterpret_cast<uint32_t*>(shm_.data());
  std::memset(buf, 0, shm_.data_size());  // transparent

  const int cols = screen_.cols(), rows = screen_.rows();
  const int cw = (W - 2 * cfg_.x_offset) / cols;
  const int ch = (H - 2 * cfg_.y_offset) / rows;
  ++frames_rendered_;
  if (cw <= 0 || ch <= 0) return;  // canvas too small / misconfigured offsets

  const int gw = font_.glyph_w, gh = font_.glyph_h;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const uint16_t cell = screen_.cell(r, c);
      const uint8_t chr = cell & 0xFF;
      if (chr == 0 || chr == 0x20) continue;  // blank (matches MspScreen)
      const int gi = chr | (((cell >> 8) & 0x3) << 8);
      if (gi >= font_.n_glyphs) continue;
      const uint32_t* g = font_.pixels + (size_t)gi * gw * gh;
      const int ox = cfg_.x_offset + c * cw, oy = cfg_.y_offset + r * ch;
      for (int dy = 0; dy < ch; ++dy) {
        const int py = oy + dy;
        if (py < 0 || py >= H) continue;
        const int sy = dy * gh / ch;
        for (int dx = 0; dx < cw; ++dx) {
          const int px = ox + dx;
          if (px < 0 || px >= W) continue;
          const uint32_t argb = g[sy * gw + (dx * gw / cw)];
          if (argb >> 24) buf[(size_t)py * W + px] = argb;  // skip transparent
        }
      }
    }
  }
}

}  // namespace maburgs
