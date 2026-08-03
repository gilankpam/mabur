#include "osd_font.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>

namespace maburplay {
namespace {

struct MfontHdr {
  uint32_t magic, version, glyph_w, glyph_h, n_glyphs, reserved[3];
};
static_assert(sizeof(MfontHdr) == 32, ".mfont header must be 32 bytes");
constexpr uint32_t kMfontMagic = 0x544E464DU;  // 'MFNT', little-endian

void replicate(const uint32_t* src, int sw, int sh, uint32_t* dst, int dw, int dh) {
  const int kx = dw / sw, ky = dh / sh;
  for (int y = 0; y < dh; ++y) {
    const uint32_t* srow = src + (size_t)(y / ky) * sw;
    uint32_t* drow = dst + (size_t)y * dw;
    for (int x = 0; x < dw; ++x) drow[x] = srow[x / kx];
  }
}

// Area average over premultiplied ARGB -- correct without un-premultiplying.
void box(const uint32_t* src, int sw, int sh, uint32_t* dst, int dw, int dh) {
  for (int y = 0; y < dh; ++y) {
    const int y0 = y * sh / dh, y1 = std::max(y0 + 1, (y + 1) * sh / dh);
    for (int x = 0; x < dw; ++x) {
      const int x0 = x * sw / dw, x1 = std::max(x0 + 1, (x + 1) * sw / dw);
      uint32_t a = 0, r = 0, g = 0, b = 0, n = 0;
      for (int sy = y0; sy < y1; ++sy) {
        for (int sx = x0; sx < x1; ++sx) {
          const uint32_t p = src[(size_t)sy * sw + sx];
          a += p >> 24;
          r += (p >> 16) & 0xFF;
          g += (p >> 8) & 0xFF;
          b += p & 0xFF;
          ++n;
        }
      }
      dst[(size_t)y * dw + x] = ((a / n) << 24) | ((r / n) << 16) | ((g / n) << 8) | (b / n);
    }
  }
}

// Bilinear sampling, for ScaleMode::kFill's fractional upscale only.
void bilinear(const uint32_t* src, int sw, int sh, uint32_t* dst, int dw, int dh) {
  for (int y = 0; y < dh; ++y) {
    const int fy = ((y * sh) << 8) / dh;
    const int y0 = std::min(fy >> 8, sh - 1), y1 = std::min(y0 + 1, sh - 1);
    const int wy = fy & 0xFF;
    for (int x = 0; x < dw; ++x) {
      const int fx = ((x * sw) << 8) / dw;
      const int x0 = std::min(fx >> 8, sw - 1), x1 = std::min(x0 + 1, sw - 1);
      const int wx = fx & 0xFF;
      const uint32_t p00 = src[(size_t)y0 * sw + x0], p01 = src[(size_t)y0 * sw + x1];
      const uint32_t p10 = src[(size_t)y1 * sw + x0], p11 = src[(size_t)y1 * sw + x1];
      uint32_t out = 0;
      for (int sh_bits = 0; sh_bits < 32; sh_bits += 8) {
        const int c00 = (p00 >> sh_bits) & 0xFF, c01 = (p01 >> sh_bits) & 0xFF;
        const int c10 = (p10 >> sh_bits) & 0xFF, c11 = (p11 >> sh_bits) & 0xFF;
        const int top = c00 + ((c01 - c00) * wx >> 8);
        const int bot = c10 + ((c11 - c10) * wx >> 8);
        const int v = top + ((bot - top) * wy >> 8);
        out |= (uint32_t)(v & 0xFF) << sh_bits;
      }
      dst[(size_t)y * dw + x] = out;
    }
  }
}

}  // namespace

OsdFont::~OsdFont() {
  if (map_) munmap(map_, map_bytes_);
}

bool OsdFont::load(const std::string& path, std::string* err) {
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    if (err) *err = "cannot open " + path;
    return false;
  }
  struct stat st {};
  if (fstat(fd, &st) != 0 || (size_t)st.st_size < sizeof(MfontHdr)) {
    ::close(fd);
    if (err) *err = path + ": too small to be a .mfont";
    return false;
  }
  void* m = mmap(nullptr, (size_t)st.st_size, PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (m == MAP_FAILED) {
    if (err) *err = path + ": mmap failed";
    return false;
  }
  MfontHdr h{};
  std::memcpy(&h, m, sizeof(h));
  const size_t need =
      sizeof(MfontHdr) + (size_t)h.glyph_w * h.glyph_h * h.n_glyphs * 4;
  if (h.magic != kMfontMagic || h.version != 1 || h.glyph_w == 0 || h.glyph_h == 0 ||
      h.n_glyphs == 0 || (size_t)st.st_size < need) {
    munmap(m, (size_t)st.st_size);
    if (err) *err = path + ": bad .mfont header or truncated payload";
    return false;
  }
  map_ = m;
  map_bytes_ = (size_t)st.st_size;
  native_ = GlyphAtlas{(int)h.glyph_w, (int)h.glyph_h, (int)h.n_glyphs,
                       reinterpret_cast<const uint32_t*>((const uint8_t*)m + sizeof(MfontHdr))};
  return true;
}

const GlyphAtlas* OsdFont::atlas_at(int w, int h, ScaleMode mode) {
  if (!native_.pixels || w <= 0 || h <= 0) return nullptr;
  if (w == native_.glyph_w && h == native_.glyph_h) return &native_;
  if (cached_.pixels && cached_.glyph_w == w && cached_.glyph_h == h && cached_mode_ == mode)
    return &cached_;

  scaled_.assign((size_t)w * h * native_.n_glyphs, 0u);
  const bool integer_up = w >= native_.glyph_w && h >= native_.glyph_h &&
                          w % native_.glyph_w == 0 && h % native_.glyph_h == 0 &&
                          w / native_.glyph_w == h / native_.glyph_h;
  const bool shrink = w <= native_.glyph_w && h <= native_.glyph_h;
  for (int gi = 0; gi < native_.n_glyphs; ++gi) {
    const uint32_t* src = native_.pixels + (size_t)gi * native_.glyph_w * native_.glyph_h;
    uint32_t* dst = scaled_.data() + (size_t)gi * w * h;
    if (integer_up) {
      replicate(src, native_.glyph_w, native_.glyph_h, dst, w, h);
    } else if (shrink) {
      box(src, native_.glyph_w, native_.glyph_h, dst, w, h);
    } else {
      bilinear(src, native_.glyph_w, native_.glyph_h, dst, w, h);
    }
  }
  cached_ = GlyphAtlas{w, h, native_.n_glyphs, scaled_.data()};
  cached_mode_ = mode;
  return &cached_;
}

}  // namespace maburplay
