#include "splash_image.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace maburplay {
namespace {

// Beyond this a header is corrupt, not an image we ship. 4096x2160 is one
// step past the largest mode the presenter can select.
constexpr uint64_t kMaxPixels = 4096ull * 2160ull;

uint32_t rd32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void fill_black(const Surface& d) {
  if (!d.pixels) return;
  for (int y = 0; y < d.height; ++y)
    for (int x = 0; x < d.width; ++x)
      d.pixels[static_cast<size_t>(y) * d.stride_px + x] = 0xff000000u;
}

// Source rectangle for a cover fit: scale by max(dw/sw, dh/sh) and centre the
// crop. Integer arithmetic throughout so the geometry is exactly testable.
void cover_crop(int sw, int sh, int dw, int dh, int* cx, int* cy, int* cw, int* ch) {
  if (static_cast<int64_t>(sw) * dh >= static_cast<int64_t>(dw) * sh) {
    // Source is wider than the destination aspect: full height, crop width.
    *ch = sh;
    *cw = static_cast<int>((static_cast<int64_t>(sh) * dw) / dh);
    if (*cw < 1) *cw = 1;
    if (*cw > sw) *cw = sw;
    *cx = (sw - *cw) / 2;
    *cy = 0;
  } else {
    *cw = sw;
    *ch = static_cast<int>((static_cast<int64_t>(sw) * dh) / dw);
    if (*ch < 1) *ch = 1;
    if (*ch > sh) *ch = sh;
    *cx = 0;
    *cy = (sh - *ch) / 2;
  }
}

void box_down(const uint32_t* src, int src_stride, int cx, int cy, int cw, int ch,
              const Surface& d) {
  for (int y = 0; y < d.height; ++y) {
    const int y0 = cy + static_cast<int>((static_cast<int64_t>(y) * ch) / d.height);
    int y1 = cy + static_cast<int>((static_cast<int64_t>(y + 1) * ch) / d.height);
    if (y1 <= y0) y1 = y0 + 1;
    for (int x = 0; x < d.width; ++x) {
      const int x0 = cx + static_cast<int>((static_cast<int64_t>(x) * cw) / d.width);
      int x1 = cx + static_cast<int>((static_cast<int64_t>(x + 1) * cw) / d.width);
      if (x1 <= x0) x1 = x0 + 1;
      uint32_t a = 0, r = 0, g = 0, b = 0, n = 0;
      for (int sy = y0; sy < y1; ++sy) {
        const uint32_t* row = src + static_cast<size_t>(sy) * src_stride;
        for (int sx = x0; sx < x1; ++sx) {
          const uint32_t p = row[sx];
          a += (p >> 24) & 0xff;
          r += (p >> 16) & 0xff;
          g += (p >> 8) & 0xff;
          b += p & 0xff;
          ++n;
        }
      }
      d.pixels[static_cast<size_t>(y) * d.stride_px + x] =
          ((a / n) << 24) | ((r / n) << 16) | ((g / n) << 8) | (b / n);
    }
  }
}

// Per-channel lerp, w in [0,65536].
uint32_t lerp_argb(uint32_t p, uint32_t q, uint32_t w) {
  uint32_t out = 0;
  for (int sh = 0; sh < 32; sh += 8) {
    const uint32_t a = (p >> sh) & 0xff, b = (q >> sh) & 0xff;
    out |= ((a * (65536u - w) + b * w + 32768u) >> 16) << sh;
  }
  return out;
}

void bilinear_up(const uint32_t* src, int src_stride, int cx, int cy, int cw, int ch,
                 const Surface& d) {
  // 16.16 fixed point. A destination pixel centre maps to
  // (i + 0.5) * c / dim - 0.5 in source coordinates; clamped at both ends, so
  // the destination corners land exactly on the source corners.
  for (int y = 0; y < d.height; ++y) {
    int64_t fy = ((static_cast<int64_t>(2 * y + 1) * ch) << 15) / d.height - (1 << 15);
    if (fy < 0) fy = 0;
    int y0 = static_cast<int>(fy >> 16);
    if (y0 > ch - 1) y0 = ch - 1;
    const int y1 = (y0 + 1 < ch) ? y0 + 1 : y0;
    const uint32_t wy = static_cast<uint32_t>(fy & 0xffff);
    const uint32_t* r0 = src + static_cast<size_t>(cy + y0) * src_stride + cx;
    const uint32_t* r1 = src + static_cast<size_t>(cy + y1) * src_stride + cx;
    for (int x = 0; x < d.width; ++x) {
      int64_t fx = ((static_cast<int64_t>(2 * x + 1) * cw) << 15) / d.width - (1 << 15);
      if (fx < 0) fx = 0;
      int x0 = static_cast<int>(fx >> 16);
      if (x0 > cw - 1) x0 = cw - 1;
      const int x1 = (x0 + 1 < cw) ? x0 + 1 : x0;
      const uint32_t wx = static_cast<uint32_t>(fx & 0xffff);
      const uint32_t top = lerp_argb(r0[x0], r0[x1], wx);
      const uint32_t bot = lerp_argb(r1[x0], r1[x1], wx);
      d.pixels[static_cast<size_t>(y) * d.stride_px + x] = lerp_argb(top, bot, wy);
    }
  }
}

}  // namespace

void resample_cover(const uint32_t* src, int src_w, int src_h, const Surface& d) {
  if (!src || !d.pixels || src_w <= 0 || src_h <= 0 || d.width <= 0 || d.height <= 0) return;
  int cx = 0, cy = 0, cw = 0, ch = 0;
  cover_crop(src_w, src_h, d.width, d.height, &cx, &cy, &cw, &ch);
  if (cw == d.width && ch == d.height) {
    for (int y = 0; y < d.height; ++y)
      std::memcpy(d.pixels + static_cast<size_t>(y) * d.stride_px,
                  src + static_cast<size_t>(cy + y) * src_w + cx,
                  static_cast<size_t>(d.width) * 4u);
    return;
  }
  // Cover scales both axes by the same factor, so the two axes are never in
  // opposite directions: one comparison picks the filter for the whole blit.
  if (d.width <= cw)
    box_down(src, src_w, cx, cy, cw, ch, d);
  else
    bilinear_up(src, src_w, cx, cy, cw, ch, d);
}

bool paint_splash(const std::string& path, const Surface& dst, std::string* err) {
  auto fail = [&](const std::string& why) {
    if (err) *err = why;
    fill_black(dst);
    return false;
  };
  if (!dst.pixels || dst.width <= 0 || dst.height <= 0) return fail("no destination surface");

  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return fail(path + ": " + std::strerror(errno));
  struct stat st {};
  if (::fstat(fd, &st) != 0) {
    const std::string e = std::strerror(errno);
    ::close(fd);
    return fail(path + ": " + e);
  }
  const uint64_t len = static_cast<uint64_t>(st.st_size);
  if (len < static_cast<uint64_t>(kSplashHeaderBytes)) {
    ::close(fd);
    return fail(path + ": shorter than the 16-byte header");
  }
  void* map = ::mmap(nullptr, static_cast<size_t>(len), PROT_READ, MAP_PRIVATE, fd, 0);
  ::close(fd);
  if (map == MAP_FAILED) return fail(path + ": mmap failed");

  const uint8_t* p = static_cast<const uint8_t*>(map);
  std::string why;
  const uint32_t w = rd32(p + 4), h = rd32(p + 8), reserved = rd32(p + 12);
  if (std::memcmp(p, "MSPL", 4) != 0)
    why = "bad magic (not a splash asset)";
  else if (reserved != 0)
    why = "reserved header word is not zero";
  else if (w == 0 || h == 0 || static_cast<uint64_t>(w) * h > kMaxPixels)
    why = "implausible dimensions";
  else if (len < static_cast<uint64_t>(kSplashHeaderBytes) + static_cast<uint64_t>(w) * h * 4u)
    why = "file is shorter than its header claims";

  if (!why.empty()) {
    ::munmap(map, static_cast<size_t>(len));
    return fail(path + ": " + why);
  }
  resample_cover(reinterpret_cast<const uint32_t*>(p + kSplashHeaderBytes), static_cast<int>(w),
                 static_cast<int>(h), dst);
  ::munmap(map, static_cast<size_t>(len));
  if (err) err->clear();
  return true;
}

}  // namespace maburplay
