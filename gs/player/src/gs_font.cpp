#include "gs_font.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>

namespace maburplay {
namespace {

constexpr uint32_t kMagic = 0x544E4647u;  // 'GFNT' LE
constexpr uint32_t kVersion = 1;
constexpr size_t kHeaderBytes = 32;
// One size-directory entry is struct.pack("<7IQ", ...) in gen_gsfont.py:
// 7 u32 fields (px, glyph_w, glyph_h, advance_x, baseline, n_glyphs, pad)
// followed by the u64 offset, which Python packs at its natural 8-byte
// alignment -- 28 bytes of u32s, then the u64 at byte 28, for 36 total.
constexpr size_t kDirEntBytes = 36;
// Sanity bound: the shipped 56 px atlas has ~40x80 cells. Anything an order
// of magnitude past that is a corrupt file, not a font.
constexpr int kMaxGlyphDim = 512;

uint32_t rd32(const uint8_t* p) {
  uint32_t v;
  std::memcpy(&v, p, 4);
  return v;
}
uint64_t rd64(const uint8_t* p) {
  uint64_t v;
  std::memcpy(&v, p, 8);
  return v;
}

bool fail(std::string* err, const char* why) {
  if (err) *err = std::string("gs font: ") + why;
  return false;
}

}  // namespace

int MaskAtlas::index_of(uint32_t cp) const {
  int lo = 0, hi = n_glyphs - 1;
  while (lo <= hi) {
    const int mid = lo + (hi - lo) / 2;
    const uint32_t v = codepoints[mid];
    if (v == cp) return mid;
    if (v < cp) lo = mid + 1;
    else hi = mid - 1;
  }
  return -1;
}

GsFont::~GsFont() {
  if (map_) ::munmap(map_, map_bytes_);
  map_ = nullptr;
  map_bytes_ = 0;
  atlases_.clear();
}

bool GsFont::load(const std::string& path, std::string* err) {
  if (map_) {
    ::munmap(map_, map_bytes_);
    map_ = nullptr;
    map_bytes_ = 0;
  }
  atlases_.clear();

  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return fail(err, ("cannot open " + path).c_str());
  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size < (off_t)kHeaderBytes) {
    ::close(fd);
    return fail(err, ("too small or unstattable: " + path).c_str());
  }
  const size_t bytes = (size_t)st.st_size;
  void* m = ::mmap(nullptr, bytes, PROT_READ, MAP_PRIVATE, fd, 0);
  ::close(fd);
  if (m == MAP_FAILED) return fail(err, ("mmap failed: " + path).c_str());

  const uint8_t* base = (const uint8_t*)m;
  if (rd32(base) != kMagic) {
    ::munmap(m, bytes);
    return fail(err, ("bad magic (not a .gfont): " + path).c_str());
  }
  if (rd32(base + 4) != kVersion) {
    ::munmap(m, bytes);
    return fail(err, ("unsupported version: " + path).c_str());
  }
  const uint32_t n_sizes = rd32(base + 8);
  if (n_sizes == 0 || n_sizes > 64 ||
      kHeaderBytes + (size_t)n_sizes * kDirEntBytes > bytes) {
    ::munmap(m, bytes);
    return fail(err, ("bad size directory: " + path).c_str());
  }

  for (uint32_t i = 0; i < n_sizes; ++i) {
    const uint8_t* d = base + kHeaderBytes + (size_t)i * kDirEntBytes;
    MaskAtlas a;
    a.px = (int)rd32(d);
    a.glyph_w = (int)rd32(d + 4);
    a.glyph_h = (int)rd32(d + 8);
    a.advance_x = (int)rd32(d + 12);
    a.baseline = (int)rd32(d + 16);
    a.n_glyphs = (int)rd32(d + 20);
    // Byte 24 is `pad` (always 0, u32); the u64 offset starts right after
    // it at byte 28 -- NOT byte 24, which would read pad's zero as the
    // high half of a bogus offset.
    const uint64_t off = rd64(d + 28);

    if (a.px <= 0 || a.glyph_w <= 0 || a.glyph_h <= 0 || a.advance_x <= 0 ||
        a.n_glyphs <= 0 || a.glyph_w > kMaxGlyphDim || a.glyph_h > kMaxGlyphDim ||
        a.baseline < 0 || a.baseline >= a.glyph_h) {
      ::munmap(m, bytes);
      return fail(err, ("bad size entry: " + path).c_str());
    }
    // Every byte the atlas can ever address must be inside the mapping --
    // checked once here so glyph() and index_of() need no bounds logic.
    const size_t cp_bytes = (size_t)a.n_glyphs * 4;
    const size_t px_bytes = (size_t)a.n_glyphs * a.glyph_w * a.glyph_h * 2;
    if (off > bytes || cp_bytes > bytes - off || px_bytes > bytes - off - cp_bytes) {
      ::munmap(m, bytes);
      return fail(err, ("glyph block runs past end of file: " + path).c_str());
    }
    a.codepoints = (const uint32_t*)(base + off);
    a.pixels = base + off + cp_bytes;
    atlases_.push_back(a);
  }

  map_ = m;
  map_bytes_ = bytes;
  return true;
}

const MaskAtlas* GsFont::atlas(int px) const {
  for (const MaskAtlas& a : atlases_)
    if (a.px == px) return &a;
  return nullptr;
}

}  // namespace maburplay
