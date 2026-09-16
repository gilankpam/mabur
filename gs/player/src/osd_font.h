#ifndef MABUR_PLAYER_OSD_FONT_H_
#define MABUR_PLAYER_OSD_FONT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace maburplay {

class ColorTrans;

// How a glyph is resized when the draw size differs from the atlas size.
// kSharp is the default and is only ever handed integer multiples or
// smaller sizes (compute_layout guarantees it), so it never blurs.
enum class ScaleMode { kSharp, kFill };

// Borrowed view of a glyph table. Glyph gi occupies
// pixels[gi*glyph_w*glyph_h ..], row-major, premultiplied ARGB32
// (little-endian 0xAARRGGBB words). gi = char | (page << 8).
struct GlyphAtlas {
  int glyph_w = 0;
  int glyph_h = 0;
  int n_glyphs = 0;
  const uint32_t* pixels = nullptr;
};

// Maps a .mfont file (see tools/msp/gen_font.py) and produces atlases at an
// exact requested draw size. All scaling in the OSD path happens here, once
// per size, so the rasterizer is a pure 1:1 blitter.
class OsdFont {
 public:
  OsdFont() = default;
  ~OsdFont();
  OsdFont(const OsdFont&) = delete;
  OsdFont& operator=(const OsdFont&) = delete;

  // On failure, *err (when non-null) gets a human-readable reason.
  bool load(const std::string& path, std::string* err);
  bool ok() const { return native_.pixels != nullptr; }
  const GlyphAtlas& native() const { return native_; }

  // The atlas as mmapped from disk, before any set_inverse(). This is what
  // the burned DVR's palette must NOT be built from once an inverse is set
  // (build_palette takes the surface-space atlas plus the forward map).
  const GlyphAtlas& native_original() const { return original_; }

  // colortrans (docs/colortrans.md): the OSD plane is blended BEFORE the
  // CRTC LUT, so every glyph pixel is pre-inverted once, here. Copies the
  // mmapped atlas into an owned buffer (un-premultiply, invert,
  // re-premultiply; alpha untouched) and repoints native() at it. Any
  // scaled atlas already built is discarded -- it was built from the old
  // pixels. Callable more than once; the last call wins.
  void set_inverse(const ColorTrans& t);

  // Atlas with glyphs exactly w x h. Cached: one non-native size at a time
  // (canvas changes are rare). Returns nullptr if not loaded, w/h <= 0, or
  // w/h exceeds the sanity bound (kMaxGlyphDim in osd_font.cpp).
  const GlyphAtlas* atlas_at(int w, int h, ScaleMode mode);

  // Number of times atlas_at() has actually built a scaled atlas (i.e. a
  // cache miss for a non-native size). Exposed for testing the one-entry
  // cache; not meaningful otherwise.
  uint64_t builds() const { return builds_; }

 private:
  void* map_ = nullptr;
  size_t map_bytes_ = 0;
  GlyphAtlas native_;
  GlyphAtlas original_;          // the mmap view
  std::vector<uint32_t> inverted_;
  GlyphAtlas cached_;
  ScaleMode cached_mode_ = ScaleMode::kSharp;
  std::vector<uint32_t> scaled_;
  uint64_t builds_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_FONT_H_
