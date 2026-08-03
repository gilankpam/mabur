#ifndef MABUR_PLAYER_OSD_FONT_H_
#define MABUR_PLAYER_OSD_FONT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace maburplay {

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

  // Atlas with glyphs exactly w x h. Cached: one non-native size at a time
  // (canvas changes are rare). Returns nullptr if not loaded or w/h <= 0.
  const GlyphAtlas* atlas_at(int w, int h, ScaleMode mode);

 private:
  void* map_ = nullptr;
  size_t map_bytes_ = 0;
  GlyphAtlas native_;
  GlyphAtlas cached_;
  ScaleMode cached_mode_ = ScaleMode::kSharp;
  std::vector<uint32_t> scaled_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_FONT_H_
