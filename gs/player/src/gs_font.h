#ifndef MABUR_PLAYER_GS_FONT_H_
#define MABUR_PLAYER_GS_FONT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace maburplay {

// Borrowed view of one pixel size's glyph table in a mmapped .gfont.
//
// Unlike the MSP GlyphAtlas (pre-coloured premultiplied ARGB), a glyph here
// is a two-channel MASK: byte 0 coverage, byte 1 shadow. The colour comes
// from the caller at blit time, so one atlas serves all seven design tokens
// instead of one atlas page per colour -- and the shadow, which the design
// puts under every glyph, is already blurred and offset.
//
// glyph_w/glyph_h INCLUDE the 4 px pad the shadow needs, so a glyph cell is
// wider than advance_x. Pen advance is advance_x; the pad overlaps between
// neighbouring glyphs, which is exactly what a real drop shadow does.
struct MaskAtlas {
  int px = 0;
  int glyph_w = 0;
  int glyph_h = 0;
  int advance_x = 0;
  int baseline = 0;  // rows from cell top down to the baseline
  int n_glyphs = 0;
  const uint32_t* codepoints = nullptr;  // ascending, n_glyphs entries
  const uint8_t* pixels = nullptr;       // n_glyphs * glyph_w * glyph_h * 2

  // Glyph index for a codepoint, or -1 if absent. NEVER falls back to
  // index 0: a missing glyph must be visibly missing, not silently a space.
  int index_of(uint32_t cp) const;

  // Borrowed pointer to glyph gi's mask, or nullptr if gi is out of range.
  const uint8_t* glyph(int gi) const {
    if (gi < 0 || gi >= n_glyphs) return nullptr;
    return pixels + (size_t)gi * glyph_w * glyph_h * 2;
  }
};

// Maps a .gfont file (see tools/msp/gen_gsfont.py). All sizes are baked at
// generation time -- there is no runtime scaling here at all, which is the
// whole point: the rasterizer stays a 1:1 blitter.
class GsFont {
 public:
  GsFont() = default;
  ~GsFont();
  GsFont(const GsFont&) = delete;
  GsFont& operator=(const GsFont&) = delete;

  // On failure *err (when non-null) gets a human-readable reason and ok()
  // stays false. Every structural field is validated against the file
  // length here so that no draw-time read can run off the mapping.
  bool load(const std::string& path, std::string* err);
  bool ok() const { return map_ != nullptr && !atlases_.empty(); }

  // Atlas for exactly `px`, or nullptr. Deliberately exact: a near-miss
  // fallback would silently ship the wrong type size.
  const MaskAtlas* atlas(int px) const;
  int n_sizes() const { return (int)atlases_.size(); }

 private:
  void* map_ = nullptr;
  size_t map_bytes_ = 0;
  std::vector<MaskAtlas> atlases_;  // ascending by px
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_FONT_H_
