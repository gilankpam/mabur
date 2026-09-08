#ifndef MABUR_PLAYER_GS_COMPACT_H_
#define MABUR_PLAYER_GS_COMPACT_H_

#include <cstdint>
#include <string>
#include <vector>

#include "gs_layer.h"
#include "osd_raster.h"  // Surface, DirtyRect

namespace maburplay {

class GsFont;
struct MaskAtlas;

// osd.gs.style = "compact": ONE plain-text line along the bottom edge.
//
//   ch:149 mcs:5 air:62% rssi:-70/-72 snr:22/20 bitrate:8.1 res:1280x720
//   fps:60 jit:5.2 lat:45/78 loss:0.3/0.0
//
// (one line on the glass; wrapped here only to fit this comment.)
//
// No status colours, no meters, no bars -- every glyph takes
// tok::kTextPrimary. The one exception is staleness: the six LINK-sourced
// items dim to tok::kTextLabel while the sideport is quiet, because the
// alternative is a frozen line that looks live, which is the failure this
// OSD exists to prevent. The five player-measured items (bitrate, res,
// fps, jit, lat) never dim -- they are current by construction.
//
// The glyph mask's baked drop shadow stays. It is part of the glyph, not
// styling: without it the line is unreadable over bright video.
//
// Same per-field dirty discipline as GsOverlay, and for the same reason --
// see the budget paragraph in gs_overlay.h. A full-line repaint is ~76 k px
// at 1080p (~1.6 ms projected on the A55), too close to the 2 ms pump
// period to do on a cadence; a typical second redraws the handful of items
// whose text actually moved.
enum class GsBarField {
  kCh = 0,
  kMcs,
  kAir,
  kRssi,
  kSnr,
  kBitrate,
  kRes,
  kFps,
  kJit,
  kLat,
  kLoss,
  kCount,
};

class GsCompactBar final : public GsLayer {
 public:
  explicit GsCompactBar(GsFont& font) : font_(font) {}

  // Picks the LARGEST baked atlas whose worst-case line (kMaxCards cards,
  // every value at the magnitude its clamp allows) fits between the
  // insets, then centres the line for however many cards are actually
  // reported. Type size is fixed at the worst case on purpose: sizing it
  // to the live card count would change the font under the pilot the first
  // time a card dropped out. Fails (with *err set) when even the smallest
  // baked size cannot fit the line.
  bool layout(int screen_w, int screen_h, std::string* err) override;

  int update(const GsSnapshot& snap, bool stale, const GsPlayerState& ps,
             const Surface& s, std::vector<DirtyRect>* out) override;
  int repaint_intersecting(const DirtyRect* rects, size_t n, const Surface& s,
                           std::vector<DirtyRect>* out) override;
  void invalidate() override;
  DirtyRect bounds() const override { return bounds_; }

  int field_count() const { return (int)GsBarField::kCount; }

  // Test hooks, mirroring GsOverlay's. `debug_field_text` formats without
  // drawing; `debug_field_box` is what the no-overflow tests measure text
  // against; `debug_atlas_px` is the size layout() resolved, 0 before it
  // has run.
  std::string debug_field_text(const GsSnapshot& snap, bool stale,
                               const GsPlayerState& ps, GsBarField id) const;
  DirtyRect debug_field_box(GsBarField id) const;
  int debug_atlas_px() const;
  // Width of the worst-case line in `a`, boxes included -- exactly what
  // layout() compares against the space between the insets. Exposed so the
  // size-choice test can assert "no larger baked size fits" by the same
  // measurement layout() makes, rather than by reimplementing it.
  static int worst_line_width(const MaskAtlas& a, int n_cards);
  // The number of card slots the line is currently drawn for, -1 before the
  // first update() has reconciled one.
  int debug_cards() const { return n_cards_; }

 private:
  struct FieldState {
    std::string text;
    uint32_t rgb = 0;
    bool operator==(const FieldState&) const = default;
  };
  struct Field {
    DirtyRect box{0, 0, 0, 0};
    int pen_x = 0;
    FieldState last;
    bool valid = false;
  };

  // The widest string field `id` can render with `n_cards` cards -- what
  // its box is sized from, and the only thing standing between a corrupt
  // datagram's 300-character number and permanent garbage on the glass
  // (draw_text clips to the SURFACE, clear_region only to the box). Every
  // clamp in state_of_ exists to keep the live string inside this one.
  static std::string worst_case(GsBarField id, int n_cards);

  FieldState state_of_(const GsSnapshot& snap, bool stale,
                       const GsPlayerState& ps, GsBarField id) const;
  // Centres the line for `n_cards` and recomputes every box. Called by
  // layout() (with kMaxCards, to reserve nothing wider than the atlas was
  // chosen for) and by update() whenever the reported card count moves.
  void place_(int n_cards);
  void draw_field_(GsBarField id, const FieldState& st, const Surface& s);
  Field& f_(GsBarField id) { return fields_[(size_t)id]; }
  const Field& f_(GsBarField id) const { return fields_[(size_t)id]; }

  GsFont& font_;
  Field fields_[(size_t)GsBarField::kCount];
  const MaskAtlas* atlas_ = nullptr;
  DirtyRect bounds_{0, 0, 0, 0};
  int screen_w_ = 0, baseline_y_ = 0, gap_ = 0, inset_x_ = 0;
  int n_cards_ = -1;  // card slots the line is placed for; -1 = never placed
  bool laid_out_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_COMPACT_H_
