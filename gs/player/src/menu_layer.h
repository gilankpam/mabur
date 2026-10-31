#ifndef MABUR_PLAYER_MENU_LAYER_H_
#define MABUR_PLAYER_MENU_LAYER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "menu_model.h"
#include "osd_raster.h"  // Surface, DirtyRect

namespace maburplay {

class GsFont;
struct MaskAtlas;

// The goggles settings menu, painted into the OSD surface: a MenuView in,
// pixels and dirty rects out. Four boxes, each a filled panel:
//
//   panel   top-left; the crumb and link pill, up to 10 rows, two help
//           lines, the keys line
//   picker  right of the panel while a value is being changed: the
//           choices, a number spinner or a go-ahead prompt; no title
//           line, the row it sits beside names it
//   toast   one line, centred above the GS bar
//   banner  centred near the top; shown with the menu closed too
//
// Every box is cut into horizontal blocks that tile it (the header, each
// row, the strip under each row the busy sweep runs in, the help, the
// keys, a picker line). paint() rebuilds nothing it does not have to: a
// box whose block rects are unchanged redraws only the blocks whose
// content changed, so moving the cursor costs two rows and the sweep
// bar costs a 2 px strip. A box whose geometry changed (rows added, the
// picker resized, the menu closed) is erased whole and drawn again.
//
// Text is composited OVER the panel fill rather than written like the
// GS bar's: the glyph mask's anti-aliased edge and its shadow tail would
// otherwise replace the panel's alpha with their own, a lighter halo of
// video around every letter.
//
// Like the bar, every pixel is premultiplied and goes through
// colour_inverse() (gs_draw.h). Unlike it, nothing here is ever burned
// into a recording.
class MenuLayer {
 public:
  explicit MenuLayer(GsFont& font);

  // Picks the atlas sizes from the surface height (row text = nearest baked
  // to 22 * h/1080, help/keys/tags = nearest to 15 * h/1080); fails only when
  // the font has no atlas at all. Panel: inset 24*h/1080, width 45 % of w.
  bool layout(int screen_w, int screen_h, std::string* err);
  void set_view(const MenuView& v);             // diffs against the last painted view
  void set_tick(uint64_t now_ms);               // advances the busy-row dots and sweep bar
  // Draws what changed (or everything after invalidate()) into s; appends the
  // rects written. Returns the rect count. A closed view erases the panel,
  // picker, toast and banner areas it last drew. A surface with no pixels
  // (the presenter gave the OSD plane up) draws nothing, returns 0 and
  // leaves the change owed to the next real surface.
  int paint(const Surface& s, std::vector<DirtyRect>* out);
  // Redraws every drawn block that intersects one of `rects`, whether or
  // not it changed: what makes the menu win a collision with the MSP grid.
  int repaint_intersecting(const DirtyRect* rects, size_t n, const Surface& s,
                           std::vector<DirtyRect>* out);
  void invalidate();
  DirtyRect bounds() const;                     // union of panel, picker, toast, banner boxes
  bool dirty() const;
  // Test hooks: every text run the last paint left on the surface, box by
  // box (panel, picker, toast, banner), each top-to-bottom, left-to-right.
  std::vector<std::string> debug_texts() const;
  DirtyRect debug_panel_box() const;
  DirtyRect debug_picker_box() const;
  // Bars of the channel picker's clean-air icon (5 = clean) from the
  // channel's busy share in percent: < 5 -> 5, < 12 -> 4, < 25 -> 3,
  // < 50 -> 2, else 1.
  static int clean_air_bars(double busy_pct);

 private:
  struct Fill {
    int x = 0, y = 0, w = 0, h = 0;
    uint32_t rgb = 0;
    bool operator==(const Fill&) const = default;
  };
  struct Run {
    const MaskAtlas* atlas = nullptr;
    int x = 0, baseline = 0;
    std::string text;
    uint32_t rgb = 0;
    bool operator==(const Run&) const = default;
  };
  // One horizontal slice of a box: its background, then fills, then text.
  struct Block {
    int x = 0, y = 0, w = 0, h = 0;
    uint32_t bg = 0;
    std::vector<Fill> fills;
    std::vector<Run> runs;
    bool operator==(const Block&) const = default;
    DirtyRect rect() const { return DirtyRect{x, y, w, h}; }
  };
  struct Box {
    std::vector<Block> blocks;   // top to bottom, tiling rect()
    DirtyRect rect() const;
    bool same_geometry(const Box& o) const;
  };
  enum { kPanel = 0, kPicker, kToast, kBanner, kBoxes };
  using Boxes = std::vector<Box>;   // kBoxes entries, in draw order

  void rebuild_();
  void build_panel_(Box* b) const;
  void build_picker_(int x, int y, Box* b) const;
  void build_note_(const std::string& text, bool banner, Box* b) const;
  void draw_block_(const Surface& s, const Block& b);
  int sc_(double design_px) const;   // a 1080p design length at this height

  GsFont& font_;
  const MaskAtlas* row_ = nullptr;     // row text, toast, banner
  const MaskAtlas* small_ = nullptr;   // header, help, keys, tags, status, notes
  const MaskAtlas* big_ = nullptr;     // the number picker's value
  int screen_w_ = 0, screen_h_ = 0;
  bool laid_out_ = false;

  MenuView view_;
  int dots_ = 0;      // busy-row dots, 0..3
  int sweep_ = 0;     // busy-row sweep step
  Boxes target_;      // what view_ looks like now
  Boxes drawn_;       // what the last paint left on the surface
  bool invalid_ = true;
  std::vector<uint32_t> scratch_;   // one text run, for compositing
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_LAYER_H_
