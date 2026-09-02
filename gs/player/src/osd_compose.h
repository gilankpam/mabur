#ifndef MABUR_PLAYER_OSD_COMPOSE_H_
#define MABUR_PLAYER_OSD_COMPOSE_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gs_overlay.h"
#include "mabur/msp_dp.h"
#include "osd_raster.h"

namespace maburplay {

// Both overlays' inputs for one composition.
struct OsdComposeIn {
  // MSP half. `screen` is null when this run has no MSP overlay at all --
  // not merely when no screen has arrived yet.
  const mabur::MspScreen* screen = nullptr;
  bool msp_fresh = false;  // a complete new screen landed this iteration
  bool msp_stale = false;  // the source has been quiet past osd.stale_ms

  // GS half. `snap` is null when this run has no GS overlay.
  const GsSnapshot* snap = nullptr;
  bool gs_stale = false;
  GsPlayerState gs_ps;
  bool gs_dirty = false;  // a GS input moved since the last composition
};

struct OsdComposeOut {
  bool published = false;       // the caller should call osd_publish()
  bool announce_blank = false;  // first buffer of a stale episode: log it once
  int gs_drawn = 0;
  int gs_reclaimed = 0;
};

// Owns everything about "what is on OSD buffer i", for both layers, and
// composes one buffer at a time.
//
// THE INVARIANT THE WHOLE DESIGN RESTS ON: a buffer only ever reaches the
// screen via compose(idx) followed by a publish and a commit, so compose()
// must leave buffer `idx` current in BOTH layers -- not only in the layer
// whose input happened to move. Two independent publishers (a fresh MSP
// screen, a moved GS figure) share one back index and one dirty flag, so
// anything that is per-buffer and refreshed by only one of them goes stale
// in the other's buffer and then strobes at the publish rate. That is why
// the MSP raster is redrawn on a GS-triggered composition (it costs a cell
// compare and writes nothing when the screen has not moved), why the stale
// blank is tracked per buffer rather than by a single latch, and why the
// GS overlay is updated on an MSP-triggered one.
//
// Shadow bookkeeping, and why there are five of them for two buffers:
//   shadow_[2]   what MSP cells buffer i holds. Per DRM buffer.
//   pre_         a copy of shadow_[idx] taken before draw(), diffed after,
//                to recover exactly the cells draw() wrote into THIS buffer
//                -- the set that can have clobbered GS pixels underneath.
//   burn_shadow_ the burned DVR's own MSP lineage: its index map is
//                refreshed on every composition, so "changed since the last
//                composition" is neither a subset nor a superset of any
//                buffer's "changed since the last composition into ME".
//   gs_[0], gs_[1], gs_[2]  the same three lineages on the GS side. gs_[2]
//                draws into the burn CANVAS (below), never into a DRM
//                buffer.
//
// The burn canvas (burn_px_/burn_canvas_): the burn feed's own heap-backed
// mirror of the picture, composed with the burn lineages above and handed
// to the sink in place of the display surface. It exists because the DRM
// dumb buffers are WRITE-COMBINE mappings on hardware: writes stream fine,
// but quantize_rects READING one back was measured at 2-23 ms per OSD
// update on the pump loop (dvr-latency probe, 2026-09-02) against the
// ~0.1-2.2 ms the cached-memory bench projects -- enough to stall decode
// drain and blow vsync release deadlines on every burned recording. The
// canvas costs one extra cached draw per update and 4 bytes/px of heap;
// reads out of it run at the benched speed. It is single (not per DRM
// buffer): it follows the SCREEN, exactly like the recorder's index map,
// which is what makes its lineage the simple one.
class OsdComposer {
 public:
  // Rect-scoped index-map update for the burned DVR. Never called with a
  // null rect array: a full quantize is ~25 ms on the A55 and has no place
  // on the pump loop.
  using BurnSink = std::function<void(const Surface&, const DirtyRect*, size_t)>;

  // Null raster => this run has no MSP overlay. Not owned.
  void set_raster(OsdRaster* r) { raster_ = r; }
  // All three or none: buffer 0, buffer 1, and the burn's lineage.
  void set_gs(std::unique_ptr<GsOverlay> b0, std::unique_ptr<GsOverlay> b1,
              std::unique_ptr<GsOverlay> burn);
  // Installing a sink resets the burn's OWN lineage -- burn_shadow_ (the
  // MSP grid), gs_[2] (the GS twin) and the canvas pixels -- because a
  // newly started BurnRecorder's index map is sized and blank, so a diff
  // against the DEAD recorder's state would leave most of the new
  // recording's overlay unpainted. The next composition therefore restates
  // everything -- a full cached draw plus quantize, single-digit ms. That
  // is the accepted startup-event cost of a deliberate button press, not a
  // cadence -- see CLAUDE.md on the 2 ms pump budget.
  void set_burn_sink(BurnSink s);

  // Lays out all three GS overlays at the same size. On failure drops all
  // three (so gs_present() goes false) and sets *err.
  bool gs_layout(int screen_w, int screen_h, std::string* err);

  bool gs_present() const { return gs_[0] != nullptr; }
  bool gs_live() const { return gs_[0] != nullptr && gs_laid_out_; }
  bool blanked(int idx) const { return blanked_[idx & 1]; }
  bool screen_blank() const { return screen_blank_; }

  // True when a composition would change what the SCREEN shows. Takes no
  // buffer index, deliberately: publishing is what reaches the screen, and
  // it publishes whichever buffer is back, so the answer cannot depend on
  // which one that is. Asking it per buffer is what made the stale blank
  // unreachable whenever the back buffer was already blank and the front
  // still carried a grid.
  bool wants(const OsdComposeIn& in) const;

  // MSP draw-or-blank, GS collision reclaim, GS update, burn feeds. Leaves
  // buffer `idx` current in both layers.
  OsdComposeOut compose(int idx, const Surface& s, const OsdComposeIn& in);

 private:
  OsdRaster* raster_ = nullptr;
  std::unique_ptr<GsOverlay> gs_[3];
  BurnSink burn_;

  ShadowGrid shadow_[2], pre_, burn_shadow_;
  // TWO DIFFERENT QUESTIONS, and conflating them is the single mistake this
  // file has made twice:
  //   blanked_[i]    does BUFFER i hold a grid? Per buffer, because clearing
  //                  one says nothing about the other, and each is cleaned
  //                  in its own composition.
  //   screen_blank_  is the grid blank in the composition that was last
  //                  PUBLISHED -- i.e. what the pilot sees, and what the
  //                  burned DVR's index map was last told. Not per buffer:
  //                  it is a property of the screen.
  // wants() and the burn feed both need the second. Using the first made the
  // blank unreachable in an MSP-only run (back already blank, front lit) and
  // left the recording showing a grid the screen had dropped.
  bool blanked_[2] = {true, true};  // nothing drawn yet == already blank
  bool screen_blank_ = false;
  DirtyRect last_grid_{0, 0, 0, 0};  // where the grid was, for the blank feed
  bool have_screen_ = false;         // a complete MSP screen has ever arrived
  bool gs_laid_out_ = false;
  // Card count buffer i's overlay has reconciled, -1 == never. The card
  // block is the ONLY part of the GS layout that moves after layout(), and
  // a box it vacates is a box it clears and never draws again -- which
  // leaves a transparent hole in the MSP grid underneath while shadow_[i]
  // still says those cells are painted. Per buffer, because each overlay
  // reconciles a count change in its own composition.
  int gs_cards_[2] = {-1, -1};
  // The same reconciliation mark for the burn canvas, which sees EVERY
  // composition rather than every other one.
  int burn_gs_cards_ = -1;

  // The burn feed's cached mirror of the picture -- see the file comment.
  // Sized lazily to the surface compose() is given; resizing resets the
  // burn lineages, same as set_burn_sink().
  std::vector<uint32_t> burn_px_;
  Surface burn_canvas_;

  // Hoisted so a steady-state composition allocates nothing.
  // gs_reclaim_ holds the boxes the MSP collision actually forced GS to
  // redraw on the DISPLAY buffer; the burn canvas resolves its own
  // collisions with its own repaints.
  std::vector<DirtyRect> msp_write_, gs_rects_, burn_rects_, gs_reclaim_;
  std::vector<DirtyRect> burn_msp_write_, burn_gs_rects_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_COMPOSE_H_
