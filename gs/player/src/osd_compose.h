#ifndef MABUR_PLAYER_OSD_COMPOSE_H_
#define MABUR_PLAYER_OSD_COMPOSE_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gs_layer.h"
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
// update on the pump loop (dvr-latency probe) against the
// ~0.1-2.2 ms the cached-memory bench projects -- enough to stall decode
// drain and blow vsync release deadlines on every burned recording. The
// canvas costs one extra cached draw per update and 4 bytes/px of heap;
// reads out of it run at the benched speed. It is single (not per DRM
// buffer), and it has ITS OWN size and ITS OWN trigger: set_burn_size()
// pins it to the recording's own dimensions (0x0, the default, follows
// the screen surface), and compose_burn()
// draws it independently of compose_screen() -- the burn's cadence is
// whatever the live recorder's housekeeping loop calls it at, not the
// screen's publish state.
class OsdComposer {
 public:
  // Rect-scoped index-map update for the burned DVR. Never called with a
  // null rect array: a full quantize is ~25 ms on the A55 and has no place
  // on the pump loop.
  using BurnSink = std::function<void(const Surface&, const DirtyRect*, size_t)>;

  // Null raster => this run has no MSP overlay. Not owned.
  void set_raster(OsdRaster* r) { raster_ = r; }
  // The MSP overlay switched off while the player runs: the raster is let
  // go at once (the caller may destroy it on return), and the grid it left
  // on each OSD buffer and on the burn canvas is erased by the next
  // composition into that buffer. wants() is true until the screen has
  // been composed once.
  void drop_raster();
  // The same for the GS overlay: all three layers go now (the all-or-none
  // rule), and the bar's area is erased from each buffer and the burn
  // canvas by its next composition. A set_gs() straight after replaces the
  // layers in place (a REC field re-sized for another dvr.target).
  void drop_gs_layers();
  // Both OSD buffers were replaced by fresh, blank ones (a rebuilt
  // presenter): forgets what each held, so the next composition into
  // either redraws both layers whole, and owes the screen one composition
  // (wants() true) even with no new input. The burn lineage is untouched:
  // the recorder is not the presenter's.
  void forget_screen();
  // All three or none: buffer 0, buffer 1, and the burn's lineage -- they
  // are three lineages of one overlay, not three overlays (see
  // make_gs_layer). Laid out at the SAME size unless a burn size is set
  // (set_burn_size), in which case gs_[2]'s size is its own.
  void set_gs(std::unique_ptr<GsLayer> b0, std::unique_ptr<GsLayer> b1,
              std::unique_ptr<GsLayer> burn);
  // Installing a sink resets the burn's OWN lineage -- burn_shadow_ (the
  // MSP grid), gs_[2] (the GS twin) and the canvas pixels -- because a
  // newly started BurnRecorder's index map is sized and blank, so a diff
  // against the DEAD recorder's state would leave most of the new
  // recording's overlay unpainted. The next composition therefore restates
  // everything -- a full cached draw plus quantize, single-digit ms. That
  // is the accepted startup-event cost of a deliberate button press, not a
  // cadence -- see CLAUDE.md on the 2 ms pump budget.
  void set_burn_sink(BurnSink s);

  // Sizes the burn canvas (and lays gs_[2] out) at w x h, independent of
  // the screen surface; 0x0 (the default) makes it follow the screen --
  // the size compose_screen() was last given, or nothing yet if
  // compose_screen() has never run. Resets the burn lineages, same shape
  // as set_burn_sink(). False (with *err) when the GS bar does not fit
  // w x h: the burn size is left cleared and gs_[2] re-laid at the screen
  // size, so the screen's GS overlay carries on and only the recording is
  // refused. Housekeeping, at recorder start: not a call the
  // per-iteration path makes.
  bool set_burn_size(int w, int h, std::string* err);

  // Lays out gs_[0] and gs_[1] at screen_w x screen_h, and gs_[2] at the
  // same size UNLESS a burn size is set (set_burn_size), in which case
  // gs_[2] keeps that size instead -- the burn's own geometry outlives a
  // screen resize. On failure drops all three (so gs_present() goes false)
  // and sets *err.
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
  // still carried a grid. Says nothing about the burn half: compose_burn()
  // runs on its own trigger, not this one.
  // Also true while the screen is owed a composition by drop_raster(),
  // drop_gs_layers() or forget_screen(), and while a running recording's
  // canvas still holds a dropped layer.
  bool wants(const OsdComposeIn& in) const;

  // The screen half of compose(): MSP draw-or-blank, GS collision reclaim,
  // GS update into buffer idx. Leaves buffer `idx` current in both layers.
  OsdComposeOut compose_screen(int idx, const Surface& s, const OsdComposeIn& in);

  // The burn half: the burn lineages into the canvas, then the burn sink.
  // Runs with or without compose_screen() this iteration -- the live
  // recorder's housekeeping calls this every iteration it is running,
  // independent of whether the screen's publish state wanted a
  // composition -- and a repeat of the same screen/GS sample draws
  // nothing and sinks nothing.
  void compose_burn(const OsdComposeIn& in);

  // compose() == compose_screen() then compose_burn(). Kept for the tests
  // and --gs-render, which still want both halves on one call.
  OsdComposeOut compose(int idx, const Surface& s, const OsdComposeIn& in);

 private:
  // Drops all three GS lineages: the all-or-none rule of gs_layout().
  void drop_gs();
  // Forgets which MSP cells buffer idx holds (keeping where its grid is,
  // which a later stale-blank needs) and invalidates its GS layer, so the
  // next composition into it redraws both layers whole.
  void forget_buffer(int idx);

  OsdRaster* raster_ = nullptr;
  std::unique_ptr<GsLayer> gs_[3];
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
  // The burn canvas's own blank latch -- screen_blank_'s counterpart, but
  // for the burn's own transition rather than the screen's. Needed because
  // compose_burn() may run on iterations compose_screen() does not (or vice
  // versa): borrowing screen_blank_ would mean a burn-only run never sees
  // the transition, and a screen-only run would flip it without the burn
  // canvas having changed at all.
  bool burn_blank_ = false;
  // The burn canvas's own size; 0x0 (the default) means "follow the
  // screen", via screen_w_/screen_h_ below. Set by set_burn_size().
  int burn_w_ = 0, burn_h_ = 0;
  // The last screen Surface compose_screen() was given -- what the burn
  // canvas falls back to sizing itself from when burn_w_/burn_h_ are 0x0.
  int screen_w_ = 0, screen_h_ = 0;
  // The last screen_w/screen_h gs_layout() was called with -- what gs_[2]
  // is re-laid out at when set_burn_size(0, 0, ...) clears the burn size
  // back to following the screen.
  int gs_screen_w_ = 0, gs_screen_h_ = 0;
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

  // Areas a dropped layer left on each OSD buffer and on the burn canvas,
  // zeroed by the next composition into that buffer (or the canvas), which
  // then redraws what remains whole. screen_owed_: the published buffer
  // still shows a dropped layer, or a fresh buffer shows nothing yet.
  std::vector<DirtyRect> erase_[2], burn_erase_;
  bool screen_owed_ = false;

  // The burn feed's cached mirror of the picture -- see the file comment.
  // Sized to burn_w_/burn_h_ when set, else lazily to the screen surface
  // compose_screen() is given; resizing resets the burn lineages, same as
  // set_burn_sink().
  std::vector<uint32_t> burn_px_;
  Surface burn_canvas_;
  // (Re)points burn_canvas_/burn_px_ at w x h (0x0 leaves it unallocated --
  // nothing has sized it yet) and resets everything that follows its
  // geometry: burn_shadow_, gs_[2]'s reconciliation mark, burn_blank_, and
  // the pixels themselves. Shared by set_burn_sink(), set_burn_size(), and
  // compose_burn()'s own resize-on-mismatch check.
  void realloc_burn_canvas(int w, int h);

  // Hoisted so a steady-state composition allocates nothing.
  // gs_reclaim_ holds the boxes the MSP collision actually forced GS to
  // redraw on the DISPLAY buffer; the burn canvas resolves its own
  // collisions with its own repaints.
  std::vector<DirtyRect> msp_write_, gs_rects_, burn_rects_, gs_reclaim_;
  std::vector<DirtyRect> burn_msp_write_, burn_gs_rects_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_COMPOSE_H_
