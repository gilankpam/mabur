#include "osd_compose.h"

#include <algorithm>

namespace maburplay {

namespace {

DirtyRect grid_of(const OsdLayout& l) {
  if (l.draw_w <= 0 || l.draw_h <= 0) return DirtyRect{0, 0, 0, 0};
  return DirtyRect{l.origin_x, l.origin_y, l.cols * l.draw_w, l.rows * l.draw_h};
}

// Makes the shadow forget every cell `r` touches, so the next draw() repaints
// them. The stored value is the COMPLEMENT of the screen's own, which is the
// only value guaranteed to compare unequal whatever the cell actually holds.
//
// This is the one thing a caller of OsdRaster occasionally needs and the
// class does not offer: "this region of the buffer no longer holds what you
// think, but the rest does". Clearing the whole ShadowGrid would work and
// would repaint the entire grid instead of a card row.
void forget_cells(ShadowGrid& sh, const mabur::MspScreen& scr, const DirtyRect& r) {
  const OsdLayout& l = sh.layout;
  if (l.draw_w <= 0 || l.draw_h <= 0 || r.w <= 0 || r.h <= 0) return;
  if (l.cols != scr.cols() || l.rows != scr.rows()) return;
  if (sh.cells.size() != (size_t)scr.rows() * (size_t)scr.cols()) return;
  const int c0 = std::max(0, (r.x - l.origin_x) / l.draw_w);
  const int r0 = std::max(0, (r.y - l.origin_y) / l.draw_h);
  const int c1 = std::min(l.cols - 1, (r.x + r.w - 1 - l.origin_x) / l.draw_w);
  const int r1 = std::min(l.rows - 1, (r.y + r.h - 1 - l.origin_y) / l.draw_h);
  for (int row = r0; row <= r1; ++row)
    for (int col = c0; col <= c1; ++col)
      sh.cells[(size_t)row * l.cols + col] = (uint16_t)~scr.cell(row, col);
}

}  // namespace

void OsdComposer::set_gs(std::unique_ptr<GsOverlay> b0, std::unique_ptr<GsOverlay> b1,
                         std::unique_ptr<GsOverlay> burn) {
  gs_[0] = std::move(b0);
  gs_[1] = std::move(b1);
  gs_[2] = std::move(burn);
  gs_laid_out_ = false;
}

void OsdComposer::set_burn_sink(BurnSink s) {
  burn_ = std::move(s);
  burn_shadow_ = ShadowGrid{};
  if (gs_[2]) gs_[2]->invalidate();
  // Blank the canvas too: pixels from the previous recording outside the
  // fresh restate's rects would otherwise linger in it, and a later rect
  // that happens to cover them would quantize stale content into the new
  // recording's map.
  std::fill(burn_px_.begin(), burn_px_.end(), 0u);
  burn_gs_cards_ = -1;
}

bool OsdComposer::gs_layout(int screen_w, int screen_h, std::string* err) {
  if (!gs_[0]) return false;
  for (auto& ov : gs_) {
    if (!ov->layout(screen_w, screen_h, err)) {
      // All three or none: a run with two of them laid out would compose a
      // buffer against a shadow that was never given any geometry.
      for (auto& o : gs_) o.reset();
      gs_laid_out_ = false;
      return false;
    }
  }
  gs_laid_out_ = true;
  return true;
}

bool OsdComposer::wants(const OsdComposeIn& in) const {
  if (gs_live() && in.snap && in.gs_dirty) return true;
  if (raster_ && in.screen) {
    if (in.msp_fresh) return true;
    // Until the blank has been PUBLISHED. Gating this on the back buffer's
    // own blanked_ flag deadlocked: with one screen ever drawn and then
    // silence, the back buffer has never held a grid, so it is already
    // blank, so nothing is composed, so nothing is published, and the front
    // buffer shows the frozen flight OSD past osd.stale_ms forever. Exactly
    // the configuration shipping today (MSP-only, no second publisher).
    // Settles after one composition: compose() sets screen_blank_.
    if (have_screen_ && in.msp_stale && !screen_blank_) return true;
  }
  // Deliberately NOT a trigger: "the other buffer's MSP grid is a screen
  // behind". A buffer that is behind is also a buffer nothing is publishing,
  // so it is not on screen; the composition that does publish it brings it
  // forward.
  return false;
}

OsdComposeOut OsdComposer::compose(int idx, const Surface& s, const OsdComposeIn& in) {
  idx &= 1;
  OsdComposeOut out;
  if (!s.pixels || s.width <= 0 || s.height <= 0) return out;

  const bool gs_on = gs_live() && in.snap != nullptr;
  const bool msp_on = raster_ != nullptr && in.screen != nullptr;
  if (msp_on && in.msp_fresh) have_screen_ = true;
  // What the grid looks like on the composition this call produces -- a
  // property of the picture, not of buffer idx, so it is what both the
  // blank trigger and the burn feed are entitled to look at. Buffer idx may
  // already agree with it (blanked_[idx]) and still need publishing, which
  // is the case both N1 and N2 fell through.
  const bool grid_blank = msp_on && have_screen_ && in.msp_stale;

  // ---------------- MSP layer ----------------
  // msp_write_ ends up holding what this composition wrote into buffer idx,
  // which is what the GS reclaim below needs. It is NOT the burn's rect set.
  msp_write_.clear();
  bool msp_drew = false;  // the draw branch ran, so there is a grid on this buffer
  if (msp_on) {
    if (have_screen_ && in.msp_stale) {
      if (!blanked_[idx]) {
        // The grid rect BEFORE clear(), which resets the shadow -- afterwards
        // there is no record of where the grid was.
        const DirtyRect grid = grid_of(shadow_[idx].layout);
        raster_->clear(s, &shadow_[idx]);
        blanked_[idx] = true;
        // clear() is scoped to the MSP GRID, not to MSP cells, so every GS
        // pixel inside that rect went with it.
        if (grid.w > 0 && grid.h > 0) msp_write_.push_back(grid);
      }
    } else if (have_screen_) {
      // Unconditional, not gated on msp_fresh. draw() is diffed against THIS
      // buffer's shadow, so on a GS-triggered composition it walks the cell
      // grid and writes nothing when the screen has not moved -- and writes
      // exactly the catch-up when it has. Without it a GS-only publish scans
      // out a buffer whose MSP grid is one screen old, i.e. the flight OSD
      // stepping backwards at the GS publish rate.
      if (gs_on) pre_ = shadow_[idx];
      raster_->draw(*in.screen, s, &shadow_[idx]);
      msp_drew = true;
      blanked_[idx] = false;
      // Remembered because a later blank may find this buffer ALREADY clear
      // and still owe the burn the rect the grid used to occupy.
      last_grid_ = grid_of(shadow_[idx].layout);
      // Exactly the cells draw() wrote: diff() shares draw()'s change
      // detection and full-repaint condition, against the same layout.
      if (gs_on) raster_->diff(*in.screen, s, &pre_, &msp_write_);
    }
  }

  // ---------------- GS layer ----------------
  gs_rects_.clear();
  gs_reclaim_.clear();
  bool geom_changed = false;
  if (gs_on) {
    // MSP drew second and clears a whole cell before blitting it, so any GS
    // pixel under a written cell is gone. The design says GS wins the
    // collision.
    //
    // How often that bites depends on the canvas, and on an HD grid the
    // answer is "constantly": a 50x18 "sharp" grid at 1080p covers ~84% of
    // the surface and reaches EVERY active GS box -- 33/33 with four cards,
    // and 140 of its 900 cells (16%) sit on one. So this is not an edge
    // case, it is the steady state: one changed cell -- a flight timer
    // ticking once a second -- lands on a GS field roughly one time in six.
    // Measured on the same fixture the coexist tests use: 137/900 cells,
    // 28/28 boxes with three cards. An SD 30x16 grid is nearer 45% and may
    // miss the corners entirely, in which case gs_reclaimed stays 0 and none
    // of the work below runs.
    // The reclaimed BOXES are collected, not just counted: the burn feed
    // below restates exactly them, and "exactly them" is the difference
    // between a few thousand pixels and every active field.
    if (!msp_write_.empty())
      out.gs_reclaimed = gs_[idx]->repaint_intersecting(msp_write_.data(),
                                                        msp_write_.size(), s, &gs_reclaim_);

    const int cards = (int)std::min<size_t>(in.snap->cards.size(), (size_t)kMaxCards);
    geom_changed = cards != gs_cards_[idx];
    gs_cards_[idx] = cards;

    // Diffed against THIS buffer, so afterwards buffer idx's GS region is
    // the current state for EVERY field, not only the ones just drawn. The
    // burn feed below relies on that: it quantizes rects this update may not
    // have touched. The rect list is wanted only when the card block moved,
    // for the repair below.
    out.gs_drawn = gs_[idx]->update(*in.snap, in.gs_stale, in.gs_ps, s,
                                    geom_changed ? &gs_rects_ : nullptr);

    // A changed card count is the only thing that moves a GS box after
    // layout(), and GsOverlay erases a box it is about to vacate. That erase
    // goes straight through to the MSP glyphs underneath, and shadow_[idx]
    // still records those cells as painted -- so without this the flight OSD
    // keeps a transparent hole the shape of a card row until whatever text
    // was there happens to change. Repaint just those cells and let GS win
    // them back; the fields that stayed put are untouched.
    if (geom_changed && msp_drew) {
      for (const DirtyRect& r : gs_rects_) forget_cells(shadow_[idx], *in.screen, r);
      pre_ = shadow_[idx];
      raster_->draw(*in.screen, s, &shadow_[idx]);
      // The repair repaints CELLS, and a cell is 38x60 px at 1080p: it spills
      // out of the vacated box and over whatever GS field is next door. So
      // the reclaim goes over what the repair actually wrote, not over the
      // rects that prompted it -- reclaiming the prompts leaves the
      // neighbours buried.
      raster_->diff(*in.screen, s, &pre_, &msp_write_);
      if (!msp_write_.empty())
        out.gs_reclaimed += gs_[idx]->repaint_intersecting(msp_write_.data(),
                                                           msp_write_.size(), s, &gs_reclaim_);
    }
  }

  // ---------------- burned DVR ----------------
  // Composed into the burn CANVAS -- the composer's own cached mirror --
  // never read back out of `s`: the DRM buffers are write-combine mappings
  // and quantize reading one was the burned-recording latency regression
  // (see osd_compose.h). The canvas runs the SAME algorithm as the display
  // path above, against the burn's own lineages (burn_shadow_, gs_[2]), so
  // it renders the same picture the screen shows: MSP first, GS repainted
  // over any cell that hit a field box, vacated card rows erased and the
  // grid repaired underneath them.
  if (burn_) {
    // (Re)size the canvas to THIS surface. A resize orphans the burn
    // lineages (their geometry followed the old size), so they reset and
    // the next feeds below restate everything -- same shape as a fresh
    // set_burn_sink().
    if (!burn_canvas_.pixels || burn_canvas_.width != s.width ||
        burn_canvas_.height != s.height) {
      burn_px_.assign((size_t)s.width * s.height, 0u);
      burn_canvas_ = Surface{burn_px_.data(), s.width, s.height, s.width};
      burn_shadow_ = ShadowGrid{};
      if (gs_[2]) gs_[2]->invalidate();
      burn_gs_cards_ = -1;
    }
    burn_rects_.clear();
    burn_msp_write_.clear();
    bool burn_msp_drew = false;
    if (msp_on && have_screen_) {
      if (grid_blank) {
        // On the TRANSITION, not per buffer: the canvas follows the screen,
        // so one clear on the episode's first composition is the whole
        // blank. clear() blanks exactly the grid the shadow records and
        // invalidates it; last_grid_ is that rect, pushed through
        // burn_msp_write_ so the GS reclaim below restores any field boxes
        // the grid overlapped.
        if (!screen_blank_) {
          raster_->clear(burn_canvas_, &burn_shadow_);
          if (last_grid_.w > 0 && last_grid_.h > 0) burn_msp_write_.push_back(last_grid_);
        }
      } else {
        raster_->draw(*in.screen, burn_canvas_, &burn_shadow_, &burn_msp_write_);
        burn_msp_drew = true;
      }
    }
    if (gs_on) {
      const int cards = (int)std::min<size_t>(in.snap->cards.size(), (size_t)kMaxCards);
      const bool burn_geom_changed = cards != burn_gs_cards_;
      burn_gs_cards_ = cards;
      // GS wins the collision in the canvas exactly as on screen: repaint
      // every field box an MSP-written cell (or the blank's grid rect)
      // touched. The repaint emits its boxes into burn_rects_ -- they are
      // quantize work whether or not update() below touches them again.
      if (!burn_msp_write_.empty())
        gs_[2]->repaint_intersecting(burn_msp_write_.data(), burn_msp_write_.size(),
                                     burn_canvas_, &burn_rects_);
      burn_gs_rects_.clear();
      gs_[2]->update(*in.snap, in.gs_stale, in.gs_ps, burn_canvas_, &burn_gs_rects_);
      // A changed card count erased boxes straight through to the canvas's
      // MSP glyphs, same hole the display path repairs above: forget those
      // cells, redraw them, and let GS win back whatever the repair spilled
      // onto -- cell rects are bigger than the boxes that prompted them.
      if (burn_geom_changed && burn_msp_drew) {
        for (const DirtyRect& r : burn_gs_rects_) forget_cells(burn_shadow_, *in.screen, r);
        const size_t repair_at = burn_msp_write_.size();
        raster_->draw(*in.screen, burn_canvas_, &burn_shadow_, &burn_msp_write_);
        if (burn_msp_write_.size() > repair_at)
          gs_[2]->repaint_intersecting(burn_msp_write_.data() + repair_at,
                                       burn_msp_write_.size() - repair_at, burn_canvas_,
                                       &burn_rects_);
      }
      for (const DirtyRect& r : burn_gs_rects_) burn_rects_.push_back(r);
    }
    for (const DirtyRect& r : burn_msp_write_) burn_rects_.push_back(r);
    // One call, one batch. Rect COUNT is worth a thought and not a worry:
    // the worst card-count transition produces ~106 rects against
    // BurnRecorder's 128 cap, and exceeding the cap does not cost a
    // quantize() on this loop -- it sets osd_full_pending, which moves a
    // ~2 MB map copy onto the recorder thread. The 25 ms path needs
    // rects == nullptr, which this never passes. Two things do accumulate
    // toward the cap though: successive set_osd() calls before the recorder
    // takes them, and OsdRaster::draw() on an adversarial screen
    // (alternating columns is legal MSP and yields 483 rects on its own).
    if (!burn_rects_.empty()) burn_(burn_canvas_, burn_rects_.data(), burn_rects_.size());
  }

  // One announcement per episode, on the transition rather than on a latch:
  // the composition that reaches the screen is the one worth logging.
  out.announce_blank = grid_blank && !screen_blank_;
  screen_blank_ = grid_blank;
  out.published = true;
  return out;
}

}  // namespace maburplay
