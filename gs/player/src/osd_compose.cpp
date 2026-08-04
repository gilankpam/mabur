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

DirtyRect OsdComposer::debug_grid_rect(int idx) const {
  return grid_of(shadow_[idx & 1].layout);
}

bool OsdComposer::wants(int idx, const OsdComposeIn& in) const {
  idx &= 1;
  if (gs_live() && in.snap && in.gs_dirty) return true;
  if (raster_ && in.screen) {
    if (in.msp_fresh) return true;
    // Per BUFFER, not a single latch: the blank has to reach both, and the
    // second one is reached on the composition after the first publish.
    if (have_screen_ && in.msp_stale && !blanked_[idx]) return true;
  }
  // Deliberately NOT a trigger: "this buffer's MSP grid is a screen behind".
  // A buffer that is behind is also a buffer nothing is publishing, so it is
  // not on screen; the composition that does publish it brings it forward.
  return false;
}

OsdComposeOut OsdComposer::compose(int idx, const Surface& s, const OsdComposeIn& in) {
  idx &= 1;
  OsdComposeOut out;
  if (!s.pixels || s.width <= 0 || s.height <= 0) return out;

  const bool gs_on = gs_live() && in.snap != nullptr;
  const bool msp_on = raster_ != nullptr && in.screen != nullptr;

  // ---------------- MSP layer ----------------
  // msp_write_ ends up holding what this composition wrote into buffer idx,
  // which is what the GS reclaim below needs. It is NOT the burn's rect set.
  msp_write_.clear();
  bool blanked_now = false;
  bool msp_drew = false;  // the draw branch ran, so there is a grid on this buffer
  if (msp_on) {
    if (in.msp_fresh) have_screen_ = true;
    if (have_screen_ && in.msp_stale) {
      if (!blanked_[idx]) {
        // The grid rect BEFORE clear(), which resets the shadow -- afterwards
        // there is no record of where the grid was.
        const DirtyRect grid = grid_of(shadow_[idx].layout);
        raster_->clear(s, &shadow_[idx]);
        blanked_[idx] = true;
        blanked_now = true;
        // clear() is scoped to the MSP GRID, not to MSP cells, so every GS
        // pixel inside that rect went with it.
        if (grid.w > 0 && grid.h > 0) msp_write_.push_back(grid);
        if (!blank_announced_) {
          blank_announced_ = true;
          out.announce_blank = true;
        }
      }
    } else if (have_screen_) {
      // Unconditional, not gated on msp_fresh. draw() is diffed against THIS
      // buffer's shadow, so on a GS-triggered composition it walks the cell
      // grid and writes nothing when the screen has not moved -- and writes
      // exactly the catch-up when it has. Without it a GS-only publish scans
      // out a buffer whose MSP grid is one screen old, i.e. the flight OSD
      // stepping backwards at the GS publish rate.
      if (gs_on) pre_ = shadow_[idx];
      out.msp_cells = raster_->draw(*in.screen, s, &shadow_[idx]);
      msp_drew = true;
      blanked_[idx] = false;
      blank_announced_ = false;
      // Exactly the cells draw() wrote: diff() shares draw()'s change
      // detection and full-repaint condition, against the same layout.
      if (gs_on) raster_->diff(*in.screen, s, &pre_, &msp_write_);
    }
  }

  // ---------------- GS layer ----------------
  gs_rects_.clear();
  bool geom_changed = false;
  if (gs_on) {
    // MSP drew second and clears a whole cell before blitting it, so any GS
    // pixel under a written cell is gone. The design says GS wins the
    // collision. At 1920x1080 with the shipped fonts the MSP grid covers
    // ~84% of the surface and every one of the 13 non-card GS fields
    // intersects it, so this is the common path, not an edge case.
    if (!msp_write_.empty())
      out.gs_reclaimed = gs_[idx]->repaint_intersecting(msp_write_.data(),
                                                        msp_write_.size(), s, nullptr);

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
                                                           msp_write_.size(), s, nullptr);
    }
  }

  // ---------------- burned DVR ----------------
  if (burn_) {
    burn_rects_.clear();
    if (msp_on && have_screen_) {
      if (blanked_now) {
        burn_shadow_ = ShadowGrid{};  // the grid really is blank now
        // Rect-scoped rather than the whole surface. This is NOT much of a
        // saving -- the grid is ~84% of a 1080p surface in "sharp" and ~99%
        // in "fill" -- the point is correctness: a whole-surface blank would
        // record the GS overlay as erased, which it is not.
        for (const DirtyRect& r : msp_write_) burn_rects_.push_back(r);
      } else if (!in.msp_stale) {
        raster_->diff(*in.screen, s, &burn_shadow_, &burn_rects_);
      }
    }
    if (gs_on) {
      // The reclaim wrote THIS BUFFER's last values, which are not the
      // lineage gs_[2] tracks -- it would report "nothing changed" over a
      // map that now disagrees with the screen. Restating every field is the
      // only granularity GsOverlay offers, and it is ~40 k px of
      // quantize_rects; see the comment on the reclaim above for how often.
      if (out.gs_reclaimed > 0) gs_[2]->invalidate();
      // invalidate() restates every ACTIVE field, which is not the same as
      // every field that MOVED: a vacated card row is inactive and would
      // never be restated, leaving the recording with the row the screen no
      // longer has.
      if (geom_changed) {
        for (const DirtyRect& r : gs_rects_) burn_rects_.push_back(r);
        // ...and the cells the repair repainted, which spill outside those
        // boxes. Only when the repair ran: on the blank path msp_write_ is
        // the grid rect, and it is already in burn_rects_ above.
        if (msp_drew)
          for (const DirtyRect& r : msp_write_) burn_rects_.push_back(r);
      }
      // Appends: update() does not clear `out`, precisely so GS and MSP
      // rects reach the encoder as one batch.
      gs_[2]->update(*in.snap, in.gs_stale, in.gs_ps, Surface{}, &burn_rects_);
    }
    if (!burn_rects_.empty()) burn_(s, burn_rects_.data(), burn_rects_.size());
  }

  out.published = true;
  return out;
}

}  // namespace maburplay
