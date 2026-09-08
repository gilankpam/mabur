#include "gs_compact.h"

#include <algorithm>
#include <cmath>

#include "gs_draw.h"
#include "gs_font.h"

namespace maburplay {
namespace {

// Design geometry at 1080p. Deliberately tighter than the essential
// overlay's 96/54 safe inset: that inset reserves the corners a broadcast
// safe area would clip, and this line is a single strip along the bottom
// whose whole point is to fit the most figures at the largest readable
// size. 32/24 still clears the panel bezel on the GS's own display.
constexpr int kInsetX = 32, kInsetY = 24;

// The eleven items, in draw order, and the row each one lands on. This IS
// the reading order on the glass: row 0 is the radio (what the link is
// doing), row 1 the picture (what came out of it). Split by source rather
// than by width -- an even split would put `snr` next to `bitrate`, which
// reads as one continuous line of unrelated figures.
//
// Row 1 is the WIDER of the two (69 worst-case characters plus 5 gaps,
// against row 0's 63 plus 4), so it is what decides the type size. Moving
// an item between rows changes the size the whole bar renders at -- which
// is one of the reasons REC is in the corner instead (see gs_compact.h).
constexpr GsBarField kOrder[] = {
    GsBarField::kCh,  GsBarField::kMcs, GsBarField::kAir,     GsBarField::kRssi,
    GsBarField::kSnr, GsBarField::kRec, GsBarField::kBitrate, GsBarField::kRes,
    GsBarField::kFps, GsBarField::kJit, GsBarField::kLat,     GsBarField::kLoss,
};
constexpr int kRow[] = {0, 0, 0, 0, 0, GsCompactBar::kCorner, 1, 1, 1, 1, 1, 1};
static_assert(sizeof(kOrder) / sizeof(kOrder[0]) == (size_t)GsBarField::kCount,
              "every field must appear exactly once in the draw order");
static_assert(sizeof(kRow) / sizeof(kRow[0]) == (size_t)GsBarField::kCount,
              "every field needs a row");

// Gap between the two rows, on top of the cell height. The boxes span the
// padded CELL, so stacking by glyph_h alone already separates them; this is
// margin, not clearance.
constexpr int kRowGap = 4;

// Top inset for the corner-anchored recording indicator. Deeper than the
// bar's own 24 px bottom inset: the bottom strip is a deliberate band of
// instrumentation, while this one sits alone in the picture and wants the
// clearance a title-safe area gives it.
constexpr int kInsetTop = 40;

bool intersects(const DirtyRect& a, const DirtyRect& b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

DirtyRect union_of(const DirtyRect& a, const DirtyRect& b) {
  if (a.w <= 0 || a.h <= 0) return b;
  if (b.w <= 0 || b.h <= 0) return a;
  const int x0 = std::min(a.x, b.x), y0 = std::min(a.y, b.y);
  const int x1 = std::max(a.x + a.w, b.x + b.w), y1 = std::max(a.y + a.h, b.y + b.h);
  return DirtyRect{x0, y0, x1 - x0, y1 - y0};
}

// Per-side shadow pad an atlas bakes in (gs_font.h: glyph_w/glyph_h include
// it, advance_x does not). A field's box spans the padded CELL -- it has to,
// because clear_region must erase the shadow with the glyph -- so the box is
// this much wider than its text on each side.
int pad_h(const MaskAtlas* a) { return (a->glyph_w - a->advance_x) / 2; }

// Space between two items. One character of advance reads as the single
// space the format string implies, but it must also keep two padded boxes
// apart: at 2*pad the boxes ABUT, and any less and one item's clear erases
// its neighbour's last column. The +2 buys a margin the no-overlap test can
// actually assert on.
int item_gap(const MaskAtlas* a) {
  return std::max(a->advance_x, (a->glyph_w - a->advance_x) + 2);
}

// Plain ASCII integer, hyphen-minus for negatives. NOT fmt_signed_int: its
// U+2212 is a typographic refinement this line does without, and the ASCII
// form is a character narrower.
std::string ascii_int(double v) { return fmt_int(v); }

std::string join_cards(const std::string& label, const GsSnapshot& snap,
                       bool snr) {
  const int n = std::min((int)snap.cards.size(), kMaxCards);
  if (n <= 0) return label + ":--";
  std::string out = label + ":";
  for (int i = 0; i < n; ++i) {
    if (i) out += "/";
    const GsCard& c = snap.cards[(size_t)i];
    if (!c.heard) {
      out += "--";
      continue;
    }
    if (snr) {
      out += c.snr_db ? ascii_int(std::clamp(*c.snr_db, -99.0, 999.0)) : "--";
    } else {
      out += c.rssi_dbm ? ascii_int(std::clamp(*c.rssi_dbm, -999.0, 999.0)) : "--";
    }
  }
  return out;
}

std::string repeat_joined(const std::string& label, const char* per, int n) {
  if (n <= 0) return label + ":--";
  std::string out = label + ":";
  for (int i = 0; i < n; ++i) {
    if (i) out += "/";
    out += per;
  }
  return out;
}

}  // namespace

std::string GsCompactBar::worst_case(GsBarField id, int n_cards) {
  const int n = std::clamp(n_cards, 0, kMaxCards);
  switch (id) {
    // "ch:--" is narrower than the numeric form, so the number sizes the box.
    case GsBarField::kCh:      return "ch:999";
    // Here the em-dash-free missing form is the WIDER one ("mcs:--" beats
    // "mcs:9"), which is exactly why every box is sized from an explicit
    // worst case rather than from whatever the live value happens to be.
    case GsBarField::kMcs:     return "mcs:--";
    case GsBarField::kAir:     return "air:100%";
    case GsBarField::kRssi:    return repeat_joined("rssi", "-999", n);
    case GsBarField::kSnr:     return repeat_joined("snr", "-99", n);
    case GsBarField::kBitrate: return "bitrate:999.9";
    case GsBarField::kRes:     return "res:9999x9999";
    case GsBarField::kFps:     return "fps:999";
    case GsBarField::kJit:     return "jit:999.9";
    case GsBarField::kLat:     return "lat:999/999";
    case GsBarField::kLoss:    return "loss:100.0/100.0";
    // Both live states are eleven glyphs wide ("● REC 99:59" and
    // "● REC FAULT"); fmt_clock saturates at 99:59 so neither can grow.
    // Armed renders nothing and leaves the box blank -- the same
    // fixed-width reservation every other item makes, and the same thing
    // the essential overlay does.
    case GsBarField::kRec:     return "● REC FAULT";
    case GsBarField::kCount:   break;
  }
  return "";
}

int GsCompactBar::row_of(GsBarField id) {
  for (int i = 0; i < (int)GsBarField::kCount; ++i)
    if (kOrder[i] == id) return kRow[i];
  return 0;
}

int GsCompactBar::worst_row_width(const MaskAtlas& a, int row, int n_cards) {
  const int gap = item_gap(&a);
  int w = 0, n = 0;
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    if (kRow[i] != row) continue;  // kCorner belongs to no row
    if (n++) w += gap;
    w += text_width(a, worst_case(kOrder[i], n_cards).c_str());
  }
  // The first box's left pad and the last one's right.
  return n ? w + 2 * pad_h(&a) : 0;
}

bool GsCompactBar::layout(int screen_w, int screen_h, std::string* err) {
  laid_out_ = false;
  atlas_ = nullptr;
  for (Field& f : fields_) f = Field{};
  bounds_ = DirtyRect{0, 0, 0, 0};
  // Same reason as GsOverlay's: a re-layout must force the next update()
  // to reconcile the card count again, even if it reports what it did
  // before this layout() threw the boxes away.
  n_cards_ = -1;

  if (screen_w <= 0 || screen_h <= 0) {
    if (err) *err = "gs osd: bad screen size";
    return false;
  }
  const double scale = (double)screen_h / 1080.0;
  const int inset_x = (int)(kInsetX * scale + 0.5);
  const int inset_y = (int)(kInsetY * scale + 0.5);
  const int avail_w = screen_w - 2 * inset_x;

  // Largest baked size whose WORST-CASE rows both fit. Unlike the essential
  // overlay this asks for no particular design size: the bar has one type
  // size and its only constraints are the width of its widest row and the
  // height of the stack, so "the biggest that fits" is the whole rule --
  // and it is what makes the bar render at a readable size on a 720p panel
  // and a bigger one on a 4K panel without a per-resolution table.
  const MaskAtlas* best = nullptr;
  int best_px = 0;
  for (int px = 1; px <= 512; ++px) {
    const MaskAtlas* a = font_.atlas(px);
    if (!a || a->advance_x <= 0) continue;
    bool fits = true;
    for (int row = 0; row < kRows; ++row)
      if (worst_row_width(*a, row, kMaxCards) > avail_w) fits = false;
    if (!fits) continue;
    // The stack also has to sit ABOVE the bottom inset without running off
    // the top of a short surface.
    const int block_h = kRows * a->glyph_h + (kRows - 1) * (int)(kRowGap * scale + 0.5);
    if (block_h + inset_y > screen_h) continue;
    if (a->px > best_px) { best_px = a->px; best = a; }
  }
  if (!best) {
    if (err) *err = "gs osd: compact bar does not fit at this screen size";
    return false;
  }

  atlas_ = best;
  screen_w_ = screen_w;
  inset_x_ = inset_x;
  gap_ = item_gap(best);
  // The cell's descender sits below the baseline, so the baseline has to
  // rise by that much for the BOX -- shadow pad included -- to clear the
  // bottom inset. Rows stack by a full glyph_h plus a margin: a shorter
  // pitch would have row 0's padded box overlap row 1's, and an overlap is
  // one row's clear erasing the other's ink (the mistake gs_overlay.cpp
  // documents at kRung's placement).
  const int row_pitch = best->glyph_h + (int)(kRowGap * scale + 0.5);
  baseline_y_[kRows - 1] = screen_h - inset_y - (best->glyph_h - best->baseline);
  for (int row = kRows - 2; row >= 0; --row)
    baseline_y_[row] = baseline_y_[row + 1] - row_pitch;
  // The corner item hangs off the TOP inset instead, so its box's top edge
  // -- shadow pad included -- sits exactly on it.
  corner_baseline_ = (int)(kInsetTop * scale + 0.5) + best->baseline;
  // Reserve the worst case until the first snapshot says how many cards
  // there really are. Nothing draws before then (update() reconciles the
  // count first), but bounds() is legitimately asked for in between, and it
  // must cover everything the bar can ever touch.
  place_(kMaxCards);
  laid_out_ = true;
  return true;
}

void GsCompactBar::place_(int n_cards) {
  if (!atlas_) return;
  const int pad = pad_h(atlas_);
  int w[(size_t)GsBarField::kCount];
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    f_(kOrder[i]).active = true;
    w[i] = text_width(*atlas_, worst_case(kOrder[i], n_cards).c_str());
  }

  // The corner item: right-flushed at the top inset, reserving the shadow
  // pad so its box does not overhang the inset it is flush against.
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    if (kRow[i] != kCorner) continue;
    Field& f = f_(kOrder[i]);
    f.pen_x = screen_w_ - inset_x_ - w[i] - pad;
    f.baseline_y = corner_baseline_;
    f.box = DirtyRect{f.pen_x - pad, corner_baseline_ - atlas_->baseline,
                      w[i] + 2 * pad, atlas_->glyph_h};
    bounds_ = union_of(bounds_, f.box);
  }

  // Each row is centred on its OWN width, not on the block's: the two rows
  // hold different amounts of text, and left-aligning the shorter one to
  // the longer one's edge reads as a layout that slipped.
  for (int row = 0; row < kRows; ++row) {
    int total = 0, n = 0;
    for (int i = 0; i < (int)GsBarField::kCount; ++i) {
      if (kRow[i] != row) continue;
      total += w[i] + (n++ ? gap_ : 0);
    }
    if (n == 0) continue;
    // Centred, but never past the inset: with kMaxCards a row can be wide
    // enough that centring and the inset disagree, and the inset wins.
    int pen = (screen_w_ - total) / 2;
    if (pen < inset_x_ + pad) pen = inset_x_ + pad;
    for (int i = 0; i < (int)GsBarField::kCount; ++i) {
      if (kRow[i] != row) continue;
      Field& f = f_(kOrder[i]);
      f.pen_x = pen;
      f.baseline_y = baseline_y_[row];
      f.box = DirtyRect{pen - pad, baseline_y_[row] - atlas_->baseline,
                        w[i] + 2 * pad, atlas_->glyph_h};
      bounds_ = union_of(bounds_, f.box);
      pen += w[i] + gap_;
    }
  }
}

GsCompactBar::FieldState GsCompactBar::state_of_(const GsSnapshot& snap,
                                                 bool stale,
                                                 const GsPlayerState& ps,
                                                 GsBarField id) const {
  FieldState st;
  // The one styling rule this line has: LINK-sourced items dim while the
  // sideport is quiet and hold their last value; player-measured ones stay
  // lit because they are current by construction. Without the dim, a dead
  // sideport leaves a line of entirely plausible frozen numbers.
  const uint32_t link = stale ? tok::kTextLabel : tok::kTextPrimary;
  st.rgb = tok::kTextPrimary;

  // Every clamp below keeps the rendered string inside worst_case(id) --
  // see the note on that function. gs_snapshot.cpp rejects non-finite and
  // non-numeric JSON but bounds no magnitude, so an unclamped air_pct of
  // 1e300 would draw a ~300-character string past its own box and never be
  // erased again.
  switch (id) {
    case GsBarField::kCh:
      st.rgb = link;
      st.text = snap.channel
                    ? "ch:" + ascii_int(std::clamp(*snap.channel, 0, 999))
                    : "ch:--";
      break;
    case GsBarField::kMcs:
      st.rgb = link;
      st.text = snap.mcs ? "mcs:" + ascii_int(std::clamp(*snap.mcs, 0, 9))
                         : "mcs:--";
      break;
    case GsBarField::kAir:
      st.rgb = link;
      st.text = snap.air_pct
                    ? "air:" + fmt_int(std::clamp(*snap.air_pct, 0.0, 100.0)) + "%"
                    : "air:--";
      break;
    case GsBarField::kRssi:
      st.rgb = link;
      st.text = join_cards("rssi", snap, /*snr=*/false);
      break;
    case GsBarField::kSnr:
      st.rgb = link;
      st.text = join_cards("snr", snap, /*snr=*/true);
      break;
    case GsBarField::kBitrate:
      // Player-measured: AU bytes off the ring, not an encoder setpoint.
      st.text = "bitrate:" + fmt_one_dp(std::clamp(ps.mbps, 0.0, 999.9));
      break;
    case GsBarField::kRes:
      st.text = (ps.vid_w > 0 && ps.vid_h > 0)
                    ? "res:" + ascii_int(std::clamp(ps.vid_w, 0, 9999)) + "x" +
                          ascii_int(std::clamp(ps.vid_h, 0, 9999))
                    : "res:--";
      break;
    case GsBarField::kFps:
      st.text = "fps:" + fmt_int(std::clamp(ps.fps, 0.0, 999.0));
      break;
    case GsBarField::kJit:
      st.text = "jit:" + fmt_one_dp(std::clamp(ps.jitter_ms, 0.0, 999.9));
      break;
    case GsBarField::kLat:
      // p50 then p99, the same order the essential overlay stacks them in.
      // No ~ marker for the relative case: this line has no room for a
      // qualifier, and lat_valid already gates the only state where the
      // number would be a fabrication.
      st.text = ps.lat_valid
                    ? "lat:" +
                          fmt_int(std::clamp((double)ps.lat_p50_e2e_ms, 0.0, 999.0)) +
                          "/" +
                          fmt_int(std::clamp((double)ps.lat_e2e_ms, 0.0, 999.0))
                    : "lat:--/--";
      break;
    case GsBarField::kLoss:
      st.rgb = link;
      st.text = "loss:";
      st.text += snap.pre_loss_pct
                     ? fmt_one_dp(std::clamp(*snap.pre_loss_pct, 0.0, 100.0))
                     : "--";
      st.text += "/";
      st.text += snap.post_loss_pct
                     ? fmt_one_dp(std::clamp(*snap.post_loss_pct, 0.0, 100.0))
                     : "--";
      break;
    case GsBarField::kRec:
      // Byte-for-byte the essential overlay's kRec, deliberately: one
      // aircraft, one recording indicator. Never dimmed -- the recorder is
      // the player's own business and says nothing about the link.
      switch (ps.rec.kind) {
        case RecState::Kind::kArmed:
          // Nothing at all. An idle placeholder clock is permanent clutter,
          // and "no REC" already reads as "not recording".
          break;
        case RecState::Kind::kRecording:
          st.text = std::string(kDotFilled) + " REC " + fmt_clock(ps.rec.elapsed_s);
          st.rgb = tok::kTextPrimary;
          st.aux = 1;  // draw_field_ paints the dot in kStatusRec
          break;
        case RecState::Kind::kFault:
          st.text = std::string(kDotFilled) + " REC FAULT";
          st.rgb = tok::kStatusCaution;
          break;
      }
      break;
    case GsBarField::kCount:
      break;
  }
  return st;
}

std::string GsCompactBar::debug_field_text(const GsSnapshot& snap, bool stale,
                                           const GsPlayerState& ps,
                                           GsBarField id) const {
  return state_of_(snap, stale, ps, id).text;
}

int GsCompactBar::debug_atlas_px() const { return atlas_ ? atlas_->px : 0; }

DirtyRect GsCompactBar::debug_field_box(GsBarField id) const {
  return f_(id).box;
}

void GsCompactBar::draw_field_(GsBarField id, const FieldState& st,
                               const Surface& s) {
  Field& f = f_(id);
  if (!atlas_ || !f.active) return;
  clear_region(s, f.box);
  if (st.text.empty()) return;  // cleared above; nothing more to draw
  if (st.aux == 1) {
    // The recording dot takes kStatusRec while the rest of the item takes
    // the field colour.
    const int adv = draw_text(s, *atlas_, f.pen_x, f.baseline_y, kDotFilled,
                              tok::kStatusRec);
    draw_text(s, *atlas_, f.pen_x + adv, f.baseline_y,
              st.text.c_str() + std::string(kDotFilled).size(), st.rgb);
    return;
  }
  draw_text(s, *atlas_, f.pen_x, f.baseline_y, st.text.c_str(), st.rgb);
}

int GsCompactBar::update(const GsSnapshot& snap, bool stale,
                         const GsPlayerState& ps, const Surface& s,
                         std::vector<DirtyRect>* out) {
  if (!laid_out_) return 0;
  int drawn = 0;

  // A changed card count changes the width of two items and therefore the
  // x of every item after them -- and, because each row is centred, of
  // every item before them on that row too. Both items live on row 0, but
  // erase EVERYTHING anyway: it costs one extra clear of a row that did not
  // move and removes the standing question of whether the split still keeps
  // the widths apart. A field's own next draw only ever clears its NEW box.
  const int n = std::min((int)snap.cards.size(), kMaxCards);
  if (n != n_cards_) {
    if (n_cards_ >= 0) {
      for (int i = 0; i < (int)GsBarField::kCount; ++i) {
        // The corner item is anchored, not centred: a card count change
        // cannot move it, so erasing it would cost a redraw for nothing.
        if (kRow[i] == kCorner) continue;
        Field& f = f_(kOrder[i]);
        if (!f.active) continue;
        clear_region(s, f.box);
        if (out) out->push_back(f.box);
        ++drawn;
      }
    }
    place_(n);
    for (Field& f : fields_) f.valid = false;
    n_cards_ = n;
  }

  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    const GsBarField id = kOrder[i];
    Field& f = f_(id);
    if (!f.active) continue;
    const FieldState st = state_of_(snap, stale, ps, id);
    if (f.valid && f.last == st) continue;
    f.last = st;
    f.valid = true;
    draw_field_(id, st, s);
    ++drawn;
    if (out) out->push_back(f.box);
  }
  return drawn;
}

int GsCompactBar::repaint_intersecting(const DirtyRect* rects, size_t n,
                                       const Surface& s,
                                       std::vector<DirtyRect>* out) {
  if (!laid_out_ || !rects || n == 0) return 0;
  int drawn = 0;
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    const GsBarField id = kOrder[i];
    Field& f = f_(id);
    if (!f.active || !f.valid) continue;
    bool hit = false;
    for (size_t r = 0; r < n; ++r)
      if (intersects(f.box, rects[r])) { hit = true; break; }
    if (!hit) continue;
    draw_field_(id, f.last, s);
    ++drawn;
    if (out) out->push_back(f.box);
  }
  return drawn;
}

void GsCompactBar::invalidate() {
  for (Field& f : fields_) f.valid = false;
}

}  // namespace maburplay
