#include "gs_overlay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "gs_draw.h"
#include "gs_font.h"

namespace maburplay {
namespace {
// Bars span -90 (nothing lit) to -45 (all six lit).
constexpr double kBarFloorDbm = -90.0;
constexpr double kBarCeilDbm = -45.0;
constexpr int kBars = 6;
}  // namespace

uint32_t status_rgb(Status s) {
  switch (s) {
    case Status::kOk: return tok::kStatusOk;
    case Status::kCaution: return tok::kStatusCaution;
    case Status::kCritical: return tok::kStatusCaution;
  }
  return tok::kStatusCaution;
}

Status rssi_status(double dbm) {
  if (dbm >= -65.0) return Status::kOk;
  if (dbm >= -80.0) return Status::kCaution;
  return Status::kCritical;
}

Status snr_status(double db) {
  if (db >= 12.0) return Status::kOk;
  if (db >= 6.0) return Status::kCaution;
  return Status::kCritical;
}

Status card_status(const GsCard& c) {
  if (!c.heard) return Status::kOk;
  const Status r = rssi_status(c.rssi_dbm.value_or(0.0));
  const Status s = snr_status(c.snr_db.value_or(0.0));
  return r > s ? r : s;  // enum order is ok < caution < critical
}

int rssi_bars(double dbm) {
  if (dbm <= kBarFloorDbm) return 0;
  if (dbm >= kBarCeilDbm) return kBars;
  const double frac = (dbm - kBarFloorDbm) / (kBarCeilDbm - kBarFloorDbm);
  // Round to nearest, not truncate: truncation understates the boundary
  // sample the design cites (-71 dBm -> 3 of 6; frac*6 = 2.53, which
  // truncates to 2). Rounding is also what keeps the mapping's two named
  // checkpoints (-58 -> 4, -71 -> 3) and monotonicity simultaneously true.
  int n = (int)std::lround(frac * kBars);
  if (n < 0) n = 0;
  if (n > kBars) n = kBars;
  return n;
}

int card_bars(const GsCard& c) {
  if (!c.heard || !c.rssi_dbm) return 0;
  return rssi_bars(*c.rssi_dbm);
}

Status airtime_status(double pct) {
  return pct >= 75.0 ? Status::kCaution : Status::kOk;
}

Status post_loss_status(double pct) {
  if (pct <= 0.0) return Status::kOk;
  if (pct <= 1.0) return Status::kCaution;
  return Status::kCritical;
}

std::string fmt_int(double v) {
  char b[32];
  double r = std::round(v);
  if (r == 0.0) r = 0.0;  // kill negative zero: "-0" would widen the box
  std::snprintf(b, sizeof(b), "%.0f", r);
  return b;
}

std::string fmt_one_dp(double v) {
  char b[32];
  // Round to one decimal place ourselves before handing off to snprintf.
  // "%.1f" alone rounds the raw binary value: 2.15 is actually stored as
  // 2.149999999999999911..., so naive "%.1f" prints "2.1", not the "2.2"
  // a human typing 2.15 expects. Scaling by 10 first lands on the nearest
  // representable double to the half-integer (21.5 is exact in binary),
  // so std::round's away-from-zero tie-break does the right thing.
  const double scaled = std::round(v * 10.0) / 10.0;
  std::snprintf(b, sizeof(b), "%.1f", scaled);
  return b;
}

std::string fmt_signed_int(double v) {
  const double r = std::round(v);
  if (r < 0.0) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.0f", -r);
    return std::string(kMinus) + b;
  }
  return fmt_int(r);
}

std::string fmt_clock(int seconds) {
  if (seconds < 0) seconds = 0;
  // Saturate rather than wrap: 60:00 reading as 00:00 mid-flight would say
  // "recording just started", which is the opposite of the truth. The cap
  // also keeps the string five characters wide forever, so the field box
  // sized at layout time stays correct.
  if (seconds > 99 * 60 + 59) seconds = 99 * 60 + 59;
  char b[16];
  std::snprintf(b, sizeof(b), "%02d:%02d", seconds / 60, seconds % 60);
  return b;
}

// --- GsOverlay ----------------------------------------------------------

namespace {

// Design sizes at 1080p (docs/superpowers/specs/2026-08-04-gs-osd-essential-
// design.md, "Layout and blocks").
constexpr int kInsetX = 96, kInsetY = 54;
constexpr int kSizeHero = 56, kSizePrimary = 38, kSizeStandard = 34;
constexpr int kSizeArrow = 26, kSizeSecondary = 24, kSizeCardId = 22;
constexpr int kSizeLabel = 19;
constexpr int kMeterW = 130, kMeterH = 10, kMeterTickW = 2;
constexpr double kAirCeilingPct = 75.0;
constexpr int kBarW = 13, kBarGap = 5;
constexpr int kBarHeights[6] = {16, 22, 28, 34, 40, 46};
constexpr int kMinRenderedPx = 18;  // the handoff's floor
// How far a design size may snap to a baked one before "nearest" stops
// being a legitimate resolution-scaling accommodation and starts being
// "the font doesn't have this size". GsFont::atlas() is deliberately
// exact-match-only (gs_font.h: "a near-miss fallback would silently ship
// the wrong type size") -- a naive nearest-of-whatever-is-loaded search
// would reintroduce exactly that near-miss fallback one layer up, e.g.
// silently substituting a 24 px atlas for a missing 56 px "hero" size
// (32 px off, ~57%) instead of failing. Proportional, not a fixed pixel
// count: a font baked across several resolutions (Task 14 bakes the eight
// design sizes at x2/3, x1, x4/3, x2 -- 720p/1080p/1440p/2160p) rounds
// `want` to a DIFFERENT nearby integer at each scale, and that rounding
// slop grows with the design size itself (a couple of px on a 56 px hero
// is a much smaller fraction than the same couple of px on a 19 px
// label). A flat pixel bound that's loose enough for the 56 px case is
// too loose to reject a bad substitute for the 19 px one, and one tight
// enough for 19 px would reject legitimate scaled hits on 56 px. Percent
// of the request scales with it either way.
constexpr int kMaxSizeSnapPctNum = 15;  // integer arithmetic: no float compares

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

// which: 0 id, 1 bars, 2 rssi, 3 dBm unit, 4 snr
GsFieldId card_field(int slot, int which) {
  return (GsFieldId)((int)GsFieldId::kCard0Id + slot * 5 + which);
}

// Per-side shadow pad an atlas bakes in (gs_font.h: glyph_w/glyph_h include
// it, advance_x does not). Text placed flush against an edge has to
// reserve this or the box -- which spans the padded CELL, not just the
// glyph, because clear_region has to erase the shadow too -- overhangs the
// edge by exactly this many pixels.
int pad_h(const MaskAtlas* a) { return (a->glyph_w - a->advance_x) / 2; }

// Vertical analogue of advance_x: the unpadded line height (ascender +
// descender). Stacking rows by glyph_h (the padded cell height) double-
// counts the shadow pad on every row -- small per row, but multiplied by
// kMaxCards=4 it was enough to push the card block's top edge up out of
// its corner and into the row-2 exclusion band the handoff reserves for
// the centre of frame. Used ONLY for the card block's row-to-row pitch
// (kMaxCards rows makes the double-count large enough to matter); the
// single-to-single line spacing elsewhere (top-right block, FPS/JIT-MBPS)
// uses plain glyph_h; there the safety we get by NOT skimping the gap --
// no two boxes ever meet, never mind overlap -- is worth more than the
// few px it costs, and line_pitch's shrink was in fact tight enough there
// to make adjacent boxes overlap (see the comment at kRung's placement).
// A field's own box still spans the full padded glyph_h always; only a
// row-to-row PITCH may use this.
int line_pitch(const MaskAtlas* a) { return a->glyph_h - (a->glyph_w - a->advance_x); }

}  // namespace

bool GsOverlay::layout(int screen_w, int screen_h, std::string* err) {
  laid_out_ = false;
  for (Field& f : fields_) f = Field{};
  bounds_ = DirtyRect{0, 0, 0, 0};
  // A re-layout (mode change, surface recreate) must force the next
  // update() to fully reconcile card activity again. Card fields start
  // active=false here (the `fields_` reset above; layout() doesn't know
  // the card count, so place_card_row_ below deliberately never sets
  // active itself -- only update()'s reconciliation does) but n_cards_
  // sitting at whatever it was from BEFORE this layout() would make that
  // reconciliation a no-op if the next snapshot happens to report the same
  // count, leaving every card field inactive even though the snapshot
  // really does have cards to show.
  n_cards_ = -1;

  if (screen_w <= 0 || screen_h <= 0) {
    if (err) *err = "gs osd: bad screen size";
    return false;
  }
  scale_ = (double)screen_h / 1080.0;

  auto pick = [&](int design_px) -> const MaskAtlas* {
    const int want = (int)(design_px * scale_ + 0.5);
    const MaskAtlas* best = nullptr;
    int best_d = 1 << 30;
    for (int px = 1; px <= 512; ++px) {
      const MaskAtlas* a = font_.atlas(px);
      if (!a) continue;
      const int d = px > want ? px - want : want - px;
      if (d < best_d) { best_d = d; best = a; }
    }
    // Integer percent, deliberately: comparing a ratio in floating point
    // here would need an epsilon anyway, and the tolerance only needs to
    // separate "a couple of px of rounding slop" from "a wrong size" --
    // it isn't a rendering measurement. want is always >= 1 in practice
    // (the smallest design size, 19 px, scaled to nothing coarser than a
    // 480p-ish surface still rounds well above 0), but max(1, ...) keeps
    // the tolerance from collapsing to 0 if a caller ever passes a
    // pathologically small design size.
    const int tol = std::max(1, want * kMaxSizeSnapPctNum / 100);
    return best_d <= tol ? best : nullptr;
  };

  const MaskAtlas* hero = pick(kSizeHero);
  const MaskAtlas* primary = pick(kSizePrimary);
  const MaskAtlas* standard = pick(kSizeStandard);
  const MaskAtlas* arrow = pick(kSizeArrow);
  const MaskAtlas* secondary = pick(kSizeSecondary);
  const MaskAtlas* cardid = pick(kSizeCardId);
  const MaskAtlas* label = pick(kSizeLabel);
  if (!hero || !primary || !standard || !arrow || !secondary || !cardid || !label) {
    if (err) *err = "gs osd: font is missing a required size";
    return false;
  }
  // The handoff's responsive floor: below 18 px rendered, drop to the two
  // bottom blocks only rather than shipping unreadable type.
  const bool drop_top = label->px < kMinRenderedPx;

  const int inset_x = (int)(kInsetX * scale_ + 0.5);
  const int inset_y = (int)(kInsetY * scale_ + 0.5);
  const int left = inset_x, right = screen_w - inset_x;
  const int top = inset_y, bottom = screen_h - inset_y;
  // THE HORIZONTAL CLEARANCE FLOOR, before you change any gapN below.
  //
  // A field's box spans the PADDED cell, so it is `(glyph_w - advance_x)`
  // wider than its text -- 8 px, PAD 4 per side. That pad is baked into the
  // atlas and is therefore scale-INVARIANT: it is 8 px at 720p and 8 px at
  // 2160p. The design gaps here are not; they scale with height/1080. So
  // the real clearance between two horizontally adjacent boxes is
  //
  //     round(gapN * scale) - 8
  //
  // which is MINIMISED at the smallest supported resolution. At 720p
  // (scale 2/3) a gap12 yields exactly 8 - 8 = 0: kFpsValue and kFpsLabel
  // ABUT with zero pixels between them. That is legal -- zero is not an
  // overlap, and pick() refuses to lay out below 720p so it cannot go
  // negative -- but the margin is gone, and the no-overlap tests assert only
  // strict non-overlap, never a minimum clearance. Anything smaller than
  // gap12 for a horizontal neighbour pair, or a new atlas with more pad,
  // makes boxes overlap at 720p and the failure will appear as one field's
  // clear erasing its neighbour's last column.
  const int gap6 = (int)(6 * scale_ + 0.5), gap8 = (int)(8 * scale_ + 0.5);
  const int gap10 = (int)(10 * scale_ + 0.5), gap12 = (int)(12 * scale_ + 0.5);
  const int gap14 = (int)(14 * scale_ + 0.5), gap16 = (int)(16 * scale_ + 0.5);
  const int gap26 = (int)(26 * scale_ + 0.5);

  auto place = [&](GsFieldId id, const MaskAtlas* a, int x, int baseline,
                   const char* worst) {
    Field& f = f_(id);
    f.atlas = a;
    f.pen_x = x;
    f.baseline_y = baseline;
    const int w = text_width(*a, worst);
    // The box spans the glyph CELL vertically, not just the em: the shadow
    // pad lives inside the cell and must be cleared with the glyph.
    f.box = DirtyRect{x - (a->glyph_w - a->advance_x) / 2, baseline - a->baseline,
                      w + (a->glyph_w - a->advance_x), a->glyph_h};
    f.active = true;
    bounds_ = union_of(bounds_, f.box);
  };

  // --- top right: rung / airtime / recording -------------------------
  if (!drop_top) {
    int y = top + standard->baseline;
    const int rung_w = text_width(*standard, "MCS 7 / FEC 100%");
    // Flush against `right`: reserve the glyph's shadow pad, or the box
    // (which spans the padded cell) overhangs the safe inset.
    place(GsFieldId::kRung, standard, right - rung_w - pad_h(standard), y,
          "MCS 7 / FEC 100%");

    // NOT line_pitch here: unlike the card block's row-to-row pitch, these
    // are single lines of the SAME size stacked close together, and
    // line_pitch's whole point (avoid double-counting the pad across many
    // rows) overshoots on just two -- it shrinks the gap enough that
    // kRung's and kAirValue's padded boxes overlap (measured: 88x2 px),
    // and a box overlap means one field's clear can erase pixels that
    // belong to its neighbour. glyph_h keeps every box here fully
    // separated at the cost of a few extra px of vertical margin.
    y += standard->glyph_h + gap6;
    // AIR label | value | meter, laid out right to left off `right`.
    const int meter_w = (int)(kMeterW * scale_ + 0.5);
    const int meter_h = (int)(kMeterH * scale_ + 0.5);
    const int val_w = text_width(*standard, "100%");
    const int lbl_w = text_width(*label, "AIR");
    const int air_x = right - meter_w - gap10 - val_w - gap10 - lbl_w;
    place(GsFieldId::kAirLabel, label, air_x, y, "AIR");
    place(GsFieldId::kAirValue, standard, air_x + lbl_w + gap10, y, "100%");
    {
      Field& m = f_(GsFieldId::kAirMeter);
      m.atlas = standard;
      m.pen_x = right - meter_w;
      // Vertically centred on the value's x-height, near the baseline.
      m.baseline_y = y - meter_h;
      // The ceiling tick overhangs the track by 3 px top and bottom.
      const int over = (int)(3 * scale_ + 0.5);
      m.box = DirtyRect{right - meter_w, y - meter_h - over, meter_w, meter_h + 2 * over};
      m.active = true;
      bounds_ = union_of(bounds_, m.box);
    }

    y += standard->glyph_h + gap6;  // see the comment on the line above
    const int rec_w = text_width(*secondary, "● REC FAULT");
    place(GsFieldId::kRec, secondary, right - rec_w - pad_h(secondary), y,
          "● REC FAULT");
  }

  // --- bottom right: video health ------------------------------------
  {
    const int sec_w = text_width(*secondary, "JIT 999 ms") + gap26 +
                      text_width(*secondary, "999.9 MBIT/S");
    const int sec_base = bottom - (secondary->glyph_h - secondary->baseline);
    // Flush against `right`, same pad reservation as the top-right block.
    const int jit_x = right - sec_w - pad_h(secondary);
    place(GsFieldId::kJit, secondary, jit_x, sec_base, "JIT 999 ms");
    place(GsFieldId::kMbps, secondary,
          jit_x + text_width(*secondary, "JIT 999 ms") + gap26, sec_base,
          "999.9 MBIT/S");

    // The FPS line stacks above the JIT/MBPS line, and unlike the top-right
    // block these two lines use DIFFERENT atlases: hero above, secondary
    // below. Stepping the baseline up by the SECONDARY's glyph_h -- what
    // this did originally -- is wrong twice over: it measures the step in
    // the wrong font, and it ignores how far the HERO cell descends below
    // its own baseline, which is what actually decides where the hero's box
    // ends. With the synthetic test font those two errors cancelled to
    // exactly 0 px of clearance, so every test passed on zero margin; with
    // real JetBrains Mono metrics they leave the two boxes OVERLAPPING by
    // 1 px at 1080p and 3 px at 2160p (measured -- see
    // asset_no_two_active_field_boxes_overlap_at_any_resolution, which is
    // the test that caught it, and note that the synthetic-font version of
    // the same test could not: it passed at exactly 0 px).
    //
    // Derive the baseline from the BOXES instead, which is what the
    // no-overlap invariant is actually about: the hero box's bottom edge is
    // fps_base + (glyph_h - baseline), the JIT/MBPS box's top edge is
    // sec_base - baseline, and gap8 separates the two.
    const int fps_base = sec_base - secondary->baseline -
                         (hero->glyph_h - hero->baseline) - gap8;
    const int fps_lbl_w = text_width(*label, "FPS");
    const int fps_val_w = text_width(*hero, "999");
    const int fps_x = right - fps_lbl_w - gap12 - fps_val_w - pad_h(label);
    place(GsFieldId::kFpsValue, hero, fps_x, fps_base, "999");
    place(GsFieldId::kFpsLabel, label, fps_x + fps_val_w + gap12, fps_base, "FPS");
  }

  // --- bottom centre: loss -------------------------------------------
  {
    const int lbl_w = text_width(*label, "LOSS");
    const int pre_w = text_width(*standard, "100.0%");
    const int arr_w = text_width(*arrow, kArrow);
    const int post_w = text_width(*standard, "100.0%");
    const int total = lbl_w + gap14 + pre_w + gap14 + arr_w + gap14 + post_w;
    int x = (screen_w - total) / 2;
    const int base = bottom - (standard->glyph_h - standard->baseline);
    place(GsFieldId::kLossLabel, label, x, base, "LOSS");
    x += lbl_w + gap14;
    place(GsFieldId::kLossPre, standard, x, base, "100.0%");
    x += pre_w + gap14;
    place(GsFieldId::kLossArrow, arrow, x, base, kArrow);
    x += arr_w + gap14;
    place(GsFieldId::kLossPost, standard, x, base, "100.0%");
  }

  // --- bottom left: per-card signal ----------------------------------
  // All kMaxCards slots get boxes here, at the positions the block would
  // use if all kMaxCards cards were reported -- this is what bounds()
  // reflects before any update() ever runs. update() re-anchors the
  // ACTIVE rows to `bottom` for however many cards are actually reported
  // (see place_card_row_ and the reconciliation branch in update()): with
  // fewer than kMaxCards cards the block must hug the bottom-left corner
  // like the loss/video blocks do, not float above it reserving space for
  // cards that were never there. Boxes are stacked UPWARD from the bottom
  // so the first card is the top row, matching the handoff's C0-above-C1
  // sample.
  {
    CardGeom& g = card_geom_;
    g.cardid = cardid;
    g.primary = primary;
    g.label = label;
    g.secondary = secondary;
    g.left = left;
    g.bottom = bottom;
    g.gap16 = gap16;
    g.bar_block_w = 6 * (int)(kBarW * scale_ + 0.5) + 5 * (int)(kBarGap * scale_ + 0.5);
    g.bar_block_h = (int)(kBarHeights[5] * scale_ + 0.5);
    g.id_w = text_width(*cardid, "C9");
    // Sized to the widest NUMERIC reading, NOT to the unheard text. Sizing it
    // for an 11-glyph string reserved ~160 px at 1080p that a 3-4 glyph value
    // never uses, so every normal row showed a visible gulf between the value
    // and "dBm". The unheard state renders the em-dash pair instead (case 2 in
    // state_of_), which fits this box -- so nothing draws past it, and the
    // most frequently changing field in the overlay now clears 2.8x less.
    g.rssi_worst = "\xE2\x88\x92" "100";
    g.rssi_w = text_width(*primary, g.rssi_worst.c_str());
    g.unit_w = text_width(*label, "dBm");
    // Row PITCH uses line_pitch (unpadded), not glyph_h -- see line_pitch's
    // comment. The BOX height below still uses the full glyph_h/baseline
    // pair so clearing still erases the shadow.
    const int row_h = std::max(line_pitch(primary), g.bar_block_h);
    g.row_pitch = row_h + gap16;

    for (int i = 0; i < kMaxCards; ++i)
      place_card_row_(i, bottom - (kMaxCards - 1 - i) * g.row_pitch);
  }

  laid_out_ = true;
  return true;
}

void GsOverlay::place_card_row_(int slot, int row_bottom) {
  const CardGeom& g = card_geom_;
  const int baseline = row_bottom - (g.primary->glyph_h - g.primary->baseline);

  // Mirrors layout()'s place() lambda -- can't reuse it directly, it's a
  // local closure over layout()'s stack frame, and this needs to run again
  // from update() whenever the active card count changes.
  auto place_text = [&](GsFieldId id, const MaskAtlas* a, int x, const char* worst) {
    Field& f = f_(id);
    f.atlas = a;
    f.pen_x = x;
    f.baseline_y = baseline;
    const int w = text_width(*a, worst);
    const int pad = a->glyph_w - a->advance_x;
    f.box = DirtyRect{x - pad / 2, baseline - a->baseline, w + pad, a->glyph_h};
    bounds_ = union_of(bounds_, f.box);
  };

  // Flush against `left`: reserve the id glyph's shadow pad, same
  // reasoning as the right-edge fields in layout().
  int x = g.left + pad_h(g.cardid);
  place_text(card_field(slot, 0), g.cardid, x, "C9");
  x += g.id_w + g.gap16;
  {
    Field& b = f_(card_field(slot, 1));
    b.atlas = g.cardid;  // unused for bars, but never left null
    b.pen_x = x;
    b.baseline_y = row_bottom;
    b.box = DirtyRect{x, row_bottom - g.bar_block_h, g.bar_block_w, g.bar_block_h};
    bounds_ = union_of(bounds_, b.box);
  }
  x += g.bar_block_w + g.gap16;
  place_text(card_field(slot, 2), g.primary, x, g.rssi_worst.c_str());
  x += g.rssi_w + g.gap16;
  place_text(card_field(slot, 3), g.label, x, "dBm");
  x += g.unit_w + g.gap16;
  place_text(card_field(slot, 4), g.secondary, x, "100 dB");
}

GsOverlay::FieldState GsOverlay::state_of_(const GsSnapshot& snap, bool stale,
                                           const GsPlayerState& ps,
                                           GsFieldId id) const {
  FieldState st;
  // Staleness dims LINK fields only. Player-measured fields (fps/jit/mbps
  // and the recording block) are current by construction and keep their
  // colours -- dimming them would say the picture is stale when it is not.
  const uint32_t link_primary = stale ? tok::kTextLabel : tok::kTextPrimary;
  const uint32_t link_secondary = stale ? tok::kTextLabel : tok::kTextSecondary;
  auto link_status_rgb = [&](Status s) {
    return stale ? tok::kTextLabel : status_rgb(s);
  };

  // Every clamp below answers the same question: what is the widest string
  // this field's box, sized in layout() from a specific worst-case string
  // (see the table above draw_field_), can actually hold? gs_snapshot.cpp's
  // JSON accessors reject non-finite and non-numeric values but do NOT
  // bound the magnitude of a well-formed one -- a hostile or corrupt
  // datagram's air_pct: 1e300 parses cleanly into a ~300-character string.
  // draw_text clips only to the SURFACE, not to this field's box, and
  // clear_region only clears the box, so an unclamped value would draw
  // past its box on one frame and never be erased on any later one --
  // permanent garbage. Clamping the INPUT here (rather than, say, adding a
  // clip rect to draw_text) is what's chosen: it keeps the fixed-width
  // invariant the whole box-sizing scheme depends on exactly true, instead
  // of merely papering over one symptom of violating it.
  switch (id) {
    case GsFieldId::kRung:
      st.rgb = link_primary;
      if (snap.mcs && snap.fec_pct) {
        const int mcs = (int)std::clamp(*snap.mcs, 0, 9);          // "MCS 7"
        const double fec = std::clamp(*snap.fec_pct, 0.0, 100.0);  // "FEC 100%"
        st.text = "MCS " + fmt_int(mcs) + " / FEC " + fmt_int(fec) + "%";
      } else {
        st.text = kEmDashPair;
      }
      break;
    case GsFieldId::kAirLabel:
      st.rgb = tok::kTextLabel;
      st.text = "AIR";
      break;
    case GsFieldId::kAirValue:
      if (snap.air_pct) {
        st.text = fmt_int(std::clamp(*snap.air_pct, 0.0, 100.0)) + "%";  // "100%"
        st.rgb = link_primary;
      } else {
        st.text = kEmDashPair;
        st.rgb = tok::kTextLabel;
      }
      break;
    case GsFieldId::kAirMeter:
      st.rgb = snap.air_pct ? link_status_rgb(airtime_status(*snap.air_pct))
                            : tok::kTrack;
      // aux is the fill width in pixels, which is what actually changes --
      // shadowing the raw percent would redraw on sub-pixel wobble. Already
      // clamped (this is a fill width, not text, but the same corrupt
      // input would otherwise overflow fill_rect's width just as badly).
      st.aux = snap.air_pct
                   ? (int)(std::clamp(*snap.air_pct, 0.0, 100.0) *
                           (kMeterW * scale_) / 100.0)
                   : 0;
      break;
    case GsFieldId::kRec:
      switch (ps.rec.kind) {
        case RecState::Kind::kAbsent:
          st.text.clear();
          st.rgb = tok::kTextLabel;
          break;
        case RecState::Kind::kArmed:
          st.text = std::string(kDotHollow) + " REC --:--";
          st.rgb = tok::kTextLabel;
          break;
        case RecState::Kind::kRecording:
          // fmt_clock saturates internally (mm:ss, cap 99:59) -- already
          // fixed-width for any int, no clamp needed here.
          st.text = std::string(kDotFilled) + " REC " + fmt_clock(ps.rec.elapsed_s);
          st.rgb = tok::kTextPrimary;
          // aux distinguishes the dot's colour from the text's; draw_field_
          // paints the leading glyph in kStatusRec when aux == 1.
          st.aux = 1;
          break;
        case RecState::Kind::kFault:
          st.text = std::string(kDotFilled) + " REC FAULT";
          st.rgb = tok::kStatusCaution;
          break;
      }
      break;
    case GsFieldId::kLossLabel:
      st.rgb = tok::kTextLabel;
      st.text = "LOSS";
      break;
    case GsFieldId::kLossPre:
      st.rgb = link_primary;
      st.text = snap.pre_loss_pct  // "100.0%"
                    ? fmt_one_dp(std::clamp(*snap.pre_loss_pct, 0.0, 100.0)) + "%"
                    : kEmDashPair;
      break;
    case GsFieldId::kLossArrow:
      st.rgb = tok::kTextLabel;
      st.text = kArrow;
      break;
    case GsFieldId::kLossPost:
      if (snap.post_loss_pct) {
        const double v = std::clamp(*snap.post_loss_pct, 0.0, 100.0);  // "100.0%"
        st.text = fmt_one_dp(v) + "%";
        st.rgb = link_status_rgb(post_loss_status(v));
      } else {
        st.text = kEmDashPair;
        st.rgb = tok::kTextLabel;
      }
      break;
    case GsFieldId::kFpsValue:
      st.rgb = tok::kTextPrimary;
      st.text = fmt_int(std::clamp(ps.fps, 0.0, 999.0));  // "999"
      break;
    case GsFieldId::kFpsLabel:
      st.rgb = tok::kTextLabel;
      st.text = "FPS";
      break;
    case GsFieldId::kJit:
      st.rgb = tok::kTextPrimary;
      st.text = "JIT " + fmt_int(std::clamp(ps.jitter_ms, 0.0, 999.0)) + " ms";
      break;
    case GsFieldId::kMbps:
      st.rgb = tok::kTextPrimary;
      st.text = fmt_one_dp(std::clamp(ps.mbps, 0.0, 999.9)) + " MBIT/S";
      break;
    default: {
      // Card slots.
      const int base = (int)GsFieldId::kCard0Id;
      const int idx = (int)id - base;
      const int slot = idx / 5, which = idx % 5;
      if (slot < 0 || slot >= (int)snap.cards.size()) return st;  // empty
      const GsCard& c = snap.cards[(size_t)slot];
      const Status cs = card_status(c);
      switch (which) {
        case 0:
          st.rgb = tok::kTextLabel;
          // "C?" for an id outside the single digit the box was sized for
          // ("C9"), not a clamp: clamping ids 10 and 11 (or a negative
          // one) both into range would silently relabel two different
          // rows identically -- and possibly identically to a genuine C9
          // -- which is a worse failure than an honestly-wrong "C?".
          st.text = (c.id >= 0 && c.id <= 9) ? "C" + fmt_int(c.id) : "C?";
          break;
        case 1:
          st.rgb = link_status_rgb(cs);
          st.aux = card_bars(c);  // rssi_bars() already clamps 0..6
          break;
        case 2:
          if (c.heard && c.rssi_dbm) {
            st.text = fmt_signed_int(std::clamp(*c.rssi_dbm, -999.0, 999.0));
            st.rgb = link_status_rgb(cs);
          } else {
            // A card that has produced no packets renders the design's own
            // never-received glyph rather than the words "never heard": it
            // fits the numeric-sized box (see rssi_worst in layout()), where
            // the words did not. The row still says "dead antenna" loudly --
            // the bars beside it are all unlit, and the dBm/SNR that would
            // follow a real reading are blank.
            st.text = kEmDashPair;
            st.rgb = tok::kTextLabel;
          }
          break;
        case 3:
          st.rgb = tok::kTextLabel;
          st.text = c.heard ? "dBm" : "";
          break;
        case 4:
          st.rgb = link_secondary;
          // fmt_signed_int, not fmt_int: a negative SNR on this row would
          // otherwise print an ASCII '-' while RSSI two fields over prints
          // U+2212 for the same sign, an inconsistency within one row.
          st.text = (c.heard && c.snr_db)  // "100 dB"
                        ? fmt_signed_int(std::clamp(*c.snr_db, -99.0, 999.0)) + " dB"
                        : "";
          break;
      }
      break;
    }
  }
  return st;
}

std::string GsOverlay::debug_field_text(const GsSnapshot& snap, bool stale,
                                        const GsPlayerState& ps,
                                        GsFieldId id) const {
  return state_of_(snap, stale, ps, id).text;
}

DirtyRect GsOverlay::debug_field_box(GsFieldId id) const {
  return f_(id).box;
}

bool GsOverlay::debug_field_active(GsFieldId id) const {
  return f_(id).active;
}

int GsOverlay::debug_field_atlas_px(GsFieldId id) const {
  const Field& f = f_(id);
  return f.atlas ? f.atlas->px : 0;
}

void GsOverlay::draw_field_(GsFieldId id, const FieldState& st, const Surface& s) {
  Field& f = f_(id);
  if (!f.active || !f.atlas) return;
  clear_region(s, f.box);

  if (id == GsFieldId::kAirMeter) {
    const int mw = (int)(kMeterW * scale_ + 0.5);
    const int mh = (int)(kMeterH * scale_ + 0.5);
    const int y = f.baseline_y;
    fill_rect(s, f.pen_x, y, mw, mh, tok::kTrack);
    if (st.aux > 0) fill_rect(s, f.pen_x, y, std::min(st.aux, mw), mh, st.rgb);
    // Ceiling tick, overhanging the track top and bottom.
    const int over = (int)(3 * scale_ + 0.5);
    const int tick_x = f.pen_x + (int)(mw * kAirCeilingPct / 100.0);
    fill_rect(s, tick_x, y - over, (int)(kMeterTickW * scale_ + 0.5), mh + 2 * over,
              tok::kStatusCaution);
    return;
  }

  const int base = (int)GsFieldId::kCard0Id;
  if ((int)id >= base && ((int)id - base) % 5 == 1) {  // a bars field
    const int bw = (int)(kBarW * scale_ + 0.5);
    const int bg = (int)(kBarGap * scale_ + 0.5);
    for (int i = 0; i < 6; ++i) {
      const int h = (int)(kBarHeights[i] * scale_ + 0.5);
      const uint32_t rgb = i < st.aux ? st.rgb : tok::kTrack;
      fill_rect(s, f.pen_x + i * (bw + bg), f.baseline_y - h, bw, h, rgb);
    }
    return;
  }

  if (st.text.empty()) return;  // cleared above; nothing more to draw

  // The recording dot takes kStatusRec while the rest of the line takes the
  // field colour -- one field, two colours, because they change together
  // and splitting them would double the dirty rects for no benefit.
  if (id == GsFieldId::kRec && st.aux == 1) {
    const int adv = draw_text(s, *f.atlas, f.pen_x, f.baseline_y, kDotFilled,
                              tok::kStatusRec);
    draw_text(s, *f.atlas, f.pen_x + adv, f.baseline_y,
              st.text.c_str() + std::string(kDotFilled).size(), st.rgb);
    return;
  }
  draw_text(s, *f.atlas, f.pen_x, f.baseline_y, st.text.c_str(), st.rgb);
}

int GsOverlay::update(const GsSnapshot& snap, bool stale, const GsPlayerState& ps,
                      const Surface& s, std::vector<DirtyRect>* out) {
  if (!laid_out_) return 0;

  int drawn = 0;

  // A changed card count changes which slots render at all -- and, since
  // active rows are anchored to `bottom` for however many are active (not
  // reserved kMaxCards-deep), it also moves every row that stays active.
  const int n = std::min((int)snap.cards.size(), kMaxCards);
  if (n != n_cards_) {
    // Clear every CURRENTLY-active slot's box before anything else moves.
    // This covers both cases at once: a slot that's about to deactivate
    // (its box is never touched again, so this is the only chance to
    // erase it -- draw_field_'s active-gated `continue` means nothing
    // else ever will) and a slot that stays active but is about to be
    // repositioned by place_card_row_ below (its OLD box needs erasing
    // just as much; only the field's own next draw clears its NEW one).
    for (int slot = 0; slot < kMaxCards; ++slot)
      for (int w = 0; w < 5; ++w) {
        Field& f = f_(card_field(slot, w));
        if (f.active) {
          clear_region(s, f.box);
          if (out) out->push_back(f.box);
          ++drawn;
        }
      }
    // Re-anchor the active rows to `bottom`, using `n` rather than the
    // kMaxCards layout() seeded -- with fewer cards than the reserved
    // maximum the block must hug the bottom-left corner like the loss and
    // video blocks do, not float above it with the unfilled rows' worth of
    // gap beneath it.
    for (int i = 0; i < n; ++i)
      place_card_row_(i, card_geom_.bottom - (n - 1 - i) * card_geom_.row_pitch);
    for (int slot = 0; slot < kMaxCards; ++slot)
      for (int w = 0; w < 5; ++w) {
        Field& f = f_(card_field(slot, w));
        f.active = slot < n;
        f.valid = false;
      }
    n_cards_ = n;
  }

  for (int i = 0; i < (int)GsFieldId::kCount; ++i) {
    const GsFieldId id = (GsFieldId)i;
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

int GsOverlay::repaint_intersecting(const DirtyRect* rects, size_t n,
                                    const Surface& s, std::vector<DirtyRect>* out) {
  if (!laid_out_ || !rects || n == 0) return 0;
  int drawn = 0;
  for (int i = 0; i < (int)GsFieldId::kCount; ++i) {
    const GsFieldId id = (GsFieldId)i;
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

void GsOverlay::invalidate() {
  for (Field& f : fields_) f.valid = false;
}

const uint32_t* GsOverlay::palette_seeds(size_t* n) {
  // Every token at a ramp of alphas, premultiplied. build_palette()'s
  // median cut needs the antialiased blends, not just the solid colours:
  // most GS pixels are partially covered glyph edges.
  static std::vector<uint32_t> seeds;
  if (seeds.empty()) {
    const uint32_t toks[] = {tok::kTextPrimary,  tok::kTextSecondary,
                             tok::kTextLabel,    tok::kTrack,
                             tok::kStatusOk,     tok::kStatusCaution,
                             tok::kStatusRec};
    for (uint32_t t : toks)
      for (int a = 255; a >= 0; a -= 17) seeds.push_back(premul(t, (uint8_t)a));
    // The shadow blend: black at a ramp of alphas, which is what a
    // shadow-only pixel looks like.
    for (int a = 255; a >= 0; a -= 17) seeds.push_back(premul(0x000000u, (uint8_t)a));
  }
  if (n) *n = seeds.size();
  return seeds.data();
}

}  // namespace maburplay
