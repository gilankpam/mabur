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

// The eleven items, in draw order. This IS the order on the glass.
constexpr GsBarField kOrder[] = {
    GsBarField::kCh,  GsBarField::kMcs, GsBarField::kAir,     GsBarField::kRssi,
    GsBarField::kSnr, GsBarField::kBitrate, GsBarField::kRes, GsBarField::kFps,
    GsBarField::kJit, GsBarField::kLat, GsBarField::kLoss,
};
static_assert(sizeof(kOrder) / sizeof(kOrder[0]) == (size_t)GsBarField::kCount,
              "every field must appear exactly once in the draw order");

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
    case GsBarField::kCount:   break;
  }
  return "";
}

int GsCompactBar::worst_line_width(const MaskAtlas& a, int n_cards) {
  const int gap = item_gap(&a);
  int w = 2 * pad_h(&a);  // the first box's left pad and the last one's right
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    if (i) w += gap;
    w += text_width(a, worst_case(kOrder[i], n_cards).c_str());
  }
  return w;
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

  // Largest baked size whose WORST-CASE line fits. Unlike the essential
  // overlay this asks for no particular design size: the bar has one type
  // size and its only constraint is the width of the line, so "the biggest
  // that fits" is the whole rule -- and it is what makes the bar render at
  // a readable size on a 720p panel and a bigger one on a 4K panel without
  // a per-resolution table.
  const MaskAtlas* best = nullptr;
  int best_px = 0;
  for (int px = 1; px <= 512; ++px) {
    const MaskAtlas* a = font_.atlas(px);
    if (!a || a->advance_x <= 0) continue;
    if (worst_line_width(*a, kMaxCards) > avail_w) continue;
    // The line also has to fit ABOVE the bottom inset without running off
    // the top of a very short surface.
    if (a->glyph_h + inset_y > screen_h) continue;
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
  // bottom inset.
  baseline_y_ = screen_h - inset_y - (best->glyph_h - best->baseline);
  // Reserve the worst case until the first snapshot says how many cards
  // there really are. Nothing draws before then (update() reconciles the
  // count first), but bounds() is legitimately asked for in between.
  place_(kMaxCards);
  laid_out_ = true;
  return true;
}

void GsCompactBar::place_(int n_cards) {
  if (!atlas_) return;
  const int pad = pad_h(atlas_);
  int total = 0;
  int w[(size_t)GsBarField::kCount];
  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    w[i] = text_width(*atlas_, worst_case(kOrder[i], n_cards).c_str());
    total += w[i] + (i ? gap_ : 0);
  }
  // Centred, but never past the inset: with kMaxCards the line can be wide
  // enough that centring and the inset disagree, and the inset wins.
  int pen = (screen_w_ - total) / 2;
  if (pen < inset_x_ + pad) pen = inset_x_ + pad;

  for (int i = 0; i < (int)GsBarField::kCount; ++i) {
    Field& f = f_(kOrder[i]);
    f.pen_x = pen;
    f.box = DirtyRect{pen - pad, baseline_y_ - atlas_->baseline, w[i] + 2 * pad,
                      atlas_->glyph_h};
    bounds_ = union_of(bounds_, f.box);
    pen += w[i] + gap_;
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
  if (!atlas_) return;
  clear_region(s, f.box);
  if (st.text.empty()) return;
  draw_text(s, *atlas_, f.pen_x, baseline_y_, st.text.c_str(), st.rgb);
}

int GsCompactBar::update(const GsSnapshot& snap, bool stale,
                         const GsPlayerState& ps, const Surface& s,
                         std::vector<DirtyRect>* out) {
  if (!laid_out_) return 0;
  int drawn = 0;

  // A changed card count changes the width of two items and therefore the
  // x of every item after them -- and, because the line is centred, of
  // every item before them too. So the whole line moves: erase it where it
  // was before anything is re-placed, since a field's own next draw only
  // ever clears its NEW box.
  const int n = std::min((int)snap.cards.size(), kMaxCards);
  if (n != n_cards_) {
    if (n_cards_ >= 0) {
      for (int i = 0; i < (int)GsBarField::kCount; ++i) {
        Field& f = f_(kOrder[i]);
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
    if (!f.valid) continue;
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
