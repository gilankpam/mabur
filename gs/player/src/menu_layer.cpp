#include "menu_layer.h"

#include <algorithm>
#include <cmath>
#include <tuple>

#include "gs_draw.h"
#include "gs_font.h"
#include "gs_layer.h"  // tok::

namespace maburplay {
namespace {

// Design geometry, in pixels at 1080p (scaled by screen_h / 1080 and the
// operator's menu scale).
constexpr double kRowPx = 22, kSmallPx = 15;
constexpr double kBigScale = 1.3;     // the number picker's value, x the row size
constexpr double kInset = 24;         // panel from the top-left corner
constexpr double kPanelW = 0.45;      // of the screen width
constexpr double kPadX = 14;          // text from a box's side
constexpr double kRowPadY = 6;        // a row's height over its glyph cell
constexpr double kLinePadY = 8;       // header/help/keys height over their lines
constexpr double kGap = 8;            // between a row's label and its tags, value and status
constexpr double kTagPad = 4;         // a tag's text inside its outline
constexpr double kPickerGap = 12;     // panel to picker
constexpr double kPickerMinW = 250, kNumberMinW = 160, kConfirmW = 320;
constexpr double kNoteGap = 16;       // a choice to its note
constexpr double kBoxPadY = 4;        // picker top/bottom margin
constexpr double kToastBottom = 90;   // above the GS bar's two rows
constexpr double kBannerTop = 0.12;   // of the screen height
constexpr double kBannerGap = 12;     // an open menu's boxes to the banner below them
constexpr size_t kMaxRows = 10;
constexpr int kHelpLines = 2;
constexpr int kConfirmLines = 4;

// Every box fill. The panel reads as a surface over video, not a hole in it.
constexpr uint8_t kAlpha = 232;

// Busy rows: dots after "applying" every 300 ms, and a lit third of the row
// sweeping along its bottom edge once per 800 ms. The sweep moves in 16
// steps, which repaints a 2 px strip at 20 Hz rather than on every pump.
constexpr uint64_t kDotMs = 300;
constexpr uint64_t kSweepMs = 800;
constexpr int kSweepSteps = 16;
constexpr int kMaxDots = 3;

bool intersects(const DirtyRect& a, const DirtyRect& b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

bool empty(const DirtyRect& r) { return r.w <= 0 || r.h <= 0; }

bool same(const DirtyRect& a, const DirtyRect& b) {
  return a.x == b.x && a.y == b.y && a.w == b.w && a.h == b.h;
}

DirtyRect union_of(const DirtyRect& a, const DirtyRect& b) {
  if (empty(a)) return b;
  if (empty(b)) return a;
  const int x0 = std::min(a.x, b.x), y0 = std::min(a.y, b.y);
  const int x1 = std::max(a.x + a.w, b.x + b.w), y1 = std::max(a.y + a.h, b.y + b.h);
  return DirtyRect{x0, y0, x1 - x0, y1 - y0};
}

int tw(const MaskAtlas* a, const std::string& s) { return text_width(*a, s.c_str()); }

std::vector<std::string> codepoints(const std::string& s) {
  std::vector<std::string> out;
  const char* p = s.c_str();
  while (*p) {
    const char* start = p;
    utf8_next(&p);
    out.emplace_back(start, p);
  }
  return out;
}

// The longest prefix that fits max_w. The atlas is monospace, so this is a
// codepoint count; a string that does not fit is cut, never wrapped past
// its box (draw_text clips only to the surface).
std::string fit(const MaskAtlas* a, const std::string& s, int max_w) {
  const int n = a->advance_x > 0 ? std::max(0, max_w / a->advance_x) : 0;
  const auto cps = codepoints(s);
  if ((int)cps.size() <= n) return s;
  std::string out;
  for (int i = 0; i < n; ++i) out += cps[(size_t)i];
  return out;
}

// Greedy word wrap into at most max_lines lines of max_w; what does not fit
// the last line is dropped.
std::vector<std::string> wrap(const MaskAtlas* a, const std::string& s, int max_w,
                              int max_lines) {
  std::vector<std::string> lines;
  const int n = a->advance_x > 0 ? std::max(1, max_w / a->advance_x) : 1;
  std::string line;
  int line_n = 0;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && s[i] == ' ') ++i;
    if (i >= s.size()) break;
    const size_t j = std::min(s.find(' ', i), s.size());
    std::string word = s.substr(i, j - i);
    i = j;
    int wn = (int)codepoints(word).size();
    if (line_n > 0 && line_n + 1 + wn <= n) {
      line += " " + word;
      line_n += 1 + wn;
      continue;
    }
    if (line_n > 0) {
      lines.push_back(line);
      if ((int)lines.size() == max_lines) return lines;
    }
    if (wn > n) {
      word = fit(a, word, n * a->advance_x);
      wn = n;
    }
    line = word;
    line_n = wn;
  }
  if (line_n > 0 && (int)lines.size() < max_lines) lines.push_back(line);
  return lines;
}

}  // namespace

DirtyRect MenuLayer::Box::rect() const {
  DirtyRect r{0, 0, 0, 0};
  for (const Block& b : blocks) r = union_of(r, b.rect());
  return r;
}

bool MenuLayer::Box::same_geometry(const Box& o) const {
  if (blocks.size() != o.blocks.size()) return false;
  for (size_t i = 0; i < blocks.size(); ++i)
    if (!same(blocks[i].rect(), o.blocks[i].rect())) return false;
  return true;
}

MenuLayer::MenuLayer(GsFont& font) : font_(font), target_(kBoxes), drawn_(kBoxes) {}

int MenuLayer::sc_(double design_px) const {
  return (int)std::lround(design_px * scale_ * screen_h_ / 1080.0);
}

int MenuLayer::clean_air_bars(double busy_pct) {
  if (busy_pct < 5) return 5;
  if (busy_pct < 12) return 4;
  if (busy_pct < 25) return 3;
  if (busy_pct < 50) return 2;
  return 1;
}

bool MenuLayer::layout(int screen_w, int screen_h, std::string* err) {
  screen_w_ = screen_w;
  screen_h_ = screen_h;
  if (!pick_atlases_()) {
    laid_out_ = false;
    if (err) *err = "menu: the GS font has no atlas";
    return false;
  }
  laid_out_ = true;
  invalid_ = true;
  rebuild_();
  return true;
}

void MenuLayer::set_scale(double scale) {
  if (scale == scale_) return;
  scale_ = scale;
  if (!laid_out_) return;
  // The font is the one layout() already found atlases in, so a re-pick
  // cannot come up empty.
  pick_atlases_();
  invalid_ = true;
  rebuild_();
}

bool MenuLayer::pick_atlases_() {
  row_ = font_.nearest(sc_(kRowPx));
  small_ = font_.nearest(sc_(kSmallPx));
  if (!row_ || !small_) return false;
  big_ = font_.nearest((int)std::lround(row_->px * kBigScale));
  return true;
}

void MenuLayer::set_view(const MenuView& v) {
  if (v == view_) return;
  view_ = v;
  rebuild_();
}

void MenuLayer::set_tick(uint64_t now_ms) {
  const int dots = (int)((now_ms / kDotMs) % (kMaxDots + 1));
  const int sweep = (int)((now_ms % kSweepMs) * kSweepSteps / kSweepMs);
  if (dots == dots_ && sweep == sweep_) return;
  dots_ = dots;
  sweep_ = sweep;
  for (const ViewRow& r : view_.rows)
    if (r.busy) {
      rebuild_();
      return;
    }
}

void MenuLayer::invalidate() { invalid_ = true; }

int MenuLayer::debug_row_px() const { return row_ ? row_->px : 0; }

DirtyRect MenuLayer::bounds() const {
  DirtyRect r{0, 0, 0, 0};
  for (const Box& b : target_) r = union_of(r, b.rect());
  return r;
}

bool MenuLayer::dirty() const {
  if (!laid_out_) return false;
  if (invalid_ && !empty(bounds())) return true;
  for (int k = 0; k < kBoxes; ++k)
    if (target_[(size_t)k].blocks != drawn_[(size_t)k].blocks) return true;
  return false;
}

DirtyRect MenuLayer::debug_panel_box() const { return target_[kPanel].rect(); }
DirtyRect MenuLayer::debug_picker_box() const { return target_[kPicker].rect(); }

std::vector<std::string> MenuLayer::debug_texts() const {
  std::vector<std::string> out;
  for (const Box& b : drawn_) {
    std::vector<const Run*> runs;
    for (const Block& bl : b.blocks)
      for (const Run& r : bl.runs) runs.push_back(&r);
    std::stable_sort(runs.begin(), runs.end(), [](const Run* a, const Run* c) {
      return std::tie(a->baseline, a->x) < std::tie(c->baseline, c->x);
    });
    for (const Run* r : runs) out.push_back(r->text);
  }
  return out;
}

void MenuLayer::rebuild_() {
  for (Box& b : target_) b.blocks.clear();
  if (!laid_out_) return;
  if (view_.open) {
    build_panel_(&target_[kPanel]);
    if (view_.picker != MenuView::Picker::kNone) {
      const DirtyRect p = target_[kPanel].rect();
      build_picker_(p.x + p.w + sc_(kPickerGap), p.y, &target_[kPicker]);
    }
  }
  // The banner last: while the menu is open it places itself below the
  // panel and picker built above.
  if (!view_.toast.empty()) build_note_(view_.toast, false, &target_[kToast]);
  if (!view_.banner.empty()) build_note_(view_.banner, true, &target_[kBanner]);
}

void MenuLayer::build_panel_(Box* box) const {
  // 45 % of the width, grown with the menu scale -- but never so wide that
  // a confirm picker (the widest fixed-width one) is squeezed against the
  // screen edge: on a 4:3 screen at the largest scale the panel gives way.
  const int x = sc_(kInset);
  const int room = screen_w_ - 2 * sc_(kInset) - sc_(kPickerGap) - sc_(kConfirmW);
  const int w = std::min((int)std::lround(kPanelW * scale_ * screen_w_), room);
  const int pad = sc_(kPadX), gap = sc_(kGap);
  const int inner = w - 2 * pad;
  int y = sc_(kInset);

  // Header: the crumb, and the link pill on the right.
  {
    Block b{x, y, w, small_->glyph_h + sc_(kLinePadY), tok::kMenuPanel, {}, {}};
    const int base = y + (b.h - small_->glyph_h) / 2 + small_->baseline;
    const std::string pill = view_.linked ? "● drone linked" : "○ no drone";
    const int pill_x = x + w - pad - tw(small_, pill);
    b.runs.push_back({small_, x + pad, base,
                      fit(small_, view_.crumb, pill_x - gap - (x + pad)), tok::kTextPrimary});
    b.runs.push_back({small_, pill_x, base, pill,
                      view_.linked ? tok::kStatusOk : tok::kTextLabel});
    b.fills.push_back({x, y + b.h - 1, w, 1, tok::kTrack});
    y += b.h;
    box->blocks.push_back(std::move(b));
  }

  // Rows: a text block, then the strip the busy sweep runs in.
  const int strip = std::max(2, sc_(2));
  const int text_h = row_->glyph_h + sc_(kRowPadY) - strip;
  for (size_t i = 0; i < view_.rows.size() && i < kMaxRows; ++i) {
    const ViewRow& r = view_.rows[i];
    const uint32_t bg = r.selected ? tok::kMenuSelect : tok::kMenuPanel;
    Block b{x, y, w, text_h, bg, {}, {}};
    const int base = y + (text_h - row_->glyph_h) / 2 + row_->baseline;
    int right = x + w - pad;

    if (!r.status.empty()) {
      std::string st = r.status;
      int reserve = tw(small_, st);
      if (r.busy) {
        // Room for three dots always, so the value does not walk as they cycle.
        st += std::string((size_t)dots_, '.');
        reserve += kMaxDots * small_->advance_x;
      }
      const uint32_t rgb = r.status_err  ? tok::kStatusCritical
                           : r.status_ok ? tok::kStatusOk
                                         : tok::kTextSecondary;
      right -= reserve;
      b.runs.push_back({small_, right, base, st, rgb});
      right -= gap;
    }
    {
      const std::string v = fit(row_, r.value, inner / 2);
      right -= tw(row_, v);
      b.runs.push_back({row_, right, base, v, r.dim ? tok::kTextLabel : tok::kTextPrimary});
      right -= row_->advance_x;
    }
    if (!r.warn.empty()) {
      const std::string full = r.warn.rfind("⚠", 0) == 0 ? r.warn : "⚠ " + r.warn;
      const std::string wt = fit(row_, full, std::max(0, right - (x + pad)) / 2);
      right -= tw(row_, wt);
      b.runs.push_back({row_, right, base, wt, tok::kStatusCaution});
      right -= row_->advance_x;
    }

    struct Tag { const char* text; uint32_t rgb; };
    std::vector<Tag> tags;
    if (r.vtx) tags.push_back({"VTX", tok::kMenuTag});
    if (r.adv) tags.push_back({"ADV", tok::kTextLabel});
    const int tpad = sc_(kTagPad);
    int tags_w = 0;
    for (const Tag& t : tags) tags_w += gap + tw(small_, t.text) + 2 * tpad;

    const std::string label = fit(row_, r.label, right - (x + pad) - tags_w);
    b.runs.push_back({row_, x + pad, base, label, r.dim ? tok::kTextLabel : tok::kTextPrimary});
    int tx = x + pad + tw(row_, label);
    for (const Tag& t : tags) {
      tx += gap;
      const int tw_ = tw(small_, t.text) + 2 * tpad;
      const int top = base - (int)std::lround(small_->px * 0.75) - sc_(3);
      const int bot = base + sc_(3);
      if (tx + tw_ > right) break;
      b.fills.push_back({tx, top, tw_, 1, t.rgb});
      b.fills.push_back({tx, bot - 1, tw_, 1, t.rgb});
      b.fills.push_back({tx, top, 1, bot - top, t.rgb});
      b.fills.push_back({tx + tw_ - 1, top, 1, bot - top, t.rgb});
      b.runs.push_back({small_, tx + tpad, base, t.text, t.rgb});
      tx += tw_;
    }
    std::stable_sort(b.runs.begin(), b.runs.end(),
                     [](const Run& a, const Run& c) { return a.x < c.x; });
    y += text_h;
    box->blocks.push_back(std::move(b));

    Block sb{x, y, w, strip, bg, {}, {}};
    if (r.busy) {
      const int seg = w / 3;
      const int x0 = std::max(x, x + sweep_ * w / kSweepSteps - seg / 2);
      const int x1 = std::min(x + w, x + sweep_ * w / kSweepSteps + seg - seg / 2);
      if (x1 > x0) sb.fills.push_back({x0, y, x1 - x0, strip, tok::kTextPrimary});
    }
    y += strip;
    box->blocks.push_back(std::move(sb));
  }

  // Help: two lines, red while it explains a failure.
  {
    Block b{x, y, w, kHelpLines * small_->glyph_h + sc_(kLinePadY), tok::kMenuPanel, {}, {}};
    b.fills.push_back({x, y, w, 1, tok::kTrack});
    const uint32_t rgb = view_.help_is_error ? tok::kStatusCritical : tok::kTextLabel;
    const auto lines = wrap(small_, view_.help, inner, kHelpLines);
    for (size_t i = 0; i < lines.size(); ++i)
      b.runs.push_back({small_, x + pad,
                        y + sc_(kLinePadY) / 2 + (int)i * small_->glyph_h + small_->baseline,
                        lines[i], rgb});
    y += b.h;
    box->blocks.push_back(std::move(b));
  }
  // Keys.
  {
    Block b{x, y, w, small_->glyph_h + sc_(kLinePadY), tok::kMenuPanel, {}, {}};
    b.runs.push_back({small_, x + pad, y + (b.h - small_->glyph_h) / 2 + small_->baseline,
                      fit(small_, view_.keys, inner), tok::kTextLabel});
    box->blocks.push_back(std::move(b));
  }
}

void MenuLayer::build_picker_(int x, int y, Box* box) const {
  const int pad = sc_(kPadX);
  const int max_w = std::max(0, screen_w_ - x - sc_(kInset));
  const int line_h = row_->glyph_h + sc_(kRowPadY);
  const std::string go = "▶ Go ahead", cancel = "◀ Cancel";
  auto choice_label = [&](size_t i) {
    return view_.choices[i].label + ((int)i == view_.picker_current ? " ·" : "");
  };

  // The width first: every block spans the whole box.
  int w = 0;
  switch (view_.picker) {
    case MenuView::Picker::kChoices:
      w = sc_(kPickerMinW);
      for (size_t i = 0; i < view_.choices.size(); ++i) {
        const Choice& c = view_.choices[i];
        const int note = c.note.empty() ? 0 : sc_(kNoteGap) + tw(small_, c.note);
        w = std::max(w, tw(row_, choice_label(i)) + note + 2 * pad);
      }
      break;
    case MenuView::Picker::kNumber:
      w = std::max({sc_(kNumberMinW), tw(big_, view_.number_text) + 2 * pad,
                    tw(small_, view_.number_range) + 2 * pad});
      break;
    case MenuView::Picker::kConfirm:
      w = std::max(sc_(kConfirmW), tw(row_, go) + 2 * pad);
      break;
    case MenuView::Picker::kNone:
      return;
  }
  w = std::min(w, max_w);
  const int inner = w - 2 * pad;

  auto pad_block = [&](int h) {
    box->blocks.push_back(Block{x, y, w, h, tok::kMenuPanel, {}, {}});
    y += h;
  };
  // One text line, centred in the box or flush left.
  auto line = [&](const MaskAtlas* a, int h, const std::string& t, uint32_t rgb, uint32_t bg,
                  bool centre) {
    Block b{x, y, w, h, bg, {}, {}};
    const std::string ft = fit(a, t, inner);
    b.runs.push_back({a, centre ? x + (w - tw(a, ft)) / 2 : x + pad,
                      y + (h - a->glyph_h) / 2 + a->baseline, ft, rgb});
    y += h;
    box->blocks.push_back(std::move(b));
  };

  pad_block(sc_(kBoxPadY));
  switch (view_.picker) {
    case MenuView::Picker::kChoices:
      for (size_t i = 0; i < view_.choices.size(); ++i) {
        const Choice& c = view_.choices[i];
        const uint32_t bg = (int)i == view_.picker_cursor ? tok::kMenuSelect : tok::kMenuPanel;
        Block b{x, y, w, line_h, bg, {}, {}};
        const int base = y + (line_h - row_->glyph_h) / 2 + row_->baseline;
        int right = x + w - pad;
        std::string note;
        if (!c.note.empty()) {
          note = fit(small_, c.note, inner / 2);
          right -= tw(small_, note) + sc_(kNoteGap);
        }
        b.runs.push_back({row_, x + pad, base, fit(row_, choice_label(i), right - (x + pad)),
                          c.pickable ? tok::kTextPrimary : tok::kTextLabel});
        if (!note.empty())
          b.runs.push_back({small_, x + w - pad - tw(small_, note), base, note, tok::kTextLabel});
        y += line_h;
        box->blocks.push_back(std::move(b));
      }
      break;
    case MenuView::Picker::kNumber:
      line(row_, line_h, "▲", tok::kTextLabel, tok::kMenuPanel, true);
      line(big_, big_->glyph_h + sc_(kRowPadY), view_.number_text, tok::kTextPrimary,
           tok::kMenuPanel, true);
      line(row_, line_h, "▼", tok::kTextLabel, tok::kMenuPanel, true);
      line(small_, small_->glyph_h + sc_(kLinePadY), view_.number_range, tok::kTextLabel,
           tok::kMenuPanel, true);
      break;
    case MenuView::Picker::kConfirm:
      for (const std::string& l : wrap(row_, view_.confirm_text, inner, kConfirmLines))
        line(row_, row_->glyph_h + sc_(2), l, tok::kTextPrimary, tok::kMenuPanel, false);
      pad_block(sc_(kBoxPadY));
      line(row_, line_h, go, tok::kTextPrimary, tok::kMenuSelect, false);
      line(row_, line_h, cancel, tok::kTextPrimary, tok::kMenuPanel, false);
      break;
    case MenuView::Picker::kNone:
      break;
  }
  pad_block(sc_(kBoxPadY));
}

void MenuLayer::build_note_(const std::string& text, bool banner, Box* box) const {
  const int pad = sc_(kPadX);
  const int max_inner = screen_w_ - 2 * sc_(kInset) - 2 * pad;
  std::vector<std::string> lines;
  size_t i = 0;
  while (i <= text.size()) {
    const size_t j = std::min(text.find('\n', i), text.size());
    lines.push_back(fit(row_, text.substr(i, j - i), max_inner));
    if (!banner) break;   // a toast is one line
    i = j + 1;
  }
  int w = 0;
  for (const std::string& l : lines) w = std::max(w, tw(row_, l));
  w += 2 * pad;
  const int line_h = row_->glyph_h + sc_(kRowPadY);
  const int h = line_h * (int)lines.size();
  const int x = (screen_w_ - w) / 2;
  // The toast area: one line, its bottom kToastBottom above the screen's.
  const int toast_top = screen_h_ - sc_(kToastBottom) - line_h;
  int y = banner ? (int)std::lround(kBannerTop * screen_h_) : toast_top;
  if (banner && view_.open) {
    // An open menu owns the top-left: the banner (a display mismatch, say)
    // is what the operator opened it to fix, so it moves below the panel
    // and picker rather than covering their values -- and above the toast
    // area when the panel reaches that far down.
    const DirtyRect menu = union_of(target_[kPanel].rect(), target_[kPicker].rect());
    y = menu.y + menu.h + sc_(kBannerGap);
    if (y + h > toast_top) y = toast_top - sc_(kBannerGap) - h;
  }
  const uint32_t rgb = banner ? tok::kStatusCaution : tok::kTextPrimary;
  for (const std::string& l : lines) {
    Block b{x, y, w, line_h, tok::kMenuPanel, {}, {}};
    b.runs.push_back({row_, x + (w - tw(row_, l)) / 2,
                      y + (line_h - row_->glyph_h) / 2 + row_->baseline, l, rgb});
    y += line_h;
    box->blocks.push_back(std::move(b));
  }
}

void MenuLayer::draw_block_(const Surface& s, const Block& b) {
  fill_rect(s, b.x, b.y, b.w, b.h, b.bg, kAlpha);
  for (const Fill& f : b.fills) fill_rect(s, f.x, f.y, f.w, f.h, f.rgb, kAlpha);
  for (const Run& r : b.runs) {
    if (r.text.empty()) continue;
    // The run's padded cell, drawn into scratch and composited over the
    // block: draw_text writes, which would replace the fill's alpha with
    // the glyph edge's.
    const MaskAtlas& a = *r.atlas;
    const int edge = a.glyph_w - a.advance_x;
    const int cw = tw(r.atlas, r.text) + edge, ch = a.glyph_h;
    const int ox = r.x - edge / 2, oy = r.baseline - a.baseline;
    scratch_.assign((size_t)cw * ch, 0u);
    const Surface t{scratch_.data(), cw, ch, cw};
    draw_text(t, a, edge / 2, a.baseline, r.text.c_str(), r.rgb);
    for (int yy = 0; yy < ch; ++yy) {
      const int py = oy + yy;
      if (py < 0 || py >= s.height) continue;
      uint32_t* row = s.pixels + (size_t)py * s.stride_px;
      const uint32_t* src = scratch_.data() + (size_t)yy * cw;
      for (int xx = 0; xx < cw; ++xx) {
        const int px = ox + xx;
        if (px < 0 || px >= s.width) continue;
        const uint32_t sp = src[xx];
        const uint32_t sa = sp >> 24;
        if (sa == 0) continue;
        if (sa == 255) {
          row[px] = sp;
          continue;
        }
        // Premultiplied OVER: out = src + dst * (1 - src_alpha), per channel.
        const uint32_t dp = row[px];
        uint32_t o = 0;
        for (int sh = 0; sh < 32; sh += 8) {
          const uint32_t c = ((sp >> sh) & 0xFF) + ((dp >> sh) & 0xFF) * (255 - sa) / 255;
          o |= std::min(c, 255u) << sh;
        }
        row[px] = o;
      }
    }
  }
}

int MenuLayer::paint(const Surface& s, std::vector<DirtyRect>* out,
                     std::vector<DirtyRect>* erased) {
  if (!s.pixels || !laid_out_) return 0;
  int n = 0;
  auto emit = [&](const DirtyRect& r) {
    if (out) out->push_back(r);
    ++n;
  };

  // Erase every box whose geometry moved before drawing any: a later box's
  // erase must not take an earlier box's fresh pixels with it.
  std::vector<DirtyRect> damage;
  bool full[kBoxes], moved[kBoxes];
  for (int k = 0; k < kBoxes; ++k) {
    const Box& old = drawn_[(size_t)k];
    moved[k] = !target_[(size_t)k].same_geometry(old);
    full[k] = invalid_ || moved[k];
    const DirtyRect r = old.rect();
    if (moved[k] && !empty(r)) {
      clear_region(s, r);
      damage.push_back(r);
      if (erased) erased->push_back(r);
      emit(r);
    }
  }

  // Draw in box order; a block under fresh damage (an erase, or an earlier
  // box's redraw) is redrawn even when unchanged.
  for (int k = 0; k < kBoxes; ++k) {
    const Box& old = drawn_[(size_t)k];
    const Box& neu = target_[(size_t)k];
    for (size_t i = 0; i < neu.blocks.size(); ++i) {
      const Block& b = neu.blocks[i];
      const DirtyRect br = b.rect();
      bool redraw = full[k] || !(b == old.blocks[i]);
      for (size_t d = 0; !redraw && d < damage.size(); ++d) redraw = intersects(br, damage[d]);
      if (!redraw) continue;
      draw_block_(s, b);
      damage.push_back(br);
      if (!full[k]) emit(br);
    }
    // A whole box is one rect; skip it when the erase above already
    // reported exactly this one.
    const DirtyRect r = neu.rect();
    if (full[k] && !empty(r) && !(moved[k] && same(r, old.rect()))) emit(r);
  }

  drawn_ = target_;
  invalid_ = false;
  return n;
}

int MenuLayer::repaint_intersecting(const DirtyRect* rects, size_t n, const Surface& s,
                                    std::vector<DirtyRect>* out) {
  if (!s.pixels || !laid_out_ || !rects || n == 0) return 0;
  std::vector<DirtyRect> damage(rects, rects + n);
  int drawn = 0;
  for (const Box& box : drawn_) {
    for (const Block& b : box.blocks) {
      const DirtyRect br = b.rect();
      bool hit = false;
      for (size_t d = 0; !hit && d < damage.size(); ++d) hit = intersects(br, damage[d]);
      if (!hit) continue;
      draw_block_(s, b);
      damage.push_back(br);
      if (out) out->push_back(br);
      ++drawn;
    }
  }
  return drawn;
}

}  // namespace maburplay
