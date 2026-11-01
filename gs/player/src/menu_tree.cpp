#include "menu_tree.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

namespace maburplay {
namespace {

using Deps = std::shared_ptr<const MenuTreeDeps>;
using Steps = std::vector<SetStep>;

constexpr size_t kMaxRungs = 8;   // maburgs' loader refuses a longer ladder
constexpr int kMaxMcs = 5;        // the bundle's link.max_mcs: a higher rung would be dropped
constexpr const char* kSplit = "Splits the recording (a new file starts).";

// The shortest decimal that reads back as `v`: "0.5", "0.25", "1". maburgs'
// loader takes a bare integer for a real key, so "1" needs no decimal point.
std::string real_text(double v) {
  char buf[64];
  for (int d = 0; d <= 15; ++d) {
    std::snprintf(buf, sizeof buf, "%.*f", d, v);
    if (std::strtod(buf, nullptr) == v) return buf;
  }
  std::snprintf(buf, sizeof buf, "%.17g", v);
  return buf;
}

std::string fx(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.2f", v);
  return buf;
}

double num(const std::string& v) { return std::strtod(v.c_str(), nullptr); }
std::string int_text(const std::string& v) { return std::to_string(std::lround(num(v))); }
std::string quoted(const std::string& s) { return "\"" + s + "\""; }
std::string bool_text(bool b) { return b ? "true" : "false"; }
std::string size_text(int w, int h) { return std::to_string(w) + "x" + std::to_string(h); }
std::string size_label(int w, int h) { return std::to_string(w) + "×" + std::to_string(h); }

std::function<std::string()> text(std::string s) {
  return [s = std::move(s)] { return s; };
}

std::vector<Choice> on_off() { return {{"true", "On", "", true}, {"false", "Off", "", true}}; }

// A mode string "WxH@R" split; false when it is not one.
bool parse_mode(const std::string& s, int* w, int* h, double* hz) {
  return std::sscanf(s.c_str(), "%dx%d@%lf", w, h, hz) == 3 && *w > 0 && *h > 0 && *hz > 0;
}

// 100 -> "100", 59.94 -> "59.94".
std::string hz_text(double hz) {
  std::string s = fx(hz);
  while (s.back() == '0') s.pop_back();
  if (s.back() == '.') s.pop_back();
  return s;
}

// The distinct progressive sizes the display offers within kModeMatchHz of
// `camera_hz` (every size while the rate is unknown), largest first.
std::vector<std::pair<int, int>> sizes_at(const std::vector<ModeCand>& modes, double camera_hz) {
  std::vector<std::pair<int, int>> out;
  for (const ModeCand& m : modes) {
    if (m.interlaced) continue;
    if (camera_hz > 0 && std::fabs(mode_hz(m) - camera_hz) > kModeMatchHz) continue;
    const std::pair<int, int> s{m.w, m.h};
    if (std::find(out.begin(), out.end(), s) == out.end()) out.push_back(s);
  }
  std::stable_sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    return static_cast<long>(a.first) * a.second > static_cast<long>(b.first) * b.second;
  });
  return out;
}

bool fixed(const Deps& d) { return d->gs->static_mcs >= 0; }
bool burning(const Deps& d) { return d->burned_rec_running && d->burned_rec_running(); }
bool recording(const Deps& d) { return d->rec_running && d->rec_running(); }

std::function<std::string()> next_rec_saved(const Deps& d) {
  return [d] { return std::string(recording(d) ? "✓ at next recording" : "✓ saved"); };
}

// A choice row with one set of `key` to `owner`; `render` turns the picked
// choice value into the TOML text the key takes.
MenuNode choice_row(std::string id, std::string label, Owner owner, std::string key,
                    std::function<std::string(const std::string&)> render = nullptr) {
  MenuNode n;
  n.id = std::move(id);
  n.label = std::move(label);
  n.kind = NodeKind::kChoice;
  n.owner = owner;
  n.steps = [owner, key = std::move(key), render = std::move(render)](const std::string& v) {
    return Steps{{owner, key, render ? render(v) : v}};
  };
  return n;
}

MenuNode group(std::string id, std::string label, std::string help,
               std::function<std::vector<MenuNode>()> kids) {
  MenuNode n;
  n.id = std::move(id);
  n.label = std::move(label);
  n.kind = NodeKind::kGroup;
  n.help = text(std::move(help));
  n.children = std::move(kids);
  return n;
}

// --- Video -------------------------------------------------------------

MenuNode display_row(const Deps& d) {
  MenuNode n = choice_row("video.display", "Display", Owner::kPlayer, "screen_mode", quoted);
  n.value_text = [d] {
    int w = 0, h = 0;
    double hz = 0;
    if (!d->running_mode || !parse_mode(d->running_mode(), &w, &h, &hz)) return std::string("—");
    return size_label(w, h) + " @ " + hz_text(hz) + " Hz";
  };
  // An empty mode list is a display that could not be read, not one that
  // lacks the camera's rate: no warning then.
  n.warn = [d] {
    const double cam = d->camera_hz ? d->camera_hz() : 0;
    const auto modes = d->modes ? d->modes() : std::vector<ModeCand>{};
    if (cam <= 0 || modes.empty() || !sizes_at(modes, cam).empty()) return std::string();
    return std::string("≠ camera");
  };
  n.choices = [d] {
    std::vector<Choice> cs{{"auto", "Auto (largest)", "", true}};
    const auto modes = d->modes ? d->modes() : std::vector<ModeCand>{};
    for (const auto& [w, h] : sizes_at(modes, d->camera_hz ? d->camera_hz() : 0))
      cs.push_back({size_text(w, h), size_label(w, h), "", true});
    return cs;
  };
  n.current = [d] {
    const ScreenWant want = parse_screen_want(d->cfg->screen_mode);
    if (!want.ok || want.kind == ScreenWant::Kind::kAuto) return std::string("auto");
    return size_text(want.w, want.h);
  };
  n.confirm = [d](const std::string&) { return std::string(burning(d) ? kSplit : ""); };
  n.help = [d] {
    if (d->snap && d->snap->low_power)
      return std::string("Drone is disarmed: its lower frame rate is on purpose, the display stays put.");
    return std::string(
        "Follows the camera's rate. Auto takes the largest size this display offers at that "
        "rate; pick a size to use that one instead.");
  };
  return n;
}

MenuNode colour_row(const Deps& d) {
  MenuNode n = choice_row("video.colour", "Colour correction", Owner::kPlayer, "colortrans.enable");
  n.choices = on_off;
  n.current = [d] { return bool_text(d->cfg->colortrans.enable); };
  n.confirm = [d](const std::string&) { return std::string(burning(d) ? kSplit : ""); };
  n.help = text("Corrects the camera's colour on the display and in burned recordings.");
  return n;
}

// --- Link --------------------------------------------------------------

// The rung Fixed pins: R3, or the top rung of a shorter ladder. The
// ladder must not be empty.
Rung current_rung(const GsLiveView& gs) {
  return gs.ladder[std::min<size_t>(3, gs.ladder.size() - 1)];
}

MenuNode width_row(const Deps& d) {
  MenuNode n;
  n.id = "link.width";
  n.label = "Width";
  n.kind = NodeKind::kChoice;
  n.owner = Owner::kGs;
  n.choices = [] { return std::vector<Choice>{{"40", "40 MHz", "", true}, {"20", "20 MHz", "", true}}; };
  n.current = [d] {
    if (fixed(d)) return std::to_string(d->gs->static_bw);
    switch (ladder_width(d->gs->ladder)) {
      case LadderWidth::k40: return std::string("40");
      case LadderWidth::k20: return std::string("20");
      case LadderWidth::kMixed: break;
    }
    return std::string("mixed");
  };
  n.value_text = [d] {
    if (fixed(d)) return std::to_string(d->gs->static_bw) + " MHz";
    switch (ladder_width(d->gs->ladder)) {
      case LadderWidth::k40: return std::string("40 MHz");
      case LadderWidth::k20: return std::string("20 MHz");
      case LadderWidth::kMixed: break;
    }
    return std::string("Mixed");
  };
  n.steps = [d](const std::string& v) {
    if (fixed(d)) return Steps{{Owner::kGs, "link.static_bw", v}};
    return Steps{{Owner::kGs, "link.ladder", render_ladder_value(default_ladder(std::atoi(v.c_str())))}};
  };
  n.confirm = [d](const std::string& v) {
    if (fixed(d) || is_default_ladder(d->gs->ladder)) return std::string();
    return "Replaces your ladder with the default " + v + " MHz ladder (" +
           std::to_string(default_ladder(std::atoi(v.c_str())).size()) + " rungs).";
  };
  n.help = [d] {
    if (fixed(d)) return std::string("Channel width for the fixed rate.");
    if (ladder_width(d->gs->ladder) == LadderWidth::kMixed)
      return std::string(
          "Your ladder mixes 20 and 40 MHz rungs (edit them in Ladder). Picking 20 or 40 loads "
          "that width's default ladder.");
    return std::string(
        "40 MHz carries about twice the data; 20 MHz reaches a few dB further at the lowest "
        "rung. Picking a width loads its default ladder.");
  };
  return n;
}

MenuNode rate_row(const Deps& d) {
  MenuNode n;
  n.id = "link.rate";
  n.label = "Rate";
  n.kind = NodeKind::kChoice;
  n.owner = Owner::kGs;
  n.choices = [] {
    return std::vector<Choice>{{"adaptive", "Adaptive", "", true}, {"fixed", "Fixed MCS", "", true}};
  };
  n.current = [d] { return std::string(fixed(d) ? "fixed" : "adaptive"); };
  n.value_text = [d] { return std::string(fixed(d) ? "Fixed MCS" : "Adaptive"); };
  // Fixed: the pair and width first -- they change nothing while
  // static_mcs is still -1 -- then static_mcs, so the link moves once.
  // With no ladder read there is no rung to pin: nothing is sent.
  n.steps = [d](const std::string& v) {
    if (v != "fixed") return Steps{{Owner::kGs, "link.static_mcs", "-1"}};
    if (d->gs->ladder.empty()) return Steps{};
    const Rung r = current_rung(*d->gs);
    return Steps{{Owner::kGs, "link.static_bw", std::to_string(r.bw)},
                 {Owner::kGs, "link.static_overhead_base", real_text(r.base)},
                 {Owner::kGs, "link.static_overhead_enh", real_text(r.enh)},
                 {Owner::kGs, "link.static_mcs", std::to_string(std::min(r.mcs, kMaxMcs))}};
  };
  n.help = [d] {
    if (fixed(d))
      return std::string(
          "Fixed holds one setting. Adaptive is one set (static_mcs = -1) and re-enters the "
          "ladder on this rung.");
    return std::string(
        "Adaptive climbs and falls the ladder with the link. Fixed pins the link where it is "
        "now: four sets, MCS last, so it moves once.");
  };
  return n;
}

MenuNode number_row(std::string id, std::string label, Owner owner, double min, double max,
                    double step, std::function<double()> get, std::function<std::string(double)> fmt) {
  MenuNode n;
  n.id = std::move(id);
  n.label = std::move(label);
  n.kind = NodeKind::kNumber;
  n.owner = owner;
  n.number.min = min;
  n.number.max = max;
  n.number.step = step;
  n.number.get = std::move(get);
  n.number.fmt = std::move(fmt);
  return n;
}

std::string mcs_fmt(double v) { return "MCS" + std::to_string(std::lround(v)); }

std::vector<MenuNode> fixed_rows(const Deps& d) {
  auto under_fixed = [d] { return fixed(d); };
  MenuNode mcs = number_row("link.mcs", "  MCS", Owner::kGs, 0, kMaxMcs, 1,
                            [d] { return static_cast<double>(d->gs->static_mcs); }, mcs_fmt);
  mcs.visible = under_fixed;
  mcs.steps = [](const std::string& v) { return Steps{{Owner::kGs, "link.static_mcs", int_text(v)}}; };
  mcs.help = text("The one modulation the link holds.");

  MenuNode base = number_row("link.fec_base", "  FEC base", Owner::kGs, 0.10, 2.00, 0.05,
                             [d] { return d->gs->static_base; }, fx);
  base.visible = under_fixed;
  // Base never reads below enh (the base layer every frame depends on
  // keeps at least the enhancement layer's protection): a base under the
  // current enh takes enh down to it first.
  base.steps = [d](const std::string& v) {
    if (num(v) < d->gs->static_enh)
      return Steps{{Owner::kGs, "link.static_overhead_enh", v},
                   {Owner::kGs, "link.static_overhead_base", v}};
    return Steps{{Owner::kGs, "link.static_overhead_base", v}};
  };
  base.help = text("Repair overhead on the base layer.");

  MenuNode enh = number_row("link.fec_enh", "  FEC enh", Owner::kGs, 0.10, d->gs->static_base, 0.05,
                            [d] { return d->gs->static_enh; }, fx);
  enh.visible = under_fixed;
  enh.steps = [](const std::string& v) { return Steps{{Owner::kGs, "link.static_overhead_enh", v}}; };
  enh.help = text("Repair overhead on the enhancement layer, at most the base.");
  return {mcs, base, enh};
}

// Rung `i` of the live ladder, or a default rung once the ladder has
// shrunk under a row built when it was longer.
Rung rung_at(const Deps& d, size_t i) { return i < d->gs->ladder.size() ? d->gs->ladder[i] : Rung{}; }

// The live key takes the whole ladder: every rung edit sends all of it.
Steps ladder_set(std::vector<Rung> l) { return Steps{{Owner::kGs, "link.ladder", render_ladder_value(l)}}; }

// Nothing when rung `i` is no longer there: the row was built for a
// longer ladder than the one now read.
Steps ladder_with(const Deps& d, size_t i, const std::function<void(Rung*)>& change) {
  std::vector<Rung> l = d->gs->ladder;
  if (i >= l.size()) return Steps{};
  change(&l[i]);
  return ladder_set(std::move(l));
}

MenuNode action_row(std::string id, std::string label, std::function<Steps()> action) {
  MenuNode n;
  n.id = std::move(id);
  n.label = std::move(label);
  n.kind = NodeKind::kAction;
  n.owner = Owner::kGs;
  n.action = std::move(action);
  return n;
}

MenuNode rung_group(const Deps& d, size_t i) {
  const std::string id = "link.ladder.r" + std::to_string(i);
  MenuNode g = group(id, "R" + std::to_string(i),
                     "One step of the adaptive link. Lower rungs reach further, higher rungs carry more.",
                     nullptr);
  g.group_value = [d, i] {
    const Rung r = rung_at(d, i);
    return "MCS" + std::to_string(r.mcs) + " · " + std::to_string(r.bw) + " MHz · " + fx(r.base) +
           "/" + fx(r.enh);
  };
  g.children = [d, i, id] {
    const Rung r = rung_at(d, i);
    MenuNode mcs = number_row(id + ".mcs", "MCS", Owner::kGs, 0, kMaxMcs, 1,
                              [d, i] { return static_cast<double>(rung_at(d, i).mcs); }, mcs_fmt);
    mcs.steps = [d, i](const std::string& v) {
      return ladder_with(d, i, [&](Rung* x) { x->mcs = static_cast<int>(std::lround(num(v))); });
    };
    mcs.help = text("Modulation for this rung (capped at 5). 0 reaches furthest.");

    MenuNode bw = choice_row(id + ".bw", "Width", Owner::kGs, "link.ladder");
    bw.choices = [] { return std::vector<Choice>{{"40", "40 MHz", "", true}, {"20", "20 MHz", "", true}}; };
    bw.current = [d, i] { return std::to_string(rung_at(d, i).bw); };
    bw.steps = [d, i](const std::string& v) {
      return ladder_with(d, i, [&](Rung* x) { x->bw = std::atoi(v.c_str()); });
    };
    bw.help = text("40 MHz carries ~2x the data; 20 MHz reaches a few dB further at MCS0.");

    MenuNode base = number_row(id + ".base", "FEC base", Owner::kGs, 0.10, 2.00, 0.05,
                               [d, i] { return rung_at(d, i).base; }, fx);
    // maburgs refuses a rung whose base is below its enh: enh follows a
    // lowered base down.
    base.steps = [d, i](const std::string& v) {
      return ladder_with(d, i, [&](Rung* x) {
        x->base = num(v);
        x->enh = std::min(x->enh, x->base);
      });
    };
    base.help = text("Repair overhead on the base layer. Must stay ≥ enhancement.");

    MenuNode enh = number_row(id + ".enh", "FEC enh", Owner::kGs, 0.10, r.base, 0.05,
                              [d, i] { return rung_at(d, i).enh; }, fx);
    enh.steps = [d, i](const std::string& v) {
      return ladder_with(d, i, [&](Rung* x) { x->enh = num(v); });
    };
    enh.help = text("Repair overhead on the enhancement layer. At most the base value.");

    MenuNode up = action_row(id + ".up", "Move up", [d, i] {
      std::vector<Rung> l = d->gs->ladder;
      if (i == 0 || i >= l.size()) return Steps{};
      std::swap(l[i - 1], l[i]);
      return ladder_set(std::move(l));
    });
    up.visible = [i] { return i > 0; };
    MenuNode down = action_row(id + ".down", "Move down", [d, i] {
      std::vector<Rung> l = d->gs->ladder;
      if (i + 1 >= l.size()) return Steps{};
      std::swap(l[i], l[i + 1]);
      return ladder_set(std::move(l));
    });
    down.visible = [d, i] { return i + 1 < d->gs->ladder.size(); };
    MenuNode del = action_row(id + ".del", "Delete rung", [d, i] {
      std::vector<Rung> l = d->gs->ladder;
      if (l.size() <= 1 || i >= l.size()) return Steps{};
      l.erase(l.begin() + static_cast<long>(i));
      return ladder_set(std::move(l));
    });
    del.visible = [d] { return d->gs->ladder.size() > 1; };
    del.confirm = [](const std::string&) { return std::string("Delete this rung?"); };
    return std::vector<MenuNode>{mcs, bw, base, enh, up, down, del};
  };
  return g;
}

MenuNode ladder_group(const Deps& d) {
  MenuNode g = group("link.ladder", "Ladder", "The steps the adaptive link moves between, slowest first.",
                     nullptr);
  g.owner = Owner::kGs;
  g.advanced = true;
  g.visible = [d] { return !fixed(d); };
  g.group_value = [d] { return std::to_string(d->gs->ladder.size()) + " rungs"; };
  g.children = [d] {
    std::vector<MenuNode> rows;
    for (size_t i = 0; i < d->gs->ladder.size(); ++i) rows.push_back(rung_group(d, i));
    MenuNode add = action_row("link.ladder.add", "+ Add rung", [d] {
      std::vector<Rung> l = d->gs->ladder;
      if (l.empty() || l.size() >= kMaxRungs) return Steps{};
      Rung r = l.back();
      r.mcs = std::min(r.mcs + 1, kMaxMcs);
      l.push_back(r);
      return ladder_set(std::move(l));
    });
    add.visible = [d] { return !d->gs->ladder.empty() && d->gs->ladder.size() < kMaxRungs; };
    rows.push_back(add);
    return rows;
  };
  return g;
}

MenuNode nack_row(const Deps& d) {
  MenuNode n = choice_row("link.nack", "Retransmit (NACK)", Owner::kGs, "link.nack.enable");
  n.advanced = true;
  n.choices = on_off;
  n.current = [d] { return bool_text(d->gs->nack); };
  n.help = text("Asks the drone to resend lost base-layer packets.");
  return n;
}

// --- Recording ---------------------------------------------------------

bool gs_records(const Deps& d) { return d->cfg->dvr.target != "vtx"; }
bool burned_rows(const Deps& d) { return gs_records(d) && d->cfg->dvr.mode == "burned"; }

std::string ordinal(int n) {
  if (n == 1) return "frame";
  if (n == 2) return "2nd";
  if (n == 3) return "3rd";
  return std::to_string(n) + "th";
}

std::vector<MenuNode> rec_rows(const Deps& d) {
  MenuNode target = choice_row("rec.target", "REC records to", Owner::kPlayer, "dvr.target", quoted);
  target.choices = [] {
    return std::vector<Choice>{{"gs", "GS", "", true}, {"vtx", "VTX SD card", "", true}, {"both", "Both", "", true}};
  };
  target.current = [d] { return d->cfg->dvr.target; };
  target.saved_text = next_rec_saved(d);
  target.help = text("Where the REC button records.");

  MenuNode mode = choice_row("rec.mode", "GS recording", Owner::kPlayer, "dvr.mode", quoted);
  mode.visible = [d] { return gs_records(d); };
  mode.choices = [] {
    return std::vector<Choice>{{"burned", "Burned (with OSD)", "", true}, {"raw", "Raw (as received)", "", true}};
  };
  mode.current = [d] { return d->cfg->dvr.mode; };
  mode.saved_text = next_rec_saved(d);
  mode.help = text("Burned re-encodes the screen with OSD and colour. Raw saves the received stream untouched.");

  auto burned = [d] { return burned_rows(d); };
  MenuNode bitrate = number_row(
      "rec.burn.bitrate", "  Bitrate", Owner::kPlayer, 2000, 20000, 1000,
      [d] { return static_cast<double>(d->cfg->dvr.burned.bitrate_kbps); },
      [](double v) { return real_text(v / 1000) + " Mb/s"; });
  bitrate.advanced = true;
  bitrate.visible = burned;
  bitrate.steps = [](const std::string& v) {
    return Steps{{Owner::kPlayer, "dvr.burned.bitrate_kbps", int_text(v)}};
  };
  bitrate.saved_text = next_rec_saved(d);
  bitrate.help = text("Burned recording bitrate.");

  MenuNode every = choice_row("rec.burn.every", "  Frame rate", Owner::kPlayer, "dvr.burned.every");
  every.advanced = true;
  every.visible = burned;
  every.choices = [d] {
    int w = 0, h = 0;
    double hz = 0;
    const bool known = d->running_mode && parse_mode(d->running_mode(), &w, &h, &hz);
    std::vector<Choice> cs;
    for (int k = 1; k <= 4; ++k) {
      const std::string each = "every " + ordinal(k);
      cs.push_back({std::to_string(k),
                    known ? std::to_string(std::lround(hz / k)) + " fps (" + each + ")" : each, "", true});
    }
    return cs;
  };
  every.current = [d] { return std::to_string(d->cfg->dvr.burned.every); };
  every.saved_text = next_rec_saved(d);
  every.help = text("Record every frame, or every Nth. Use every 2nd at 100 Hz.");

  MenuNode size = choice_row("rec.burn.size", "  Size", Owner::kPlayer, "dvr.burned.size", quoted);
  size.advanced = true;
  size.visible = burned;
  size.choices = [] {
    return std::vector<Choice>{{"", "Same as video", "", true},
                               {"1920x1080", size_label(1920, 1080), "", true},
                               {"1280x720", size_label(1280, 720), "", true},
                               {"2560x1440", size_label(2560, 1440), "", true}};
  };
  size.current = [d] {
    const auto& b = d->cfg->dvr.burned;
    return b.size_w == 0 && b.size_h == 0 ? std::string() : size_text(b.size_w, b.size_h);
  };
  size.saved_text = next_rec_saved(d);
  size.help = text("Burned recording size. A different shape gets black bars, never a stretch.");
  return {target, mode, bitrate, every, size};
}

// --- OSD ---------------------------------------------------------------

std::vector<MenuNode> osd_rows(const Deps& d) {
  // Turning an overlay on rebuilds the presenter when it adds the OSD
  // plane; turning one off never splits.
  auto split_on = [d](const std::string& v) {
    return std::string(v == "true" && burning(d) ? kSplit : "");
  };
  MenuNode bar = choice_row("osd.gs", "Link stats bar", Owner::kPlayer, "osd.gs.enable");
  bar.choices = on_off;
  bar.current = [d] { return bool_text(d->cfg->osd.gs.enable); };
  bar.confirm = split_on;
  bar.help = text("The two rows of link and picture numbers along the bottom.");

  // Two halves: maburplay's render and maburgs' MSP feed. The render goes
  // on first and off last, so it never waits on a feed that is off.
  MenuNode fc;
  fc.id = "osd.fc";
  fc.label = "Flight controller OSD";
  fc.kind = NodeKind::kChoice;
  fc.owner = Owner::kGs;
  fc.choices = on_off;
  fc.current = [d] {
    const bool render = d->cfg->osd.enable, feed = d->gs->msp;
    if (render && feed) return std::string("true");
    if (!render && !feed) return std::string("false");
    return std::string("mixed");
  };
  fc.value_text = [d] {
    const bool render = d->cfg->osd.enable, feed = d->gs->msp;
    return std::string(render && feed ? "On" : !render && !feed ? "Off" : "mixed");
  };
  fc.steps = [](const std::string& v) {
    if (v == "true")
      return Steps{{Owner::kPlayer, "osd.enable", "true"}, {Owner::kGs, "msp.enable", "true"}};
    return Steps{{Owner::kGs, "msp.enable", "false"}, {Owner::kPlayer, "osd.enable", "false"}};
  };
  fc.confirm = split_on;
  fc.help = [d] {
    const bool render = d->cfg->osd.enable, feed = d->gs->msp;
    if (render && !feed)
      return std::string(
          "The OSD render is on but the GS's MSP feed is off, so nothing reaches it. Pick On or "
          "Off to set both.");
    if (!render && feed)
      return std::string(
          "The GS's MSP feed is on but the OSD render is off, so nothing is drawn. Pick On or Off "
          "to set both.");
    return std::string("Draws the flight controller's OSD (Betaflight / INAV).");
  };
  return {bar, fc};
}

// --- System ------------------------------------------------------------

std::vector<MenuNode> sys_rows(const Deps& d) {
  MenuNode adv = choice_row("sys.adv", "Show advanced settings", Owner::kPlayer, "menu.show_advanced");
  adv.choices = on_off;
  adv.current = [d] { return bool_text(d->cfg->menu.show_advanced); };
  adv.help = text("Shows the ADV rows in every group.");

  MenuNode size = choice_row(kMenuSizeRow, "Menu size", Owner::kPlayer, "menu.scale", quoted);
  size.choices = [] {
    return std::vector<Choice>{{"normal", "Normal", "", true},
                               {"large", "Large", "", true},
                               {"larger", "Larger", "", true}};
  };
  size.current = [d] { return d->cfg->menu.scale; };
  size.help = text("Makes this menu bigger; ▲▼ shows each size, ◀ keeps the old one. The OSDs keep their size.");

  MenuNode debug = choice_row("sys.debug", "Debug log", Owner::kGs, "debug_log.enable");
  debug.choices = on_off;
  debug.current = [d] { return bool_text(d->gs->debug_log); };
  debug.help = text("Writes a per-session debug log to the GS DVR card.");
  return {adv, size, debug};
}

// --- Unread GS values ----------------------------------------------------

// A GS row reads "—" until maburgs' first `get` lands: until then the view
// holds GsLiveView's defaults, which are not maburgs' values. A row without
// a value_text of its own gets the text the model would show for it.
void dash_until_read(const Deps& d, MenuNode* n) {
  if (n->kind == NodeKind::kGroup) {
    n->group_value = [d, gv = n->group_value] {
      return d->gs->have ? (gv ? gv() : std::string()) : std::string("—");
    };
    return;
  }
  std::function<std::string()> shown = n->value_text;
  if (!shown && n->kind == NodeKind::kChoice && n->choices && n->current) {
    shown = [choices = n->choices, current = n->current] {
      const std::string cur = current();
      for (const Choice& c : choices())
        if (c.value == cur) return c.label;
      return cur;
    };
  }
  if (!shown && n->kind == NodeKind::kNumber && n->number.get && n->number.fmt)
    shown = [s = n->number] { return s.fmt(s.get()); };
  if (!shown) return;
  n->value_text = [d, shown] { return d->gs->have ? shown() : std::string("—"); };
}

// Applies dash_until_read to every GS value row under `rows`, at any depth.
std::vector<MenuNode> guarded(const Deps& d, std::vector<MenuNode> rows) {
  for (MenuNode& n : rows) {
    if (n.kind == NodeKind::kGroup && n.children)
      n.children = [d, kids = n.children] { return guarded(d, kids()); };
    if (n.owner == Owner::kGs && n.kind != NodeKind::kAction && n.kind != NodeKind::kInfo)
      dash_until_read(d, &n);
  }
  return rows;
}

}  // namespace

std::vector<MenuNode> build_menu_tree(const MenuTreeDeps& deps) {
  const Deps d = std::make_shared<const MenuTreeDeps>(deps);
  return guarded(d, {
      group("video", "Video", "Camera, display and picture.",
            [d] { return std::vector<MenuNode>{display_row(d), colour_row(d)}; }),
      group("link", "Link", "How the video crosses the air.",
            [d] {
              std::vector<MenuNode> rows{width_row(d), rate_row(d)};
              for (MenuNode& n : fixed_rows(d)) rows.push_back(std::move(n));
              rows.push_back(ladder_group(d));
              rows.push_back(nack_row(d));
              return rows;
            }),
      group("rec", "Recording", "What REC records, and where.", [d] { return rec_rows(d); }),
      group("osd", "OSD", "What is drawn over the video.", [d] { return osd_rows(d); }),
      group("sys", "System", "GS housekeeping.", [d] { return sys_rows(d); }),
  });
}

}  // namespace maburplay
