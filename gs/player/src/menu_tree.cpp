#include "menu_tree.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <utility>

#include "mabur/camera_modes.h"
#include "mabur/config_params.h"

namespace maburplay {

int clean_air_level(double busy_pct) {
  if (busy_pct < 5) return 5;
  if (busy_pct < 12) return 4;
  if (busy_pct < 25) return 3;
  if (busy_pct < 50) return 2;
  return 1;
}

std::string clean_air_bars(std::optional<double> busy_pct) {
  const int level = busy_pct ? clean_air_level(*busy_pct) : 0;
  std::string out;
  for (int i = 0; i < 5; ++i) out += i < level ? "●" : "○";
  return out;
}

std::string age_text(int64_t age_ms) {
  const int64_t s = age_ms / 1000;
  if (s < 60) return std::to_string(s) + " s ago";
  return std::to_string(s / 60) + " min ago";
}

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

// --- Video drone rows ---------------------------------------------------

// A pause every Rebuild-apply drone row shares: the encoder pipeline stops
// and restarts in-process.
constexpr const char* kPause = "Video pauses about half a second.";

// Non-empty sentences joined with one space, rebuild sentence first.
std::string join_confirm(std::initializer_list<std::string> parts) {
  std::string out;
  for (const std::string& p : parts) {
    if (p.empty()) continue;
    if (!out.empty()) out += " ";
    out += p;
  }
  return out;
}

// The row's value text off the sideport, or nullopt with no snapshot or no
// such row.
std::optional<std::string> vtx(const Deps& d, const char* id) {
  if (!d->snap) return std::nullopt;
  return vtx_value(*d->snap, id);
}

// What stops a drone row from applying, checked after the model's own
// "Needs the drone linked": a table mismatch refuses every drone row, and
// a row whose own value has not landed yet refuses until it has -- with
// "This setting did not answer." once the list has settled "partial"
// (some rows answered, this one never will), rather than "Reading the
// drone's settings…" forever for a row that is in fact done waiting.
std::function<std::string()> drone_refuse(const Deps& d, const char* id) {
  return [d, id] {
    if (d->snap && d->snap->vtx && d->snap->vtx->table_mismatch)
      return std::string("Drone settings differ from this GS — update");
    if (!vtx(d, id)) {
      if (d->snap && d->snap->vtx && d->snap->vtx->list == "partial")
        return std::string("This setting did not answer.");
      return std::string("Reading the drone's settings…");
    }
    return std::string();
  };
}

// A choice row that sets one drone parameter, reading its current value
// straight off the sideport.
MenuNode drone_choice(const Deps& d, std::string id, std::string label, const char* param_id) {
  MenuNode n;
  n.id = std::move(id);
  n.label = std::move(label);
  n.kind = NodeKind::kChoice;
  n.owner = Owner::kDrone;
  n.refuse = drone_refuse(d, param_id);
  n.current = [d, param_id] { return vtx(d, param_id).value_or(std::string()); };
  n.steps = [param_id](const std::string& v) { return Steps{{Owner::kDrone, param_id, v}}; };
  return n;
}

// A drone_choice over a Bool param: the wire carries "1"/"0", the row
// trades in the model's own "true"/"false".
MenuNode drone_bool(const Deps& d, std::string id, std::string label, const char* param_id) {
  MenuNode n = drone_choice(d, std::move(id), std::move(label), param_id);
  n.choices = on_off;
  n.current = [d, param_id] {
    const auto v = vtx(d, param_id);
    return std::string(v && *v == "1" ? "true" : "false");
  };
  n.steps = [param_id](const std::string& v) {
    return Steps{{Owner::kDrone, param_id, v == "true" ? "1" : "0"}};
  };
  return n;
}

// A number row over a drone parameter: `scale` converts between the wire
// text (what the drone takes) and the number the picker edits -- e.g. kbps
// on the wire, Mb/s on the picker.
MenuNode drone_number(const Deps& d, std::string id, std::string label, const char* param_id,
                      double min, double max, double step, double scale,
                      std::function<std::string(double)> fmt) {
  MenuNode n;
  n.id = std::move(id);
  n.label = std::move(label);
  n.kind = NodeKind::kNumber;
  n.owner = Owner::kDrone;
  n.refuse = drone_refuse(d, param_id);
  n.number.min = min;
  n.number.max = max;
  n.number.step = step;
  n.number.get = [d, param_id, scale] {
    const auto v = vtx(d, param_id);
    return v ? num(*v) * scale : 0.0;
  };
  n.number.fmt = std::move(fmt);
  n.steps = [param_id, scale](const std::string& v) {
    return Steps{{Owner::kDrone, param_id, real_text(num(v) / scale)}};
  };
  return n;
}

struct CamMode {
  int fps = 0, w = 0, h = 0;
  bool binned = false;
};

// The drone's camera mode, parsed off VENC_MODE; nullopt while that row has
// no value yet or its text does not parse.
std::optional<CamMode> camera_mode(const Deps& d) {
  const auto v = vtx(d, "VENC_MODE");
  if (!v) return std::nullopt;
  CamMode m;
  if (!mabur::cfgparam::parse_mode_text(*v, &m.fps, &m.w, &m.h, &m.binned)) return std::nullopt;
  return m;
}

// Camera mode's steps: REC_FPS first, dragged down to `fps`, when the
// onboard recorder is on and asking for more than the new rate can give --
// the recorder never outruns the camera.
Steps camera_steps(const Deps& d, const std::string& mode_text) {
  Steps out;
  int fps = 0, w = 0, h = 0;
  bool binned = false;
  if (mabur::cfgparam::parse_mode_text(mode_text, &fps, &w, &h, &binned)) {
    const auto rec_enable = vtx(d, "REC_ENABLE");
    const auto rec_fps = vtx(d, "REC_FPS");
    if (rec_enable && *rec_enable == "1" && rec_fps && std::atoi(rec_fps->c_str()) > fps)
      out.push_back({Owner::kDrone, "REC_FPS", std::to_string(fps)});
  }
  out.push_back({Owner::kDrone, "VENC_MODE", mode_text});
  return out;
}

// Camera mode's confirm: the rebuild pause, plus the SD frame rate note
// whenever camera_steps would drag REC_FPS down with it.
std::string camera_confirm(const Deps& d, const std::string& mode_text) {
  std::string extra;
  int fps = 0, w = 0, h = 0;
  bool binned = false;
  if (mabur::cfgparam::parse_mode_text(mode_text, &fps, &w, &h, &binned)) {
    const auto rec_enable = vtx(d, "REC_ENABLE");
    const auto rec_fps = vtx(d, "REC_FPS");
    if (rec_enable && *rec_enable == "1" && rec_fps && std::atoi(rec_fps->c_str()) > fps)
      extra = "Also lowers the SD frame rate to " + std::to_string(fps) + ".";
  }
  return join_confirm({kPause, extra});
}

// The Camera mode choice list: binned sizes then native sizes, for every
// rate the display shows (every rate when `modes` is empty -- the host
// test build and a connector that could not be read); the 120 fps choices
// are letterboxed. The drone's current value is pulled to the front when
// it is off that list (an old REC_SIZE-shaped leftover, or a rate the
// display cannot show), noting which, and stays listed-but-unpickable in
// that second case (keep_current_if_unpickable on the row).
std::vector<Choice> camera_choices(const Deps& d) {
  std::vector<Choice> out;
  const auto modes = d->modes ? d->modes() : std::vector<ModeCand>{};
  for (int rate : mabur::cammode::kRates) {
    if (!modes.empty() && sizes_at(modes, rate).empty()) continue;
    for (bool binned : {true, false}) {
      const mabur::cammode::SensorMode* m = mabur::cammode::mode_for(rate, binned);
      if (!m) continue;
      for (const mabur::cammode::Size& sz : m->sizes) {
        out.push_back({mabur::cfgparam::render_mode_text(rate, sz.w, sz.h, binned),
                       mabur::cammode::choice_label(rate, sz.w, sz.h, binned),
                       rate == 120 ? "letterboxed 2.1:1" : "", true});
      }
    }
  }
  const auto cur = vtx(d, "VENC_MODE");
  if (cur) {
    const bool listed =
        std::any_of(out.begin(), out.end(), [&](const Choice& c) { return c.value == *cur; });
    if (!listed) {
      int fps = 0, w = 0, h = 0;
      bool binned = false;
      if (mabur::cfgparam::parse_mode_text(*cur, &fps, &w, &h, &binned)) {
        const bool shown = modes.empty() || !sizes_at(modes, fps).empty();
        const std::string note =
            !shown ? "display can't show" : fps == 120 ? "letterboxed 2.1:1" : "";
        out.insert(out.begin(), {*cur, mabur::cammode::choice_label(fps, w, h, binned), note, shown});
      }
    }
  }
  return out;
}

MenuNode camera_row(const Deps& d) {
  MenuNode n = drone_choice(d, "video.camera", "Camera mode", "VENC_MODE");
  n.choices = [d] { return camera_choices(d); };
  n.keep_current_if_unpickable = true;
  n.confirm = [d](const std::string& v) { return camera_confirm(d, v); };
  n.steps = [d](const std::string& v) { return camera_steps(d, v); };
  n.help = text(
      "Camera frame rate, size and view. bin merges sensor pixels 2×2 for the widest "
      "view; without it the sensor is read pixel for pixel, sharper but zoomed in. Changing it "
      "pauses video about half a second.");
  return n;
}

// The display-shown rate nearest the camera's current one (a tie keeps the
// higher rate), at the camera's own size when that rate's sensor mode
// offers it, else that mode's first size; same readout (binned/native).
std::optional<CamMode> fix_target(const Deps& d) {
  const auto cm = camera_mode(d);
  if (!cm) return std::nullopt;
  const auto modes = d->modes ? d->modes() : std::vector<ModeCand>{};
  int best = -1;
  double best_d = 0;
  for (int rate : mabur::cammode::kRates) {
    if (sizes_at(modes, rate).empty()) continue;
    const double dist = std::fabs(rate - cm->fps);
    if (best < 0 || dist < best_d || (dist == best_d && rate > best)) {
      best = rate;
      best_d = dist;
    }
  }
  if (best < 0) return std::nullopt;
  const mabur::cammode::SensorMode* m = mabur::cammode::mode_for(best, cm->binned);
  if (!m || m->sizes.empty()) return std::nullopt;
  int w = cm->w, h = cm->h;
  const bool offers =
      std::any_of(m->sizes.begin(), m->sizes.end(),
                  [&](const mabur::cammode::Size& s) { return s.w == w && s.h == h; });
  if (!offers) {
    w = m->sizes[0].w;
    h = m->sizes[0].h;
  }
  return CamMode{best, w, h, cm->binned};
}

MenuNode fix_row(const Deps& d) {
  MenuNode n;
  n.id = "video.fix";
  n.label = "Fix: set camera to …";
  n.kind = NodeKind::kAction;
  n.owner = Owner::kDrone;
  n.refuse = drone_refuse(d, "VENC_MODE");
  n.visible = [d] {
    const auto cm = camera_mode(d);
    if (!cm) return false;
    const auto modes = d->modes ? d->modes() : std::vector<ModeCand>{};
    // Also requires a fix_target: a display offering none of the fixed
    // rates (60/90/100/120) leaves nothing to pick, so the row would send
    // nothing and still read "✓ saved".
    return !modes.empty() && sizes_at(modes, cm->fps).empty() && fix_target(d).has_value();
  };
  const auto target = fix_target(d);
  if (target) {
    n.label = "Fix: set camera to " + std::to_string(target->fps) + " fps · " +
              size_label(target->w, target->h);
  }
  n.action = [d, target] {
    if (!target) return Steps{};
    return camera_steps(
        d, mabur::cfgparam::render_mode_text(target->fps, target->w, target->h, target->binned));
  };
  n.confirm = [d, target](const std::string&) {
    if (!target) return std::string();
    return camera_confirm(
        d, mabur::cfgparam::render_mode_text(target->fps, target->w, target->h, target->binned));
  };
  return n;
}

MenuNode mirror_row(const Deps& d) {
  MenuNode n = drone_bool(d, "video.mirror", "Mirror", "VENC_MIRROR");
  n.confirm = [](const std::string&) { return std::string(kPause); };
  n.help = text("Flip the picture left-right, for a camera mounted mirrored.");
  return n;
}

MenuNode flip_row(const Deps& d) {
  MenuNode n = drone_bool(d, "video.flip", "Flip", "VENC_FLIP");
  n.confirm = [](const std::string&) { return std::string(kPause); };
  n.help = text("Flip the picture upside down, for a camera mounted upside down.");
  return n;
}

MenuNode slices_row(const Deps& d) {
  MenuNode n = drone_choice(d, "video.slices", "Row slices", "VENC_SLICES");
  n.advanced = true;
  n.choices = [] { return std::vector<Choice>{{"4", "On", "", true}, {"1", "Off", "", true}}; };
  n.current = [d] {
    const auto v = vtx(d, "VENC_SLICES");
    return std::string(v && num(*v) > 1 ? "4" : "1");
  };
  n.confirm = [](const std::string&) { return std::string(kPause); };
  n.help = text("Splits each frame into row slices the GS can patch when one is lost.");
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

// Who moves to hold the phase lock: the drone's camera (the GS steers its
// sensor rate), or the GS's own display (refresh sync, nudge or follow).
MenuNode refresh_sync_row(const Deps& d) {
  MenuNode n = choice_row("video.refresh_sync", "Phase lock by", Owner::kPlayer, "display.refresh_sync", quoted);
  n.advanced = true;
  n.choices = [] {
    return std::vector<Choice>{{"off", "Camera", "", true},
                               {"nudge", "Display (nudge)", "", true},
                               {"follow", "Display (follow)", "", true}};
  };
  n.current = [d] { return d->cfg->display.refresh_sync; };
  // A display choice that cannot act here (no phase lock, no frame-length
  // control as on the Orange Pi, or the controller's total refused): the set
  // still saves, and the camera holds the lock instead. Camera itself always
  // can.
  n.warn = [d] {
    const bool display = d->cfg->display.refresh_sync != "off";
    return display && d->refresh_sync_available && !d->refresh_sync_available() ? std::string("n/a")
                                                                                : std::string();
  };
  n.help = text(
      "Camera: the drone shifts its camera to the screen; works everywhere. Display: the GS "
      "bends the screen's timing to the camera instead — nudge is gentle but takes about half "
      "a minute to lock; follow locks in seconds but more screens flicker. Go back to Camera "
      "if yours blanks.");
  return n;
}

// --- Link --------------------------------------------------------------

// The rung Fixed pins: the live rung off the sideport (the first ladder
// rung matching both GsSnapshot::mcs and ::bw) when known, else R3, or the
// top rung of a shorter ladder. The ladder must not be empty.
Rung current_rung(const Deps& d) {
  const GsLiveView& gs = *d->gs;
  if (d->snap && d->snap->mcs && d->snap->bw) {
    for (const Rung& r : gs.ladder)
      if (r.mcs == *d->snap->mcs && r.bw == *d->snap->bw) return r;
  }
  return gs.ladder[std::min<size_t>(3, gs.ladder.size() - 1)];
}

// The receiver width maburgs would derive from `gs`'s live keys: the
// page-side mirror of maburgs' derived_width, since this binary does not
// link maburgs' config code.
int receiver_width(const GsLiveView& gs) {
  if (gs.static_mcs >= 0) return gs.static_bw;
  for (const Rung& r : gs.ladder)
    if (r.bw == 40) return 40;
  return 20;
}

// Whether the sideport snapshot is current (MenuTreeDeps::snap_stale; no
// staleness input reads fresh).
bool snap_fresh(const Deps& d) { return !d->snap_stale || !d->snap_stale(); }

// The link's own channel right now: the sideport's link.channel while the
// session holds on a fresh snapshot (unlinked, a stale link.channel from
// before the drop is not "the link", and the row has nothing to point two
// sources at; a stale sideport says nothing about the link at all).
std::optional<int> link_channel(const Deps& d) {
  if (d->snap && snap_fresh(d) && d->snap->session && d->snap->channel) return d->snap->channel;
  return std::nullopt;
}

// "auto" or the pinned channel's number -- the row's own value key, read
// off maburgs' live radio.channel (GsLiveView::pinned_channel), never the
// sideport (the two sources, docs/gs-menu.md "link.channel").
std::string channel_current(const Deps& d) {
  return d->gs->pinned_channel ? std::to_string(*d->gs->pinned_channel) : std::string("auto");
}

// The Channel row: pins or unpins radio.channel, reading the live-air bars
// from scan.members while the picker holds maburgs' measuring lease open
// (ChannelObserve, main.cpp). See docs/gs-menu.md "link.channel".
MenuNode channel_row(const Deps& d) {
  MenuNode n;
  n.id = kChannelRow;
  n.label = "Channel";
  n.kind = NodeKind::kChoice;
  n.owner = Owner::kGs;
  n.current = [d] { return channel_current(d); };
  n.value_text = [d] {
    const std::optional<int> link = link_channel(d);
    if (!d->gs->pinned_channel)
      return "Auto (on " + (link ? std::to_string(*link) : std::string("—")) + ")";
    const int pin = *d->gs->pinned_channel;
    if (link && *link == pin) return std::to_string(pin) + " ● link";
    return std::to_string(pin);
  };
  n.warn = [d] {
    if (!d->gs->pinned_channel) return std::string();
    const std::optional<int> link = link_channel(d);
    if (link && *link != *d->gs->pinned_channel) return "link on " + std::to_string(*link);
    return std::string();
  };
  n.choices = [d] {
    const std::optional<int> link = link_channel(d);
    const std::string current = channel_current(d);
    std::vector<Choice> out;
    out.push_back({"auto", "Auto", "on " + (link ? std::to_string(*link) : std::string("—")), true});
    // A stale sideport's figures are not the air now: every member draws
    // unmeasured.
    const bool fresh = snap_fresh(d);
    if (d->snap && !d->snap->scan_members.empty()) {
      for (const ScanMember& m : d->snap->scan_members) {
        const std::string value = std::to_string(m.ch);
        std::string label = value;
        if (m.pair_lo && m.pair_hi)
          label += " · " + std::to_string(*m.pair_lo) + "+" + std::to_string(*m.pair_hi);
        std::string note;
        ChoiceTone tone;
        if (!m.busy_pct || !fresh) {
          note = clean_air_bars(std::nullopt) + " · not measured";
          tone = ChoiceTone::kDim;
        } else if (m.age_ms && *m.age_ms > kFreshMs) {
          note = clean_air_bars(m.busy_pct) + " · " + age_text(*m.age_ms);
          tone = ChoiceTone::kDim;
        } else {
          note = clean_air_bars(m.busy_pct);
          const int level = clean_air_level(*m.busy_pct);
          tone = level >= 4 ? ChoiceTone::kGood : (level >= 2 ? ChoiceTone::kWarn : ChoiceTone::kBad);
        }
        if (link && *link == m.ch && value != current) note += " · link";
        out.push_back({value, label, note, true, tone});
      }
    } else if (d->gs->pinned_channel) {
      // No scan.members (none exported yet): the pin still needs a choice
      // of its own so the current value stays visible, unmeasured.
      const std::string value = std::to_string(*d->gs->pinned_channel);
      out.push_back({value, value, clean_air_bars(std::nullopt) + " · not measured", true, ChoiceTone::kDim});
    }
    return out;
  };
  n.confirm = [d](const std::string& v) {
    if (v == "auto") return std::string();
    const std::optional<int> link = link_channel(d);
    if (link && std::to_string(*link) == v) return std::string("Auto stops choosing for you.");
    return "The link moves to " + v + " and stays there: Auto stops choosing for you.";
  };
  n.steps = [](const std::string& v) {
    return Steps{{Owner::kGs, "radio.channel", v == "auto" ? "\"auto\"" : v}};
  };
  n.help = [d] {
    const std::string state = d->observe_state && snap_fresh(d) ? d->observe_state() : std::string();
    if (state == "one_card") return std::string("Measuring — video pauses each second.");
    if (state == "two_card" || state == "relay" || state == "search") return std::string("Measuring.");
    const std::optional<int> link = link_channel(d);
    if (d->gs->pinned_channel) {
      const int pin = *d->gs->pinned_channel;
      if (link && *link != pin) {
        // The pin's own relocate failed: its reason stays here for as long
        // as the link is still elsewhere (the reply's error is timed).
        const std::optional<ScanRelocate>& r = d->snap->scan_relocate;
        if (r && r->state == "failed" && r->ch == pin)
          return "Pinned here, link still on " + std::to_string(*link) + " — " + r->why + ".";
        return "Pinned here, link still on " + std::to_string(*link) + ".";
      }
      if (!link)
        return "Pinned to " + std::to_string(pin) + " — the link moves here when it comes back.";
    }
    return std::string(
        "Auto picks the cleanest channel and moves the link off a busy one. Pick a channel to "
        "pin the link there.");
  };
  return n;
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
    std::string base;
    if (!fixed(d) && !is_default_ladder(d->gs->ladder)) {
      base = "Replaces your ladder with the default " + v + " MHz ladder (" +
             std::to_string(default_ladder(std::atoi(v.c_str())).size()) + " rungs).";
    }
    if (receiver_width(*d->gs) != std::atoi(v.c_str()))
      return join_confirm({base, "Retunes the receiver."});
    return base;
  };
  n.help = [d] {
    if (fixed(d)) return std::string("Channel width for the fixed rate.");
    if (ladder_width(d->gs->ladder) == LadderWidth::kMixed)
      return std::string(
          "Your ladder mixes 20 and 40 MHz rungs (edit them in Ladder). Picking 20 or 40 loads "
          "that width's default ladder. The receiver retunes to the ladder's width.");
    return std::string(
        "40 MHz carries about twice the data; 20 MHz reaches a few dB further at the lowest "
        "rung. Picking a width loads its default ladder. The receiver retunes to the ladder's "
        "width.");
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
    const Rung r = current_rung(d);
    return Steps{{Owner::kGs, "link.static_bw", std::to_string(r.bw)},
                 {Owner::kGs, "link.static_overhead_base", real_text(r.base)},
                 {Owner::kGs, "link.static_overhead_enh", real_text(r.enh)},
                 {Owner::kGs, "link.static_mcs", std::to_string(std::min(r.mcs, kMaxMcs))}};
  };
  // Fixed pins the live rung's width, Adaptive goes back to the ladder's:
  // when that moves the receiver width, the cards retune (the Width row's
  // rule).
  n.confirm = [d](const std::string& v) {
    GsLiveView after = *d->gs;
    if (v == "fixed") {
      if (after.ladder.empty()) return std::string();   // nothing is sent
      after.static_bw = current_rung(d).bw;
      after.static_mcs = std::max(after.static_mcs, 0);
    } else {
      after.static_mcs = -1;
    }
    if (receiver_width(after) != receiver_width(*d->gs)) return std::string("Retunes the receiver.");
    return std::string();
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

// --- Link drone rows -----------------------------------------------------

MenuNode bitrate_row(const Deps& d) {
  MenuNode n = drone_number(d, "link.bitrate", "Max bitrate", "ENC_BITRATE_MAX", 1, 100, 1, 0.001,
                            [](double v) { return std::to_string(std::lround(v)) + " Mb/s"; });
  n.help = text("Ceiling on the video bitrate.");
  return n;
}

MenuNode airtime_row(const Deps& d) {
  MenuNode n = drone_number(d, "link.airtime", "Airtime share", "ENC_AIRTIME", 5, 100, 5, 100,
                            [](double v) { return std::to_string(std::lround(v)) + " %"; });
  n.advanced = true;
  n.help = text("Share of the link's real capacity the video may use.");
  return n;
}

MenuNode ldpc_row(const Deps& d) {
  MenuNode n = drone_bool(d, "link.ldpc", "LDPC coding", "RADIO_LDPC");
  n.advanced = true;
  // The drone's own calibration keeps a wall-margin note tied to LDPC; a
  // row that just turned it off (or on) keeps showing it rather than
  // fading like a plain "✓ saved".
  n.warn_after_ok = [d] {
    const VtxRow* r = d->snap ? vtx_row(*d->snap, "RADIO_LDPC") : nullptr;
    if (r && r->st == "accepted" && !r->status.empty()) return r->status;
    return std::string();
  };
  n.help = text("LDPC coding on every frame; turn off only for a ground card that cannot decode it.");
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

// The drone's onboard recorder is enabled (REC_ENABLE == "1").
bool rec_vtx_on(const Deps& d) {
  const auto v = vtx(d, "REC_ENABLE");
  return v && *v == "1";
}

// The camera's fps and its whole divisors >= 30, descending: SD frame rate
// outruns nothing above the camera's own rate, and a rate under 30 fps is
// not worth offering.
std::vector<int> rec_fps_divisors(int fps) {
  std::vector<int> out;
  for (int i = 30; i <= fps; ++i)
    if (fps % i == 0) out.push_back(i);
  std::sort(out.rbegin(), out.rend());
  return out;
}

// REC_FPS choices: the camera's divisors (empty with no known camera
// mode), plus the current value first when off that list -- so the row
// stays usable (one value, the current one) even with no camera mode read
// yet.
std::vector<Choice> rec_fps_choices(const Deps& d) {
  std::vector<Choice> out;
  const auto cm = camera_mode(d);
  if (cm)
    for (int fps : rec_fps_divisors(cm->fps))
      out.push_back({std::to_string(fps), std::to_string(fps) + " fps", "", true});
  const auto cur = vtx(d, "REC_FPS");
  if (cur) {
    const bool listed =
        std::any_of(out.begin(), out.end(), [&](const Choice& c) { return c.value == *cur; });
    if (!listed) out.insert(out.begin(), {*cur, *cur + " fps", "", true});
  }
  return out;
}

// REC_SIZE choices: the camera's own sensor mode's sizes, the camera's own
// size appended when it is off that list, plus the current value first
// when it is off the resulting list (same "stays usable" rule as fps).
std::vector<Choice> rec_size_choices(const Deps& d) {
  std::vector<Choice> out;
  const auto cm = camera_mode(d);
  if (cm) {
    if (const mabur::cammode::SensorMode* m = mabur::cammode::mode_for(cm->fps, cm->binned))
      for (const mabur::cammode::Size& sz : m->sizes)
        out.push_back({size_text(sz.w, sz.h), size_label(sz.w, sz.h), "", true});
    const std::string own = size_text(cm->w, cm->h);
    const bool has_own = std::any_of(out.begin(), out.end(), [&](const Choice& c) { return c.value == own; });
    if (!has_own) out.push_back({own, size_label(cm->w, cm->h), "", true});
  }
  const auto cur = vtx(d, "REC_SIZE");
  if (cur) {
    const bool listed =
        std::any_of(out.begin(), out.end(), [&](const Choice& c) { return c.value == *cur; });
    if (!listed) {
      int w = 0, h = 0;
      const std::string label = std::sscanf(cur->c_str(), "%dx%d", &w, &h) == 2 ? size_label(w, h) : *cur;
      out.insert(out.begin(), {*cur, label, "", true});
    }
  }
  return out;
}

std::vector<MenuNode> rec_rows(const Deps& d) {
  MenuNode target = choice_row("rec.target", "REC records to", Owner::kPlayer, "dvr.target", quoted);
  target.choices = [] {
    return std::vector<Choice>{{"gs", "GS", "", true}, {"vtx", "VTX SD card", "", true}, {"both", "Both", "", true}};
  };
  target.current = [d] { return d->cfg->dvr.target; };
  target.saved_text = next_rec_saved(d);
  // The REC button has nowhere to write when it is routed at least partly
  // to the VTX SD card and the drone's own recorder is off.
  target.warn = [d] {
    if (d->cfg->dvr.target != "vtx" && d->cfg->dvr.target != "both") return std::string();
    const auto rec_enable = vtx(d, "REC_ENABLE");
    return rec_enable && *rec_enable == "0" ? std::string("SD recorder off") : std::string();
  };
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

  MenuNode vtx_enable = drone_bool(d, "rec.vtx", "VTX SD recorder", "REC_ENABLE");
  vtx_enable.confirm = [](const std::string&) { return std::string(kPause); };
  vtx_enable.help = text("Records straight to the VTX's own microSD card, independent of the GS.");

  auto vtx_on = [d] { return rec_vtx_on(d); };

  MenuNode vtx_bitrate = drone_number(d, "rec.vtx.bitrate", "  SD bitrate", "REC_BITRATE", 2, 80, 1,
                                      0.001, [](double v) { return std::to_string(std::lround(v)) + " Mb/s"; });
  vtx_bitrate.visible = vtx_on;
  vtx_bitrate.confirm = [](const std::string&) { return std::string(kPause); };
  vtx_bitrate.help = text("Constant bitrate of the onboard recording.");

  MenuNode vtx_fps = drone_choice(d, "rec.vtx.fps", "  SD frame rate", "REC_FPS");
  vtx_fps.visible = vtx_on;
  vtx_fps.choices = [d] { return rec_fps_choices(d); };
  vtx_fps.confirm = [](const std::string&) { return std::string(kPause); };
  vtx_fps.help = text("Onboard recording frame rate, at most the camera's fps.");

  MenuNode vtx_size = drone_choice(d, "rec.vtx.size", "  SD size", "REC_SIZE");
  vtx_size.visible = vtx_on;
  vtx_size.choices = [d] { return rec_size_choices(d); };
  vtx_size.confirm = [](const std::string&) { return std::string(kPause); };
  vtx_size.help = text("Onboard recording resolution, same aspect ratio as the camera's.");

  return {target, mode, bitrate, every, size, vtx_enable, vtx_bitrate, vtx_fps, vtx_size};
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
  // The FC OSD draws nothing while the drone's own MSP link is off.
  fc.warn = [d] {
    const auto msp_enable = vtx(d, "MSP_ENABLE");
    return msp_enable && *msp_enable == "0" ? std::string("FC link off") : std::string();
  };
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

// --- Power & FC ----------------------------------------------------------

MenuNode lowpwr_row(const Deps& d) {
  MenuNode n = drone_bool(d, "power.lowpwr", "Low power when disarmed", "LOWPWR_ENABLE");
  // Low power needs the MSP link for arm state: turning it on with that
  // link off takes the link on first.
  n.steps = [d](const std::string& v) {
    if (v == "true") {
      const auto msp_enable = vtx(d, "MSP_ENABLE");
      if (msp_enable && *msp_enable == "0")
        return Steps{{Owner::kDrone, "MSP_ENABLE", "1"}, {Owner::kDrone, "LOWPWR_ENABLE", "1"}};
    }
    return Steps{{Owner::kDrone, "LOWPWR_ENABLE", v == "true" ? "1" : "0"}};
  };
  n.confirm = [d](const std::string& v) {
    if (v == "true") {
      const auto msp_enable = vtx(d, "MSP_ENABLE");
      if (msp_enable && *msp_enable == "0") return std::string("Also turns the FC link on.");
    }
    return std::string();
  };
  n.help = text(
      "Lower bitrate and frame rate while disarmed to keep the VTX cool. Needs the FC link for "
      "arm state.");
  return n;
}

MenuNode msp_row(const Deps& d) {
  MenuNode n = drone_bool(d, "power.msp", "FC link (MSP)", "MSP_ENABLE");
  // Turning the MSP link off under Low power takes Low power off first --
  // it depends on this link for arm state.
  n.steps = [d](const std::string& v) {
    if (v == "false") {
      const auto lowpwr = vtx(d, "LOWPWR_ENABLE");
      if (lowpwr && *lowpwr == "1")
        return Steps{{Owner::kDrone, "LOWPWR_ENABLE", "0"}, {Owner::kDrone, "MSP_ENABLE", "0"}};
    }
    return Steps{{Owner::kDrone, "MSP_ENABLE", v == "true" ? "1" : "0"}};
  };
  n.confirm = [d](const std::string& v) {
    if (v == "false") {
      const auto lowpwr = vtx(d, "LOWPWR_ENABLE");
      if (lowpwr && *lowpwr == "1") return std::string("Also turns Low power off.");
    }
    return std::string();
  };
  n.help = text("Flight-controller link for the OSD and arm state.");
  return n;
}

std::vector<MenuNode> power_rows(const Deps& d) { return {lowpwr_row(d), msp_row(d)}; }

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

std::string drone_header(const GsSnapshot* snap, bool stale) {
  if (!snap || stale || !snap->vtx) return std::string();
  const VtxState& vtx = *snap->vtx;
  if (vtx.table_mismatch) return "Drone settings differ from this GS — update";
  if (vtx.list == "pending") {
    const bool missing = std::any_of(vtx.rows.begin(), vtx.rows.end(),
                                     [](const VtxRow& r) { return !r.value.has_value(); });
    if (missing) return "Reading the drone's settings…";
  }
  if (vtx.list == "partial") return "Some settings did not answer.";
  return std::string();
}

std::string drone_pill_mark(const GsSnapshot* snap, bool stale) {
  if (!snap || stale || !snap->vtx) return std::string();
  const VtxState& vtx = *snap->vtx;
  if (vtx.linked && vtx.alive.has_value() && !*vtx.alive) return "no heartbeat";
  return std::string();
}

std::vector<MenuNode> build_menu_tree(const MenuTreeDeps& deps) {
  const Deps d = std::make_shared<const MenuTreeDeps>(deps);
  return guarded(d, {
      group("video", "Video", "Camera, display and picture.",
            [d] {
              return std::vector<MenuNode>{camera_row(d),       fix_row(d),   display_row(d),
                                           colour_row(d),        refresh_sync_row(d),
                                           mirror_row(d),        flip_row(d), slices_row(d)};
            }),
      group("link", "Link", "How the video crosses the air.",
            [d] {
              std::vector<MenuNode> rows{channel_row(d), width_row(d), rate_row(d)};
              for (MenuNode& n : fixed_rows(d)) rows.push_back(std::move(n));
              rows.push_back(ladder_group(d));
              rows.push_back(nack_row(d));
              rows.push_back(bitrate_row(d));
              rows.push_back(airtime_row(d));
              rows.push_back(ldpc_row(d));
              return rows;
            }),
      group("rec", "Recording", "What REC records, and where.", [d] { return rec_rows(d); }),
      group("osd", "OSD", "What is drawn over the video.", [d] { return osd_rows(d); }),
      group("power", "Power & FC", "The drone's power saving and its flight-controller link.",
            [d] { return power_rows(d); }),
      group("sys", "System", "GS housekeeping.", [d] { return sys_rows(d); }),
  });
}

}  // namespace maburplay
