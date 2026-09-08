#include "player_config.h"

#include <fstream>
#include <stdexcept>

#include "gs_layer.h"  // parse_gs_style
#include "mabur/toml.h"

namespace maburplay {
namespace {

namespace toml = mabur::toml;   // maburgs/maburplay are not inside mabur
using mabur::toml::Value;

// Set for the duration of load_config; collects keys that fell back to their
// struct default. Not reentrant, which is fine: it is called once at boot.
std::vector<std::string>* g_defaulted = nullptr;

std::string to_text(bool v) { return v ? "true" : "false"; }
std::string to_text(const std::string& v) { return v; }
template <typename T>
std::string to_text(const T& v) { return std::to_string(v); }

void note_default(const std::string& prefix, const char* key,
                  const std::string& value) {
  if (g_defaulted == nullptr) return;
  g_defaulted->push_back((prefix.empty() ? std::string(key)
                                         : prefix + "." + key) + "=" + value);
}

std::string g_file;      // set by load_config, "" outside it
// Line of the LAST key read, not necessarily the key currently being
// validated: a cross-key check runs after both keys have been read, so it
// reports whichever key assign_if_present touched last, not necessarily the
// one actually at fault. Known, accepted limitation (see
// drone/src/config.cpp for the full rationale).
int g_line = 0;

[[noreturn]] void fail(const std::string& field, const std::string& why) {
  std::string where = "config: ";
  if (!g_file.empty() && g_line > 0)
    where += g_file + ":" + std::to_string(g_line) + ": ";
  throw std::runtime_error(where + field + ": " + why);
}

void check_keys(const Value& o, const std::string& where,
                std::initializer_list<const char*> allowed) {
  for (auto it = o.begin(); it != o.end(); ++it) {
    const std::string& k = it.key();
    bool ok = false;
    for (const char* a : allowed)
      if (k == a) { ok = true; break; }
    if (!ok) fail(where.empty() ? k : where + "." + k, "unknown key");
  }
}

long get_int(const Value& o, const char* key, long dflt, long lo, long hi,
             const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, to_text(dflt)); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_number_integer()) fail(where + "." + key, "not an integer");
  const long v = o[key].get<long>();
  if (v < lo || v > hi) fail(where + "." + key, "out of range");
  return v;
}

std::string get_str(const Value& o, const char* key, const std::string& dflt,
                    const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, dflt); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_string()) fail(where + "." + key, "not a string");
  return o[key].get<std::string>();
}

// New: bools went through inline `if (o.contains(...))` blocks, which meant a
// missing bool could not be reported. Same shape as the others now.
bool get_bool(const Value& o, const char* key, bool dflt,
              const std::string& where) {
  if (!o.contains(key)) { note_default(where, key, to_text(dflt)); return dflt; }
  g_line = o[key].line();
  if (!o[key].is_boolean()) fail(where + "." + key, "not a boolean");
  return o[key].get<bool>();
}
}  // namespace

Config load_config(const std::string& path, std::vector<std::string>* defaulted) {
  Value j;
  try {
    j = toml::parse_toml_file(path);
  } catch (const toml::Error& e) {
    throw std::runtime_error(std::string("config: ") + e.what());
  }
  g_defaulted = defaulted;
  g_file = path;
  struct Clear {
    ~Clear() { g_defaulted = nullptr; g_file.clear(); g_line = 0; }
  } clear_on_exit;

  for (const char* sec : {"dvr", "osd", "input", "display"})
    if (!j.contains(sec)) note_default("", sec, "(section absent)");

  check_keys(j, "",
             {"ring_path", "socket", "backend", "screen_mode", "dvr", "osd",
              "input", "display"});
  Config c;

  c.ring_path = get_str(j, "ring_path", "/dev/shm/mabur-au", "");
  c.socket = get_str(j, "socket", "/run/mabur-au.sock", "");
  c.backend = get_str(j, "backend", "mpp", "");

  if (c.backend != "mpp" && c.backend != "null") {
    fail("backend", "must be \"mpp\" or \"null\"");
  }

  c.screen_mode = get_str(j, "screen_mode", "1920x1080@60", "");

  if (j.contains("dvr")) {
    const Value& r = j["dvr"];
    check_keys(r, "dvr", {"autostart", "dir", "fragment_ms", "mode", "burned"});
    c.dvr.autostart = get_bool(r, "autostart", c.dvr.autostart, "dvr");
    c.dvr.dir = get_str(r, "dir", "/media/dvr", "dvr");
    c.dvr.fragment_ms = static_cast<int>(get_int(r, "fragment_ms", 1000, 100, 10000, "dvr"));
    c.dvr.mode = get_str(r, "mode", "raw", "dvr");
    if (c.dvr.mode != "raw" && c.dvr.mode != "burned")
      fail("dvr.mode", "must be \"raw\" or \"burned\"");
    if (r.contains("burned")) {
      const Value& b = r["burned"];
      check_keys(b, "dvr.burned", {"bitrate_kbps", "fps_cap"});
      c.dvr.burned.bitrate_kbps =
          static_cast<int>(get_int(b, "bitrate_kbps", 12000, 500, 100000, "dvr.burned"));
      c.dvr.burned.fps_cap =
          static_cast<int>(get_int(b, "fps_cap", 30, 1, 120, "dvr.burned"));
    } else {
      // Whole sub-table absent: one line, same style as a missing top-level
      // section, rather than two separate per-key lines the operator would
      // have to mentally group back together.
      note_default("dvr", "burned", "(section absent)");
    }
  }

  if (j.contains("osd")) {
    const Value& o = j["osd"];
    check_keys(o, "osd", {"enable", "port", "font", "scale", "stale_ms", "gs"});
    c.osd.enable = get_bool(o, "enable", c.osd.enable, "osd");
    c.osd.port = static_cast<int>(get_int(o, "port", 14560, 1, 65535, "osd"));
    c.osd.font = get_str(o, "font", c.osd.font, "osd");
    c.osd.scale = get_str(o, "scale", "sharp", "osd");
    if (c.osd.scale != "sharp" && c.osd.scale != "fill")
      fail("osd.scale", "must be \"sharp\" or \"fill\"");
    // Default mirrors OsdCfg::stale_ms (see player_config.h for why 5000).
    c.osd.stale_ms = static_cast<int>(get_int(o, "stale_ms", 5000, 0, 60000, "osd"));

    if (o.contains("gs")) {
      const Value& g = o["gs"];
      check_keys(g, "osd.gs", {"enable", "port", "font", "style", "stale_ms"});
      c.osd.gs.enable = get_bool(g, "enable", c.osd.gs.enable, "osd.gs");
      c.osd.gs.port = static_cast<int>(get_int(g, "port", c.osd.gs.port, 1, 65535, "osd.gs"));
      c.osd.gs.font = get_str(g, "font", c.osd.gs.font, "osd.gs");
      c.osd.gs.style = get_str(g, "style", c.osd.gs.style, "osd.gs");
      if (!parse_gs_style(c.osd.gs.style, nullptr))
        fail("osd.gs.style", "must be \"compact\" or \"essential\"");
      // Default mirrors OsdCfg::GsCfg::stale_ms (see player_config.h).
      c.osd.gs.stale_ms =
          static_cast<int>(get_int(g, "stale_ms", c.osd.gs.stale_ms, 0, 60000, "osd.gs"));
    } else {
      // Whole sub-table absent: one line, same style as a missing top-level
      // section, rather than four separate per-key lines the operator would
      // have to mentally group back together.
      note_default("osd", "gs", "(section absent)");
    }
  }

  if (j.contains("input")) {
    const Value& in = j["input"];
    check_keys(in, "input", {"rec"});
    if (in.contains("rec")) {
      const Value& rc = in["rec"];
      check_keys(rc, "input.rec", {"pin", "active_low", "bias"});
      // Required, not defaulted: a rec block with no pin is a typo, and
      // defaulting it would silently claim some unrelated line.
      if (!rc.contains("pin")) fail("input.rec.pin", "required");
      // Upper bound is generous on purpose: header pins run to 40 on this
      // board, but a GPIO<n>-naming kernel can go far higher.
      c.input.rec.pin = static_cast<int>(get_int(rc, "pin", 0, 1, 512, "input.rec"));
      c.input.rec.active_low = get_bool(rc, "active_low", c.input.rec.active_low, "input.rec");
      c.input.rec.bias = get_str(rc, "bias", "pull-up", "input.rec");
      if (c.input.rec.bias != "pull-up" && c.input.rec.bias != "pull-down" &&
          c.input.rec.bias != "none") {
        fail("input.rec.bias", "must be \"pull-up\", \"pull-down\" or \"none\"");
      }
      c.input.rec.configured = true;
    } else {
      // Whole sub-table absent: one line, same style as a missing top-level
      // section, rather than three separate per-key lines the operator would
      // have to mentally group back together.
      note_default("input", "rec", "(section absent)");
    }
  }

  if (j.contains("display")) {
    const Value& d = j["display"];
    check_keys(d, "display",
               {"regulate_ms", "vsync_lock", "vsync_lead_ms", "chain_budget"});
    c.display.regulate_ms =
        static_cast<int>(get_int(d, "regulate_ms", 12, 0, 100, "display"));
    c.display.vsync_lock = get_bool(d, "vsync_lock", c.display.vsync_lock, "display");
    c.display.vsync_lead_ms =
        static_cast<int>(get_int(d, "vsync_lead_ms", 6, 1, 10, "display"));
    c.display.chain_budget =
        static_cast<int>(get_int(d, "chain_budget", 3, 0, 60, "display"));
  }

  return c;
}

}  // namespace maburplay
