#include "player_config.h"

#include <fstream>
#include <stdexcept>

#include "json.hpp"

namespace maburplay {
namespace {
using nlohmann::json;

[[noreturn]] void fail(const std::string& field, const std::string& why) {
  throw std::runtime_error("config: " + field + ": " + why);
}

void check_keys(const json& o, const std::string& where,
                std::initializer_list<const char*> allowed) {
  for (auto& [k, v] : o.items()) {
    bool ok = false;
    for (const char* a : allowed)
      if (k == a) { ok = true; break; }
    if (!ok) fail(where + "." + k, "unknown key");
  }
}

long get_int(const json& o, const char* key, long dflt, long lo, long hi,
             const std::string& where) {
  if (!o.contains(key)) return dflt;
  if (!o[key].is_number_integer()) fail(where + "." + key, "not an integer");
  const long v = o[key].get<long>();
  if (v < lo || v > hi) fail(where + "." + key, "out of range");
  return v;
}

std::string get_str(const json& o, const char* key, const std::string& dflt,
                    const std::string& where) {
  if (!o.contains(key)) return dflt;
  if (!o[key].is_string()) fail(where + "." + key, "not a string");
  return o[key].get<std::string>();
}
}  // namespace

Config load_config(const std::string& path) {
  std::ifstream f(path);
  if (!f) fail(path, "cannot open");
  json j;
  try {
    j = json::parse(f);
  } catch (const std::exception& e) {
    fail(path, std::string("parse error: ") + e.what());
  }
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
    const json& r = j["dvr"];
    check_keys(r, "dvr", {"autostart", "dir", "fragment_ms", "mode", "burned"});
    if (r.contains("autostart")) {
      if (!r["autostart"].is_boolean()) fail("dvr.autostart", "not a boolean");
      c.dvr.autostart = r["autostart"].get<bool>();
    }
    c.dvr.dir = get_str(r, "dir", "/media/dvr", "dvr");
    c.dvr.fragment_ms = static_cast<int>(get_int(r, "fragment_ms", 1000, 100, 10000, "dvr"));
    c.dvr.mode = get_str(r, "mode", "raw", "dvr");
    if (c.dvr.mode != "raw" && c.dvr.mode != "burned")
      fail("dvr.mode", "must be \"raw\" or \"burned\"");
    if (r.contains("burned")) {
      const json& b = r["burned"];
      check_keys(b, "dvr.burned", {"bitrate_kbps", "fps_cap"});
      c.dvr.burned.bitrate_kbps =
          static_cast<int>(get_int(b, "bitrate_kbps", 12000, 500, 100000, "dvr.burned"));
      c.dvr.burned.fps_cap =
          static_cast<int>(get_int(b, "fps_cap", 30, 1, 120, "dvr.burned"));
    }
  }

  if (j.contains("osd")) {
    const json& o = j["osd"];
    check_keys(o, "osd", {"enable", "port", "font", "scale", "stale_ms", "gs"});
    if (o.contains("enable")) {
      if (!o["enable"].is_boolean()) fail("osd.enable", "not a boolean");
      c.osd.enable = o["enable"].get<bool>();
    }
    c.osd.port = static_cast<int>(get_int(o, "port", 14560, 1, 65535, "osd"));
    c.osd.font = get_str(o, "font", c.osd.font, "osd");
    c.osd.scale = get_str(o, "scale", "sharp", "osd");
    if (c.osd.scale != "sharp" && c.osd.scale != "fill")
      fail("osd.scale", "must be \"sharp\" or \"fill\"");
    // Default mirrors OsdCfg::stale_ms (see player_config.h for why 5000).
    c.osd.stale_ms = static_cast<int>(get_int(o, "stale_ms", 5000, 0, 60000, "osd"));

    if (o.contains("gs")) {
      const json& g = o["gs"];
      check_keys(g, "osd.gs", {"enable", "port", "font", "stale_ms"});
      if (g.contains("enable")) {
        if (!g["enable"].is_boolean()) fail("osd.gs.enable", "not a boolean");
        c.osd.gs.enable = g["enable"].get<bool>();
      }
      c.osd.gs.port = static_cast<int>(get_int(g, "port", c.osd.gs.port, 1, 65535, "osd.gs"));
      c.osd.gs.font = get_str(g, "font", c.osd.gs.font, "osd.gs");
      // Default mirrors OsdCfg::GsCfg::stale_ms (see player_config.h).
      c.osd.gs.stale_ms =
          static_cast<int>(get_int(g, "stale_ms", c.osd.gs.stale_ms, 0, 60000, "osd.gs"));
    }
  }

  if (j.contains("input")) {
    const json& in = j["input"];
    check_keys(in, "input", {"rec"});
    if (in.contains("rec")) {
      const json& rc = in["rec"];
      check_keys(rc, "input.rec", {"pin", "active_low", "bias"});
      // Required, not defaulted: a rec block with no pin is a typo, and
      // defaulting it would silently claim some unrelated line.
      if (!rc.contains("pin")) fail("input.rec.pin", "required");
      // Upper bound is generous on purpose: header pins run to 40 on this
      // board, but a GPIO<n>-naming kernel can go far higher.
      c.input.rec.pin = static_cast<int>(get_int(rc, "pin", 0, 1, 512, "input.rec"));
      if (rc.contains("active_low")) {
        if (!rc["active_low"].is_boolean()) fail("input.rec.active_low", "not a boolean");
        c.input.rec.active_low = rc["active_low"].get<bool>();
      }
      c.input.rec.bias = get_str(rc, "bias", "pull-up", "input.rec");
      if (c.input.rec.bias != "pull-up" && c.input.rec.bias != "pull-down" &&
          c.input.rec.bias != "none") {
        fail("input.rec.bias", "must be \"pull-up\", \"pull-down\" or \"none\"");
      }
      c.input.rec.configured = true;
    }
  }

  if (j.contains("display")) {
    const json& d = j["display"];
    check_keys(d, "display",
               {"regulate_ms", "vsync_lock", "vsync_lead_ms", "lat_log_dir"});
    c.display.regulate_ms =
        static_cast<int>(get_int(d, "regulate_ms", 12, 0, 100, "display"));
    if (d.contains("vsync_lock")) {
      if (!d["vsync_lock"].is_boolean()) fail("display.vsync_lock", "not a boolean");
      c.display.vsync_lock = d["vsync_lock"].get<bool>();
    }
    c.display.vsync_lead_ms =
        static_cast<int>(get_int(d, "vsync_lead_ms", 6, 1, 10, "display"));
    c.display.lat_log_dir = get_str(d, "lat_log_dir", "/media/dvr/log", "display");
  }

  return c;
}

}  // namespace maburplay
