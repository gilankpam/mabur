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
  check_keys(j, "", {"ring_path", "socket", "backend", "screen_mode", "dvr"});
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
    check_keys(r, "dvr", {"enabled", "dir", "fragment_ms"});
    if (r.contains("enabled")) {
      if (!r["enabled"].is_boolean()) fail("dvr.enabled", "not a boolean");
      c.dvr.enabled = r["enabled"].get<bool>();
    }
    c.dvr.dir = get_str(r, "dir", "/media/dvr", "dvr");
    c.dvr.fragment_ms = static_cast<int>(get_int(r, "fragment_ms", 1000, 100, 10000, "dvr"));
  }

  return c;
}

}  // namespace maburplay
