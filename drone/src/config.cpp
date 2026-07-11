#include "config.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "json.hpp"

namespace mabur {
namespace {

using nlohmann::json;

[[noreturn]] void fail(const std::string& field, const std::string& why) {
  throw std::runtime_error("config: " + field + ": " + why);
}

// Rejects any key in `j` (a JSON object) that isn't in `known`. `prefix` is
// the dotted field-path prefix used in the error message (e.g. "fec").
void check_known_keys(const json& j, const std::vector<std::string>& known,
                       const std::string& prefix) {
  if (!j.is_object()) return;
  for (auto it = j.begin(); it != j.end(); ++it) {
    const std::string& key = it.key();
    if (std::find(known.begin(), known.end(), key) == known.end()) {
      std::string field = prefix.empty() ? key : prefix + "." + key;
      fail(field, "unknown key");
    }
  }
}

template <typename T>
void assign_if_present(const json& j, const char* key, T& out,
                       const std::string& prefix = "") {
  if (j.contains(key)) {
    try {
      out = j.at(key).get<T>();
    } catch (const json::exception& e) {
      std::string field = prefix.empty() ? key : prefix + "." + key;
      fail(field, "wrong type");
    }
  }
}

void parse_radio(const json& j, RadioCfg& r) {
  check_known_keys(j, {"usb_vid", "usb_pid", "channel", "width", "bw_set",
                        "max_txagc", "thermal_max_delta"},
                   "radio");
  assign_if_present(j, "usb_vid", r.usb_vid, "radio");
  assign_if_present(j, "usb_pid", r.usb_pid, "radio");
  assign_if_present(j, "channel", r.channel, "radio");
  assign_if_present(j, "width", r.width, "radio");
  if (j.contains("bw_set")) {
    r.bw_set.clear();
    try {
      for (auto& v : j.at("bw_set")) r.bw_set.push_back(v.get<uint8_t>());
    } catch (const json::exception& e) {
      fail("radio.bw_set", "wrong type");
    }
  }
  assign_if_present(j, "max_txagc", r.max_txagc, "radio");
  assign_if_present(j, "thermal_max_delta", r.thermal_max_delta, "radio");

  if (r.channel < 1 || r.channel > 177) fail("radio.channel", "must be in [1,177]");

  uint8_t prev = 0;
  for (uint8_t bw : r.bw_set) {
    if (bw != 20 && bw != 40 && bw != 80) fail("radio.bw_set", "values must be in {20,40,80}");
    if (bw <= prev) fail("radio.bw_set", "values must be strictly ascending");
    prev = bw;
  }

  // B7 (docs/bench-validation.md): a device tuned to radio.width cannot emit
  // a valid PPDU wider than that — such probe rungs are dead air (transmitted
  // but unreceivable). Drop them with a warning rather than flying them.
  std::erase_if(r.bw_set, [&](uint8_t bw) {
    if (bw <= r.width) return false;
    std::fprintf(stderr,
                 "mabur config: radio.bw_set rung %u > radio.width %u — "
                 "dropped (a %u MHz-tuned device cannot emit %u MHz frames)\n",
                 bw, r.width, r.width, bw);
    return true;
  });
}

void parse_fec(const json& j, FecCfg& f) {
  check_known_keys(j, {"k", "symbol_size", "blocks_per_body", "base_overhead", "flush_ms"}, "fec");
  assign_if_present(j, "k", f.k, "fec");
  assign_if_present(j, "symbol_size", f.symbol_size, "fec");
  if (j.contains("blocks_per_body")) {
    auto& arr = j.at("blocks_per_body");
    if (!arr.is_array() || arr.size() != 4) fail("fec.blocks_per_body", "must be an array of 4 ints");
    try {
      for (size_t i = 0; i < 4; ++i) f.blocks_per_body[i] = arr.at(i).get<int>();
    } catch (const json::exception& e) {
      fail("fec.blocks_per_body", "wrong type");
    }
  }
  assign_if_present(j, "base_overhead", f.base_overhead, "fec");
  assign_if_present(j, "flush_ms", f.flush_ms, "fec");

  if (f.k < 2 || f.k > 32) fail("fec.k", "must be in [2,32]");
  if (f.symbol_size < 16 || f.symbol_size > 1024) fail("fec.symbol_size", "must be in [16,1024]");
  for (int b : f.blocks_per_body)
    if (b < 1 || b > 255) fail("fec.blocks_per_body", "must be in [1,255]");
  if (f.base_overhead < 0.05 || f.base_overhead > 2.0) fail("fec.base_overhead", "must be in [0.05,2.0]");
}

void parse_waybeam(const json& j, WaybeamCfg& w) {
  check_known_keys(j,
                    {"host", "port", "idr_path", "bitrate_min_kbps", "bitrate_max_kbps",
                     "airtime_budget", "roi_threshold_kbps", "roi_qp_low", "roi_qp_normal"},
                    "waybeam");
  assign_if_present(j, "host", w.host, "waybeam");
  assign_if_present(j, "port", w.port, "waybeam");
  assign_if_present(j, "idr_path", w.idr_path, "waybeam");
  assign_if_present(j, "bitrate_min_kbps", w.bitrate_min_kbps, "waybeam");
  assign_if_present(j, "bitrate_max_kbps", w.bitrate_max_kbps, "waybeam");
  assign_if_present(j, "airtime_budget", w.airtime_budget, "waybeam");
  assign_if_present(j, "roi_threshold_kbps", w.roi_threshold_kbps, "waybeam");
  assign_if_present(j, "roi_qp_low", w.roi_qp_low, "waybeam");
  assign_if_present(j, "roi_qp_normal", w.roi_qp_normal, "waybeam");

  if (w.port < 1 || w.port > 65535) fail("waybeam.port", "must be in [1,65535]");
  if (w.bitrate_min_kbps < 100) fail("waybeam.bitrate_min_kbps", "must be >= 100");
  if (w.bitrate_min_kbps >= w.bitrate_max_kbps)
    fail("waybeam.bitrate_min_kbps", "must be < waybeam.bitrate_max_kbps");
  if (w.airtime_budget <= 0.0 || w.airtime_budget > 1.0)
    fail("waybeam.airtime_budget", "must be in (0,1]");
  if (w.roi_threshold_kbps < 0) fail("waybeam.roi_threshold_kbps", "must be >= 0");
}

void parse_link(const json& j, LinkCfg& l) {
  check_known_keys(j, {"vtx_id", "failsafe_ms", "rendezvous_ms", "tick_ms"}, "link");
  assign_if_present(j, "vtx_id", l.vtx_id, "link");
  assign_if_present(j, "failsafe_ms", l.failsafe_ms, "link");
  assign_if_present(j, "rendezvous_ms", l.rendezvous_ms, "link");
  assign_if_present(j, "tick_ms", l.tick_ms, "link");

  if (l.vtx_id == 0) fail("link.vtx_id", "must be non-zero");
}

void parse_flags(const json& j, rc::FlagPolicy& fp) {
  check_known_keys(j, {"crit_ldpc", "crit_stbc", "t0_ldpc", "t0_stbc"}, "flags");
  assign_if_present(j, "crit_ldpc", fp.crit_ldpc, "flags");
  assign_if_present(j, "crit_stbc", fp.crit_stbc, "flags");
  assign_if_present(j, "t0_ldpc", fp.t0_ldpc, "flags");
  assign_if_present(j, "t0_stbc", fp.t0_stbc, "flags");
}

}  // namespace

std::array<UepLayerCfg, 4> Config::uep_layers() const {
  std::array<UepLayerCfg, 4> layers;
  for (int sid = 0; sid < 4; ++sid) {
    double overhead = uep_layer_overhead(sid, fec.base_overhead);
    layers[static_cast<size_t>(sid)].fec = RsConfig{fec.k, fec.symbol_size, overhead};
    layers[static_cast<size_t>(sid)].blocks_per_body = fec.blocks_per_body[static_cast<size_t>(sid)];
  }
  return layers;
}

Config load_config(const std::string& path) {
  std::ifstream in(path);
  if (!in) fail("file", "cannot open '" + path + "'");

  json j;
  try {
    j = json::parse(in);
  } catch (const json::parse_error& e) {
    fail("file", std::string("invalid JSON: ") + e.what());
  }

  if (!j.is_object()) fail("file", "top-level JSON must be an object");

  check_known_keys(j,
                    {"radio", "fec", "waybeam", "link", "ring_name", "flags", "power_offset_db"},
                    "");

  Config cfg;
  if (j.contains("radio")) parse_radio(j.at("radio"), cfg.radio);
  if (j.contains("fec")) parse_fec(j.at("fec"), cfg.fec);
  if (j.contains("waybeam")) parse_waybeam(j.at("waybeam"), cfg.waybeam);
  if (j.contains("link")) parse_link(j.at("link"), cfg.link);
  assign_if_present(j, "ring_name", cfg.ring_name, "");
  if (j.contains("flags")) parse_flags(j.at("flags"), cfg.flags);
  if (j.contains("power_offset_db")) {
    auto& arr = j.at("power_offset_db");
    if (!arr.is_array() || arr.size() != 4) fail("power_offset_db", "must be an array of 4 ints");
    try {
      for (size_t i = 0; i < 4; ++i) cfg.power_offset_db[i] = arr.at(i).get<int8_t>();
    } catch (const json::exception& e) {
      fail("power_offset_db", "wrong type");
    }
  }

  return cfg;
}

}  // namespace mabur
