#include "config.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "json.hpp"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"

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
  check_known_keys(j, {"usb_vid", "usb_pid", "channel", "width",
                        "thermal_max_delta", "power_mode",
                        "power_offset_qdb", "tx_threads", "rate_walls_idx",
                        "legacy_wall_idx", "wall_margin_db",
                        "min_offset_qdb", "base_ref_idx"},
                   "radio");
  assign_if_present(j, "usb_vid", r.usb_vid, "radio");
  assign_if_present(j, "usb_pid", r.usb_pid, "radio");
  assign_if_present(j, "channel", r.channel, "radio");
  assign_if_present(j, "width", r.width, "radio");
  assign_if_present(j, "thermal_max_delta", r.thermal_max_delta, "radio");
  assign_if_present(j, "power_mode", r.power_mode, "radio");
  assign_if_present(j, "power_offset_qdb", r.power_offset_qdb, "radio");
  assign_if_present(j, "tx_threads", r.tx_threads, "radio");

  bool rate_walls_idx_present = j.contains("rate_walls_idx");
  if (rate_walls_idx_present) {
    auto& arr = j.at("rate_walls_idx");
    if (!arr.is_array() || arr.size() != 8)
      fail("radio.rate_walls_idx", "must be an array of 8 ints");
    try {
      for (size_t i = 0; i < 8; ++i)
        r.rate_walls_idx[i] = arr.at(i).get<int>();
    } catch (const json::exception&) {
      fail("radio.rate_walls_idx", "wrong type");
    }
  }
  assign_if_present(j, "legacy_wall_idx", r.legacy_wall_idx, "radio");
  assign_if_present(j, "wall_margin_db", r.wall_margin_db, "radio");
  assign_if_present(j, "min_offset_qdb", r.min_offset_qdb, "radio");
  assign_if_present(j, "base_ref_idx", r.base_ref_idx, "radio");

  if (r.channel < 1 || r.channel > 177) fail("radio.channel", "must be in [1,177]");
  if (r.tx_threads < 1 || r.tx_threads > 8)
    fail("radio.tx_threads", "must be in [1,8]");
  if (r.power_mode != "override" && r.power_mode != "offset" &&
      r.power_mode != "none")
    fail("radio.power_mode", "must be \"override\", \"offset\" or \"none\"");
  if (r.power_offset_qdb < -128 || r.power_offset_qdb > 128)
    fail("radio.power_offset_qdb", "must be in [-128,128]");

  if (r.power_mode == "offset" && !rate_walls_idx_present)
    fail("radio.rate_walls_idx", "required when radio.power_mode is \"offset\"");
  for (int w : r.rate_walls_idx)
    if (w < 0 || w > 127) fail("radio.rate_walls_idx", "values must be in [0,127]");
  if (r.legacy_wall_idx < 0 || r.legacy_wall_idx > 127)
    fail("radio.legacy_wall_idx", "must be in [0,127]");
  if (r.wall_margin_db < 0.0 || r.wall_margin_db > 6.0)
    fail("radio.wall_margin_db", "must be in [0,6]");
  if (r.min_offset_qdb < -64 || r.min_offset_qdb > 0)
    fail("radio.min_offset_qdb", "must be in [-64,0]");
  if (r.base_ref_idx < 0 || r.base_ref_idx > 127)
    fail("radio.base_ref_idx", "must be in [0,127]");

  // The 8822E's per-rate diff field is 7-bit two's complement (devourer's
  // pack_rate_diff_word masks & 0x7f), so every diff make_power_plan will
  // derive — walls[r] - m - base_ref_idx, and the same for legacy_wall_idx —
  // must land in [-64, 63]. Outside that range the value silently wraps on
  // air (e.g. +70 becomes -58) with no error, sign-flipping per-rate power.
  // A miscalibrated config (e.g. base_ref_idx left at 0) must refuse to
  // boot here rather than let power_plan.h's clamp silently paper over it.
  if (r.power_mode == "offset") {
    const int m = static_cast<int>(std::lround(r.wall_margin_db * 4.0));
    for (int w : r.rate_walls_idx) {
      const int diff = w - m - r.base_ref_idx;
      if (diff < -64 || diff > 63)
        fail("radio.rate_walls_idx",
             "derived diff (wall - wall_margin_db*4 - base_ref_idx) = " +
                 std::to_string(diff) +
                 " is out of the 7-bit hardware field range [-64,63] — "
                 "check base_ref_idx/wall_margin_db calibration");
    }
    const int legacy_diff = r.legacy_wall_idx - m - r.base_ref_idx;
    if (legacy_diff < -64 || legacy_diff > 63)
      fail("radio.legacy_wall_idx",
           "derived diff (legacy_wall_idx - wall_margin_db*4 - base_ref_idx) = " +
               std::to_string(legacy_diff) +
               " is out of the 7-bit hardware field range [-64,63] — "
               "check base_ref_idx/wall_margin_db calibration");
  }
}

void parse_fec(const json& j, FecCfg& f) {
  check_known_keys(j, {"symbol_size", "window", "blocks_per_body", "base_overhead", "flush_ms"}, "fec");
  if (j.contains("symbol_size")) {
    auto& s = j.at("symbol_size");
    if (s.is_array()) {
      if (s.size() != 4) fail("fec.symbol_size", "array must have 4 ints");
      try {
        for (size_t i = 0; i < 4; ++i) f.symbol_size[i] = s.at(i).get<int>();
      } catch (const json::exception&) {
        fail("fec.symbol_size", "wrong type");
      }
    } else {
      int v = 0;
      try { v = s.get<int>(); } catch (const json::exception&) {
        fail("fec.symbol_size", "wrong type");
      }
      f.symbol_size.fill(v);
    }
  }
  assign_if_present(j, "window", f.window, "fec");
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

  for (int s : f.symbol_size)
    if (s < 32 || s > 1500) fail("fec.symbol_size", "must be in [32,1500]");
  if (f.window < 2 || f.window > 255) fail("fec.window", "must be in [2,255]");
  for (int b : f.blocks_per_body)
    if (b < 1 || b > 255) fail("fec.blocks_per_body", "must be in [1,255]");
  for (size_t i = 0; i < 4; ++i) {
    const int body = f.blocks_per_body[i] *
                     (static_cast<int>(sw::kSwHeaderLen) + f.symbol_size[i]);
    if (body > kMaxBodyBytes)
      fail("fec", "layer body bytes exceed kMaxBodyBytes (2900)");
  }
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

void parse_msp(const json& j, MspCfg& m) {
  check_known_keys(j, {"enable", "serial", "baud", "update_rate_hz",
                        "symbol_size", "window", "overhead"}, "msp");
  assign_if_present(j, "enable", m.enable, "msp");
  assign_if_present(j, "serial", m.serial, "msp");
  assign_if_present(j, "baud", m.baud, "msp");
  assign_if_present(j, "update_rate_hz", m.update_rate_hz, "msp");
  assign_if_present(j, "symbol_size", m.symbol_size, "msp");
  assign_if_present(j, "window", m.window, "msp");
  assign_if_present(j, "overhead", m.overhead, "msp");

  if (m.update_rate_hz <= 0) fail("msp.update_rate_hz", "must be > 0");
  if (m.symbol_size < 16 || m.symbol_size > 2048)
    fail("msp.symbol_size", "must be in [16,2048]");
  if (m.window < 2 || m.window > 255) fail("msp.window", "must be in [2,255]");
  if (m.overhead < 0.0 || m.overhead > 4.0) fail("msp.overhead", "must be in [0,4]");
  if (m.baud <= 0) fail("msp.baud", "must be > 0");
}

}  // namespace

std::array<UepLayerCfg, 4> Config::uep_layers() const {
  std::array<UepLayerCfg, 4> layers;
  for (int sid = 0; sid < 4; ++sid) {
    double overhead = uep_layer_overhead(sid, fec.base_overhead);
    layers[static_cast<size_t>(sid)].fec =
        SwConfig{fec.symbol_size[static_cast<size_t>(sid)], fec.window, overhead};
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
                    {"radio", "fec", "waybeam", "link", "msp",
                     "frame_ring_name"},
                    "");

  Config cfg;
  if (j.contains("radio")) parse_radio(j.at("radio"), cfg.radio);
  if (j.contains("fec")) parse_fec(j.at("fec"), cfg.fec);
  if (j.contains("waybeam")) parse_waybeam(j.at("waybeam"), cfg.waybeam);
  if (j.contains("link")) parse_link(j.at("link"), cfg.link);
  if (j.contains("msp")) parse_msp(j.at("msp"), cfg.msp);
  assign_if_present(j, "frame_ring_name", cfg.frame_ring_name, "");

  return cfg;
}

}  // namespace mabur
