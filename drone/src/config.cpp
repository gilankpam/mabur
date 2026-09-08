#include "config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "mabur/toml.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"

namespace mabur {
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
// validated: a cross-key check (e.g. "up_util >= down_util") runs after both
// keys have been read, so it reports whichever key assign_if_present touched
// last, not necessarily the one actually at fault. This is a known, accepted
// limitation, not a bug to "fix": the field NAME in a fail() call is always
// the right one, so the operator is never misdirected about WHICH key is
// bad — only the cited line can be off by a key when two interacting keys
// disagree.
int g_line = 0;

[[noreturn]] void fail(const std::string& field, const std::string& why) {
  std::string where = "config: ";
  if (!g_file.empty() && g_line > 0)
    where += g_file + ":" + std::to_string(g_line) + ": ";
  throw std::runtime_error(where + field + ": " + why);
}

// Rejects any key in `j` (a TOML table) that isn't in `known`. `prefix` is
// the dotted field-path prefix used in the error message (e.g. "fec").
void check_known_keys(const Value& j, const std::vector<std::string>& known,
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
void assign_if_present(const Value& j, const char* key, T& out,
                       const std::string& prefix = "") {
  if (!j.contains(key)) {
    note_default(prefix, key, to_text(out));
    return;
  }
  try {
    g_line = j.at(key).line();
    out = j.at(key).get<T>();
  } catch (const toml::Error& e) {
    std::string field = prefix.empty() ? key : prefix + "." + key;
    fail(field, "wrong type");
  }
}

void parse_radio(const Value& j, RadioCfg& r) {
  check_known_keys(j, {"usb_vid", "usb_pid", "channel", "width",
                        "power_mode", "tx_threads", "rate_walls_idx",
                        "legacy_wall_idx", "wall_margin_db",
                        "base_ref_idx"},
                   "radio");
  assign_if_present(j, "usb_vid", r.usb_vid, "radio");
  assign_if_present(j, "usb_pid", r.usb_pid, "radio");
  assign_if_present(j, "channel", r.channel, "radio");
  assign_if_present(j, "width", r.width, "radio");
  assign_if_present(j, "power_mode", r.power_mode, "radio");
  assign_if_present(j, "tx_threads", r.tx_threads, "radio");

  bool rate_walls_idx_present = j.contains("rate_walls_idx");
  if (rate_walls_idx_present) {
    auto& arr = j.at("rate_walls_idx");
    g_line = arr.line();
    if (!arr.is_array() || arr.size() != 8)
      fail("radio.rate_walls_idx", "must be an array of 8 ints");
    try {
      for (size_t i = 0; i < 8; ++i)
        r.rate_walls_idx[i] = arr.at(i).get<int>();
    } catch (const toml::Error&) {
      fail("radio.rate_walls_idx", "wrong type");
    }
  }
  if (!rate_walls_idx_present) note_default("radio", "rate_walls_idx", "(unit defaults)");
  assign_if_present(j, "legacy_wall_idx", r.legacy_wall_idx, "radio");
  assign_if_present(j, "wall_margin_db", r.wall_margin_db, "radio");
  assign_if_present(j, "base_ref_idx", r.base_ref_idx, "radio");

  if (r.channel < 1 || r.channel > 177) fail("radio.channel", "must be in [1,177]");
  if (r.tx_threads < 1 || r.tx_threads > 8)
    fail("radio.tx_threads", "must be in [1,8]");
  if (r.power_mode != "offset" && r.power_mode != "none")
    fail("radio.power_mode", "must be \"offset\" or \"none\"");

  if (r.power_mode == "offset" && !rate_walls_idx_present)
    fail("radio.rate_walls_idx", "required when radio.power_mode is \"offset\"");
  for (int w : r.rate_walls_idx)
    if (w < 0 || w > 127) fail("radio.rate_walls_idx", "values must be in [0,127]");
  if (r.legacy_wall_idx < 0 || r.legacy_wall_idx > 127)
    fail("radio.legacy_wall_idx", "must be in [0,127]");
  if (r.wall_margin_db < 0.0 || r.wall_margin_db > 6.0)
    fail("radio.wall_margin_db", "must be in [0,6]");
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

void parse_fec(const Value& j, FecCfg& f) {
  check_known_keys(j, {"symbol_size", "window", "blocks_per_body", "base_overhead", "flush_ms", "feed_batch"}, "fec");
  if (j.contains("symbol_size")) {
    auto& s = j.at("symbol_size");
    g_line = s.line();
    if (s.is_array()) {
      if (s.size() != 2) fail("fec.symbol_size", "array must have 2 ints");
      try {
        for (size_t i = 0; i < 2; ++i) f.symbol_size[i] = s.at(i).get<int>();
      } catch (const toml::Error&) {
        fail("fec.symbol_size", "wrong type");
      }
    } else {
      int v = 0;
      try { v = s.get<int>(); } catch (const toml::Error&) {
        fail("fec.symbol_size", "wrong type");
      }
      f.symbol_size.fill(v);
    }
  } else {
    note_default("fec", "symbol_size", to_text(f.symbol_size[0]));
  }
  assign_if_present(j, "window", f.window, "fec");
  if (j.contains("blocks_per_body")) {
    auto& arr = j.at("blocks_per_body");
    g_line = arr.line();
    if (!arr.is_array() || arr.size() != 2) fail("fec.blocks_per_body", "must be an array of 2 ints");
    try {
      for (size_t i = 0; i < 2; ++i) f.blocks_per_body[i] = arr.at(i).get<int>();
    } catch (const toml::Error& e) {
      fail("fec.blocks_per_body", "wrong type");
    }
  } else {
    note_default("fec", "blocks_per_body", to_text(f.blocks_per_body[0]));
  }
  assign_if_present(j, "base_overhead", f.base_overhead, "fec");
  assign_if_present(j, "flush_ms", f.flush_ms, "fec");
  assign_if_present(j, "feed_batch", f.feed_batch, "fec");
  if (f.feed_batch < 0 || f.feed_batch > 8)
    fail("fec.feed_batch", "must be in [0,8] (0 = streaming push)");

  for (int s : f.symbol_size)
    if (s < 32 || s > 1500) fail("fec.symbol_size", "must be in [32,1500]");
  if (f.window < 2 || f.window > 255) fail("fec.window", "must be in [2,255]");
  for (int b : f.blocks_per_body)
    if (b < 1 || b > 255) fail("fec.blocks_per_body", "must be in [1,255]");
  for (size_t i = 0; i < 2; ++i) {
    const int body = f.blocks_per_body[i] *
                     (static_cast<int>(sw::kSwHeaderLen) + f.symbol_size[i]);
    if (body > kMaxBodyBytes)
      fail("fec", "layer body bytes exceed kMaxBodyBytes (2900)");
  }
  if (f.base_overhead < 0.1 || f.base_overhead > 2.0) fail("fec.base_overhead", "must be in [0.1,2.0]");
}

void parse_encoder(const Value& j, EncoderCfg& e) {
  check_known_keys(j,
                    {"bitrate_min_kbps", "bitrate_max_kbps", "airtime_budget",
                     "roi_threshold_kbps", "roi_qp_low", "roi_qp_normal"},
                    "encoder");
  assign_if_present(j, "bitrate_min_kbps", e.bitrate_min_kbps, "encoder");
  assign_if_present(j, "bitrate_max_kbps", e.bitrate_max_kbps, "encoder");
  assign_if_present(j, "airtime_budget", e.airtime_budget, "encoder");
  assign_if_present(j, "roi_threshold_kbps", e.roi_threshold_kbps, "encoder");
  assign_if_present(j, "roi_qp_low", e.roi_qp_low, "encoder");
  assign_if_present(j, "roi_qp_normal", e.roi_qp_normal, "encoder");

  if (e.bitrate_min_kbps < 100) fail("encoder.bitrate_min_kbps", "must be >= 100");
  if (e.bitrate_min_kbps >= e.bitrate_max_kbps)
    fail("encoder.bitrate_min_kbps", "must be < encoder.bitrate_max_kbps");
  if (e.airtime_budget <= 0.0 || e.airtime_budget > 1.0)
    fail("encoder.airtime_budget", "must be in (0,1]");
  if (e.roi_threshold_kbps < 0) fail("encoder.roi_threshold_kbps", "must be >= 0");
}

// venc: boot-time encoder pipeline config -> VencCfg (spec 2026-08-28
// venc-foldin §3). The venc core is a pure mechanism with zero policy, so
// there is intentionally NO "bitrate" key here — it's simply absent from
// the known-key set below, so one lands on the ordinary unknown-key path
// like any other stale key (global constraint: no venc.bitrate ever).
void parse_venc(const Value& j, VencSectionCfg& v) {
  check_known_keys(j,
                    {"sensor_bin", "size", "fps", "gop_s", "qp_delta",
                     "max_ipprop", "min_iqp", "superframe_p_pct",
                     "intra_refresh_rows", "intra_refresh_qp",
                     "ref_base", "ref_enhance", "ref_pred",
                     "roi", "ae_fps", "awb_fps", "snapshot_quality",
                     "debug_port", "module_loader"},
                    "venc");

  // Real compiled defaults, for accurate reporting only. Every branch below
  // reads into a local temp (or validates in place) before the struct field
  // is ever touched, so assign_if_present's own absent-branch never fires
  // for a key here -- report the actual compiled default explicitly instead
  // of letting a temp's zero-init sentinel masquerade as one (a bare
  // "venc.fps=0" would be a lie: the real default is 60).
  const VencSectionCfg kDef{};

  if (j.contains("sensor_bin")) {
    std::string s;
    assign_if_present(j, "sensor_bin", s, "venc");
    if (s.size() >= sizeof(v.core.sensor_bin))
      fail("venc.sensor_bin", "too long");
    std::snprintf(v.core.sensor_bin, sizeof(v.core.sensor_bin), "%s", s.c_str());
  } else {
    note_default("venc", "sensor_bin", "(required, no default)");
  }

  if (j.contains("size")) {
    std::string s;
    assign_if_present(j, "size", s, "venc");
    auto x = s.find('x');
    int w = 0, h = 0;
    bool ok = x != std::string::npos && x > 0 && x + 1 < s.size();
    if (ok) {
      try {
        size_t wend = 0, hend = 0;
        w = std::stoi(s.substr(0, x), &wend);
        h = std::stoi(s.substr(x + 1), &hend);
        ok = wend == x && hend == s.size() - x - 1;
      } catch (const std::exception&) {
        ok = false;
      }
    }
    if (!ok || w <= 0 || h <= 0)
      fail("venc.size", "malformed, expected WIDTHxHEIGHT (e.g. \"1920x1080\")");
    v.core.width = static_cast<uint16_t>(w);
    v.core.height = static_cast<uint16_t>(h);
  } else {
    note_default("venc", "size",
                  to_text(kDef.core.width) + "x" + to_text(kDef.core.height));
  }

  if (j.contains("fps")) {
    int fps = 0;
    assign_if_present(j, "fps", fps, "venc");
    if (fps < 1 || fps > 120) fail("venc.fps", "must be in [1,120]");
    v.core.fps = static_cast<uint16_t>(fps);
  } else {
    note_default("venc", "fps", to_text(kDef.core.fps));
  }

  if (j.contains("gop_s")) {
    assign_if_present(j, "gop_s", v.core.gop_s, "venc");
    if (v.core.gop_s < 0.5 || v.core.gop_s > 10.0)
      fail("venc.gop_s", "must be in [0.5,10]");
  } else {
    note_default("venc", "gop_s", to_text(kDef.core.gop_s));
  }

  if (j.contains("qp_delta")) {
    int qp = 0;
    assign_if_present(j, "qp_delta", qp, "venc");
    if (qp < -12 || qp > 12) fail("venc.qp_delta", "must be in [-12,12]");
    v.core.qp_delta = static_cast<int8_t>(qp);
  } else {
    note_default("venc", "qp_delta", to_text(static_cast<int>(kDef.core.qp_delta)));
  }

  if (j.contains("max_ipprop")) {
    int prop = 0;
    assign_if_present(j, "max_ipprop", prop, "venc");
    if (prop < 0 || prop > 100) fail("venc.max_ipprop", "must be in [0,100]");
    v.core.max_ipprop = static_cast<uint8_t>(prop);
  } else {
    note_default("venc", "max_ipprop", to_text(static_cast<int>(kDef.core.max_ipprop)));
  }

  if (j.contains("min_iqp")) {
    int q = 0;
    assign_if_present(j, "min_iqp", q, "venc");
    if (q < 0 || q > 51) fail("venc.min_iqp", "must be in [0,51]");
    v.core.min_iqp = static_cast<uint8_t>(q);
  } else {
    note_default("venc", "min_iqp", to_text(static_cast<int>(kDef.core.min_iqp)));
  }

  if (j.contains("superframe_p_pct")) {
    int p = 0;
    assign_if_present(j, "superframe_p_pct", p, "venc");
    if (p != 0 && (p < 100 || p > 1000))
      fail("venc.superframe_p_pct", "must be 0 (off) or in [100,1000]");
    v.core.superframe_p_pct = static_cast<uint16_t>(p);
  } else {
    note_default("venc", "superframe_p_pct", to_text(kDef.core.superframe_p_pct));
  }

  // Error-resilience structure: the five components venc.resilience used to
  // name a preset for (deleted 2026-09-04). Each lands on an MI struct field
  // verbatim, so the checks below are the hardware's own limits, not policy.
  // Parsed AFTER "size" because the rows bound depends on the picture height.
  if (j.contains("intra_refresh_rows")) {
    int rows = 0;
    assign_if_present(j, "intra_refresh_rows", rows, "venc");
    if (rows < 0 || rows > UINT16_MAX)
      fail("venc.intra_refresh_rows", "must be >= 0");
    v.core.intra_refresh_rows = static_cast<uint16_t>(rows);
  } else {
    note_default("venc", "intra_refresh_rows", to_text(kDef.core.intra_refresh_rows));
  }

  if (j.contains("intra_refresh_qp")) {
    int qp = 0;
    assign_if_present(j, "intra_refresh_qp", qp, "venc");
    if (qp < 1 || qp > 51) fail("venc.intra_refresh_qp", "must be in [1,51]");
    v.core.intra_refresh_qp = static_cast<uint8_t>(qp);
  } else {
    note_default("venc", "intra_refresh_qp", to_text(static_cast<int>(kDef.core.intra_refresh_qp)));
  }

  if (j.contains("ref_base")) {
    int base = 0;
    assign_if_present(j, "ref_base", base, "venc");
    if (base < 0 || base > 255)
      fail("venc.ref_base", "must be 0 (SVC-T off) or in [1,255]");
    v.core.ref_base = static_cast<uint8_t>(base);
  } else {
    note_default("venc", "ref_base", to_text(static_cast<int>(kDef.core.ref_base)));
  }

  if (j.contains("ref_enhance")) {
    int enh = 0;
    assign_if_present(j, "ref_enhance", enh, "venc");
    if (enh < 0 || enh > 255) fail("venc.ref_enhance", "must be in [0,255]");
    v.core.ref_enhance = static_cast<uint8_t>(enh);
  } else {
    note_default("venc", "ref_enhance", to_text(static_cast<int>(kDef.core.ref_enhance)));
  }

  assign_if_present(j, "ref_pred", v.core.ref_pred, "venc");

  // u32Enhance is a period (one non-referenced frame per enhance+1), so 0
  // has no meaning while SVC-T is on. The apply site used to paper over it
  // with `enhance ? enhance : 1`, silently running a structure the config
  // did not ask for. Checked after both keys are read so either order works.
  if (v.core.ref_base != 0 && v.core.ref_enhance == 0)
    fail("venc.ref_enhance", "must be >= 1 when venc.ref_base is nonzero");

  if (j.contains("roi")) {
    const Value& r = j.at("roi");
    check_known_keys(r, {"enabled", "steps", "center"}, "venc.roi");
    assign_if_present(r, "enabled", v.core.roi_enabled, "venc.roi");
    if (r.contains("steps")) {
      int steps = 0;
      assign_if_present(r, "steps", steps, "venc.roi");
      if (steps < 1 || steps > 4) fail("venc.roi.steps", "must be in [1,4]");
      v.core.roi_steps = static_cast<uint8_t>(steps);
    } else {
      note_default("venc.roi", "steps", to_text(static_cast<int>(kDef.core.roi_steps)));
    }
    if (r.contains("center")) {
      assign_if_present(r, "center", v.core.roi_center, "venc.roi");
      if (v.core.roi_center < 0.0 || v.core.roi_center > 1.0)
        fail("venc.roi.center", "must be in [0,1]");
    } else {
      note_default("venc.roi", "center", to_text(kDef.core.roi_center));
    }
  } else {
    // Whole sub-table absent: one line, same style as a missing top-level
    // section, rather than three separate per-key lines the operator would
    // have to mentally group back together.
    note_default("venc", "roi", "(section absent)");
  }

  // Range-checked BEFORE the uint16 cast: unchecked, ae_fps -1 wrapped to
  // 65535 and 0 sailed through as "run the ISP loop at no rate at all",
  // both of which reach the MI ISP as a legal-looking value and misbehave
  // on hardware rather than failing boot.
  if (j.contains("ae_fps")) {
    int v_ae = 0;
    assign_if_present(j, "ae_fps", v_ae, "venc");
    if (v_ae < 1 || v_ae > 60) fail("venc.ae_fps", "must be in [1,60]");
    v.core.ae_fps = static_cast<uint16_t>(v_ae);
  } else {
    note_default("venc", "ae_fps", to_text(kDef.core.ae_fps));
  }
  if (j.contains("awb_fps")) {
    int v_awb = 0;
    assign_if_present(j, "awb_fps", v_awb, "venc");
    if (v_awb < 1 || v_awb > 60) fail("venc.awb_fps", "must be in [1,60]");
    v.core.awb_fps = static_cast<uint16_t>(v_awb);
  } else {
    note_default("venc", "awb_fps", to_text(kDef.core.awb_fps));
  }

  if (j.contains("snapshot_quality")) {
    int q = 0;
    assign_if_present(j, "snapshot_quality", q, "venc");
    if (q < 1 || q > 100) fail("venc.snapshot_quality", "must be in [1,100]");
    v.core.snapshot_quality = static_cast<uint8_t>(q);
  } else {
    note_default("venc", "snapshot_quality", to_text(static_cast<int>(kDef.core.snapshot_quality)));
  }

  if (j.contains("debug_port")) {
    assign_if_present(j, "debug_port", v.debug_port, "venc");
    if (v.debug_port < 1024 || v.debug_port > 65535)
      fail("venc.debug_port", "must be in [1024,65535]");
  } else {
    note_default("venc", "debug_port", to_text(kDef.debug_port));
  }

  if (j.contains("module_loader")) {
    assign_if_present(j, "module_loader", v.module_loader, "venc");
  } else {
    note_default("venc", "module_loader", kDef.module_loader);
  }

  // A stripe cannot be wider than the picture. Checked here rather than
  // inside the key's own block for two reasons: it depends on venc.size,
  // which is parsed above, and the DEFAULT rows value has to face the same
  // bound — omitting the key must not buy a pass on a picture too short to
  // hold the default sweep. The preset path clamped instead and warned on
  // stderr, which ran a sweep the config did not describe; boot failure is
  // the forcing function. (The apply site keeps its own clamp for the
  // narrower case where the sensor delivers a shorter picture than
  // venc.size asked for — see star6e_pipeline_apply_intra_refresh.)
  {
    const int max_rows = venc_cfg_ctu_rows(v.core.height);
    if (v.core.intra_refresh_rows > max_rows)
      fail("venc.intra_refresh_rows",
           "must be 0 (off) or in [1," + std::to_string(max_rows) +
               "] CTU rows for this venc.size");
  }

  // sensor_bin is the ONE venc key with no default (venc_cfg_defaults()
  // seeds every other field, see venc_cfg.c): it names a device-specific
  // ISP calibration blob, and guessing one gets you a booted encoder
  // producing garbage colour rather than an honest boot failure. Checked
  // LAST so a config that is wrong in several ways still reports the more
  // specific key first.
  if (v.core.sensor_bin[0] == '\0') fail("venc.sensor_bin", "is required");
}

void parse_link(const Value& j, LinkCfg& l) {
  check_known_keys(j, {"vtx_id", "failsafe_ms", "rendezvous_ms", "tick_ms",
                       "rc_drain_ms"}, "link");
  assign_if_present(j, "vtx_id", l.vtx_id, "link");
  assign_if_present(j, "failsafe_ms", l.failsafe_ms, "link");
  assign_if_present(j, "rendezvous_ms", l.rendezvous_ms, "link");
  assign_if_present(j, "tick_ms", l.tick_ms, "link");
  assign_if_present(j, "rc_drain_ms", l.rc_drain_ms, "link");

  if (l.vtx_id == 0) fail("link.vtx_id", "must be non-zero");
  // tick_ms is the agent loop's housekeeping deadline (TickGate). Unbounded
  // it was merely a hot spin at 0; behind the gate a non-positive value casts
  // to a ~1.8e19 ms period and the gate fires once at startup and never
  // again — no failsafe transition, no rendezvous fallback, no congestion
  // guard, no watchdog, no telemetry, and nothing logged to say so (review
  // finding 2026-08-14). Same [1,1000] idiom as rc_drain_ms below.
  if (l.tick_ms < 1 || l.tick_ms > 1000)
    fail("link.tick_ms", "must be in [1,1000]");
  if (l.rc_drain_ms < 1 || l.rc_drain_ms > 1000)
    fail("link.rc_drain_ms", "must be in [1,1000]");
  // The drain period is the loop's WAKE interval and tick_ms the deadline
  // behind it, so a drain slower than the tick silently retimes every per-tick
  // job to rc_drain_ms (TickGate then fires on every wake). Equality is the
  // legacy single-cadence loop and stays legal.
  if (l.rc_drain_ms > l.tick_ms)
    fail("link.rc_drain_ms", "must be <= link.tick_ms");
}

void parse_msp(const Value& j, MspCfg& m) {
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

void parse_ampdu(const Value& j, AmpduCfg& a) {
  check_known_keys(j, {"max_num", "max_time"}, "ampdu");
  assign_if_present(j, "max_num", a.max_num, "ampdu");
  assign_if_present(j, "max_time", a.max_time, "ampdu");
  if (a.max_num < 0 || a.max_num > 31)
    fail("ampdu.max_num", "must be in [0,31] (5-bit MAX_AGG_NUM; 0 = off)");
  if (a.max_time < 0 || a.max_time > 255)
    fail("ampdu.max_time", "must be in [0,255] (raw 0x455 register value)");
  if (a.max_time >= 1 && a.max_time <= 8)
    fail("ampdu.max_time",
         "1..8 is the 0x455 register cliff (silently disables aggregation); "
         "use 0 for the chip default or >= 9");
}

void parse_air_clock(const Value& j, AirClockCfg& a) {
  check_known_keys(j, {"shed_ms", "efficiency", "body_us"}, "air_clock");
  assign_if_present(j, "shed_ms", a.shed_ms, "air_clock");
  assign_if_present(j, "efficiency", a.efficiency, "air_clock");
  assign_if_present(j, "body_us", a.body_us, "air_clock");
  if (a.shed_ms < 0 || a.shed_ms > 60000)
    fail("air_clock.shed_ms", "must be in [0,60000] (0 = observe only)");
  if (a.efficiency <= 0.0 || a.efficiency > 1.0)
    fail("air_clock.efficiency", "must be in (0,1]");
  if (a.body_us < 0) fail("air_clock.body_us", "must be >= 0");
}

}  // namespace

std::array<UepLayerCfg, 2> Config::uep_layers() const {
  std::array<UepLayerCfg, 2> layers;
  for (int sid = 0; sid < 2; ++sid) {
    layers[static_cast<size_t>(sid)].fec = SwConfig{
        fec.symbol_size[static_cast<size_t>(sid)], fec.window, fec.base_overhead};
    layers[static_cast<size_t>(sid)].blocks_per_body =
        fec.blocks_per_body[static_cast<size_t>(sid)];
  }
  return layers;
}

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

  static const char* kSections[] = {"radio", "fec", "encoder", "venc",
                                    "link", "msp", "ampdu", "air_clock"};
  check_known_keys(j, {"radio", "fec", "encoder", "venc", "link", "msp", "ampdu", "air_clock"}, "");

  // A whole missing section means none of its keys are visited below, so
  // report the section itself. Dropping a [table] while hand-transcribing is
  // exactly the mistake this line exists to catch.
  for (const char* sec : kSections)
    if (!j.contains(sec)) note_default("", sec, "(section absent)");

  Config cfg;
  if (j.contains("radio")) parse_radio(j.at("radio"), cfg.radio);
  if (j.contains("fec")) parse_fec(j.at("fec"), cfg.fec);
  if (j.contains("encoder")) parse_encoder(j.at("encoder"), cfg.encoder);
  if (j.contains("venc")) parse_venc(j.at("venc"), cfg.venc);
  if (j.contains("link")) parse_link(j.at("link"), cfg.link);
  if (j.contains("msp")) parse_msp(j.at("msp"), cfg.msp);
  if (j.contains("ampdu")) parse_ampdu(j.at("ampdu"), cfg.ampdu);
  if (j.contains("air_clock")) parse_air_clock(j.at("air_clock"), cfg.air_clock);

  return cfg;
}

}  // namespace mabur
