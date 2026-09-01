#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

#include "mtest.h"
#include "config.h"
using namespace mabur;

namespace {

// Path to the committed default bundle config, resolved relative to this
// source file's known repo layout (tests/ -> ../bundle/mabur.default.json).
std::string default_config_path() {
  return std::string(MABUR_BUNDLE_DIR) + "/mabur.default.json";
}

// Writes `contents` to a fresh temp file and returns its path. Caller is
// responsible for cleanup (tests remove it at the end).
std::filesystem::path write_temp_json(const std::string& contents) {
  static std::atomic<int> counter{0};
  auto path = std::filesystem::temp_directory_path() /
              ("mabur_test_config_" + std::to_string(counter++) + ".json");
  std::ofstream f(path);
  f << contents;
  f.close();
  return path;
}

std::string what_of(const std::function<void()>& fn) {
  try {
    fn();
  } catch (const std::exception& e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST(load_config_default_file_matches_struct_defaults) {
  Config cfg = load_config(default_config_path());
  Config def;  // struct defaults

  CHECK(cfg.radio.usb_vid == def.radio.usb_vid);
  CHECK(cfg.radio.usb_pid == def.radio.usb_pid);
  CHECK(cfg.radio.channel == def.radio.channel);
  CHECK(cfg.radio.width == def.radio.width);
  CHECK(cfg.radio.power_mode == "none");

  // Default bundle ships power-inert ("none" = efuse/kernel per-rate table
  // untouched). "offset" is the adaptive opt-in at deploy time. Bundle
  // carries unit's measured wall-equalization values (Task 9) alongside
  // the inert power mode.
  CHECK((cfg.radio.rate_walls_idx ==
         std::array<int, 8>{91, 91, 91, 91, 73, 56, 51, 49}));
  CHECK(cfg.radio.legacy_wall_idx == 91);
  CHECK(cfg.radio.wall_margin_db == 1.0);
  CHECK(cfg.radio.base_ref_idx == 53);

  // The bundle intentionally diverges from struct defaults for fec, so check
  // against the bundle's actual values rather than the struct defaults used
  // for everything else. 332/w32/bpb4 is the 2026-07-29 geometry: same CPU/
  // air profile as the 2026-07-25 gated 328 (docs/fec-symbol-size-328.md),
  // shifted +4 because 328x4 = 1396 B air frames sit exactly in the
  // mcs6+STBC PHY hole (docs/mcs6-bench-anomaly.md — air MPDUs 1392-1400 B
  // vanish whole at RX). Any new size needs the all-8-MCS hole-scan.
  CHECK((cfg.fec.symbol_size == std::array<int, 2>{332, 332}));
  CHECK(cfg.fec.window == 32);
  CHECK((cfg.fec.blocks_per_body == std::array<int, 2>{4, 4}));
  CHECK(cfg.fec.base_overhead == def.fec.base_overhead);
  CHECK(cfg.fec.flush_ms == 25);

  // waybeam is retired (Task B5). Since the 2026-08-29 flag day the bundle
  // carries the config that was actually validated on hardware and deployed
  // (1000/10000/0.70, roi_qp_low -24) rather than the pre-fold-in waybeam-era
  // tuning it was seeded with — same "bundle diverges from struct defaults on
  // purpose" pattern as fec above (EncoderCfg compiles 2000/10000/0.60/+8).
  // NOTE the sign: apply_roi_qp() takes a QP OFFSET for the centre region, so
  // the useful low-bitrate value is NEGATIVE (better centre quality). The
  // struct default +8 has the opposite sign and is left alone deliberately —
  // nothing deployed relies on it, and changing a compiled default is not a
  // flag-day concern.
  CHECK(cfg.encoder.bitrate_min_kbps == 1000);
  CHECK(cfg.encoder.bitrate_max_kbps == 10000);
  CHECK(cfg.encoder.airtime_budget == 0.70);
  CHECK(cfg.encoder.roi_threshold_kbps == 3000);
  CHECK(cfg.encoder.roi_qp_low == -24);
  CHECK(cfg.encoder.roi_qp_normal == 0);

  // venc: boot-time encoder pipeline config (Task B5), also bundle-pinned
  // rather than struct-default (struct defaults are all-zero/empty, not a
  // bootable encoder configuration).
  CHECK(cfg.venc.core.sensor_bin ==
        std::string("/etc/sensors/imx415_greg_fpvXIX_colortrans.bin"));
  CHECK(cfg.venc.core.width == 1920);
  CHECK(cfg.venc.core.height == 1080);
  CHECK(cfg.venc.core.fps == 60);
  CHECK(cfg.venc.core.gop_s == 2.0);
  CHECK(cfg.venc.core.qp_delta == -4);
  CHECK(cfg.venc.core.max_ipprop == 0);
  CHECK(std::string(cfg.venc.core.resilience) == "rally");
  CHECK(cfg.venc.core.roi_enabled == true);
  CHECK(cfg.venc.core.roi_steps == 2);
  CHECK(cfg.venc.core.roi_center == 0.4);
  CHECK(cfg.venc.core.ae_fps == 15);
  CHECK(cfg.venc.core.awb_fps == 15);
  CHECK(cfg.venc.core.snapshot_quality == 80);
  CHECK(cfg.venc.debug_port == 8301);

  CHECK(cfg.link.vtx_id == def.link.vtx_id);
  CHECK(cfg.link.failsafe_ms == def.link.failsafe_ms);
  CHECK(cfg.link.rendezvous_ms == def.link.rendezvous_ms);
  CHECK(cfg.link.tick_ms == def.link.tick_ms);

  auto layers = cfg.uep_layers();
  // Literal passthrough (Task 3): no uep_layer_overhead ladder translation
  // left — every layer's overhead is exactly fec.base_overhead.
  CHECK(layers[0].fec.overhead == cfg.fec.base_overhead);
  CHECK(layers[1].fec.overhead == cfg.fec.base_overhead);
}

TEST(load_config_missing_file_throws) {
  bool threw = false;
  std::string msg;
  try {
    load_config("/nonexistent/path/does/not/exist/mabur.json");
  } catch (const std::runtime_error& e) {
    threw = true;
    msg = e.what();
  }
  CHECK(threw);
  CHECK(msg.find("config:") == 0);
}

TEST(load_config_out_of_range_field_throws_naming_field) {
  auto path = write_temp_json(R"({"fec":{"window":9999}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.window") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_unknown_top_level_key_throws_naming_it) {
  auto path = write_temp_json(R"({"typo":1})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("typo") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_unknown_nested_key_throws_naming_it) {
  auto path = write_temp_json(R"({"fec":{"kx":8}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.kx") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_type_mismatch_fec_window_string_throws_runtime_error_with_dotted_path) {
  auto path = write_temp_json(R"({"fec":{"window":"wide"}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("fec.window") != std::string::npos);
  CHECK(msg.find("wrong type") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_unknown_radio_bw_set_key_throws) {
  // radio.bw_set (bandwidth-probe schedule) was removed 2026-07-27 (SDD
  // ladder-controller Task 5): the ladder controller never varies bw
  // independently of the commanded rung, so the probe schedule and its
  // config key are dead. Strict-keys config load (PR #7) must reject it
  // like any other unknown key rather than silently ignoring it.
  auto path = write_temp_json(R"({"radio":{"bw_set":[20,40]}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.bw_set") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// waybeam is retired (spec 2026-08-28 venc-foldin, Task B5): the section
// and its host/port/idr_path keys are gone entirely, strict keys reject any
// config that still carries it. See waybeam_section_is_now_unknown below.
// The surviving bitrate/ROI policy fields moved to Config::encoder.

TEST(load_config_encoder_bitrate_min_not_less_than_max_throws_naming_field) {
  auto path = write_temp_json(R"({"encoder":{"bitrate_min_kbps":5000,"bitrate_max_kbps":5000}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.bitrate_min_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_encoder_bitrate_min_below_floor_throws_naming_field) {
  auto path = write_temp_json(R"({"encoder":{"bitrate_min_kbps":50}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.bitrate_min_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_encoder_airtime_budget_out_of_range_throws_naming_field) {
  auto path = write_temp_json(R"({"encoder":{"airtime_budget":1.5}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.airtime_budget") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_encoder_roi_threshold_negative_throws_naming_field) {
  auto path = write_temp_json(R"({"encoder":{"roi_threshold_kbps":-1}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("encoder.roi_threshold_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

// ---- Task B5: venc section / waybeam retirement -------------------------

// A full, valid venc block matching the brief's fixture (spec 2026-08-28
// venc-foldin Task B5 Step 1), reused across the tests below.
std::string valid_venc_block() {
  return R"("venc":{"sensor_bin":"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin",)"
         R"("size":"1920x1080","fps":60,"gop_s":2.0,"qp_delta":-4,)"
         R"("resilience":"rally",)"
         R"("roi":{"enabled":true,"steps":2,"center":0.4},)"
         R"("ae_fps":15,"awb_fps":15,"snapshot_quality":80,)"
         R"("debug_port":8301})";
}

TEST(venc_section_parses_and_validates) {
  auto path = write_temp_json(
      "{" + valid_venc_block() +
      R"(,"encoder":{"bitrate_min_kbps":1000,"bitrate_max_kbps":20000,)"
      R"("airtime_budget":0.65,"roi_threshold_kbps":3000,)"
      R"("roi_qp_low":8,"roi_qp_normal":0}})");
  Config c = load_config(path.string());
  CHECK(c.venc.core.width == 1920);
  CHECK(c.venc.core.height == 1080);
  CHECK(std::string(c.venc.core.resilience) == "rally");
  CHECK(c.encoder.airtime_budget == 0.65);
  std::filesystem::remove(path);
}

TEST(waybeam_section_is_now_unknown) {
  auto path = write_temp_json(R"({"waybeam":{"host":"127.0.0.1"}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_rejects_bitrate_key) {
  // spec 2026-08-28 venc-foldin §3: no venc.bitrate key ever exists. It
  // simply isn't in venc's known-key set, so this hits the same unknown-key
  // path as any other stale key.
  auto path = write_temp_json(
      R"({"venc":{"sensor_bin":"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin",)"
      R"("size":"1920x1080","fps":60,"gop_s":2.0,"qp_delta":-4,)"
      R"("resilience":"rally","bitrate":8000,)"
      R"("roi":{"enabled":true,"steps":2,"center":0.4},)"
      R"("ae_fps":15,"awb_fps":15,"snapshot_quality":80,)"
      R"("debug_port":8301}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.bitrate") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_rejects_unknown_resilience) {
  auto path = write_temp_json(
      R"({"venc":{"sensor_bin":"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin",)"
      R"("size":"1920x1080","fps":60,"gop_s":2.0,"qp_delta":-4,)"
      R"("resilience":"yolo",)"
      R"("roi":{"enabled":true,"steps":2,"center":0.4},)"
      R"("ae_fps":15,"awb_fps":15,"snapshot_quality":80,)"
      R"("debug_port":8301}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.resilience") != std::string::npos);
  std::filesystem::remove(path);
}

// Absent venc keys fall back to the spec §3 values (venc_cfg_defaults(),
// drone/venc/venc_cfg.c), NOT to the all-zero a plain `VencCfg core{}`
// would give: a zeroed VencCfg is fps 0 / 0x0 / gop 0.0 / resilience "",
// which is a malformed pipeline dressed up as a default.
// REVERT CHECK: delete the VencSectionCfg() constructor in config.h (or the
// body of venc_cfg_defaults) and every CHECK below reads 0/"".
TEST(venc_absent_keys_fall_back_to_spec_defaults) {
  // Only the one REQUIRED key present; everything else omitted.
  auto path = write_temp_json(
      R"({"venc":{"sensor_bin":"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin"}})");
  Config c = load_config(path.string());
  CHECK(c.venc.core.fps == 60);
  CHECK(c.venc.core.width == 1920);
  CHECK(c.venc.core.height == 1080);
  CHECK(c.venc.core.gop_s == 2.0);
  CHECK(c.venc.core.qp_delta == -4);
  CHECK(c.venc.core.max_ipprop == 0);
  CHECK(std::string(c.venc.core.resilience) == "rally");
  CHECK(c.venc.core.roi_enabled == true);
  CHECK(c.venc.core.roi_steps == 2);
  CHECK(c.venc.core.roi_center == 0.4);
  CHECK(c.venc.core.ae_fps == 15);
  CHECK(c.venc.core.awb_fps == 15);
  CHECK(c.venc.core.snapshot_quality == 80);
  CHECK(c.venc.debug_port == 8301);
  // The default resilience must survive the preset authority, or the
  // fallback would boot-fail on a config that named nothing wrong.
  CHECK(venc_cfg_preset_known(c.venc.core.resilience) != 0);
  std::filesystem::remove(path);
}

// max_ipprop is optional: absent -> 0 (spec default, tested above), a
// legal in-range value lands verbatim in VencCfg.
TEST(venc_max_ipprop_parses) {
  auto path = write_temp_json(
      R"({"venc":{"sensor_bin":"/etc/sensors/imx415_greg_fpvXIX_colortrans.bin",)"
      R"("max_ipprop":2}})");
  Config c = load_config(path.string());
  CHECK(c.venc.core.max_ipprop == 2);
  std::filesystem::remove(path);
}

// sensor_bin is the ONE venc key with no default: it names a device-specific
// ISP calibration blob, and there is no value that is right for an unknown
// camera. Absent => boot failure, per the project's config-strict policy.
// REVERT CHECK: remove the sensor_bin[0] check at the end of parse_venc and
// this load succeeds with an empty sensor_bin.
TEST(venc_sensor_bin_is_required) {
  auto path = write_temp_json(R"({"venc":{"fps":60}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.sensor_bin") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_size_malformed_throws) {
  auto path = write_temp_json(R"({"venc":{"size":"1920"}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("venc.size") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(venc_range_checks) {
  // Each case names the field its own out-of-range value should be
  // reported against (review finding 2026-08-29: a loose "venc." find()
  // let 10/14 cases silently pass on a DIFFERENT field's error message
  // — the venc.gop_s-validated-unconditionally bug masked here because
  // every case happened to also fail gop_s's range check first).
  struct Case { const char* json; const char* field; };
  for (const Case& c : {
           Case{R"({"venc":{"fps":0}})", "venc.fps"},
           Case{R"({"venc":{"fps":121}})", "venc.fps"},
           Case{R"({"venc":{"gop_s":0.1}})", "venc.gop_s"},
           Case{R"({"venc":{"gop_s":11}})", "venc.gop_s"},
           Case{R"({"venc":{"qp_delta":-13}})", "venc.qp_delta"},
           Case{R"({"venc":{"qp_delta":13}})", "venc.qp_delta"},
           Case{R"({"venc":{"max_ipprop":-1}})", "venc.max_ipprop"},
           Case{R"({"venc":{"max_ipprop":101}})", "venc.max_ipprop"},
           Case{R"({"venc":{"snapshot_quality":0}})", "venc.snapshot_quality"},
           Case{R"({"venc":{"snapshot_quality":101}})", "venc.snapshot_quality"},
           Case{R"({"venc":{"debug_port":1023}})", "venc.debug_port"},
           Case{R"({"venc":{"debug_port":65536}})", "venc.debug_port"},
           Case{R"({"venc":{"roi":{"steps":0}}})", "venc.roi.steps"},
           Case{R"({"venc":{"roi":{"steps":5}}})", "venc.roi.steps"},
           Case{R"({"venc":{"roi":{"center":-0.1}}})", "venc.roi.center"},
           Case{R"({"venc":{"roi":{"center":1.1}}})", "venc.roi.center"},
           // ae_fps/awb_fps are range-checked BEFORE the uint16 cast: -1
           // used to wrap to 65535 and 0 used to sail through as "run the
           // ISP loop at no rate at all", both of which reach the MI ISP
           // looking legal and misbehave on hardware instead of failing
           // boot.
           // REVERT CHECK: drop the `< 1` half of either range check and the
           // -1 and 0 cases stop throwing (the load succeeds).
           Case{R"({"venc":{"ae_fps":-1}})", "venc.ae_fps"},
           Case{R"({"venc":{"ae_fps":0}})", "venc.ae_fps"},
           Case{R"({"venc":{"ae_fps":61}})", "venc.ae_fps"},
           Case{R"({"venc":{"awb_fps":-1}})", "venc.awb_fps"},
           Case{R"({"venc":{"awb_fps":0}})", "venc.awb_fps"},
           Case{R"({"venc":{"awb_fps":61}})", "venc.awb_fps"},
       }) {
    auto path = write_temp_json(c.json);
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find(c.field) != std::string::npos);
    std::filesystem::remove(path);
  }
}

TEST(uep_layers_overhead_is_literal_base_overhead) {
  Config cfg;  // defaults: base_overhead = 0.5, literal (Task 3)
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.overhead == cfg.fec.base_overhead);
  CHECK(layers[1].fec.overhead == cfg.fec.base_overhead);
  CHECK(layers[0].fec.window == cfg.fec.window);
  CHECK(layers[0].fec.symbol_size == cfg.fec.symbol_size[0]);
  CHECK(layers[0].blocks_per_body == cfg.fec.blocks_per_body[0]);
  CHECK(layers[1].blocks_per_body == cfg.fec.blocks_per_body[1]);
}

TEST(fec_symbol_size_scalar_fans_out) {
  // 164 keeps every layer's body (bpb*(hdr+symbol_size)) within
  // kMaxBodyBytes at the default blocks_per_body {4,8}: 8*(14+164)=1424 <
  // 2900.
  auto path = write_temp_json(R"({"fec":{"symbol_size":164}})");
  Config cfg = load_config(path.string());
  for (int s = 0; s < 2; ++s) CHECK(cfg.fec.symbol_size[s] == 164);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_array_per_layer) {
  auto path = write_temp_json(
      R"({"fec":{"symbol_size":[164,1312],"blocks_per_body":[4,1]}})");
  Config cfg = load_config(path.string());
  CHECK(cfg.fec.symbol_size[0] == 164);
  CHECK(cfg.fec.symbol_size[1] == 1312);
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.symbol_size == 164);
  CHECK(layers[1].fec.symbol_size == 1312);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_rejects_wrong_len_array) {
  auto path = write_temp_json(R"({"fec":{"symbol_size":[164,1312,164]}})");
  bool threw = false;
  try {
    (void)load_config(path.string());
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_rejects_oversize_body) {
  // 1312B symbols at bpb 8 -> 8*(14+1312) = 10608 > kMaxBodyBytes 2900
  auto path = write_temp_json(
      R"({"fec":{"symbol_size":1312,"blocks_per_body":[8,8]}})");
  bool threw = false;
  try {
    (void)load_config(path.string());
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_bounds) {
  {
    auto path = write_temp_json(R"({"fec":{"symbol_size":16}})");  // <32
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_json(R"({"fec":{"symbol_size":1600}})");  // >1500
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw);
    std::filesystem::remove(path);
  }
}

TEST(msp_defaults_and_parse) {
  // Defaults: disabled, ttyS2, 1 Hz.
  {
    auto path = write_temp_json("{}");
    auto cfg = load_config(path.string());
    CHECK(cfg.msp.enable == false);
    CHECK(cfg.msp.serial == "/dev/ttyS2");
    CHECK(cfg.msp.baud == 115200);
    CHECK(cfg.msp.update_rate_hz == 1.0);
    CHECK(cfg.msp.symbol_size == 1312);
    std::filesystem::remove(path);
  }
  // Explicit values.
  {
    auto path = write_temp_json(
        R"({"msp":{"enable":true,"serial":"/dev/ttyS1","baud":230400,)"
        R"("update_rate_hz":2.0,"symbol_size":1024,"window":32,"overhead":0.5}})");
    auto cfg = load_config(path.string());
    CHECK(cfg.msp.enable == true);
    CHECK(cfg.msp.serial == "/dev/ttyS1");
    CHECK(cfg.msp.baud == 230400);
    CHECK(cfg.msp.update_rate_hz == 2.0);
    CHECK(cfg.msp.symbol_size == 1024);
    CHECK(cfg.msp.window == 32);
    std::filesystem::remove(path);
  }
}

TEST(msp_rejects_bad_values) {
  {
    auto path = write_temp_json(R"({"msp":{"update_rate_hz":0}})");
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::exception&) {
      threw = true;
    }
    CHECK(threw == true);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_json(R"({"msp":{"nonsense":1}})");
    bool threw = false;
    try {
      (void)load_config(path.string());
    } catch (const std::exception&) {
      threw = true;
    }
    CHECK(threw == true);
    std::filesystem::remove(path);
  }
}

TEST(radio_wall_equalization_keys_parse) {
  auto path = write_temp_json(
      R"({"radio":{"power_mode":"offset",)"
      R"("rate_walls_idx":[91,91,91,91,73,56,51,49],)"
      R"("legacy_wall_idx":91,"wall_margin_db":2.0,)"
      R"("base_ref_idx":50}})");
  Config cfg = load_config(path.string());
  CHECK((cfg.radio.rate_walls_idx ==
         std::array<int, 8>{91, 91, 91, 91, 73, 56, 51, 49}));
  CHECK(cfg.radio.legacy_wall_idx == 91);
  CHECK(cfg.radio.wall_margin_db == 2.0);
  CHECK(cfg.radio.base_ref_idx == 50);
  std::filesystem::remove(path);
}

TEST(radio_rate_walls_idx_wrong_length_rejected) {
  auto path = write_temp_json(
      R"({"radio":{"rate_walls_idx":[91,91,91]}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.rate_walls_idx") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(radio_power_mode_offset_requires_rate_walls_idx) {
  auto path = write_temp_json(R"({"radio":{"power_mode":"offset"}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.rate_walls_idx") != std::string::npos);
  std::filesystem::remove(path);
}

// The 8822E's per-rate diff field is 7-bit two's complement: diff[r] =
// walls[r] - wall_margin_db*4 - base_ref_idx must land in [-64,63], or the
// value silently wraps on air (e.g. +70 -> -58, sign-flipping per-rate
// power) with no error. base_ref_idx left at 0 (a plausible miscalibration:
// forgetting to set the unit's efuse anchor) drives every wall straight out
// of range, so config load must fail loudly rather than let power_plan.h's
// clamp paper over it silently.
TEST(radio_offset_diff_out_of_range_rejected) {
  auto path = write_temp_json(
      R"({"radio":{"power_mode":"offset",)"
      R"("rate_walls_idx":[127,127,127,127,127,127,127,127],)"
      R"("legacy_wall_idx":91,"wall_margin_db":0.0,)"
      R"("base_ref_idx":0}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.rate_walls_idx") != std::string::npos);
  CHECK(msg.find("[-64,63]") != std::string::npos);
  std::filesystem::remove(path);
}

// The transitional async gate was removed after hardware acceptance (plan
// 2026-07-17 Task 7): async is the only mode. A stale config still carrying
// the key must fail loudly, not be silently ignored.
TEST(fec_stale_async_worker_key_throws) {
  auto p = write_temp_json(R"({"fec":{"async_worker":true}})");
  std::string w = what_of([&] { load_config(p.string()); });
  CHECK(w.find("async_worker") != std::string::npos);
  std::filesystem::remove(p);
}

// frame_ring_name was deleted (spec 2026-08-28 venc-foldin, controller
// ruling on Task B5): the ring name's single authority is now the
// compile-time VENC_RING_NAME in drone/venc/venc_cfg.h. A config that still
// carries the key hits the ordinary unknown-key path — see
// stale_video_input_and_ring_name_keys_throw below for the sibling
// pre-frame-shm keys that already went through this.
TEST(stale_frame_ring_name_key_throws) {
  auto path = write_temp_json(R"({"frame_ring_name": "mabur_f"})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("frame_ring_name") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// video_input/ring_name selected and named the pre-frame-shm RTP-packet ring.
// Their accept-and-warn grace release has passed and the drone's live
// /etc/mabur.json no longer carries them, so they now hit the blanket
// unknown-key check like any other stale key.
TEST(stale_video_input_and_ring_name_keys_throw) {
  auto path = write_temp_json(R"({"video_input": "frame_ring"})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("video_input") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);

  auto path2 = write_temp_json(R"({"ring_name": "mabur"})");
  std::string msg2 = what_of([&] { (void)load_config(path2.string()); });
  CHECK(msg2.find("ring_name") != std::string::npos);
  CHECK(msg2.find("unknown key") != std::string::npos);
  std::filesystem::remove(path2);
}

// The flags block tuned per-rung LDPC/STBC policy. Removed 2026-07-26:
// LDPC+STBC are now hardcoded true on every rung in both ladder builders
// (the deployed all-true config was the only shape ever flown; flags-off
// T1/T2 measured 2-3 dB weaker on air). A stale block fails the boot.
TEST(stale_flags_key_throws) {
  auto path = write_temp_json(R"({"flags":{"crit_ldpc":true}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("flags") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// radio.max_txagc was the legacy TXAGC-index ceiling; Task 11 moved the
// power path to qdB offsets and nothing has read it since. The live config
// was scrubbed 2026-07-26, so a stale key fails the boot loudly (note: the
// pre-sym328 rollback config still carries it — edit before rolling back).
TEST(stale_radio_max_txagc_key_throws) {
  auto path = write_temp_json(R"({"radio":{"max_txagc":40}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("radio.max_txagc") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

// power_offset_db fed the ladder's carried-but-never-emitted per-rung field
// (radio_tx.h: no DBM_TX_POWER radiotap is ever written). Scrubbed from the
// live config 2026-07-26; stale key fails the boot.
TEST(stale_power_offset_db_key_throws) {
  auto path = write_temp_json(R"({"power_offset_db":[0,0,0,0]})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(msg.find("power_offset_db") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(config_rejects_removed_power_keys) {
  // Each removed key must fail boot loudly. Reverting the deletion from
  // check_known_keys() in drone/src/config.cpp makes these keys parse again
  // and this test fails.
  for (const char* key : {"thermal_max_delta", "min_offset_qdb",
                          "power_offset_qdb"}) {
    std::string js = std::string("{\"radio\":{\"") + key + "\":1}}";
    auto path = write_temp_json(js);
    bool threw = false;
    try {
      load_config(path.string());
    } catch (const std::runtime_error& e) {
      threw = true;
      CHECK(std::string(e.what()).find("unknown key") != std::string::npos);
    }
    CHECK(threw);
    std::filesystem::remove(path);
  }
}

TEST(config_rejects_power_mode_override) {
  // Reverting the removal of "override" from the accepted set in
  // parse_radio() makes this load successfully and the test fails.
  auto path = write_temp_json(R"({"radio":{"power_mode":"override"}})");
  bool threw = false;
  try {
    load_config(path.string());
  } catch (const std::runtime_error& e) {
    threw = true;
    CHECK(std::string(e.what()).find("power_mode") != std::string::npos);
  }
  CHECK(threw);
  std::filesystem::remove(path);
}

TEST(link_rc_drain_ms_default_and_bounds) {
  // Absent: the agent loop wakes every 5 ms to drain RCFs (spec 2026-08-14
  // fade-demote §3b). This is an optional key on a strict-keys config, so a
  // deployed drone with no `link.rc_drain_ms` must still boot.
  {
    auto path = write_temp_json("{}");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.rc_drain_ms == 5);
    std::filesystem::remove(path);
  }
  // Explicit value inside the range is taken verbatim (50 <= the default
  // 100 ms tick_ms, so the cross-check below is satisfied).
  {
    auto path = write_temp_json(R"({"link":{"rc_drain_ms":50}})");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.rc_drain_ms == 50);
    std::filesystem::remove(path);
  }
  // Out of range fails boot, naming the field. 0 would spin the agent
  // thread; > 1000 would make actuation slower than the legacy loop.
  for (int bad : {0, 1001}) {
    auto path = write_temp_json(std::string(R"({"link":{"rc_drain_ms":)") +
                                std::to_string(bad) + "}}");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("link.rc_drain_ms") != std::string::npos);
    std::filesystem::remove(path);
  }
}

// Review finding 2026-08-14 (final whole-branch review, finding 4): tick_ms
// was unvalidated, and the TickGate the agent loop now runs its housekeeping
// behind turns a bad value from "spins hot" into "silently loses the
// failsafe" — TickGate(now, -1) casts to a ~1.8e19 ms period, so the gate
// fires once at startup and never again: no failsafe transition, no
// rendezvous fallback, no watchdog, no telemetry, and nothing in the log to
// say it stopped.
TEST(link_tick_ms_bounds) {
  // Absent: the historical 100 ms housekeeping cadence.
  {
    auto path = write_temp_json("{}");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.tick_ms == 100);
    std::filesystem::remove(path);
  }
  // In range, taken verbatim.
  {
    auto path = write_temp_json(R"({"link":{"tick_ms":20}})");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.tick_ms == 20);
    std::filesystem::remove(path);
  }
  // Out of range fails boot, naming the field.
  for (int bad : {-1, 0, 1001}) {
    auto path = write_temp_json(std::string(R"({"link":{"tick_ms":)") +
                                std::to_string(bad) + R"(,"rc_drain_ms":1}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("link.tick_ms") != std::string::npos);
    std::filesystem::remove(path);
  }
}

TEST(link_rc_drain_ms_must_not_exceed_tick_ms) {
  // rc_drain_ms is the loop's WAKE period and tick_ms the housekeeping
  // deadline behind it; a drain slower than the tick silently retimes every
  // per-tick job to rc_drain_ms instead (TickGate degenerates to firing on
  // every wake). Equality is legal — that is exactly the legacy loop.
  {
    auto path = write_temp_json(R"({"link":{"tick_ms":50,"rc_drain_ms":50}})");
    auto cfg = load_config(path.string());
    CHECK(cfg.link.rc_drain_ms == 50);
    CHECK(cfg.link.tick_ms == 50);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_json(R"({"link":{"tick_ms":50,"rc_drain_ms":51}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("link.rc_drain_ms") != std::string::npos);
    std::filesystem::remove(path);
  }
}

// ---- Task 3: ampdu block (spec 2026-09-01-ampdu-design.md) --------------

TEST(ampdu_defaults_when_absent) {
  // A config with no "ampdu" block gets the shipped defaults — aggregation
  // OFF since the 2026-09-01 bench verdict (no fec win, RF-report damage).
  auto path = write_temp_json("{}");
  auto cfg = load_config(path.string());
  CHECK(cfg.ampdu.max_num == 0);
  CHECK(cfg.ampdu.max_time == 32);
  std::filesystem::remove(path);
}

TEST(ampdu_block_parses) {
  auto path = write_temp_json(
      R"({"ampdu": {"max_num": 4, "max_time": 48}})");
  auto cfg = load_config(path.string());
  CHECK(cfg.ampdu.max_num == 4);
  CHECK(cfg.ampdu.max_time == 48);
  std::filesystem::remove(path);
}

TEST(ampdu_zero_disables) {
  auto path = write_temp_json(R"({"ampdu": {"max_num": 0}})");
  auto cfg = load_config(path.string());
  CHECK(cfg.ampdu.max_num == 0);
  std::filesystem::remove(path);
}

TEST(ampdu_rejects_bad_values) {
  // max_num out of the 5-bit MAX_AGG_NUM field.
  {
    auto path = write_temp_json(R"({"ampdu": {"max_num": 32}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_num") != std::string::npos);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_json(R"({"ampdu": {"max_num": -1}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_num") != std::string::npos);
    std::filesystem::remove(path);
  }
  // max_time 1..8 is the register cliff (aggregation silently disabled).
  {
    auto path = write_temp_json(R"({"ampdu": {"max_time": 8}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_time") != std::string::npos);
    std::filesystem::remove(path);
  }
  {
    auto path = write_temp_json(R"({"ampdu": {"max_time": 256}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.max_time") != std::string::npos);
    std::filesystem::remove(path);
  }
  // Unknown key inside the block fails boot (config-strict).
  {
    auto path = write_temp_json(R"({"ampdu": {"depth": 4}})");
    std::string msg = what_of([&] { (void)load_config(path.string()); });
    CHECK(!msg.empty());
    CHECK(msg.find("ampdu.depth") != std::string::npos);
    CHECK(msg.find("unknown key") != std::string::npos);
    std::filesystem::remove(path);
  }
}

MTEST_MAIN
