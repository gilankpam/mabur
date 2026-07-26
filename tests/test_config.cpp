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
  CHECK(cfg.radio.thermal_max_delta == def.radio.thermal_max_delta);
  CHECK(cfg.radio.power_mode == "none");
  CHECK(cfg.radio.power_offset_qdb == 0);

  // Default bundle ships power-inert ("none" = efuse/kernel per-rate table
  // untouched). "offset" is the adaptive opt-in at deploy time; "override"
  // is bench-diagnostic only. Bundle carries unit's measured wall-equalization
  // values (Task 9) alongside the inert power mode.
  CHECK((cfg.radio.rate_walls_idx ==
         std::array<int, 8>{91, 91, 91, 91, 73, 56, 51, 49}));
  CHECK(cfg.radio.legacy_wall_idx == 91);
  CHECK(cfg.radio.wall_margin_db == 1.0);
  CHECK(cfg.radio.min_offset_qdb == -40);
  CHECK(cfg.radio.base_ref_idx == 53);

  // The bundle intentionally diverges from struct defaults for fec, so check
  // against the bundle's actual values rather than the struct defaults used
  // for everything else. 328/w32/bpb4 is the 2026-07-25 gated geometry
  // (docs/fec-symbol-size-328.md): same ~1.4kB body and ~11kB window span as
  // scalar-164/w64/bpb8 but ~5% less airtime and -7.5% maburd CPU, quality
  // parity on-air.
  CHECK((cfg.fec.symbol_size == std::array<int, 4>{328, 328, 328, 328}));
  CHECK(cfg.fec.window == 32);
  CHECK((cfg.fec.blocks_per_body == std::array<int, 4>{4, 4, 4, 4}));
  CHECK(cfg.fec.base_overhead == def.fec.base_overhead);
  CHECK(cfg.fec.flush_ms == 25);

  CHECK(cfg.waybeam.host == def.waybeam.host);
  CHECK(cfg.waybeam.port == def.waybeam.port);
  CHECK(cfg.waybeam.idr_path == def.waybeam.idr_path);
  CHECK(cfg.waybeam.bitrate_min_kbps == def.waybeam.bitrate_min_kbps);
  CHECK(cfg.waybeam.bitrate_max_kbps == def.waybeam.bitrate_max_kbps);
  CHECK(cfg.waybeam.airtime_budget == def.waybeam.airtime_budget);
  CHECK(cfg.waybeam.roi_threshold_kbps == def.waybeam.roi_threshold_kbps);
  CHECK(cfg.waybeam.roi_qp_low == def.waybeam.roi_qp_low);
  CHECK(cfg.waybeam.roi_qp_normal == def.waybeam.roi_qp_normal);

  CHECK(cfg.link.vtx_id == def.link.vtx_id);
  CHECK(cfg.link.failsafe_ms == def.link.failsafe_ms);
  CHECK(cfg.link.rendezvous_ms == def.link.rendezvous_ms);
  CHECK(cfg.link.tick_ms == def.link.tick_ms);

  CHECK(cfg.frame_ring_name == def.frame_ring_name);

  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.overhead == 1.0);  // base_overhead 0.25 -> sid0 ref 1.00
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

TEST(load_config_waybeam_port_out_of_range_throws_naming_field) {
  auto path = write_temp_json(R"({"waybeam":{"port":70000}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam.port") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_waybeam_bitrate_min_not_less_than_max_throws_naming_field) {
  auto path = write_temp_json(R"({"waybeam":{"bitrate_min_kbps":5000,"bitrate_max_kbps":5000}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam.bitrate_min_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_waybeam_bitrate_min_below_floor_throws_naming_field) {
  auto path = write_temp_json(R"({"waybeam":{"bitrate_min_kbps":50}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam.bitrate_min_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_waybeam_airtime_budget_out_of_range_throws_naming_field) {
  auto path = write_temp_json(R"({"waybeam":{"airtime_budget":1.5}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam.airtime_budget") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(load_config_waybeam_roi_threshold_negative_throws_naming_field) {
  auto path = write_temp_json(R"({"waybeam":{"roi_threshold_kbps":-1}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("waybeam.roi_threshold_kbps") != std::string::npos);
  std::filesystem::remove(path);
}

TEST(uep_layers_overhead_ladder_at_base_0_25) {
  Config cfg;  // defaults: base_overhead = 0.25
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.overhead == 1.0);
  CHECK(layers[1].fec.overhead == 0.75);
  CHECK(layers[2].fec.overhead == 0.5);
  CHECK(layers[3].fec.overhead == 0.25);
  CHECK(layers[0].fec.window == cfg.fec.window);
  CHECK(layers[0].fec.symbol_size == cfg.fec.symbol_size[0]);
  CHECK(layers[0].blocks_per_body == cfg.fec.blocks_per_body[0]);
  CHECK(layers[3].blocks_per_body == cfg.fec.blocks_per_body[3]);
}

TEST(fec_symbol_size_scalar_fans_out) {
  // 164 (not an arbitrary probe value): with the default blocks_per_body
  // {4,8,16,16}, this is the largest symbol_size that keeps every layer's
  // body (bpb*(hdr+symbol_size)) within kMaxBodyBytes (16*(14+164)=2848 <
  // 2900) — a bigger scalar here would trip the oversize-body guard before
  // fan-out could even be observed.
  auto path = write_temp_json(R"({"fec":{"symbol_size":164}})");
  Config cfg = load_config(path.string());
  for (int s = 0; s < 4; ++s) CHECK(cfg.fec.symbol_size[s] == 164);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_array_per_layer) {
  auto path = write_temp_json(
      R"({"fec":{"symbol_size":[164,1312,1312,1312],"blocks_per_body":[4,1,1,1]}})");
  Config cfg = load_config(path.string());
  CHECK(cfg.fec.symbol_size[0] == 164);
  CHECK(cfg.fec.symbol_size[3] == 1312);
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.symbol_size == 164);
  CHECK(layers[3].fec.symbol_size == 1312);
  std::filesystem::remove(path);
}

TEST(fec_symbol_size_rejects_wrong_len_array) {
  auto path = write_temp_json(R"({"fec":{"symbol_size":[164,1312]}})");
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
      R"({"fec":{"symbol_size":1312,"blocks_per_body":[8,8,8,8]}})");
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
      R"("min_offset_qdb":-32,"base_ref_idx":50}})");
  Config cfg = load_config(path.string());
  CHECK((cfg.radio.rate_walls_idx ==
         std::array<int, 8>{91, 91, 91, 91, 73, 56, 51, 49}));
  CHECK(cfg.radio.legacy_wall_idx == 91);
  CHECK(cfg.radio.wall_margin_db == 2.0);
  CHECK(cfg.radio.min_offset_qdb == -32);
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
      R"("min_offset_qdb":-32,"base_ref_idx":0}})");
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

// Video ingest is frame-shm only: the frame ring is named by frame_ring_name
// and there is no mode to select.
TEST(frame_ring_name_default) {
  auto path = write_temp_json("{}");
  auto cfg = load_config(path.string());
  CHECK(cfg.frame_ring_name == "mabur_f");
  std::filesystem::remove(path);

  auto path2 = write_temp_json(R"({"frame_ring_name": "other"})");
  CHECK(load_config(path2.string()).frame_ring_name == "other");
  std::filesystem::remove(path2);
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

MTEST_MAIN
