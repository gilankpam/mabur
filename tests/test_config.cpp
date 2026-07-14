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
  CHECK(cfg.radio.bw_set == def.radio.bw_set);
  CHECK(cfg.radio.max_txagc == def.radio.max_txagc);
  CHECK(cfg.radio.thermal_max_delta == def.radio.thermal_max_delta);
  CHECK(cfg.radio.power_mode == "override");
  CHECK(cfg.radio.power_offset_qdb == 0);

  CHECK(cfg.fec.symbol_size == def.fec.symbol_size);
  CHECK(cfg.fec.window == def.fec.window);
  CHECK(cfg.fec.blocks_per_body == def.fec.blocks_per_body);
  CHECK(cfg.fec.base_overhead == def.fec.base_overhead);
  CHECK(cfg.fec.flush_ms == def.fec.flush_ms);
  CHECK(cfg.fec.window == 128);

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

  CHECK(cfg.ring_name == def.ring_name);
  CHECK(cfg.flags.crit_ldpc == def.flags.crit_ldpc);
  CHECK(cfg.flags.crit_stbc == def.flags.crit_stbc);
  CHECK(cfg.flags.t0_ldpc == def.flags.t0_ldpc);
  CHECK(cfg.flags.t0_stbc == def.flags.t0_stbc);
  CHECK(cfg.power_offset_db == def.power_offset_db);

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

TEST(load_config_bw_set_rungs_above_width_are_dropped) {
  // B7 (docs/bench-validation.md): a 20 MHz-tuned baseband cannot emit a
  // valid 40 MHz PPDU — probe rungs wider than radio.width are dead air, so
  // config load drops them (with a stderr warning) instead of flying them.
  auto path = write_temp_json(R"({"radio":{"width":20,"bw_set":[20,40]}})");
  Config cfg = load_config(path.string());
  CHECK((cfg.radio.bw_set == std::vector<uint8_t>{20}));
  std::filesystem::remove(path);
}

TEST(load_config_bw_set_rungs_within_width_are_kept) {
  auto path = write_temp_json(R"({"radio":{"width":40,"bw_set":[20,40]}})");
  Config cfg = load_config(path.string());
  CHECK((cfg.radio.bw_set == std::vector<uint8_t>{20, 40}));
  std::filesystem::remove(path);
}

TEST(load_config_bw_set_all_rungs_dropped_leaves_empty_probe_set) {
  auto path = write_temp_json(R"({"radio":{"width":20,"bw_set":[40,80]}})");
  Config cfg = load_config(path.string());
  CHECK(cfg.radio.bw_set.empty());
  std::filesystem::remove(path);
}

TEST(load_config_type_mismatch_radio_bw_set_string_throws_runtime_error_with_dotted_path) {
  auto path = write_temp_json(R"({"radio":{"bw_set":"wide"}})");
  std::string msg = what_of([&] { (void)load_config(path.string()); });
  CHECK(!msg.empty());
  CHECK(msg.find("radio.bw_set") != std::string::npos);
  CHECK(msg.find("wrong type") != std::string::npos);
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
  CHECK(layers[0].fec.symbol_size == cfg.fec.symbol_size);
  CHECK(layers[0].blocks_per_body == cfg.fec.blocks_per_body[0]);
  CHECK(layers[3].blocks_per_body == cfg.fec.blocks_per_body[3]);
}

MTEST_MAIN
