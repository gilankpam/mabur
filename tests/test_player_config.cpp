#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include "mtest.h"
#include "player_config.h"

static std::string write_tmp_play(const char* text) {
  std::string path = "/tmp/maburplay_test_config.toml";
  std::ofstream f(path);
  f << text;
  return path;
}

// dvr.target (VTX recorder): which recorders the record button drives. Default
// "gs" (identical to pre-feature behaviour).
TEST(dvr_target_values_and_strictness) {
  auto c = maburplay::load_config(write_tmp_play("[dvr]\ntarget = \"both\"\n"));
  CHECK(c.dvr.target == "both");
  c = maburplay::load_config(write_tmp_play("[dvr]\ntarget = \"vtx\"\n"));
  CHECK(c.dvr.target == "vtx");
  c = maburplay::load_config(write_tmp_play("[dvr]\nautostart = false\n"));
  CHECK(c.dvr.target == "gs");
  std::string msg;
  try { maburplay::load_config(write_tmp_play("[dvr]\ntarget = \"sd\"\n")); }
  catch (const std::exception& e) { msg = e.what(); }
  CHECK(msg.find("dvr.target") != std::string::npos);
}

TEST(values_and_strictness) {
  auto c = maburplay::load_config(write_tmp_play(
      "backend = \"null\"\n\n[dvr]\nautostart = false\nfragment_ms = 500\n"));
  CHECK(c.backend == "null");
  CHECK(!c.dvr.autostart);
  CHECK(c.dvr.fragment_ms == 500);
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("bogus = 1\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);
  threw = false;
  try { maburplay::load_config(write_tmp_play("backend = \"vaapi\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // backend must be one of mpp|v4l2req|null
  threw = false;
  try { maburplay::load_config(write_tmp_play("[dvr]\nfragment_ms = 50\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // floor 100
}

TEST(the_old_dvr_enabled_key_is_rejected) {
  // The rename is breaking on purpose: an un-updated /etc/maburplay.toml
  // must fail boot loudly rather than silently reverting to the default.
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[dvr]\nenabled = true\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("enabled") != std::string::npos;
  }
  CHECK(threw);
}

TEST(osd_defaults_are_off_and_conventional) {
  auto c = maburplay::load_config(write_tmp_play("backend = \"null\"\n"));
  CHECK(c.osd.enable == false);
  CHECK(c.osd.port == 14560);
  CHECK(c.osd.scale == "sharp");
  // 5 s = 5 missed snapshots at the drone's default msp.update_rate_hz of
  // 1 Hz; a shorter default strobes the overlay on a single dropped one.
  CHECK(c.osd.stale_ms == 5000);
  CHECK(c.osd.font == "/usr/local/share/mabur/font_btfl.mfont");
}

TEST(osd_block_is_parsed) {
  auto c = maburplay::load_config(write_tmp_play(
      "backend = \"null\"\n\n[osd]\nenable = true\nport = 15000\n"
      "font = \"/tmp/f.mfont\"\nscale = \"fill\"\nstale_ms = 0\n"));
  CHECK(c.osd.enable == true);
  CHECK(c.osd.port == 15000);
  CHECK(c.osd.font == "/tmp/f.mfont");
  CHECK(c.osd.scale == "fill");
  CHECK(c.osd.stale_ms == 0);
}

TEST(osd_rejects_unknown_keys_and_bad_scale) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[osd]\nenabl = true\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play("[osd]\nscale = \"blurry\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(dvr_mode_defaults_to_raw) {
  auto c = maburplay::load_config(write_tmp_play("backend = \"null\"\n"));
  CHECK(c.dvr.mode == "raw");
  CHECK(c.dvr.burned.bitrate_kbps == 12000);
  CHECK(c.dvr.burned.fps_cap == 30);
}

TEST(dvr_burned_block_parses) {
  auto c = maburplay::load_config(write_tmp_play(
      "backend = \"null\"\n\n[dvr]\nmode = \"burned\"\n\n"
      "[dvr.burned]\nbitrate_kbps = 20000\nfps_cap = 60\n"));
  CHECK(c.dvr.mode == "burned");
  CHECK(c.dvr.burned.bitrate_kbps == 20000);
  CHECK(c.dvr.burned.fps_cap == 60);
}

TEST(dvr_rejects_bad_mode_and_unknown_keys) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[dvr]\nmode = \"burnt\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play("[dvr.burned]\nbitrate = 1\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(dvr_burned_bounds_are_enforced) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[dvr.burned]\nfps_cap = 0\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[dvr.burned]\nbitrate_kbps = 500000\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(gs_osd_defaults_are_off_on_8302) {
  auto c = maburplay::load_config(write_tmp_play("backend = \"null\"\n"));
  CHECK(c.osd.gs.enable == false);
  CHECK(c.osd.gs.port == 8302);
  // 3 s = 6 missed samples at the sideport's 500 ms cadence.
  CHECK(c.osd.gs.stale_ms == 3000);
  CHECK(c.osd.gs.font == "/usr/local/share/mabur/gs_osd.gfont");
}

// An empty osd block must leave the gs defaults alone: parsing "osd" and
// parsing "osd.gs" are separate conditionals, and a regression that hung
// the gs defaults off the presence of the outer block would only show here.
TEST(gs_osd_defaults_survive_an_osd_block_without_gs) {
  auto c = maburplay::load_config(write_tmp_play(
      "backend = \"null\"\n\n[osd]\nenable = true\nport = 15000\n"));
  CHECK(c.osd.enable == true);
  CHECK(c.osd.gs.enable == false);
  CHECK(c.osd.gs.port == 8302);
  CHECK(c.osd.gs.stale_ms == 3000);
  CHECK(c.osd.gs.font == "/usr/local/share/mabur/gs_osd.gfont");
}

TEST(gs_osd_keys_parse) {
  auto c = maburplay::load_config(write_tmp_play(
      "[osd.gs]\nenable = true\nport = 9000\nfont = \"/x.gfont\"\n"
      "stale_ms = 1500\n"));
  CHECK(c.osd.gs.enable == true);
  CHECK(c.osd.gs.port == 9000);
  CHECK(c.osd.gs.font == "/x.gfont");
  CHECK(c.osd.gs.stale_ms == 1500);
}

// There is one GS layout, so there is no style key: a config still
// carrying one must refuse to boot like any other unknown key, not be
// quietly accepted.
TEST(gs_osd_style_key_is_gone) {
  for (const char* v : {"compact", "essential"}) {
    bool threw = false;
    try {
      maburplay::load_config(write_tmp_play(
          (std::string("[osd.gs]\nstyle = \"") + v + "\"\n").c_str()));
    } catch (const std::exception&) { threw = true; }
    CHECK(threw == true);
  }
}

// The GS-only topology -- no MSP-capable FC -- is a supported configuration
// and the one this whole overlay exists for. Pinned because main.cpp's
// want_osd is the OR of the two, and a config that cannot express this
// would make that unreachable.
TEST(gs_osd_alone_is_expressible) {
  auto c = maburplay::load_config(write_tmp_play(
      "[osd]\nenable = false\n\n[osd.gs]\nenable = true\n"));
  CHECK(c.osd.enable == false);
  CHECK(c.osd.gs.enable == true);
}

// Strict config: an unknown key under osd.gs must refuse to boot rather
// than silently ignore a typo'd port.
TEST(unknown_gs_key_is_rejected) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[osd.gs]\nprot = 8302\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(gs_osd_bounds_and_types_are_enforced) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[osd.gs]\nport = 0\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play("[osd.gs]\nport = 70000\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play("[osd.gs]\nstale_ms = -1\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play("[osd.gs]\nenable = \"yes\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play("[osd.gs]\nfont = 7\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(input_rec_defaults_to_absent) {
  auto c = maburplay::load_config(write_tmp_play("backend = \"null\"\n"));
  CHECK(c.input.rec.configured == false);
}

TEST(input_rec_is_parsed_with_button_to_ground_defaults) {
  auto c = maburplay::load_config(write_tmp_play(
      "backend = \"null\"\n\n[input.rec]\npin = 32\n"));
  CHECK(c.input.rec.configured == true);
  CHECK(c.input.rec.pin == 32);
  // The assumed wiring: button between the pin and GND, kernel pull-up,
  // line requested active-low so "pressed" reads 1.
  CHECK(c.input.rec.active_low == true);
  CHECK(c.input.rec.bias == "pull-up");
}

TEST(input_rec_honours_explicit_wiring) {
  auto c = maburplay::load_config(write_tmp_play(
      "backend = \"null\"\n\n[input.rec]\npin = 11\nactive_low = false\n"
      "bias = \"pull-down\"\n"));
  CHECK(c.input.rec.pin == 11);
  CHECK(c.input.rec.active_low == false);
  CHECK(c.input.rec.bias == "pull-down");
}

TEST(input_rec_rejects_a_missing_pin_bad_bias_and_unknown_keys) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[input.rec]\nactive_low = true\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("pin") != std::string::npos;
  }
  CHECK(threw);  // pin is required inside input.rec

  threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[input.rec]\npin = 32\nbias = \"floating\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // bias must be pull-up|pull-down|none

  threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[input.rec]\npin = 32\ndebounce_ms = 50\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // debounce is a constant, not a key

  threw = false;
  try { maburplay::load_config(write_tmp_play("[input.menu]\npin = 11\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // one button, one job

  threw = false;
  try { maburplay::load_config(write_tmp_play("[input.rec]\npin = 0\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // pin range floor
}

TEST(display_vsync_defaults) {
  const auto cfg = maburplay::load_config(write_tmp_play(""));
  CHECK(cfg.display.vsync_lock == true);
  CHECK(cfg.display.vsync_lead_ms == 5);
}

TEST(display_vsync_lead_range_enforced) {
  // vsync_lead_ms 0 must fail load (range [1,10]) -- write a minimal
  // config with display.vsync_lead_ms: 0 via the tmp-file helper and
  // CHECK the load throws, matching the file's existing bad-value tests.
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[display]\nvsync_lead_ms = 0\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(
      "[display]\nvsync_lead_ms = 11\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

// display.lat_log_dir went with the debug-log consolidation: maburplay holds
// no logging config at all now, it follows /tmp/mabur-session.
TEST(removed_lat_log_dir_is_rejected) {
  bool threw = false;
  try {
    maburplay::load_config(write_tmp_play(
        "[display]\nlat_log_dir = \"/media/dvr/log\"\n"));
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(player_load_config_top_level_scalars_then_tables) {
  const std::string path = write_tmp_play(
      "backend = \"null\"\n"
      "screen_mode = \"1280x720@60\"\n"
      "\n"
      "[dvr]\n"
      "autostart = true\n"
      "mode = \"burned\"\n"
      "\n"
      "[dvr.burned]\n"
      "bitrate_kbps = 8000\n"
      "\n"
      "[osd.gs]\n"
      "enable = true\n"
      "port = 8302\n");
  auto c = maburplay::load_config(path);
  CHECK(c.backend == "null");
  CHECK(c.screen_mode == "1280x720@60");
  CHECK(c.dvr.autostart == true);
  CHECK(c.dvr.mode == "burned");
  CHECK(c.dvr.burned.bitrate_kbps == 8000);
  CHECK(c.osd.gs.enable == true);
  CHECK(c.osd.gs.port == 8302);
}

TEST(player_load_config_reports_defaulted_keys) {
  const std::string path = write_tmp_play("backend = \"null\"\n");
  std::vector<std::string> defaulted;
  maburplay::load_config(path, &defaulted);
  bool saw_key = false, saw_section = false;
  for (const std::string& d : defaulted) {
    if (d == "screen_mode=1920x1080@60") saw_key = true;   // top-level
    if (d == "display=(section absent)") saw_section = true;
  }
  CHECK(saw_key);
  CHECK(saw_section);
}

TEST(player_load_config_errors_carry_file_and_line) {
  // vsync_lead_ms is range-checked to [1,10]; 99 trips it on line 4.
  const std::string path = write_tmp_play(
      "backend = \"null\"\n\n[display]\nvsync_lead_ms = 99\n");
  std::string msg;
  try {
    maburplay::load_config(path);
  } catch (const std::exception& e) {
    msg = e.what();
  }
  CHECK(msg.find("display.vsync_lead_ms") != std::string::npos);
  CHECK(msg.find(".toml:4:") != std::string::npos);
}

MTEST_MAIN

TEST(display_chain_budget_key) {
  // display.chain_budget: frames a sequential-slot chain may run before
  // the regulator cuts it with one drop. Default 0 = unbounded, the
  // bundle's value too, so a config that omits the key gets what ships;
  // range [0, 60].
  const auto bare = maburplay::load_config(write_tmp_play(""));
  CHECK(bare.display.chain_budget == 0);
  const auto empty_display = maburplay::load_config(write_tmp_play("[display]\n"));
  CHECK(empty_display.display.chain_budget == 0);
  const auto cfg = maburplay::load_config(
      write_tmp_play("[display]\nchain_budget = 3\n"));
  CHECK(cfg.display.chain_budget == 3);
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[display]\nchain_budget = 61\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(display_sched_cap_ms_key) {
  // display.sched_cap_ms: capture-time scheduling's allowance cap. 0 = off
  // (arrival-based servo) in code and bundle; range [0, 50].
  const auto bare = maburplay::load_config(write_tmp_play(""));
  CHECK(bare.display.sched_cap_ms == 0);
  const auto cfg = maburplay::load_config(
      write_tmp_play("[display]\nsched_cap_ms = 16\n"));
  CHECK(cfg.display.sched_cap_ms == 16);
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[display]\nsched_cap_ms = 51\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

// "Every knob is in the bundle": the loader reports each known key the file
// did not set, so an empty report IS the completeness gate. Adding a config
// key without writing it into the bundle fails here, which is the point --
// a knob that only exists in a struct default is a knob nobody knows about.
TEST(bundle_default_sets_every_known_key) {
  std::vector<std::string> defaulted;
  maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.toml", &defaulted);
  for (const std::string& d : defaulted)
    std::fprintf(stderr, "  bundle leaves defaulted: %s\n", d.c_str());
  CHECK(defaulted.empty());
}

TEST(colortrans_enable_key) {
  auto on = maburplay::load_config(write_tmp_play("[colortrans]\nenable = true\n"));
  CHECK(on.colortrans.enable);
  // Absent block: off, so a pre-colortrans config boots unchanged.
  std::vector<std::string> defaulted;
  auto off = maburplay::load_config(write_tmp_play("backend = \"null\"\n"), &defaulted);
  CHECK(!off.colortrans.enable);
  bool noted = false;
  for (const auto& d : defaulted) if (d.find("colortrans") != std::string::npos) noted = true;
  CHECK(noted);
  // Only `enable` exists: the constants live in code.
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[colortrans]\ngain = 2.0\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("colortrans.gain") != std::string::npos;
  }
  CHECK(threw);
}

TEST(decoder_stream_key) {
  // [decoder] stream: A/B and kill switch for the streamed decode. Default on;
  // the bundle sets it.
  CHECK(maburplay::load_config(write_tmp_play("")).decoder.stream);
  CHECK(!maburplay::load_config(write_tmp_play("[decoder]\nstream = false\n")).decoder.stream);
  std::vector<std::string> defaulted;
  maburplay::load_config(write_tmp_play("backend = \"null\"\n"), &defaulted);
  bool noted = false;
  for (const auto& d : defaulted) if (d.find("decoder") != std::string::npos) noted = true;
  CHECK(noted);                               // absent section is reported
  std::string msg;
  try { maburplay::load_config(write_tmp_play("[decoder]\nstreams = true\n")); }
  catch (const std::exception& e) { msg = e.what(); }
  CHECK(msg.find("decoder.streams") != std::string::npos);
  msg.clear();
  try { maburplay::load_config(write_tmp_play("[decoder]\nstream = 1\n")); }
  catch (const std::exception& e) { msg = e.what(); }
  CHECK(msg.find("decoder.stream") != std::string::npos);
}

// backend "v4l2req" is a known name (the Cedrus/V4L2-request decoder).
TEST(backend_v4l2req_accepted) {
  auto c = maburplay::load_config(write_tmp_play("backend = \"v4l2req\"\n"));
  CHECK(c.backend == "v4l2req");
  std::string msg;
  try { maburplay::load_config(write_tmp_play("backend = \"cedrus\"\n")); }
  catch (const std::exception& e) { msg = e.what(); }
  CHECK(msg.find("v4l2req") != std::string::npos);  // the error lists the valid names
}

// A config is checked against what this binary was built with.
TEST(unsupported_on_build_rules) {
  maburplay::Config c;
  maburplay::BuildCaps mpp{true, false}, v4l{false, true}, host{false, false};

  c.backend = "mpp"; c.dvr.mode = "burned";
  CHECK(maburplay::unsupported_on_build(c, mpp).empty());
  CHECK(!maburplay::unsupported_on_build(c, v4l).empty());   // no mpp backend here

  c.backend = "v4l2req"; c.dvr.mode = "raw";
  CHECK(maburplay::unsupported_on_build(c, v4l).empty());
  CHECK(!maburplay::unsupported_on_build(c, mpp).empty());   // no v4l2req backend there

  c.backend = "v4l2req"; c.dvr.mode = "burned";
  CHECK(maburplay::unsupported_on_build(c, v4l) ==
        "dvr.mode \"burned\" needs the MPP encoder; this build has none -- set dvr.mode = \"raw\"");

  // The null backend decodes nothing, so burned mode is merely inert there
  // (host e2e runs the shipped bundle's burned default under --backend null).
  c.backend = "null"; c.dvr.mode = "burned";
  CHECK(maburplay::unsupported_on_build(c, host).empty());
  CHECK(maburplay::unsupported_on_build(c, v4l).empty());
}

// A Radxa toml (backend "mpp", dvr.mode "burned") started on the OPi image
// (v4l2req only) fails two independent checks; both must be reported in one
// string, joined, so the operator does not have to fix one, restart, and
// only then see the second (S97maburplay does not respawn on exit 2).
TEST(unsupported_on_build_reports_every_problem) {
  maburplay::Config c;
  maburplay::BuildCaps v4l{false, true};
  c.backend = "mpp"; c.dvr.mode = "burned";
  CHECK(maburplay::unsupported_on_build(c, v4l) ==
        "backend \"mpp\" is not in this build (Rockchip MPP decoder); this build has: null, v4l2req; "
        "dvr.mode \"burned\" needs the MPP encoder; this build has none -- set dvr.mode = \"raw\"");
}

// The "not in this build" message names the backend(s) this binary DOES
// carry, so the operator knows what to put in the config instead of
// guessing from the board.
TEST(unsupported_on_build_names_available_backends) {
  maburplay::Config c;
  maburplay::BuildCaps mpp{true, false}, v4l{false, true}, host{false, false};

  c.backend = "v4l2req"; c.dvr.mode = "raw";
  CHECK(maburplay::unsupported_on_build(c, mpp) ==
        "backend \"v4l2req\" is not in this build (V4L2-request decoder); this build has: null, mpp");

  c.backend = "mpp"; c.dvr.mode = "raw";
  CHECK(maburplay::unsupported_on_build(c, v4l) ==
        "backend \"mpp\" is not in this build (Rockchip MPP decoder); this build has: null, v4l2req");
  CHECK(maburplay::unsupported_on_build(c, host) ==
        "backend \"mpp\" is not in this build (Rockchip MPP decoder); this build has: null");
}
