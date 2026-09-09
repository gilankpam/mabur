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

TEST(defaults_from_bundle) {
  auto c = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.toml");
  CHECK(c.ring_path == "/dev/shm/mabur-au");
  CHECK(c.socket == "/run/mabur-au.sock");
  CHECK(c.backend == "mpp");
  CHECK(c.screen_mode == "1920x1080@60");
  // The bundle is the flight config: the record button arms the DVR, so
  // autostart ships OFF even though DvrCfg::autostart defaults to true.
  CHECK(c.dvr.autostart == false);
  CHECK(c.dvr.dir == "/media/dvr");
  CHECK(c.dvr.fragment_ms == 1000);
  // The shipped bundle records with the OSD burned in. Pinned here because
  // it is a product decision, not a code default -- DvrCfg::mode still
  // defaults to "raw" so an omitted key keeps the pristine remux.
  CHECK(c.dvr.mode == "burned");
  CHECK(c.dvr.burned.bitrate_kbps == 8000);
  CHECK(c.dvr.burned.fps_cap == 60);
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
  CHECK(threw);  // backend must be one of mpp|null
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

TEST(bundle_default_parses_with_osd_enabled) {
  auto c = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.toml");
  CHECK(c.osd.enable == true);
  // The shipped bundle draws the compact bar. Pinned for the same reason
  // dvr.mode is: it is a product decision, and the file is what lands on
  // the GS at the next wipe.
  CHECK(c.osd.gs.style == "compact");
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
  // The one-line bottom bar is the default layout, not the corner blocks.
  CHECK(c.osd.gs.style == "compact");
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
      "style = \"essential\"\nstale_ms = 1500\n"));
  CHECK(c.osd.gs.enable == true);
  CHECK(c.osd.gs.port == 9000);
  CHECK(c.osd.gs.font == "/x.gfont");
  CHECK(c.osd.gs.style == "essential");
  CHECK(c.osd.gs.stale_ms == 1500);
}

// Both layouts are expressible, and nothing else is. A typo'd style must
// fail the load rather than silently pick one: the two look nothing alike,
// so "whichever the default was" is not a recoverable outcome in flight.
TEST(gs_osd_style_accepts_both_layouts_and_rejects_anything_else) {
  auto c = maburplay::load_config(write_tmp_play(
      "[osd.gs]\nstyle = \"compact\"\n"));
  CHECK(c.osd.gs.style == "compact");
  for (const char* bad : {"Compact", "bar", "", "essentail"}) {
    bool threw = false;
    try {
      maburplay::load_config(write_tmp_play(
          (std::string("[osd.gs]\nstyle = \"") + bad + "\"\n").c_str()));
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

// The shipped bundle must parse, and ships with the GS overlay ON -- the
// bundle is the flight config, and maburgs' sideport ships enabled to feed
// it. The STRUCT default stays false (see gs_osd_defaults_are_off_on_8302).
TEST(bundle_default_parses_with_gs_osd_on) {
  auto c = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.toml");
  CHECK(c.osd.gs.enable == true);
  CHECK(c.osd.gs.port == 8302);
  CHECK(c.osd.gs.stale_ms == 3000);
  CHECK(c.osd.gs.font == "/usr/local/share/mabur/gs_osd.gfont");
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
  // Bundle default toml must carry the new keys with spec defaults.
  const auto cfg = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.toml");
  CHECK(cfg.display.vsync_lock == true);
  CHECK(cfg.display.vsync_lead_ms == 6);
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

// display.lat_log_dir went with the 2026-09-06 consolidation: maburplay holds
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
  // the regulator cuts it with one drop. Default 3 (bench A/B
  // 2026-09-02, operator choice); 0 = unbounded; range [0, 60].
  const auto def = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.toml");
  CHECK(def.display.chain_budget == 3);
  const auto bare = maburplay::load_config(write_tmp_play(""));
  CHECK(bare.display.chain_budget == 3);
  const auto cfg = maburplay::load_config(
      write_tmp_play("[display]\nchain_budget = 0\n"));
  CHECK(cfg.display.chain_budget == 0);
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("[display]\nchain_budget = 61\n")); }
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
