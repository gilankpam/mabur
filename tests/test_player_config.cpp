#include <fstream>
#include "mtest.h"
#include "player_config.h"

static std::string write_tmp_play(const char* text) {
  std::string path = "/tmp/maburplay_test_config.json";
  std::ofstream f(path);
  f << text;
  return path;
}

TEST(defaults_from_bundle) {
  auto c = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.json");
  CHECK(c.ring_path == "/dev/shm/mabur-au");
  CHECK(c.socket == "/run/mabur-au.sock");
  CHECK(c.backend == "mpp");
  CHECK(c.screen_mode == "1920x1080@60");
  CHECK(c.dvr.autostart);
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
      "{\"backend\": \"null\", \"dvr\": {\"autostart\": false, \"fragment_ms\": 500}}"));
  CHECK(c.backend == "null");
  CHECK(!c.dvr.autostart);
  CHECK(c.dvr.fragment_ms == 500);
  bool threw = false;
  try { maburplay::load_config(write_tmp_play("{\"bogus\": 1}")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);
  threw = false;
  try { maburplay::load_config(write_tmp_play("{\"backend\": \"vaapi\"}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // backend must be one of mpp|null
  threw = false;
  try { maburplay::load_config(write_tmp_play("{\"dvr\": {\"fragment_ms\": 50}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // floor 100
}

TEST(the_old_dvr_enabled_key_is_rejected) {
  // The rename is breaking on purpose: an un-updated /etc/maburplay.json
  // must fail boot loudly rather than silently reverting to the default.
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"dvr":{"enabled":true}})")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("enabled") != std::string::npos;
  }
  CHECK(threw);
}

TEST(osd_defaults_are_off_and_conventional) {
  auto c = maburplay::load_config(write_tmp_play(R"({"backend":"null"})"));
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
      R"({"backend":"null","osd":{"enable":true,"port":15000,)"
      R"("font":"/tmp/f.mfont","scale":"fill","stale_ms":0}})"));
  CHECK(c.osd.enable == true);
  CHECK(c.osd.port == 15000);
  CHECK(c.osd.font == "/tmp/f.mfont");
  CHECK(c.osd.scale == "fill");
  CHECK(c.osd.stale_ms == 0);
}

TEST(osd_rejects_unknown_keys_and_bad_scale) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"enabl":true}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"scale":"blurry"}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(bundle_default_parses_with_osd_enabled) {
  auto c = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.json");
  CHECK(c.osd.enable == true);
}

TEST(dvr_mode_defaults_to_raw) {
  auto c = maburplay::load_config(write_tmp_play(R"({"backend":"null"})"));
  CHECK(c.dvr.mode == "raw");
  CHECK(c.dvr.burned.bitrate_kbps == 12000);
  CHECK(c.dvr.burned.fps_cap == 30);
}

TEST(dvr_burned_block_parses) {
  auto c = maburplay::load_config(write_tmp_play(
      R"({"backend":"null","dvr":{"mode":"burned",)"
      R"("burned":{"bitrate_kbps":20000,"fps_cap":60}}})"));
  CHECK(c.dvr.mode == "burned");
  CHECK(c.dvr.burned.bitrate_kbps == 20000);
  CHECK(c.dvr.burned.fps_cap == 60);
}

TEST(dvr_rejects_bad_mode_and_unknown_keys) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"dvr":{"mode":"burnt"}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"dvr":{"burned":{"bitrate":1}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(dvr_burned_bounds_are_enforced) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(
      R"({"dvr":{"burned":{"fps_cap":0}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(
      R"({"dvr":{"burned":{"bitrate_kbps":500000}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(gs_osd_defaults_are_off_on_8302) {
  auto c = maburplay::load_config(write_tmp_play(R"({"backend":"null"})"));
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
      R"({"backend":"null","osd":{"enable":true,"port":15000}})"));
  CHECK(c.osd.enable == true);
  CHECK(c.osd.gs.enable == false);
  CHECK(c.osd.gs.port == 8302);
  CHECK(c.osd.gs.stale_ms == 3000);
  CHECK(c.osd.gs.font == "/usr/local/share/mabur/gs_osd.gfont");
}

TEST(gs_osd_keys_parse) {
  auto c = maburplay::load_config(write_tmp_play(
      R"({"osd":{"gs":{"enable":true,"port":9000,"font":"/x.gfont",)"
      R"("stale_ms":1500}}})"));
  CHECK(c.osd.gs.enable == true);
  CHECK(c.osd.gs.port == 9000);
  CHECK(c.osd.gs.font == "/x.gfont");
  CHECK(c.osd.gs.stale_ms == 1500);
}

// The GS-only topology -- no MSP-capable FC -- is a supported configuration
// and the one this whole overlay exists for. Pinned because main.cpp's
// want_osd is the OR of the two, and a config that cannot express this
// would make that unreachable.
TEST(gs_osd_alone_is_expressible) {
  auto c = maburplay::load_config(write_tmp_play(
      R"({"osd":{"enable":false,"gs":{"enable":true}}})"));
  CHECK(c.osd.enable == false);
  CHECK(c.osd.gs.enable == true);
}

// Strict config: an unknown key under osd.gs must refuse to boot rather
// than silently ignore a typo'd port.
TEST(unknown_gs_key_is_rejected) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"gs":{"prot":8302}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(gs_osd_bounds_and_types_are_enforced) {
  bool threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"gs":{"port":0}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"gs":{"port":70000}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"gs":{"stale_ms":-1}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"gs":{"enable":"yes"}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburplay::load_config(write_tmp_play(R"({"osd":{"gs":{"font":7}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

// The shipped bundle must parse, and must ship with the overlay off --
// matching how the MSP OSD and maburgs' sideport itself ship.
TEST(bundle_default_parses_with_gs_osd_off) {
  auto c = maburplay::load_config(
      std::string(MABUR_PLAY_BUNDLE_DIR) + "/maburplay.default.json");
  CHECK(c.osd.gs.enable == false);
  CHECK(c.osd.gs.port == 8302);
  CHECK(c.osd.gs.stale_ms == 3000);
  CHECK(c.osd.gs.font == "/usr/local/share/mabur/gs_osd.gfont");
}

MTEST_MAIN
