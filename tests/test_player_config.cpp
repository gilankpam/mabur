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
  CHECK(c.dvr.enabled);
  CHECK(c.dvr.dir == "/media/dvr");
  CHECK(c.dvr.fragment_ms == 1000);
}

TEST(values_and_strictness) {
  auto c = maburplay::load_config(write_tmp_play(
      "{\"backend\": \"null\", \"dvr\": {\"enabled\": false, \"fragment_ms\": 500}}"));
  CHECK(c.backend == "null");
  CHECK(!c.dvr.enabled);
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

MTEST_MAIN
