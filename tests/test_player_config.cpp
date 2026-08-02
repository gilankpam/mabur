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

MTEST_MAIN
