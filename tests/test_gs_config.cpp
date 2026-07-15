#include <cstdio>
#include <fstream>
#include "mtest.h"
#include "config.h"

static std::string write_tmp(const char* text) {
  std::string path = "/tmp/maburgs_test_config.json";
  std::ofstream f(path);
  f << text;
  return path;
}

TEST(default_bundle_config_loads) {
  auto cfg = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.json");
  CHECK(cfg.radio.channel == 149);
  CHECK(cfg.radio.cards.size() == 1);
  CHECK(cfg.radio.tx_card == -1);
  CHECK(cfg.fec.decode_deadline_ms == 200);
  CHECK(cfg.fec.seq_horizon == 512);
  CHECK(cfg.link.vtx_id == 1);
  CHECK(cfg.video_out.port == 5600);
  auto L = cfg.uep_layers();
  CHECK(L[0].fec.overhead == 1.00);
  CHECK(L[3].fec.overhead == 0.25);
}

TEST(missing_keys_fall_back_to_defaults) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.radio.channel == 149);
  CHECK(cfg.video_out.host == "127.0.0.1");
  CHECK(cfg.link.video_silence_ms == 3000);
}

TEST(errors_are_fail_fast) {
  bool threw = false;
  try { maburgs::load_config("/nonexistent/x.json"); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("{\"radio\": {\"chanel\": 149}}")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("chanel") != std::string::npos; }
  CHECK(threw);  // unknown key named in the error
  threw = false;
  try { maburgs::load_config(write_tmp("{\"video_out\": {\"port\": 99999}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // out of range
  threw = false;
  try { maburgs::load_config(write_tmp("{\"radio\": {\"cards\": []}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // zero cards is a config error
}

TEST(fec_symbol_size_array_per_layer) {
  auto cfg = maburgs::load_config(
      write_tmp(R"({"fec":{"symbol_size":[164,1312,1312,1312]}})"));
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.symbol_size == 164);
  CHECK(layers[3].fec.symbol_size == 1312);
}

TEST(fec_symbol_size_scalar_fans_out) {
  auto cfg = maburgs::load_config(write_tmp(R"({"fec":{"symbol_size":328}})"));
  auto layers = cfg.uep_layers();
  for (int s = 0; s < 4; ++s) CHECK(layers[(size_t)s].fec.symbol_size == 328);
}

TEST(fec_symbol_size_bounds) {
  bool threw = false;
  try {
    maburgs::load_config(
        write_tmp(R"({"fec":{"symbol_size":[164,1312,1312,1600]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);  // 1600 > 1500 upper bound
}

TEST(tx_card_validates_against_effective_card_list) {
  // Test: tx_card 0 with default single card should load without error
  auto cfg = maburgs::load_config(write_tmp("{\"radio\": {\"tx_card\": 0}}"));
  CHECK(cfg.radio.cards.size() == 1);
  CHECK(cfg.radio.tx_card == 0);

  // Test: tx_card 5 with default single card should fail (out of range)
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"radio\": {\"tx_card\": 5}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // out of range against the effective single default card
}
TEST(gs_msp_defaults_and_parse) {
  {
    auto cfg = maburgs::load_config(write_tmp("{}"));
    CHECK(cfg.msp.enable == false);
    CHECK(cfg.msp.out_host == "127.0.0.1");
    CHECK(cfg.msp.out_port == 14560);
    CHECK(cfg.msp.symbol_size == 1312);
    CHECK(cfg.msp.window == 16);
  }
  {
    auto cfg = maburgs::load_config(write_tmp(
        R"({"msp":{"enable":true,"out":{"host":"10.0.0.9","port":15000},)"
        R"("symbol_size":1024,"window":32}})"));
    CHECK(cfg.msp.enable == true);
    CHECK(cfg.msp.out_host == "10.0.0.9");
    CHECK(cfg.msp.out_port == 15000);
    CHECK(cfg.msp.symbol_size == 1024);
    CHECK(cfg.msp.window == 32);
  }
}
MTEST_MAIN

// link.src_bitrate_mbps drives the controller's energy-model design point
// (rungs that can't carry src*(1+overhead) are infeasible). Optional,
// fractional, defaults to the Python controller's 4 Mbps.
TEST(src_bitrate_mbps_parses_and_defaults) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.src_bitrate_mbps > 3.999 && cfg.link.src_bitrate_mbps < 4.001);
  cfg = maburgs::load_config(
      write_tmp("{\"link\": {\"src_bitrate_mbps\": 17.5}}"));
  CHECK(cfg.link.src_bitrate_mbps > 17.499 && cfg.link.src_bitrate_mbps < 17.501);
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"link\": {\"src_bitrate_mbps\": 99}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // out of range
  cfg = maburgs::load_config(write_tmp("{\"link\": {\"margin_db\": 35}}"));
  CHECK(cfg.link.margin_db > 34.999 && cfg.link.margin_db < 35.001);
}
