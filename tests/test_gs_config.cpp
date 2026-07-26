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
}

// link.video_silence_ms claimed to tune the video-silence escape valve, but
// the valve (VrxRzConfig.link_lost_ms, rendezvous.cpp) has been hardcoded to
// 1000 ms since the GS scaffold (34fe0b9) — the key was never wired to it.
// Removed 2026-07-26 in favor of the hardcode every bench run validated; a
// stale key fails the boot like any other unknown key.
TEST(stale_video_silence_ms_key_throws) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"link\": {\"video_silence_ms\": 3000}}")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("video_silence_ms") != std::string::npos;
  }
  CHECK(threw);
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
TEST(gs_msp_render_mode_and_shm) {
  {  // default render is udp
    auto cfg = maburgs::load_config(write_tmp("{}"));
    CHECK(cfg.msp.render == "udp");
    CHECK(cfg.msp.shm_name == "msp");
    CHECK(cfg.msp.shm_x_offset == 0);
  }
  {  // explicit shm mode
    auto cfg = maburgs::load_config(write_tmp(
        R"({"msp":{"enable":true,"render":"shm","shm":{"name":"osd","x_offset":8,"y_offset":4}}})"));
    CHECK(cfg.msp.render == "shm");
    CHECK(cfg.msp.shm_name == "osd");
    CHECK(cfg.msp.shm_x_offset == 8);
    CHECK(cfg.msp.shm_y_offset == 4);
  }
  {  // invalid render value rejected
    bool threw = false;
    try { maburgs::load_config(write_tmp(R"({"msp":{"render":"drm"}})")); }
    catch (const std::exception&) { threw = true; }
    CHECK(threw == true);
  }
}

TEST(stats_defaults_disabled) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(!cfg.stats.enable);
  CHECK(cfg.stats.host == "127.0.0.1");
  CHECK(cfg.stats.port == 8300);
  CHECK(cfg.stats.interval_ms == 500);
}

TEST(stats_section_parses_and_validates) {
  auto cfg = maburgs::load_config(write_tmp(
      "{\"stats\": {\"enable\": true, \"host\": \"10.0.0.2\","
      " \"port\": 9000, \"interval_ms\": 250}}"));
  CHECK(cfg.stats.enable);
  CHECK(cfg.stats.host == "10.0.0.2");
  CHECK(cfg.stats.port == 9000);
  CHECK(cfg.stats.interval_ms == 250);
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"stats\": {\"interval_ms\": 50}}")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("interval_ms") != std::string::npos; }
  CHECK(threw);  // below the 100 ms floor
  threw = false;
  try { maburgs::load_config(write_tmp("{\"stats\": {\"prot\": 1}}")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("prot") != std::string::npos; }
  CHECK(threw);  // unknown key fail-fast, like every other section
}

MTEST_MAIN

// static_txagc was renamed to static_offset_qdb (USER-FACING, qdB offset
// semantics since 2026-07-17).
TEST(static_offset_qdb_key_parses_and_defaults) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.static_offset_qdb == 0);

  cfg = maburgs::load_config(write_tmp(R"({"link":{"static_offset_qdb":-12}})"));
  CHECK(cfg.link.static_offset_qdb == -12);

  // Old key name is now unknown -> rejected.
  bool threw = false;
  try { maburgs::load_config(write_tmp(R"({"link":{"static_txagc":63}})")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("static_txagc") != std::string::npos;
  }
  CHECK(threw);
}

// src_bitrate_mbps/margin_db/min_offset_qdb/max_offset_qdb/base_ref_idx were
// the old model-driven (Controller/LinkTable) energy-model + qdB-rail keys.
// The measured-loss ladder controller (SDD 2026-07-27) replaces that
// resolver entirely; each is now an unknown key and must fail the boot like
// any other stale key (config.cpp:check_keys, "link").
TEST(deleted_model_controller_keys_now_throw) {
  const char* deleted_keys[] = {"src_bitrate_mbps", "margin_db",
                                "min_offset_qdb", "max_offset_qdb",
                                "base_ref_idx"};
  for (const char* key : deleted_keys) {
    bool threw = false;
    const std::string json =
        std::string(R"({"link":{")") + key + R"(":1}})";
    try {
      maburgs::load_config(write_tmp(json.c_str()));
    } catch (const std::exception& e) {
      threw = std::string(e.what()).find(key) != std::string::npos;
    }
    CHECK(threw);
  }
}

// frame_gap_timeout_ms/frame_lookahead: FrameStream tuning knobs for the
// session-negotiated frame-wire tail (Task 10). JSON keys under video_out.
TEST(video_out_frame_keys) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.video_out.frame_gap_timeout_ms == 50);
  CHECK(cfg.video_out.frame_lookahead == 8);
  auto cfg2 = maburgs::load_config(write_tmp(
      "{\"video_out\": {\"frame_gap_timeout_ms\": 30, \"frame_lookahead\": 4}}"));
  CHECK(cfg2.video_out.frame_gap_timeout_ms == 30);
  CHECK(cfg2.video_out.frame_lookahead == 4);
}

// link.ladder: measured-loss ladder controller rungs + max_mcs filter +
// thresholds (SDD 2026-07-27 ladder-controller Task 2).
TEST(ladder_defaults_to_spec_six_rung_ladder) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 6);
  CHECK(L[0].mcs == 0); CHECK(L[0].overhead > 0.999 && L[0].overhead < 1.001);
  CHECK(L[1].mcs == 2); CHECK(L[1].overhead > 0.499 && L[1].overhead < 0.501);
  CHECK(L[2].mcs == 4); CHECK(L[2].overhead > 0.249 && L[2].overhead < 0.251);
  CHECK(L[3].mcs == 5); CHECK(L[3].overhead > 0.249 && L[3].overhead < 0.251);
  CHECK(L[4].mcs == 6); CHECK(L[4].overhead > 0.149 && L[4].overhead < 0.151);
  CHECK(L[5].mcs == 7); CHECK(L[5].overhead > 0.099 && L[5].overhead < 0.101);
}

TEST(ladder_parses_explicit_array_in_order) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"ladder":[{"mcs":0,"overhead":1.0},{"mcs":3,"overhead":0.4}]}})"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 2);
  CHECK(L[0].mcs == 0);
  CHECK(L[1].mcs == 3);
  CHECK(L[1].overhead > 0.399 && L[1].overhead < 0.401);
}

TEST(ladder_rung_unknown_key_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":0,"overhead":1.0,"bogus":1}]}})"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_rung_mcs_out_of_range_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"ladder":[{"mcs":8,"overhead":0.5}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(ladder_rung_overhead_out_of_range_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"ladder":[{"mcs":0,"overhead":0.01}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"ladder":[{"mcs":0,"overhead":1.5}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(ladder_rung_overhead_boundary_values_accepted) {
  // overhead exactly at the [0.05, 1.0] boundary must load, not throw.
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"ladder":[{"mcs":0,"overhead":0.05},{"mcs":7,"overhead":1.0}]}})"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 2);
  CHECK(cfg.link.ladder_cfg.ladder[0].overhead > 0.0499 && cfg.link.ladder_cfg.ladder[0].overhead < 0.0501);
  CHECK(cfg.link.ladder_cfg.ladder[1].overhead > 0.999 && cfg.link.ladder_cfg.ladder[1].overhead < 1.001);
}

TEST(ladder_empty_array_rejected) {
  bool threw = false;
  try { maburgs::load_config(write_tmp(R"({"link":{"ladder":[]}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

// Spec: ladder must have 1-8 entries. A 9-rung ladder must be rejected even
// though every individual rung is otherwise valid.
TEST(ladder_over_eight_entries_rejected) {
  std::string json = R"({"link":{"ladder":[)";
  for (int i = 0; i < 9; ++i) {
    if (i) json += ",";
    json += "{\"mcs\":" + std::to_string(i % 8) + ",\"overhead\":0.25}";
  }
  json += "]}}";
  bool threw = false;
  try { maburgs::load_config(write_tmp(json.c_str())); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.ladder") != std::string::npos;
  }
  CHECK(threw);
}

TEST(max_mcs_filters_effective_ladder) {
  auto cfg = maburgs::load_config(write_tmp(R"({"link":{"max_mcs":5}})"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 4);
  for (auto& r : L) CHECK(r.mcs <= 5);
}

TEST(max_mcs_zero_keeps_single_mcs0_rung) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"ladder":[{"mcs":0,"overhead":1.0},{"mcs":4,"overhead":0.25}],)"
      R"("max_mcs":0}})"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 1);
  CHECK(cfg.link.ladder_cfg.ladder[0].mcs == 0);
}

TEST(max_mcs_filter_leaving_no_rungs_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":4,"overhead":0.25},{"mcs":6,"overhead":0.15}],)"
        R"("max_mcs":2}})"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("empty after max_mcs filter") != std::string::npos;
  }
  CHECK(threw);
}

TEST(up_util_must_be_less_than_down_util) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"up_util":0.6,"down_util":0.6}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"up_util":0.7,"down_util":0.6}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

// Spec: 0 < up_util < down_util <= 1. up_util == 0 satisfies the
// less-than-down_util check but must still be rejected: at 0 the clean
// window (u < up_util) can never be satisfied (u is never negative), so a
// misconfigured link would be permanently stuck unable to promote off
// rung 0.
TEST(up_util_must_be_strictly_positive) {
  bool threw = false;
  try { maburgs::load_config(write_tmp(R"({"link":{"up_util":0.0}})")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.up_util") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_threshold_keys_parse_with_defaults) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.ladder_cfg.down_util > 0.599 && cfg.link.ladder_cfg.down_util < 0.601);
  CHECK(cfg.link.ladder_cfg.up_util > 0.149 && cfg.link.ladder_cfg.up_util < 0.151);
  CHECK(cfg.link.ladder_cfg.confirm_ms == 250);
  CHECK(cfg.link.ladder_cfg.clean_ms == 5000);
  CHECK(cfg.link.ladder_cfg.probation_ms == 3000);
  CHECK(cfg.link.ladder_cfg.penalty_base_ms == 10000);
  CHECK(cfg.link.ladder_cfg.penalty_max_ms == 60000);
  CHECK(cfg.link.ladder_cfg.hold_after_down_ms == 4000);
  CHECK(cfg.link.ladder_cfg.min_between_changes_ms == 150);
  CHECK(cfg.link.ladder_cfg.feedback_timeout_ms == 1000);

  auto cfg2 = maburgs::load_config(write_tmp(
      R"({"link":{"down_util":0.5,"clean_ms":4000,"penalty_max_ms":30000}})"));
  CHECK(cfg2.link.ladder_cfg.down_util > 0.499 && cfg2.link.ladder_cfg.down_util < 0.501);
  CHECK(cfg2.link.ladder_cfg.clean_ms == 4000);
  CHECK(cfg2.link.ladder_cfg.penalty_max_ms == 30000);
}
