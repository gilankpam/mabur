#include <cmath>
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
  CHECK(cfg.fec.seq_horizon == 512);
  CHECK(cfg.link.vtx_id == 1);
  CHECK(cfg.video.frame_gap_timeout_ms == 50);
  auto L = cfg.uep_layers();
  CHECK(L[0].fec.overhead == 0.50);
  CHECK(L[1].fec.overhead == 0.50);
}

TEST(missing_keys_fall_back_to_defaults) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.radio.channel == 149);
  CHECK(cfg.video.frame_lookahead == 8);
}

// Actual-overhead ranges (airtime-balance-uep): static_overhead's default
// and validation range are doubled from cmd-value semantics ([0.10, 1.0]
// default 0.25) to actual-air semantics ([0.1, 2.0] default 0.5).
// Same-rate-fixed-pairs (Task 3): static_overhead split into a base/enh
// pair, both keeping the old scalar's default/range.
TEST(static_overhead_default_and_range) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.static_overhead_base > 0.499 && cfg.link.static_overhead_base < 0.501);
  CHECK(cfg.link.static_overhead_enh > 0.499 && cfg.link.static_overhead_enh < 0.501);
  auto cfg2 = maburgs::load_config(write_tmp(R"({"link":{}})"));
  CHECK(cfg2.link.static_overhead_base > 0.499 && cfg2.link.static_overhead_base < 0.501);
  CHECK(cfg2.link.static_overhead_enh > 0.499 && cfg2.link.static_overhead_enh < 0.501);

  bool threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"static_overhead_base":2.1}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"static_overhead_enh":0.05}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);

  auto cfg3 = maburgs::load_config(write_tmp(
      R"({"link":{"static_overhead_base":2.0,"static_overhead_enh":0.1}})"));
  CHECK(cfg3.link.static_overhead_base > 1.999 && cfg3.link.static_overhead_base < 2.001);
  CHECK(cfg3.link.static_overhead_enh > 0.0999 && cfg3.link.static_overhead_enh < 0.1001);
}

// Old scalar key is gone: a config carrying it fails boot (intended forcing
// function, no compat shim -- CLAUDE.md compatibility policy).
TEST(stale_static_overhead_key_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(R"({"link":{"static_overhead":0.5}})"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("unknown key") != std::string::npos;
  }
  CHECK(threw);
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

// Removed 2026-08-15 when the s3 residual demote became instant: with
// attribution unconditional, the confirm window had nothing left to guard
// against. Strict keys mean a tuned device config naming it must be edited
// by hand BEFORE the new binary starts, or maburgs crash-loops at 2 s.
TEST(stale_s3_residual_confirm_ms_key_throws) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"link\": {\"s3_residual_confirm_ms\": 500}}")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("s3_residual_confirm_ms") != std::string::npos;
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
  try { maburgs::load_config(write_tmp("{\"video\": {\"frame_gap_timeout_ms\": 5}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // out of range
  threw = false;
  try { maburgs::load_config(write_tmp("{\"radio\": {\"cards\": []}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // zero cards is a config error
}

TEST(fec_symbol_size_array_per_layer) {
  auto cfg = maburgs::load_config(
      write_tmp(R"({"fec":{"symbol_size":[164,1312]}})"));
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.symbol_size == 164);
  CHECK(layers[1].fec.symbol_size == 1312);
}

TEST(fec_symbol_size_scalar_fans_out) {
  auto cfg = maburgs::load_config(write_tmp(R"({"fec":{"symbol_size":328}})"));
  auto layers = cfg.uep_layers();
  for (int s = 0; s < 2; ++s) CHECK(layers[(size_t)s].fec.symbol_size == 328);
}

TEST(fec_symbol_size_bounds) {
  bool threw = false;
  try {
    maburgs::load_config(
        write_tmp(R"({"fec":{"symbol_size":[164,1600]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);  // 1600 > 1500 upper bound
}

// 2-stream config (airtime-balance-uep): a 4-entry symbol_size array is no
// longer a valid shape and must fail parse with a specific message, not
// silently truncate or index out of bounds.
TEST(fec_symbol_size_wrong_length_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(
        write_tmp(R"({"fec":{"symbol_size":[164,1312,1312,1312]}})"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("array must have 2 ints") != std::string::npos;
  }
  CHECK(threw);
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
TEST(msp_render_and_shm_keys_are_rejected) {
  bool threw = false;
  try { maburgs::load_config(write_tmp(R"({"msp":{"enable":true,"render":"shm"}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburgs::load_config(write_tmp(R"({"msp":{"enable":true,"shm":{"name":"msp"}}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(msp_udp_keys_still_parse) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"msp":{"enable":true,"out":{"host":"127.0.0.1","port":14560},)"
      R"("symbol_size":1312,"window":16}})"));
  CHECK(cfg.msp.enable == true);
  CHECK(cfg.msp.out_port == 14560);
  CHECK(cfg.msp.symbol_size == 1312);
}

TEST(stats_defaults_disabled) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(!cfg.stats.enable);
  REQUIRE(cfg.stats.out.size() == 1);
  CHECK(cfg.stats.out[0].host == "127.0.0.1");
  CHECK(cfg.stats.out[0].port == 8300);
  CHECK(cfg.stats.interval_ms == 500);
}

TEST(stats_section_parses_and_validates) {
  auto cfg = maburgs::load_config(write_tmp(
      "{\"stats\": {\"enable\": true, \"host\": \"10.0.0.2\","
      " \"port\": 9000, \"interval_ms\": 250}}"));
  CHECK(cfg.stats.enable);
  REQUIRE(cfg.stats.out.size() == 1);
  CHECK(cfg.stats.out[0].host == "10.0.0.2");
  CHECK(cfg.stats.out[0].port == 9000);
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

TEST(stats_defaults_to_one_destination) {
  const std::string p = write_tmp("{}");
  const maburgs::Config c = maburgs::load_config(p);
  REQUIRE(c.stats.out.size() == 1);
  CHECK(c.stats.out[0].host == "127.0.0.1");
  CHECK(c.stats.out[0].port == 8300);
  std::remove(p.c_str());
}

TEST(legacy_host_port_still_works) {
  const std::string p = write_tmp(
      "{\"stats\":{\"enable\":true,\"host\":\"10.0.0.5\",\"port\":9999}}");
  const maburgs::Config c = maburgs::load_config(p);
  REQUIRE(c.stats.out.size() == 1);
  CHECK(c.stats.out[0].host == "10.0.0.5");
  CHECK(c.stats.out[0].port == 9999);
  std::remove(p.c_str());
}

TEST(out_list_yields_every_destination_in_order) {
  const std::string p = write_tmp(
      "{\"stats\":{\"enable\":true,\"out\":["
      "{\"host\":\"127.0.0.1\",\"port\":8300},"
      "{\"host\":\"127.0.0.1\",\"port\":8302}]}}");
  const maburgs::Config c = maburgs::load_config(p);
  REQUIRE(c.stats.out.size() == 2);
  CHECK(c.stats.out[0].port == 8300);
  CHECK(c.stats.out[1].port == 8302);
  std::remove(p.c_str());
}

// Strict config: `out` and `host`/`port` together is ambiguous, so it is a
// boot failure rather than a silent precedence rule nobody can remember.
TEST(out_together_with_host_or_port_is_rejected) {
  for (const char* body : {
           "{\"stats\":{\"out\":[{\"host\":\"127.0.0.1\",\"port\":8300}],\"host\":\"127.0.0.1\"}}",
           "{\"stats\":{\"out\":[{\"host\":\"127.0.0.1\",\"port\":8300}],\"port\":8300}}"}) {
    const std::string p = write_tmp(body);
    bool threw = false;
    try { maburgs::load_config(p); } catch (const std::exception&) { threw = true; }
    CHECK(threw);
    std::remove(p.c_str());
  }
}

TEST(empty_out_list_is_rejected) {
  const std::string p = write_tmp("{\"stats\":{\"out\":[]}}");
  bool threw = false;
  try { maburgs::load_config(p); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  std::remove(p.c_str());
}

TEST(out_entry_missing_a_port_is_rejected) {
  const std::string p = write_tmp("{\"stats\":{\"out\":[{\"host\":\"127.0.0.1\"}]}}");
  bool threw = false;
  try { maburgs::load_config(p); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  std::remove(p.c_str());
}

TEST(stale_video_out_key_throws) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"video_out\": {\"port\": 5600}}")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("video_out") != std::string::npos;
  }
  CHECK(threw);
}

MTEST_MAIN

TEST(gs_config_rejects_static_offset_qdb) {
  // Reverting the removal of "static_offset_qdb" from the link known-keys
  // list in gs/src/config.cpp makes this load successfully and the test
  // fails.
  bool threw = false;
  try {
    maburgs::load_config(write_tmp("{\"link\":{\"static_offset_qdb\":0}}"));
  } catch (const std::runtime_error& e) {
    threw = true;
    CHECK(std::string(e.what()).find("unknown key") != std::string::npos);
  }
  CHECK(threw);
}

// src_bitrate_mbps/margin_db/min_offset_qdb/max_offset_qdb/base_ref_idx were
// the old model-driven controller/link-table energy-model + qdB-rail keys.
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
// session-negotiated frame-wire tail (Task 10). JSON keys under video
// (PR C: the section was video_out until the RTP destination was deleted;
// a stale video_out key now fails boot like any other unknown key).
TEST(video_frame_keys) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.video.frame_gap_timeout_ms == 50);
  CHECK(cfg.video.frame_lookahead == 8);
  auto cfg2 = maburgs::load_config(write_tmp(
      "{\"video\": {\"frame_gap_timeout_ms\": 30, \"frame_lookahead\": 4}}"));
  CHECK(cfg2.video.frame_gap_timeout_ms == 30);
  CHECK(cfg2.video.frame_lookahead == 4);
}

// link.ladder: measured-loss ladder controller rungs + max_mcs filter +
// thresholds (SDD 2026-07-27 ladder-controller Task 2).
// Pins the C++ struct default (no "link.ladder" key at all -- config.cpp
// never touches c.link.ladder_cfg.ladder in that case, so this is the
// literal member-initializer in gs/src/config.h). Actual-air overhead
// (airtime-balance-uep): old cmd-value defaults x2, same global rule as
// the bundle json's ladder -- a regression back to half-scale values here
// would parse and validate silently (all in-range), so this test exists
// to catch exactly that.
TEST(ladder_defaults_to_spec_six_rung_ladder) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 6);
  CHECK(L[0].mcs == 0); CHECK(L[0].overhead_base > 1.999 && L[0].overhead_base < 2.001);
  CHECK(L[1].mcs == 2); CHECK(L[1].overhead_base > 0.999 && L[1].overhead_base < 1.001);
  CHECK(L[2].mcs == 4); CHECK(L[2].overhead_base > 0.499 && L[2].overhead_base < 0.501);
  CHECK(L[3].mcs == 5); CHECK(L[3].overhead_base > 0.499 && L[3].overhead_base < 0.501);
  // mcs6 rung at 0.5 (cmd-value 0.25, not the spec's cmd-value 0.15) since
  // 2026-07-29 — see the ladder_cfg comment in gs/src/config.h and
  // docs/mcs6-bench-anomaly.md.
  CHECK(L[4].mcs == 6); CHECK(L[4].overhead_base > 0.499 && L[4].overhead_base < 0.501);
  CHECK(L[5].mcs == 7); CHECK(L[5].overhead_base > 0.199 && L[5].overhead_base < 0.201);
  // Same-rate-fixed-pairs (Task 3): the struct default duplicates each
  // rung's value into overhead_enh too.
  for (auto& r : L) CHECK(std::abs(r.overhead_enh - r.overhead_base) < 1e-9);
}

// Same-rate-fixed-pairs (Task 3): rung overhead is now a base/enh pair.
TEST(rung_overhead_pair_parses) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"ladder":[{"mcs":1,"overhead_base":1.0,"overhead_enh":0.5}]}})"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 1);
  CHECK(L[0].mcs == 1);
  CHECK(std::abs(L[0].overhead_base - 1.0) < 1e-9);
  CHECK(std::abs(L[0].overhead_enh - 0.5) < 1e-9);
}

// Old scalar "overhead" key is gone: a config carrying it fails boot
// (intended forcing function, no compat shim -- CLAUDE.md compat policy).
TEST(rung_old_overhead_key_fails_boot) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":1,"overhead":1.0}]}})"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("unknown key") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_parses_explicit_array_in_order) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"ladder":[{"mcs":0,"overhead_base":1.0,"overhead_enh":1.0},)"
      R"({"mcs":3,"overhead_base":0.4,"overhead_enh":0.4}]}})"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 2);
  CHECK(L[0].mcs == 0);
  CHECK(L[1].mcs == 3);
  CHECK(L[1].overhead_base > 0.399 && L[1].overhead_base < 0.401);
}

TEST(ladder_rung_unknown_key_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":0,"overhead_base":1.0,"overhead_enh":1.0,"bogus":1}]}})"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_rung_mcs_out_of_range_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":8,"overhead_base":0.5,"overhead_enh":0.5}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(ladder_rung_overhead_out_of_range_rejected) {
  // Actual-overhead ranges (airtime-balance-uep): rung overhead accepts
  // [0.1, 2.0], not the old cmd-value [0.05, 1.0]. Same-rate-fixed-pairs
  // (Task 3): both overhead_base and overhead_enh are validated.
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":0,"overhead_base":0.01,"overhead_enh":1.0}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":0,"overhead_base":2.1,"overhead_enh":1.0}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":0,"overhead_base":1.0,"overhead_enh":0.01}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":0,"overhead_base":1.0,"overhead_enh":2.1}]}})"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(ladder_rung_overhead_boundary_values_accepted) {
  // overhead exactly at the [0.1, 2.0] boundary must load, not throw.
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"ladder":[)"
      R"({"mcs":0,"overhead_base":0.1,"overhead_enh":0.1},)"
      R"({"mcs":7,"overhead_base":1.9,"overhead_enh":1.9},)"
      R"({"mcs":6,"overhead_base":2.0,"overhead_enh":2.0}]}})"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 3);
  CHECK(cfg.link.ladder_cfg.ladder[0].overhead_base > 0.0999 && cfg.link.ladder_cfg.ladder[0].overhead_base < 0.1001);
  CHECK(cfg.link.ladder_cfg.ladder[1].overhead_base > 1.899 && cfg.link.ladder_cfg.ladder[1].overhead_base < 1.901);
  CHECK(cfg.link.ladder_cfg.ladder[2].overhead_base > 1.999 && cfg.link.ladder_cfg.ladder[2].overhead_base < 2.001);
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
    json += "{\"mcs\":" + std::to_string(i % 8) +
            ",\"overhead_base\":0.25,\"overhead_enh\":0.25}";
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
      R"({"link":{"ladder":[{"mcs":0,"overhead_base":1.0,"overhead_enh":1.0},)"
      R"({"mcs":4,"overhead_base":0.25,"overhead_enh":0.25}],)"
      R"("max_mcs":0}})"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 1);
  CHECK(cfg.link.ladder_cfg.ladder[0].mcs == 0);
}

TEST(max_mcs_filter_leaving_no_rungs_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        R"({"link":{"ladder":[{"mcs":4,"overhead_base":0.25,"overhead_enh":0.25},)"
        R"({"mcs":6,"overhead_base":0.15,"overhead_enh":0.15}],)"
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

// Default bundle ladder rewritten to actual-air overhead (airtime-balance-
// uep): old cmd values x2, with mcs6/mcs7 landing on genuinely different
// bytes now (0.3 vs 0.2) instead of the 16ths grid collapsing 0.15/0.10
// to the same repair-symbol count.
TEST(default_bundle_ladder_is_actual_overhead) {
  auto c = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.json");
  auto& L = c.link.ladder_cfg.ladder;
  CHECK(L.size() == 6);
  CHECK(L[0].mcs == 0); CHECK(L[0].overhead_base > 1.999 && L[0].overhead_base < 2.001);
  CHECK(L[1].mcs == 2); CHECK(L[1].overhead_base > 0.999 && L[1].overhead_base < 1.001);
  CHECK(L[2].mcs == 4); CHECK(L[2].overhead_base > 0.499 && L[2].overhead_base < 0.501);
  CHECK(L[3].mcs == 5); CHECK(L[3].overhead_base > 0.499 && L[3].overhead_base < 0.501);
  CHECK(L[4].mcs == 6); CHECK(L[4].overhead_base > 0.299 && L[4].overhead_base < 0.301);
  CHECK(L[5].mcs == 7); CHECK(L[5].overhead_base > 0.199 && L[5].overhead_base < 0.201);
  // Same-rate-fixed-pairs (Task 3): the bundle's ladder duplicates base
  // into enh for every rung.
  for (auto& r : L) CHECK(std::abs(r.overhead_enh - r.overhead_base) < 1e-9);
  CHECK(c.link.static_overhead_base > 0.499 && c.link.static_overhead_base < 0.501);
  CHECK(c.link.static_overhead_enh > 0.499 && c.link.static_overhead_enh < 0.501);
  auto layers = c.uep_layers();
  CHECK(layers[0].fec.symbol_size == 332);
  CHECK(layers[1].fec.symbol_size == 332);
}

TEST(au_ring_defaults) {
  auto c = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.json");
  // PR C: the ring IS the video output, so the shipped bundle enables it;
  // the STRUCT default stays false (empty config checked below).
  CHECK(c.au_ring.enable);
  CHECK(!maburgs::load_config(write_tmp("{}")).au_ring.enable);
  CHECK(c.au_ring.path == "/dev/shm/mabur-au");
  CHECK(c.au_ring.socket == "/run/mabur-au.sock");
  CHECK(c.au_ring.slot_kb == 512);
  CHECK(c.au_ring.slot_count == 16);
}

TEST(au_ring_values_load) {
  auto c = maburgs::load_config(write_tmp(
      "{\"au_ring\": {\"enable\": true, \"path\": \"/tmp/r\","
      " \"socket\": \"/tmp/s\", \"slot_kb\": 256, \"slot_count\": 8}}"));
  CHECK(c.au_ring.enable);
  CHECK(c.au_ring.path == "/tmp/r");
  CHECK(c.au_ring.socket == "/tmp/s");
  CHECK(c.au_ring.slot_kb == 256);
  CHECK(c.au_ring.slot_count == 8);
}

// link.probe_*/s3_*/ctl_log keys: s3-probe-before-promote + s3 steady-state
// demote tuning (LadderCfg, Task 4) plus the dedicated adaptive-link log
// (spec 2026-08-05 section 5). Strict keys apply here like every other
// "link" key.
TEST(probe_and_s3_keys_parse) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"probe_ms":3000,"probe_max_util":0.5,"s3_demote":false,)"
      R"("s3_down_util":0.4,"ctl_log":true,"ctl_log_dir":"/tmp/x"}})"));
  CHECK(cfg.link.ladder_cfg.probe_ms == 3000);
  CHECK(cfg.link.ladder_cfg.probe_max_util > 0.499 && cfg.link.ladder_cfg.probe_max_util < 0.501);
  CHECK(!cfg.link.ladder_cfg.s3_demote);
  CHECK(cfg.link.ladder_cfg.s3_down_util > 0.399 && cfg.link.ladder_cfg.s3_down_util < 0.401);
  CHECK(cfg.link.ctl_log);
  CHECK(cfg.link.ctl_log_dir == "/tmp/x");
}

// probe_max_util/s3_down_util absent (< 0 sentinel) resolve to the loaded
// down_util, not the struct default 0.6 -- sentinel resolution must happen
// AFTER down_util parses.
TEST(probe_defaults_and_sentinel_resolution) {
  auto cfg = maburgs::load_config(write_tmp(R"({"link":{"down_util":0.35}})"));
  auto& lc = cfg.link.ladder_cfg;
  CHECK(lc.probe_ms == 2000);
  CHECK(lc.probe_settle_ms == 150);
  CHECK(lc.probe_max_util > 0.349 && lc.probe_max_util < 0.351);  // resolved to down_util
  CHECK(lc.s3_down_util > 0.349 && lc.s3_down_util < 0.351);
  CHECK(lc.s3_demote);
  CHECK(lc.s3_settle_ms == 300);
  CHECK(lc.probe_s3_min_syms == 50);
  CHECK(lc.probe_s3_silence_ms == 500);
  CHECK(!cfg.link.ctl_log);
  CHECK(cfg.link.ctl_log_dir == "/media/dvr");
}

TEST(unknown_probe_key_still_fails) {
  bool threw = false;
  try { maburgs::load_config(write_tmp(R"({"link":{"probe_msx":1}})")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("probe_msx") != std::string::npos;
  }
  CHECK(threw);
}

// Amendment (Task 4 review): a negative/zero probe_s3_min_syms cast to
// uint64_t in the controller would silently disable all probing, so it
// must be rejected loudly at load, not clamped.
TEST(probe_s3_min_syms_rejects_non_positive) {
  bool threw = false;
  try { maburgs::load_config(write_tmp(R"({"link":{"probe_s3_min_syms":0}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp(R"({"link":{"probe_s3_min_syms":-1}})")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(au_ring_strictness) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"au_ring\": {\"bogus\": 1}}")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);  // unknown key named in the error
  threw = false;
  try { maburgs::load_config(write_tmp("{\"au_ring\": {\"slot_kb\": 16}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // below the 64 KiB floor
  threw = false;
  try { maburgs::load_config(write_tmp("{\"au_ring\": {\"slot_count\": 2}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // below the 4-slot floor
}

TEST(rung_stats_defaults) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.ladder_cfg.rung_stats.half_life_samples == 600);
  CHECK(cfg.link.rung_log_period_s == 10);
}

TEST(rung_stats_parses_and_validates) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link":{"rung_stats":{"half_life_samples":100,"rung_log_period_s":5}}})"));
  CHECK(cfg.link.ladder_cfg.rung_stats.half_life_samples == 100);
  CHECK(cfg.link.rung_log_period_s == 5);
  try {  // out-of-range fails boot (strict config)
    maburgs::load_config(
        write_tmp(R"({"link":{"rung_stats":{"half_life_samples":0}}})"));
    CHECK(false);
  } catch (const std::exception&) {}
  try {  // unknown nested key fails boot
    maburgs::load_config(write_tmp(R"({"link":{"rung_stats":{"bogus":1}}})"));
    CHECK(false);
  } catch (const std::exception&) {}
}

// Removed 2026-08-15: attribution is unconditional. The switch's remaining
// value was reproducing pre-attribution numbers, and the instant s3
// residual demote makes attrib=false unsafe rather than merely different.
// link.ctl_log_period_ms (2026-08-15): the S-line cadence became configurable
// so the ctl log can run fast enough to resolve the fade trigger's 300 ms
// sustain, which a 1 Hz record cannot. Optional with a live default, so a
// config that never names it is unaffected.
TEST(ctl_log_period_ms_defaults_and_bounds) {
  auto c = maburgs::load_config(write_tmp("{}"));
  CHECK(c.link.ctl_log_period_ms == 1000);
  auto c2 = maburgs::load_config(write_tmp("{\"link\": {\"ctl_log_period_ms\": 50}}"));
  CHECK(c2.link.ctl_log_period_ms == 50);
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"link\": {\"ctl_log_period_ms\": 49}}")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // below the 50 ms floor
}

TEST(stale_link_attrib_key_throws) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("{\"link\": {\"attrib\": true}}")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("attrib") != std::string::npos;
  }
  CHECK(threw);
}

// link.fade: config surface for fade-aware demotes (spec 2026-08-14
// fade-demote). This task adds ONLY the config block; nothing consumes
// cfg_.fade yet.
TEST(link_fade_defaults) {
  auto cfg = maburgs::load_config(write_tmp(R"({"link": {}})"));
  CHECK(cfg.link.ladder_cfg.fade.cascade == true);
  CHECK(cfg.link.ladder_cfg.fade.predict == true);
  CHECK(cfg.link.ladder_cfg.fade.hold_ms == 2500);
  CHECK(cfg.link.ladder_cfg.fade.confirm_ms == 100);
  CHECK(cfg.link.ladder_cfg.fade.rssi_db == 8.0);
  CHECK(cfg.link.ladder_cfg.fade.snr_db == 4.0);
  CHECK(cfg.link.ladder_cfg.fade.trigger_ms == 300);
  CHECK(cfg.link.ladder_cfg.fade.min_rung == 2);
}

TEST(link_fade_explicit_values_and_kill_switches) {
  auto cfg = maburgs::load_config(write_tmp(
      R"({"link": {"fade": {"cascade": false, "predict": false,
      "hold_ms": 1000, "confirm_ms": 50, "rssi_db": 6.0, "snr_db": 3.0,
      "trigger_ms": 200, "min_rung": 1}}})"));
  CHECK(cfg.link.ladder_cfg.fade.cascade == false);
  CHECK(cfg.link.ladder_cfg.fade.predict == false);
  CHECK(cfg.link.ladder_cfg.fade.hold_ms == 1000);
  CHECK(cfg.link.ladder_cfg.fade.confirm_ms == 50);
  CHECK(cfg.link.ladder_cfg.fade.rssi_db == 6.0);
  CHECK(cfg.link.ladder_cfg.fade.snr_db == 3.0);
  CHECK(cfg.link.ladder_cfg.fade.trigger_ms == 200);
  CHECK(cfg.link.ladder_cfg.fade.min_rung == 1);
}

TEST(link_fade_rejects_unknown_key_and_bad_ranges) {
  // strict keys inside the block
  try {
    maburgs::load_config(write_tmp(R"({"link": {"fade": {"bogus": 1}}})"));
    CHECK(false);
  } catch (const std::exception&) {}
  // bounds: confirm_ms 20-1000, trigger_ms 50-5000, hold_ms 0-60000,
  // min_rung 0-15, rssi_db/snr_db 0.5-40
  try {
    maburgs::load_config(write_tmp(R"({"link": {"fade": {"confirm_ms": 5}}})"));
    CHECK(false);
  } catch (const std::exception&) {}
  try {
    maburgs::load_config(write_tmp(R"({"link": {"fade": {"rssi_db": 0.1}}})"));
    CHECK(false);
  } catch (const std::exception&) {}
}

// --- link.rcf_repeat_* (rcf-uplink-loss findings 2026-08-14) ----------------

TEST(rcf_repeat_defaults_when_absent) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.rcf_repeat_copies == 3);
  CHECK(cfg.link.rcf_repeat_ms == 10);
}

TEST(rcf_repeat_explicit_values_parse) {
  auto cfg = maburgs::load_config(write_tmp(
      "{\"link\": {\"feedback_ms\": 50, \"rcf_repeat_copies\": 4, "
      "\"rcf_repeat_ms\": 8}}"));
  CHECK(cfg.link.rcf_repeat_copies == 4);
  CHECK(cfg.link.rcf_repeat_ms == 8);
}

// --- link.rcf_slot_hold_ms (gs-uplink-self-blanking 2026-09-02) --------------
TEST(rcf_slot_hold_defaults_when_absent) {
  auto cfg = maburgs::load_config(write_tmp("{}"));
  CHECK(cfg.link.rcf_slot_hold_ms == 30);
}

TEST(rcf_slot_hold_explicit_and_zero_parse) {
  auto a = maburgs::load_config(
      write_tmp("{\"link\": {\"rcf_slot_hold_ms\": 33}}"));
  CHECK(a.link.rcf_slot_hold_ms == 33);
  auto b = maburgs::load_config(
      write_tmp("{\"link\": {\"rcf_slot_hold_ms\": 0}}"));
  CHECK(b.link.rcf_slot_hold_ms == 0);
}

// The burst must fit inside one feedback period: a burst spanning the next
// regular RCF slot silently raises the steady-state control rate. Same
// fail-fast stance as the drone's rc_drain_ms <= tick_ms cross-check.
// ⚠ Breaking shape: feedback_ms < 40 with the repeat keys left at defaults
// (3*10=30 needs feedback_ms > 30). Nothing deployed sets feedback_ms
// below 50.
TEST(rcf_repeat_burst_must_fit_inside_feedback_period) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "{\"link\": {\"feedback_ms\": 50, \"rcf_repeat_copies\": 5, "
        "\"rcf_repeat_ms\": 10}}"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("rcf_repeat") != std::string::npos;
  }
  CHECK(threw);
  // copies 0 disables the burst and the cross-check with it.
  auto cfg = maburgs::load_config(write_tmp(
      "{\"link\": {\"feedback_ms\": 20, \"rcf_repeat_copies\": 0}}"));
  CHECK(cfg.link.rcf_repeat_copies == 0);
}

TEST(rcf_repeat_copies_out_of_range_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp("{\"link\": {\"rcf_repeat_copies\": 17}}"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("rcf_repeat_copies") != std::string::npos;
  }
  CHECK(threw);
}
