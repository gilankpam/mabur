#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include "mtest.h"
#include "config.h"

static std::string write_tmp(const std::string& text) {
  std::string path = "/tmp/maburgs_test_config.toml";
  std::ofstream f(path);
  f << text;
  return path;
}

TEST(default_bundle_config_loads) {
  auto cfg = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.toml");
  CHECK(cfg.radio.channel == 136);
  // The shipped default lists no cards: the GS auto-scans the bus.
  CHECK(cfg.radio.auto_scan == true);
  CHECK(cfg.radio.cards.empty());
  CHECK(cfg.radio.tx_card == -1);
  CHECK(cfg.fec.seq_horizon == 512);
  CHECK(cfg.link.vtx_id == 1);
  CHECK(cfg.video.frame_gap_timeout_ms == 50);
  auto L = cfg.uep_layers();
  CHECK(L[0].fec.overhead == 0.50);
  CHECK(L[1].fec.overhead == 0.50);
}

TEST(missing_keys_fall_back_to_defaults) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.radio.channel == 149);
  CHECK(cfg.video.frame_lookahead == 8);
}

// Actual-overhead ranges (airtime-balance-uep): static_overhead's default
// and validation range are doubled from cmd-value semantics ([0.10, 1.0]
// default 0.25) to actual-air semantics ([0.1, 2.0] default 0.5).
// Same-rate-fixed-pairs (Task 3): static_overhead split into a base/enh
// pair, both keeping the old scalar's default/range.
TEST(static_overhead_default_and_range) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.link.static_overhead_base > 0.499 && cfg.link.static_overhead_base < 0.501);
  CHECK(cfg.link.static_overhead_enh > 0.499 && cfg.link.static_overhead_enh < 0.501);
  auto cfg2 = maburgs::load_config(write_tmp("[link]\n"));
  CHECK(cfg2.link.static_overhead_base > 0.499 && cfg2.link.static_overhead_base < 0.501);
  CHECK(cfg2.link.static_overhead_enh > 0.499 && cfg2.link.static_overhead_enh < 0.501);

  bool threw = false;
  try {
    maburgs::load_config(write_tmp("[link]\nstatic_overhead_base = 2.1\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp("[link]\nstatic_overhead_enh = 0.05\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);

  auto cfg3 = maburgs::load_config(write_tmp(
      "[link]\nstatic_overhead_base = 2.0\nstatic_overhead_enh = 0.1\n"));
  CHECK(cfg3.link.static_overhead_base > 1.999 && cfg3.link.static_overhead_base < 2.001);
  CHECK(cfg3.link.static_overhead_enh > 0.0999 && cfg3.link.static_overhead_enh < 0.1001);
}

// Old scalar key is gone: a config carrying it fails boot (intended forcing
// function, no compat shim -- CLAUDE.md compatibility policy).
TEST(stale_static_overhead_key_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp("[link]\nstatic_overhead = 0.5\n"));
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
  try { maburgs::load_config(write_tmp("[link]\nvideo_silence_ms = 3000\n")); }
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
  try { maburgs::load_config(write_tmp("[link]\ns3_residual_confirm_ms = 500\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("s3_residual_confirm_ms") != std::string::npos;
  }
  CHECK(threw);
}

TEST(errors_are_fail_fast) {
  bool threw = false;
  try { maburgs::load_config("/nonexistent/x.toml"); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nchanel = 149\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("chanel") != std::string::npos; }
  CHECK(threw);  // unknown key named in the error
  threw = false;
  try { maburgs::load_config(write_tmp("[video]\nframe_gap_timeout_ms = 5\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // out of range
  threw = false;
  try { maburgs::load_config(write_tmp("[radio]\ncards = []\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // zero cards is a config error
}

TEST(fec_symbol_size_array_per_layer) {
  auto cfg = maburgs::load_config(
      write_tmp("[fec]\nsymbol_size = [164, 1312]\n"));
  auto layers = cfg.uep_layers();
  CHECK(layers[0].fec.symbol_size == 164);
  CHECK(layers[1].fec.symbol_size == 1312);
}

TEST(fec_symbol_size_scalar_fans_out) {
  auto cfg = maburgs::load_config(write_tmp("[fec]\nsymbol_size = 328\n"));
  auto layers = cfg.uep_layers();
  for (int s = 0; s < 2; ++s) CHECK(layers[(size_t)s].fec.symbol_size == 328);
}

TEST(fec_symbol_size_bounds) {
  bool threw = false;
  try {
    maburgs::load_config(
        write_tmp("[fec]\nsymbol_size = [164, 1600]\n"));
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
        write_tmp("[fec]\nsymbol_size = [164, 1312, 1312, 1312]\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("array must have 2 ints") != std::string::npos;
  }
  CHECK(threw);
}

TEST(tx_card_validates_against_an_explicit_card_list_only) {
  // An explicit list is a fact at load time, so it is range-checked here.
  const std::string two_cards =
      "[[radio.cards]]\nindex = 0\n\n[[radio.cards]]\nindex = 1\n";
  auto cfg = maburgs::load_config(write_tmp(two_cards + "\n[radio]\ntx_card = 1\n"));
  CHECK(cfg.radio.tx_card == 1);

  bool threw = false;
  try { maburgs::load_config(write_tmp(two_cards + "\n[radio]\ntx_card = 5\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // out of range against the two cards actually listed

  // Under auto-scan the count is a hardware fact discovered after load, so
  // a pin cannot be range-checked here -- main.cpp warns and falls back to
  // auto-select if the scan comes back with fewer cards than the pin.
  auto autocfg = maburgs::load_config(write_tmp("[radio]\ntx_card = 5\n"));
  CHECK(autocfg.radio.auto_scan == true);
  CHECK(autocfg.radio.tx_card == 5);
}
TEST(gs_msp_defaults_and_parse) {
  {
    auto cfg = maburgs::load_config(write_tmp(""));
    CHECK(cfg.msp.enable == false);
    CHECK(cfg.msp.out_host == "127.0.0.1");
    CHECK(cfg.msp.out_port == 14560);
    CHECK(cfg.msp.symbol_size == 1312);
    CHECK(cfg.msp.window == 16);
  }
  {
    auto cfg = maburgs::load_config(write_tmp(
        "[msp]\nenable = true\nsymbol_size = 1024\nwindow = 32\n"
        "\n[msp.out]\nhost = \"10.0.0.9\"\nport = 15000\n"));
    CHECK(cfg.msp.enable == true);
    CHECK(cfg.msp.out_host == "10.0.0.9");
    CHECK(cfg.msp.out_port == 15000);
    CHECK(cfg.msp.symbol_size == 1024);
    CHECK(cfg.msp.window == 32);
  }
}
TEST(msp_render_and_shm_keys_are_rejected) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[msp]\nenable = true\nrender = \"shm\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);

  threw = false;
  try { maburgs::load_config(write_tmp("[msp]\nenable = true\n\n[msp.shm]\nname = \"msp\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw == true);
}

TEST(msp_udp_keys_still_parse) {
  auto cfg = maburgs::load_config(write_tmp(
      "[msp]\nenable = true\nsymbol_size = 1312\nwindow = 16\n"
      "\n[msp.out]\nhost = \"127.0.0.1\"\nport = 14560\n"));
  CHECK(cfg.msp.enable == true);
  CHECK(cfg.msp.out_port == 14560);
  CHECK(cfg.msp.symbol_size == 1312);
}

TEST(stats_defaults_disabled) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(!cfg.stats.enable);
  REQUIRE(cfg.stats.out.size() == 1);
  CHECK(cfg.stats.out[0].host == "127.0.0.1");
  CHECK(cfg.stats.out[0].port == 8300);
  CHECK(cfg.stats.interval_ms == 500);
}

TEST(stats_section_parses_and_validates) {
  auto cfg = maburgs::load_config(write_tmp(
      "[stats]\nenable = true\nhost = \"10.0.0.2\"\n"
      "port = 9000\ninterval_ms = 250\n"));
  CHECK(cfg.stats.enable);
  REQUIRE(cfg.stats.out.size() == 1);
  CHECK(cfg.stats.out[0].host == "10.0.0.2");
  CHECK(cfg.stats.out[0].port == 9000);
  CHECK(cfg.stats.interval_ms == 250);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[stats]\ninterval_ms = 50\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("interval_ms") != std::string::npos; }
  CHECK(threw);  // below the 100 ms floor
  threw = false;
  try { maburgs::load_config(write_tmp("[stats]\nprot = 1\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("prot") != std::string::npos; }
  CHECK(threw);  // unknown key fail-fast, like every other section
}

TEST(stats_defaults_to_one_destination) {
  const std::string p = write_tmp("");
  const maburgs::Config c = maburgs::load_config(p);
  REQUIRE(c.stats.out.size() == 1);
  CHECK(c.stats.out[0].host == "127.0.0.1");
  CHECK(c.stats.out[0].port == 8300);
  std::remove(p.c_str());
}

TEST(legacy_host_port_still_works) {
  const std::string p = write_tmp(
      "[stats]\nenable = true\nhost = \"10.0.0.5\"\nport = 9999\n");
  const maburgs::Config c = maburgs::load_config(p);
  REQUIRE(c.stats.out.size() == 1);
  CHECK(c.stats.out[0].host == "10.0.0.5");
  CHECK(c.stats.out[0].port == 9999);
  std::remove(p.c_str());
}

TEST(out_list_yields_every_destination_in_order) {
  const std::string p = write_tmp(
      "[stats]\nenable = true\n"
      "\n[[stats.out]]\nhost = \"127.0.0.1\"\nport = 8300\n"
      "\n[[stats.out]]\nhost = \"127.0.0.1\"\nport = 8302\n");
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
           "[stats]\nhost = \"127.0.0.1\"\n\n[[stats.out]]\nhost = \"127.0.0.1\"\nport = 8300\n",
           "[stats]\nport = 8300\n\n[[stats.out]]\nhost = \"127.0.0.1\"\nport = 8300\n"}) {
    const std::string p = write_tmp(body);
    bool threw = false;
    try { maburgs::load_config(p); } catch (const std::exception&) { threw = true; }
    CHECK(threw);
    std::remove(p.c_str());
  }
}

TEST(empty_out_list_is_rejected) {
  const std::string p = write_tmp("[stats]\nout = []\n");
  bool threw = false;
  try { maburgs::load_config(p); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  std::remove(p.c_str());
}

TEST(out_entry_missing_a_port_is_rejected) {
  const std::string p = write_tmp("[[stats.out]]\nhost = \"127.0.0.1\"\n");
  bool threw = false;
  try { maburgs::load_config(p); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  std::remove(p.c_str());
}

TEST(stale_video_out_key_throws) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[video_out]\nport = 5600\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("video_out") != std::string::npos;
  }
  CHECK(threw);
}

TEST(gs_load_config_parses_arrays_of_tables) {
  const std::string path = write_tmp(
      "[[radio.cards]]\n"
      "usb_vid = 3034\n"
      "index = 0\n"
      "\n"
      "[[radio.cards]]\n"
      "usb_vid = 3034\n"
      "index = 1\n"
      "\n"
      "[link]\n"
      "vtx_id = 1\n"
      "\n"
      "[[link.ladder]]\n"
      "mcs = 2\n"
      "overhead_base = 1.0\n"
      "overhead_enh = 1.0\n"
      "\n"
      "[[link.ladder]]\n"
      "mcs = 5\n"
      "overhead_base = 0.5\n"
      "overhead_enh = 0.5\n");
  auto cfg = maburgs::load_config(path);
  CHECK(cfg.radio.cards.size() == 2);
  CHECK(cfg.radio.cards[1].index == 1);
  CHECK(cfg.radio.auto_scan == false);  // an explicit list pins; no scan
  CHECK(cfg.link.ladder_cfg.ladder.size() == 2);
  CHECK(cfg.link.ladder_cfg.ladder[0].mcs == 2);
  CHECK(cfg.link.ladder_cfg.ladder[1].overhead_enh == 0.5);
}

TEST(gs_load_config_reports_defaulted_keys) {
  // The sections must be PRESENT for their keys to be visited: a whole
  // missing section is reported as the section, not key by key.
  const std::string path = write_tmp(
      "[link]\nvtx_id = 1\n"
      "\n[stats]\ninterval_ms = 500\n");
  std::vector<std::string> defaulted;
  auto cfg = maburgs::load_config(path, &defaulted);
  CHECK(cfg.link.feedback_ms == 100);
  bool saw_bool = false, saw_int = false, saw_section = false;
  for (const std::string& d : defaulted) {
    if (d == "stats.enable=false") saw_bool = true;       // via get_bool
    if (d == "link.feedback_ms=100") saw_int = true;      // via get_int
    if (d == "radio=(section absent)") saw_section = true;
  }
  CHECK(saw_bool);
  CHECK(saw_int);
  CHECK(saw_section);
}

TEST(gs_load_config_errors_carry_file_and_line) {
  // up_util must stay below down_util; this trips that rule on line 3.
  const std::string path =
      write_tmp("[link]\ndown_util = 0.6\nup_util = 0.9\n");
  std::string msg;
  try {
    maburgs::load_config(path);
  } catch (const std::exception& e) {
    msg = e.what();
  }
  CHECK(msg.find("link.up_util") != std::string::npos);
  CHECK(msg.find(".toml:3:") != std::string::npos);
}

// Fix round 1 (task-4 review): five keys were guarded by a bare
// `if (contains(...))` with no `get_*` call and no `else note_default`,
// so their absence silently vanished from the defaulted-key report --
// fec.symbol_size, radio.cards, link.ladder (all reported by the drone's
// equivalent parse_radio/parse_fec), plus the two sentinel-guarded keys
// link.s3_down_util and link.probe.max_util (whose guard must stay: an
// absent value resolves to link.down_util, not to get_num's own default).
TEST(gs_load_config_reports_previously_invisible_defaults) {
  const std::string path = write_tmp(
      "[radio]\nchannel = 149\n"
      "\n[fec]\nseq_horizon = 512\n"
      "\n[link]\nvtx_id = 1\ndown_util = 0.4\n"
      "\n[link.probe]\nenable = true\n");
  std::vector<std::string> defaulted;
  auto cfg = maburgs::load_config(path, &defaulted);
  CHECK(cfg.radio.auto_scan == true);
  CHECK(cfg.link.ladder_cfg.ladder.size() == 6);

  bool saw_cards = false, saw_symbol_size = false, saw_ladder = false,
       saw_s3_down_util = false, saw_probe_max_util = false;
  for (const std::string& d : defaulted) {
    if (d == "radio.cards=(auto-scan)") saw_cards = true;
    if (d == "fec.symbol_size=64") saw_symbol_size = true;
    if (d == "link.ladder=(6 default rungs)") saw_ladder = true;
    if (d == "link.s3_down_util=(defaults to link.down_util)") saw_s3_down_util = true;
    if (d == "link.probe.max_util=(defaults to link.down_util)") saw_probe_max_util = true;
    // The "report a fake number" trap: an absent link.s3_down_util must
    // never be reported as get_num's own default (0.35) -- it resolves to
    // link.down_util (0.4 in this fixture, not 0.35) after this function
    // returns, so a line naming 0.35 would be a lie.
    if (d.rfind("link.s3_down_util=", 0) == 0)
      CHECK(d.find("0.35") == std::string::npos);
  }
  CHECK(saw_cards);
  CHECK(saw_symbol_size);
  CHECK(saw_ladder);
  CHECK(saw_s3_down_util);
  CHECK(saw_probe_max_util);
  // Confirms the sentinel resolution actually ran to down_util (0.4), not
  // to get_num's own out-of-band default (0.35).
  CHECK(std::abs(cfg.link.ladder_cfg.s3_down_util - 0.4) < 1e-9);
  CHECK(std::abs(cfg.link.ladder_cfg.probe.max_util - 0.4) < 1e-9);
}

MTEST_MAIN

TEST(gs_config_rejects_static_offset_qdb) {
  // Reverting the removal of "static_offset_qdb" from the link known-keys
  // list in gs/src/config.cpp makes this load successfully and the test
  // fails.
  bool threw = false;
  try {
    maburgs::load_config(write_tmp("[link]\nstatic_offset_qdb = 0\n"));
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
    const std::string toml_body =
        std::string("[link]\n") + key + " = 1\n";
    try {
      maburgs::load_config(write_tmp(toml_body));
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
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.video.frame_gap_timeout_ms == 50);
  CHECK(cfg.video.frame_lookahead == 8);
  auto cfg2 = maburgs::load_config(write_tmp(
      "[video]\nframe_gap_timeout_ms = 30\nframe_lookahead = 4\n"));
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
  auto cfg = maburgs::load_config(write_tmp(""));
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
      "[[link.ladder]]\nmcs = 1\noverhead_base = 1.0\noverhead_enh = 0.5\n"));
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
        "[[link.ladder]]\nmcs = 1\noverhead = 1.0\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("unknown key") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_parses_explicit_array_in_order) {
  auto cfg = maburgs::load_config(write_tmp(
      "[[link.ladder]]\nmcs = 0\noverhead_base = 1.0\noverhead_enh = 1.0\n"
      "\n[[link.ladder]]\nmcs = 3\noverhead_base = 0.4\noverhead_enh = 0.4\n"));
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
        "[[link.ladder]]\nmcs = 0\noverhead_base = 1.0\noverhead_enh = 1.0\nbogus = 1\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_rung_mcs_out_of_range_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 8\noverhead_base = 0.5\noverhead_enh = 0.5\n"));
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
        "[[link.ladder]]\nmcs = 0\noverhead_base = 0.01\noverhead_enh = 1.0\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\noverhead_base = 2.1\noverhead_enh = 1.0\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\noverhead_base = 1.0\noverhead_enh = 0.01\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\noverhead_base = 1.0\noverhead_enh = 2.1\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(ladder_rung_overhead_boundary_values_accepted) {
  // overhead exactly at the [0.1, 2.0] boundary must load, not throw.
  auto cfg = maburgs::load_config(write_tmp(
      "[[link.ladder]]\nmcs = 0\noverhead_base = 0.1\noverhead_enh = 0.1\n"
      "\n[[link.ladder]]\nmcs = 7\noverhead_base = 1.9\noverhead_enh = 1.9\n"
      "\n[[link.ladder]]\nmcs = 6\noverhead_base = 2.0\noverhead_enh = 2.0\n"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 3);
  CHECK(cfg.link.ladder_cfg.ladder[0].overhead_base > 0.0999 && cfg.link.ladder_cfg.ladder[0].overhead_base < 0.1001);
  CHECK(cfg.link.ladder_cfg.ladder[1].overhead_base > 1.899 && cfg.link.ladder_cfg.ladder[1].overhead_base < 1.901);
  CHECK(cfg.link.ladder_cfg.ladder[2].overhead_base > 1.999 && cfg.link.ladder_cfg.ladder[2].overhead_base < 2.001);
}

TEST(ladder_empty_array_rejected) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link]\nladder = []\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

// Spec: ladder must have 1-8 entries. A 9-rung ladder must be rejected even
// though every individual rung is otherwise valid.
TEST(ladder_over_eight_entries_rejected) {
  std::string toml_body;
  for (int i = 0; i < 9; ++i) {
    toml_body += "[[link.ladder]]\nmcs = " + std::to_string(i % 8) +
                 "\noverhead_base = 0.25\noverhead_enh = 0.25\n\n";
  }
  bool threw = false;
  try { maburgs::load_config(write_tmp(toml_body)); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.ladder") != std::string::npos;
  }
  CHECK(threw);
}

TEST(max_mcs_filters_effective_ladder) {
  auto cfg = maburgs::load_config(write_tmp("[link]\nmax_mcs = 5\n"));
  auto& L = cfg.link.ladder_cfg.ladder;
  CHECK(L.size() == 4);
  for (auto& r : L) CHECK(r.mcs <= 5);
}

TEST(max_mcs_zero_keeps_single_mcs0_rung) {
  auto cfg = maburgs::load_config(write_tmp(
      "[link]\nmax_mcs = 0\n"
      "\n[[link.ladder]]\nmcs = 0\noverhead_base = 1.0\noverhead_enh = 1.0\n"
      "\n[[link.ladder]]\nmcs = 4\noverhead_base = 0.25\noverhead_enh = 0.25\n"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 1);
  CHECK(cfg.link.ladder_cfg.ladder[0].mcs == 0);
}

TEST(max_mcs_filter_leaving_no_rungs_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[link]\nmax_mcs = 2\n"
        "\n[[link.ladder]]\nmcs = 4\noverhead_base = 0.25\noverhead_enh = 0.25\n"
        "\n[[link.ladder]]\nmcs = 6\noverhead_base = 0.15\noverhead_enh = 0.15\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("empty after max_mcs filter") != std::string::npos;
  }
  CHECK(threw);
}

TEST(up_util_must_be_less_than_down_util) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp("[link]\nup_util = 0.6\ndown_util = 0.6\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp("[link]\nup_util = 0.7\ndown_util = 0.6\n"));
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
  try { maburgs::load_config(write_tmp("[link]\nup_util = 0.0\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.up_util") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_threshold_keys_parse_with_defaults) {
  auto cfg = maburgs::load_config(write_tmp(""));
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
      "[link]\ndown_util = 0.5\nclean_ms = 4000\npenalty_max_ms = 30000\n"));
  CHECK(cfg2.link.ladder_cfg.down_util > 0.499 && cfg2.link.ladder_cfg.down_util < 0.501);
  CHECK(cfg2.link.ladder_cfg.clean_ms == 4000);
  CHECK(cfg2.link.ladder_cfg.penalty_max_ms == 30000);
}

// The bundle carries the FLIGHT ladder, not a neutral seed: seven written
// rungs mcs 0..5, none of which link.max_mcs = 5 filters out. mcs 0 is the
// failsafe floor every controller starts from and falls back to -- pinned
// here because "the failsafe rung moved" is the kind of change that must be
// deliberate. Overheads are actual-air (airtime-balance-uep), and the flight
// ladder splits them: base 1.0 / enh 0.5 on every rung.
TEST(default_bundle_ladder_is_the_flight_ladder) {
  auto c = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.toml");
  auto& L = c.link.ladder_cfg.ladder;
  CHECK(L.size() == 6);
  for (size_t i = 0; i < L.size(); ++i) {
    CHECK(L[i].mcs == static_cast<int>(i));
    CHECK(L[i].overhead_base > 0.999 && L[i].overhead_base < 1.001);
    CHECK(L[i].overhead_enh > 0.499 && L[i].overhead_enh < 0.501);
  }
  // Operator rule (uep-base-protection-constraint): base protection must
  // never fall below enh on any rung.
  for (auto& r : L) CHECK(r.overhead_base >= r.overhead_enh);
  CHECK(c.link.static_overhead_base > 0.499 && c.link.static_overhead_base < 0.501);
  CHECK(c.link.static_overhead_enh > 0.499 && c.link.static_overhead_enh < 0.501);
  auto layers = c.uep_layers();
  CHECK(layers[0].fec.symbol_size == 332);
  CHECK(layers[1].fec.symbol_size == 332);
}

TEST(au_ring_defaults) {
  auto c = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.toml");
  // PR C: the ring IS the video output, so the shipped bundle enables it;
  // the STRUCT default stays false (empty config checked below).
  CHECK(c.au_ring.enable);
  CHECK(!maburgs::load_config(write_tmp("")).au_ring.enable);
  CHECK(c.au_ring.path == "/dev/shm/mabur-au");
  CHECK(c.au_ring.socket == "/run/mabur-au.sock");
  CHECK(c.au_ring.slot_kb == 512);
  CHECK(c.au_ring.slot_count == 16);
}

TEST(au_ring_values_load) {
  auto c = maburgs::load_config(write_tmp(
      "[au_ring]\nenable = true\npath = \"/tmp/r\"\n"
      "socket = \"/tmp/s\"\nslot_kb = 256\nslot_count = 8\n"));
  CHECK(c.au_ring.enable);
  CHECK(c.au_ring.path == "/tmp/r");
  CHECK(c.au_ring.socket == "/tmp/s");
  CHECK(c.au_ring.slot_kb == 256);
  CHECK(c.au_ring.slot_count == 8);
}

// link.s3_* keys: s3 steady-state demote tuning (LadderCfg). Strict keys
// apply here like every other "link" key. The five flat probe_* keys went
// with the s3 probe (spec 2026-09-04); link.probe replaces them.
TEST(s3_keys_parse) {
  auto cfg = maburgs::load_config(write_tmp(
      "[link]\ns3_demote = false\ns3_down_util = 0.4\n"));
  CHECK(!cfg.link.ladder_cfg.s3_demote);
  CHECK(cfg.link.ladder_cfg.s3_down_util > 0.399 && cfg.link.ladder_cfg.s3_down_util < 0.401);
}

// An absent s3_down_util (< 0 sentinel) resolves to the loaded down_util,
// not the struct default 0.6 -- sentinel resolution must happen AFTER
// down_util parses.
TEST(s3_defaults_and_sentinel_resolution) {
  auto cfg = maburgs::load_config(write_tmp("[link]\ndown_util = 0.35\n"));
  auto& lc = cfg.link.ladder_cfg;
  CHECK(lc.s3_down_util > 0.349 && lc.s3_down_util < 0.351);
  CHECK(lc.s3_demote);
  CHECK(lc.s3_settle_ms == 300);
}

// The four keys the 2026-09-06 consolidation removed. Unknown keys fail boot
// by design (PR #7), which is what makes an old config a hard stop rather
// than a silently half-configured GS.
TEST(removed_ctl_log_keys_are_rejected) {
  auto throws = [](const char* text) {
    try {
      maburgs::load_config(write_tmp(text));
    } catch (const std::exception&) {
      return true;
    }
    return false;
  };
  CHECK(throws("[link]\nctl_log = true\n"));
  CHECK(throws("[link]\nctl_log_dir = \"/x\"\n"));
  CHECK(throws("[link]\nctl_log_period_ms = 500\n"));
  CHECK(throws("[link.rung_stats]\nrung_log_period_s = 5\n"));
}

TEST(unknown_probe_key_still_fails) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link]\nprobe_msx = 1\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("probe_msx") != std::string::npos;
  }
  CHECK(threw);
}

TEST(au_ring_strictness) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[au_ring]\nbogus = 1\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);  // unknown key named in the error
  threw = false;
  try { maburgs::load_config(write_tmp("[au_ring]\nslot_kb = 16\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // below the 64 KiB floor
  threw = false;
  try { maburgs::load_config(write_tmp("[au_ring]\nslot_count = 2\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);  // below the 4-slot floor
}

TEST(rung_stats_defaults) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.link.ladder_cfg.rung_stats.half_life_samples == 600);
}

TEST(rung_stats_parses_and_validates) {
  auto cfg = maburgs::load_config(
      write_tmp("[link.rung_stats]\nhalf_life_samples = 100\n"));
  CHECK(cfg.link.ladder_cfg.rung_stats.half_life_samples == 100);
  try {  // out-of-range fails boot (strict config)
    maburgs::load_config(
        write_tmp("[link.rung_stats]\nhalf_life_samples = 0\n"));
    CHECK(false);
  } catch (const std::exception&) {}
  try {  // unknown nested key fails boot
    maburgs::load_config(write_tmp("[link.rung_stats]\nbogus = 1\n"));
    CHECK(false);
  } catch (const std::exception&) {}
}

// Removed 2026-08-15: attribution is unconditional. The switch's remaining
// value was reproducing pre-attribution numbers, and the instant s3
// residual demote makes attrib=false unsafe rather than merely different.
TEST(stale_link_attrib_key_throws) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link]\nattrib = true\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("attrib") != std::string::npos;
  }
  CHECK(threw);
}

// link.fade: config surface for fade-aware demotes (spec 2026-08-14
// fade-demote). This task adds ONLY the config block; nothing consumes
// cfg_.fade yet.
TEST(link_fade_defaults) {
  auto cfg = maburgs::load_config(write_tmp("[link]\n"));
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
      "[link.fade]\ncascade = false\npredict = false\n"
      "hold_ms = 1000\nconfirm_ms = 50\nrssi_db = 6.0\nsnr_db = 3.0\n"
      "trigger_ms = 200\nmin_rung = 1\n"));
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
    maburgs::load_config(write_tmp("[link.fade]\nbogus = 1\n"));
    CHECK(false);
  } catch (const std::exception&) {}
  // bounds: confirm_ms 20-1000, trigger_ms 50-5000, hold_ms 0-60000,
  // min_rung 0-15, rssi_db/snr_db 0.5-40
  try {
    maburgs::load_config(write_tmp("[link.fade]\nconfirm_ms = 5\n"));
    CHECK(false);
  } catch (const std::exception&) {}
  try {
    maburgs::load_config(write_tmp("[link.fade]\nrssi_db = 0.1\n"));
    CHECK(false);
  } catch (const std::exception&) {}
}

// --- link.rcf_slot_hold_ms (gs-uplink-self-blanking 2026-09-02) --------------
TEST(rcf_slot_hold_defaults_when_absent) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.link.rcf_slot_hold_ms == 30);
}

TEST(rcf_slot_hold_explicit_and_zero_parse) {
  auto a = maburgs::load_config(
      write_tmp("[link]\nrcf_slot_hold_ms = 33\n"));
  CHECK(a.link.rcf_slot_hold_ms == 33);
  auto b = maburgs::load_config(
      write_tmp("[link]\nrcf_slot_hold_ms = 0\n"));
  CHECK(b.link.rcf_slot_hold_ms == 0);
}

// --- link.probe block (spec 2026-09-04 sections 4.2, 5) ---------------------
TEST(probe_block_defaults) {
  auto c = maburgs::load_config(write_tmp(""));
  const auto& p = c.link.ladder_cfg.probe;
  CHECK(p.enable); CHECK(p.rung_offset == 1); CHECK(p.clean_ms == 2000);
  CHECK(p.max_util == c.link.ladder_cfg.down_util);  // sentinel resolved
  CHECK(p.min_syms == 40); CHECK(p.silence_ms == 500); CHECK(p.pin_mcs == -1);
  CHECK(c.link.ladder_cfg.s3_min_syms == 50);
}

TEST(probe_block_parses_and_bounds) {
  auto c = maburgs::load_config(write_tmp(
      "[link]\ns3_min_syms = 30\n"
      "\n[link.probe]\nenable = false\nrung_offset = 2\nclean_ms = 1500\n"
      "max_util = 0.4\nmin_syms = 20\nsilence_ms = 800\npin_mcs = 5\n"));
  const auto& p = c.link.ladder_cfg.probe;
  CHECK(!p.enable); CHECK(p.rung_offset == 2); CHECK(p.clean_ms == 1500);
  CHECK(std::abs(p.max_util - 0.4) < 1e-9); CHECK(p.min_syms == 20);
  CHECK(p.silence_ms == 800); CHECK(p.pin_mcs == 5);
  CHECK(c.link.ladder_cfg.s3_min_syms == 30);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link.probe]\nrung_offset = 0\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
  // link.probe.min_syms is bounded [4, 100000] -- 0 must reject too.
  threw = false;
  try { maburgs::load_config(write_tmp("[link.probe]\nmin_syms = 0\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(old_flat_probe_keys_fail_boot) {
  for (const char* k : {"probe_ms", "probe_settle_ms", "probe_max_util",
                        "probe_s3_min_syms", "probe_s3_silence_ms"}) {
    bool threw = false;
    try { maburgs::load_config(write_tmp(std::string("[link]\n") + k + " = 1\n")); }
    catch (const std::exception& e) { threw = std::string(e.what()).find(k) != std::string::npos; }
    CHECK(threw);
  }
}

// Carried from the Task 7 review: s3_min_syms feeds s3_usable()'s
// h.s3_expected_syms >= s3_min_syms comparison directly -- a non-positive
// value would silently disable every s3 demote.
TEST(s3_min_syms_rejects_non_positive) {
  for (const char* v : {"0", "-5"}) {
    bool threw = false;
    try { maburgs::load_config(write_tmp(std::string("[link]\ns3_min_syms = ") + v + "\n")); }
    catch (const std::exception& e) { threw = std::string(e.what()).find("s3_min_syms") != std::string::npos; }
    CHECK(threw);
  }
}

// debug_log (2026-09-06 consolidation): one knob for the whole GS. Default
// OFF -- nothing is written until it is set.
TEST(debug_log_defaults) {
  auto c = maburgs::load_config(write_tmp(""));
  CHECK(!c.debug_log.enable);
  CHECK(c.debug_log.dir == "/media/dvr/log");
  CHECK(c.debug_log.ctl_period_ms == 1000);
  CHECK(c.debug_log.rung_period_s == 10);
}

TEST(debug_log_parses_values) {
  auto c = maburgs::load_config(write_tmp(
      "[debug_log]\nenable = true\ndir = \"/tmp/x\"\n"
      "ctl_period_ms = 250\nrung_period_s = 30\n"));
  CHECK(c.debug_log.enable);
  CHECK(c.debug_log.dir == "/tmp/x");
  CHECK(c.debug_log.ctl_period_ms == 250);
  CHECK(c.debug_log.rung_period_s == 30);
}

TEST(debug_log_rejects_out_of_range_and_unknown_keys) {
  auto throws = [](const char* text) {
    try {
      maburgs::load_config(write_tmp(text));
    } catch (const std::exception&) {
      return true;
    }
    return false;
  };
  CHECK(throws("[debug_log]\nctl_period_ms = 49\n"));   // below the floor
  CHECK(throws("[debug_log]\nrung_period_s = 0\n"));    // below the floor
  CHECK(throws("[debug_log]\nenable = \"yes\"\n"));     // wrong type
  CHECK(throws("[debug_log]\nnope = 1\n"));             // strict keys
}

// "Every knob is in the bundle": the loader reports each known key the file
// did not set, so the report IS the completeness gate. Adding a config key
// without writing it into the bundle fails here, which is the point -- a
// knob that only exists in a struct default is a knob nobody knows about.
//
// radio.cards is the one permitted omission, and it is not laziness: its
// ABSENCE is the auto-scan setting (a list pins cards and skips the probe),
// so there is no value the bundle could write that means "scan the bus".
TEST(bundle_default_sets_every_known_key_but_radio_cards) {
  std::vector<std::string> defaulted;
  maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.toml",
                       &defaulted);
  for (const std::string& d : defaulted)
    std::fprintf(stderr, "  bundle leaves defaulted: %s\n", d.c_str());
  CHECK(defaulted.size() == 1);
  CHECK(!defaulted.empty() && defaulted[0] == "radio.cards=(auto-scan)");
}
