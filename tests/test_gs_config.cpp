#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include "mtest.h"
#include "config.h"
#include "mabur/link_key.h"

static std::string write_tmp(const std::string& text) {
  std::string path = "/tmp/maburgs_test_config.toml";
  std::ofstream f(path);
  f << text;
  return path;
}

static std::string what_of(const std::function<void()>& fn) {
  try {
    fn();
  } catch (const std::exception& e) {
    return e.what();
  }
  return "";
}

// The shipped bundle must load through the real loader. Its values are
// tuning and deliberately not pinned -- retuning must not fail a test.
// (bundle_default_sets_every_known_key_but_radio_cards keeps it complete.)
TEST(default_bundle_config_loads) {
  auto cfg = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.toml");
  CHECK(!cfg.link.ladder_cfg.ladder.empty());
}

TEST(missing_keys_fall_back_to_defaults) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.radio.channels.front() == 40);
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

// Software NACK (spec 2026-10-05 fec-nack section 7): [link.nack] is
// optional and absent = off; the spike's settle_ms knob is gone (settle is
// adaptive), and lookback must stay inside the decoder's seq horizon.
TEST(link_nack_section_parses_and_defaults_off) {
  auto off = maburgs::load_config(write_tmp(""));
  CHECK(!off.link.nack.enable && off.link.nack.lookback == 256 &&
        off.link.nack.repeat_ms == 16 && off.link.nack.max_tries == 2);
  CHECK(off.link.nack.min_lead_ms == 12);
  auto on = maburgs::load_config(write_tmp(
      "[link.nack]\nenable = true\nlookback = 128\nrepeat_ms = 20\nmax_tries = 1\nmin_lead_ms = 8\n"));
  CHECK(on.link.nack.enable && on.link.nack.lookback == 128 &&
        on.link.nack.repeat_ms == 20 && on.link.nack.max_tries == 1);
  CHECK(on.link.nack.min_lead_ms == 8);
  bool lead_threw = false;
  try { maburgs::load_config(write_tmp("[link.nack]\nmin_lead_ms = 0\n")); } catch (const std::exception&) { lead_threw = true; }
  CHECK(lead_threw);  // 0 would send requests that cannot be answered in time
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link.nack]\nsettle_ms = 5\n")); } catch (const std::exception&) { threw = true; }
  CHECK(threw);   // the spike's knob is gone; settle is adaptive
  threw = false;
  try { maburgs::load_config(write_tmp("[link.nack]\nlookback = 600\n")); } catch (const std::exception&) { threw = true; }
  CHECK(threw);   // must stay below fec.seq_horizon (512 default)
  // ...and the bound follows fec.seq_horizon rather than a fixed number.
  auto wide = maburgs::load_config(write_tmp(
      "[fec]\nseq_horizon = 1024\n[link.nack]\nlookback = 600\n"));
  CHECK(wide.link.nack.lookback == 600);
}

TEST(nack_disabled_sends_nothing) {
  // Config off => the exporter never sees a block and the tracker is never
  // constructed enabled; pinned here at the config/exporter seam, and in
  // test_nack_tracker's disabled_tracker_is_inert for the tracker itself.
  auto off = maburgs::load_config(write_tmp(""));
  CHECK(!off.link.nack.enable);
  auto bundle = maburgs::load_config(std::string(MABUR_GS_BUNDLE_DIR) + "/maburgs.default.toml");
  CHECK(!bundle.link.nack.enable);
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
      "\n"
      "[[link.ladder]]\n"
      "mcs = 2\n"
      "bw = 20\n"
      "overhead_base = 1.0\n"
      "overhead_enh = 1.0\n"
      "\n"
      "[[link.ladder]]\n"
      "mcs = 5\n"
      "bw = 20\n"
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
      "[link]\n"
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
      "[radio]\n"
      "\n[fec]\nseq_horizon = 512\n"
      "\n[link]\ndown_util = 0.4\n"
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

TEST(radio_channels_default_set_and_auto) {
  auto cfg = maburgs::load_config(write_tmp(""));
  REQUIRE(cfg.radio.channels.size() == 4);
  CHECK(cfg.radio.channels[0] == 40 && cfg.radio.channels[1] == 64 &&
        cfg.radio.channels[2] == 112 && cfg.radio.channels[3] == 144);
  CHECK(!cfg.radio.pin.has_value());                 // "auto"
  CHECK(cfg.radio.scan.dwell_ms == 250);
  CHECK(cfg.radio.scan.settle_ms == 30);
  CHECK(cfg.radio.scan.min_rounds == 3);
  CHECK(cfg.radio.scan.search_ms == 100);
  CHECK(cfg.radio.scan.op_window_ms == 300);
  CHECK(cfg.radio.scan.search_after_ms == 5000);
  CHECK(cfg.radio.scan.pick_margin == 20);
  CHECK(cfg.radio.scan.one_card_ms == 5000);
  CHECK(cfg.radio.scan.max_ms == 30000);
}

TEST(radio_channel_auto_string_or_member_pin) {
  auto a = maburgs::load_config(write_tmp("[radio]\nchannel = \"auto\"\nchannels = [136, 144]\nwidth = 40\n"));
  CHECK(!a.radio.pin.has_value());
  REQUIRE(a.radio.channels.size() == 2);
  auto p = maburgs::load_config(write_tmp("[radio]\nchannel = 144\nchannels = [136, 144]\nwidth = 40\n"));
  REQUIRE(p.radio.pin.has_value()); CHECK(*p.radio.pin == 144);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nchannel = 112\nchannels = [136, 144]\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("radio.channel") != std::string::npos; }
  CHECK(threw);                                        // pin must be a member
  threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nchannel = \"manual\"\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

// The pin is range-checked BEFORE the uint8_t cast: 296 wraps to 40, a
// member of the default set, and used to be accepted as a 40 pin.
// Revert (cast first): no throw.
TEST(radio_channel_pin_out_of_range_fails_before_the_cast) {
  for (const char* body : {"[radio]\nchannel = 296\n", "[radio]\nchannel = 0\n",
                           "[radio]\nchannel = -216\n"}) {
    bool threw = false;
    try { maburgs::load_config(write_tmp(body)); }
    catch (const std::exception& e) { threw = std::string(e.what()).find("radio.channel") != std::string::npos; }
    CHECK(threw);
  }
}

TEST(radio_channels_validated_by_channel_set_rules) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nchannels = [40, 36]\nwidth = 40\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("radio.channels") != std::string::npos; }
  CHECK(threw);                                        // mixed offsets
  threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nchannels = []\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nchannels = [40, 40]\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(radio_scan_parses_and_validates) {
  auto cfg = maburgs::load_config(write_tmp(
      "[radio.scan]\ndwell_ms = 500\nsettle_ms = 40\nmin_rounds = 2\nsearch_ms = 60\n"
      "op_window_ms = 400\nsearch_after_ms = 8000\npick_margin = 5\none_card_ms = 0\nmax_ms = 20000\n"));
  CHECK(cfg.radio.scan.dwell_ms == 500);
  CHECK(cfg.radio.scan.search_ms == 60);
  CHECK(cfg.radio.scan.op_window_ms == 400);
  CHECK(cfg.radio.scan.search_after_ms == 8000);
  CHECK(cfg.radio.scan.pick_margin == 5);
  CHECK(cfg.radio.scan.one_card_ms == 0);
  CHECK(cfg.radio.scan.max_ms == 20000);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[radio.scan]\nsearch_ms = 10\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);                                        // [40,2000]
}

TEST(removed_scan_and_home_keys_fail_boot) {
  for (const char* body : {"[radio.scan]\nenable = true\n", "[radio.scan]\ncandidates = [144]\n",
                           "[radio.scan]\nhome_window_ms = 300\n", "[radio.scan]\nsplit_after_ms = 5000\n",
                           "[radio.scan]\nhome_margin = 20\n"}) {
    bool threw = false;
    try { maburgs::load_config(write_tmp(body)); } catch (const std::exception&) { threw = true; }
    CHECK(threw);
  }
}

TEST(hop_defaults_when_absent) {
  auto cfg = maburgs::load_config(write_tmp(""));
  CHECK(cfg.hop.window_ms == 150 && cfg.hop.persist == 2);
  CHECK(cfg.hop.dwell_observe_ms == 5 && cfg.hop.dwell_period_ms == 333);
  CHECK(cfg.hop.confirm_ms == 500 && cfg.hop.verify_ms == 1000 && cfg.hop.cooldown_ms == 2000);
  CHECK(cfg.hop.max_hops_per_min == 4 && cfg.hop.backoff_ms == 30000 && cfg.hop.one_card_repeats == 5);
  CHECK(cfg.hop.verdict.loss_pct == 3.0 && cfg.hop.verdict.fa_pps == 100 && cfg.hop.verdict.weak_rssi_dbm == -78);
  CHECK(cfg.radio.scan.busy.busy_dbm == -83 && cfg.radio.scan.busy.blocked_pct == 50.0);
}
// 2026-10-04: pin is static. The mode knob is radio.channel alone; the
// busy-air thresholds belong to every measurer, so they live under
// radio.scan. Removed keys fail boot, as every removed key does.
TEST(removed_hop_keys_fail_boot) {
  for (const char* body : {"[hop]\nenable = true\n", "[hop]\nscout_when_disabled = false\n",
                           "[hop.verdict]\nbusy_dbm = -83\n", "[hop.verdict]\nblocked_pct = 50\n"}) {
    bool threw = false;
    try { maburgs::load_config(write_tmp(body)); } catch (const std::exception&) { threw = true; }
    CHECK(threw);
  }
}
TEST(hop_parses_and_validates) {
  auto cfg = maburgs::load_config(write_tmp(
      "[hop]\nwindow_ms = 200\npersist = 3\ndwell_observe_ms = 8\n"
      "[hop.verdict]\nfa_pps = 250\nweak_rssi_dbm = -80\n"));
  CHECK(cfg.hop.window_ms == 200 && cfg.hop.persist == 3 && cfg.hop.dwell_observe_ms == 8);
  CHECK(cfg.hop.verdict.fa_pps == 250 && cfg.hop.verdict.weak_rssi_dbm == -80);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[hop]\nwindow_ms = 10\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("hop.window_ms") != std::string::npos; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[hop]\npersist = 4\n")); }   // > 3
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("hop.persist") != std::string::npos; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[hop.verdict]\nbogus = 1\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("hop.verdict.bogus") != std::string::npos; }
  CHECK(threw);
}
TEST(hop_relay_burst_period_ms_default_and_range) {
  CHECK(maburgs::load_config(write_tmp("")).hop.relay_burst_period_ms == 1000);
  CHECK(maburgs::load_config(write_tmp("[hop]\nrelay_burst_period_ms = 500\n")).hop.relay_burst_period_ms == 500);
  CHECK(what_of([] { maburgs::load_config(write_tmp("[hop]\nrelay_burst_period_ms = 50\n")); })
            .find("relay_burst_period_ms") != std::string::npos);
  // min 500: under ~450 ms a burst can re-fire on the tick the trigger
  // returns after a result and starve the controller (final review item 5)
  CHECK(what_of([] { maburgs::load_config(write_tmp("[hop]\nrelay_burst_period_ms = 400\n")); })
            .find("relay_burst_period_ms") != std::string::npos);
}
TEST(radio_scan_energy_period_ms_is_gone) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[radio.scan]\nenergy_period_ms = 1000\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("radio.scan.energy_period_ms") != std::string::npos; }
  CHECK(threw);
}

// ---- per-rung width (2026-09-24, 40 MHz rungs) ---------------------------

TEST(ladder_rung_bw_is_required_and_20_or_40) {
  auto cfg = maburgs::load_config(write_tmp(
      "[radio]\nwidth = 40\n"
      "[[link.ladder]]\nmcs = 4\nbw = 20\noverhead_base = 0.5\noverhead_enh = 0.25\n"
      "[[link.ladder]]\nmcs = 3\nbw = 40\noverhead_base = 0.5\noverhead_enh = 0.25\n"));
  auto& L = cfg.link.ladder_cfg.ladder;
  REQUIRE(L.size() == 2);
  CHECK(L[0].bw == 20);
  CHECK(L[1].mcs == 3 && L[1].bw == 40);

  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 4\noverhead_base = 0.5\noverhead_enh = 0.25\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.ladder[0].bw") != std::string::npos;
  }
  CHECK(threw);  // missing bw fails boot: no default, no compat shim
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 4\nbw = 80\noverhead_base = 0.5\noverhead_enh = 0.25\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.ladder[0].bw") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_rung_bw_40_needs_radio_width_40) {
  // A 20-tuned receiver cannot hear HT40 at all (findings "Mixed width"):
  // commanding a 40 rung from a GS tuned 20 would blank the video silently.
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[radio]\nwidth = 20\n"
        "[[link.ladder]]\nmcs = 3\nbw = 40\noverhead_base = 0.5\noverhead_enh = 0.25\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("link.ladder[0].bw") != std::string::npos &&
            std::string(e.what()).find("radio.width") != std::string::npos;
  }
  CHECK(threw);
}

TEST(radio_width_is_20_or_40_and_40_needs_a_pair) {
  CHECK(maburgs::load_config(write_tmp("[radio]\nwidth = 40\n")).radio.width == 40);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[radio]\nwidth = 80\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("radio.width") != std::string::npos; }
  CHECK(threw);
  threw = false;
  // A channel-set member with no 40 MHz pair is caught by channel_set_issue
  // on radio.channels now, not a bare radio.channel pin check.
  try { maburgs::load_config(write_tmp("[radio]\nchannels = [165]\nwidth = 40\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("radio.channels") != std::string::npos &&
            std::string(e.what()).find("165") != std::string::npos;
  }
  CHECK(threw);
}

TEST(width_issue_helpers_match_the_loader) {
  // The web GS validates its page channel/width override with these same
  // helpers; load_config's radio.width / ladder checks go through them.
  CHECK(!maburgs::radio_width_issue(136, 40));
  CHECK(!maburgs::radio_width_issue(165, 20));
  auto e = maburgs::radio_width_issue(165, 40);
  REQUIRE(e.has_value());
  CHECK(e->field == "radio.width" && e->why.find("165") != std::string::npos);
  CHECK(maburgs::radio_width_issue(136, 80).has_value());
  maburgs::LinkCfg link;
  CHECK(!maburgs::link_width_issue(link, 20));   // default ladder is all 20
  link.ladder_cfg.ladder.back().bw = 40;
  auto l = maburgs::link_width_issue(link, 20);
  REQUIRE(l.has_value());
  CHECK(l->field.rfind("link.ladder[", 0) == 0);
  CHECK(!maburgs::link_width_issue(link, 40));
}

TEST(static_bw_defaults_20_and_40_needs_radio_width_40) {
  CHECK(maburgs::load_config(write_tmp("")).link.static_bw == 20);
  auto cfg = maburgs::load_config(write_tmp(
      "[radio]\nwidth = 40\n[link]\nstatic_mcs = 3\nstatic_bw = 40\n"));
  CHECK(cfg.link.static_bw == 40);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link]\nstatic_mcs = 3\nstatic_bw = 40\n")); }
  catch (const std::exception& e) { threw = std::string(e.what()).find("link.static_bw") != std::string::npos; }
  CHECK(threw);
}

// Operator rule: the base layer, which every frame depends on, never gets
// less FEC than the droppable enhance layer. Enforced by the loader so any
// config -- not just the shipped one -- fails boot on an inverted rung.
TEST(ladder_rung_with_base_below_enh_overhead_is_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 0.25\noverhead_enh = 0.5\n"));
  } catch (const std::runtime_error& e) {
    threw = std::string(e.what()).find("link.ladder[0]") != std::string::npos;
  }
  CHECK(threw);
  auto ok = maburgs::load_config(write_tmp(
      "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 0.5\noverhead_enh = 0.5\n"));
  CHECK(ok.link.ladder_cfg.ladder.size() == 1);
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
  for (auto& r : L) CHECK(r.bw == 20);
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
      "[[link.ladder]]\nmcs = 1\nbw = 20\noverhead_base = 1.0\noverhead_enh = 0.5\n"));
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
        "[[link.ladder]]\nmcs = 1\nbw = 20\noverhead = 1.0\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("unknown key") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_parses_explicit_array_in_order) {
  auto cfg = maburgs::load_config(write_tmp(
      "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 1.0\noverhead_enh = 1.0\n"
      "\n[[link.ladder]]\nmcs = 3\nbw = 20\noverhead_base = 0.4\noverhead_enh = 0.4\n"));
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
        "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 1.0\noverhead_enh = 1.0\nbogus = 1\n"));
  } catch (const std::exception& e) {
    threw = std::string(e.what()).find("bogus") != std::string::npos;
  }
  CHECK(threw);
}

TEST(ladder_rung_mcs_out_of_range_rejected) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 8\nbw = 20\noverhead_base = 0.5\noverhead_enh = 0.5\n"));
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
        "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 0.01\noverhead_enh = 1.0\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 2.1\noverhead_enh = 1.0\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 1.0\noverhead_enh = 0.01\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 1.0\noverhead_enh = 2.1\n"));
  } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

TEST(ladder_rung_overhead_boundary_values_accepted) {
  // overhead exactly at the [0.1, 2.0] boundary must load, not throw.
  auto cfg = maburgs::load_config(write_tmp(
      "[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 0.1\noverhead_enh = 0.1\n"
      "\n[[link.ladder]]\nmcs = 7\nbw = 20\noverhead_base = 1.9\noverhead_enh = 1.9\n"
      "\n[[link.ladder]]\nmcs = 6\nbw = 20\noverhead_base = 2.0\noverhead_enh = 2.0\n"));
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
                 "\nbw = 20\noverhead_base = 0.25\noverhead_enh = 0.25\n\n";
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
      "\n[[link.ladder]]\nmcs = 0\nbw = 20\noverhead_base = 1.0\noverhead_enh = 1.0\n"
      "\n[[link.ladder]]\nmcs = 4\nbw = 20\noverhead_base = 0.25\noverhead_enh = 0.25\n"));
  CHECK(cfg.link.ladder_cfg.ladder.size() == 1);
  CHECK(cfg.link.ladder_cfg.ladder[0].mcs == 0);
}

TEST(max_mcs_filter_leaving_no_rungs_throws) {
  bool threw = false;
  try {
    maburgs::load_config(write_tmp(
        "[link]\nmax_mcs = 2\n"
        "\n[[link.ladder]]\nmcs = 4\nbw = 20\noverhead_base = 0.25\noverhead_enh = 0.25\n"
        "\n[[link.ladder]]\nmcs = 6\nbw = 20\noverhead_base = 0.15\noverhead_enh = 0.15\n"));
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

TEST(au_ring_defaults) {
  // The STRUCT default stays off; the shipped bundle turns it on.
  auto c = maburgs::load_config(write_tmp(""));
  CHECK(!c.au_ring.enable);
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

// --- link.arrival_guard_syms (cca-on 2026-09-23, from tx-windows §5.2) ------
TEST(arrival_guard_defaults_and_parses) {
  CHECK(maburgs::load_config(write_tmp("")).link.arrival_guard_syms == 192);
  CHECK(maburgs::load_config(write_tmp("[link]\narrival_guard_syms = 96\n")).link.arrival_guard_syms == 96);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link]\narrival_guard_syms = 8\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("link.arrival_guard_syms") != std::string::npos; }
  CHECK(threw);
}

// --- link.probe block (spec 2026-09-04 sections 4.2, 5) ---------------------
TEST(probe_block_defaults) {
  auto c = maburgs::load_config(write_tmp(""));
  const auto& p = c.link.ladder_cfg.probe;
  CHECK(p.enable); CHECK(p.rung_offset == 1); CHECK(p.clean_bodies == 90);
  CHECK(p.max_util == c.link.ladder_cfg.down_util);  // sentinel resolved
  CHECK(p.min_syms == 16); CHECK(p.silence_ms == 500); CHECK(p.pin_mcs == -1);
  CHECK(c.link.ladder_cfg.s3_min_syms == 50);
}

TEST(probe_block_parses_and_bounds) {
  auto c = maburgs::load_config(write_tmp(
      "[link]\ns3_min_syms = 30\n"
      "\n[link.probe]\nenable = false\nrung_offset = 2\nclean_bodies = 180\n"
      "max_util = 0.4\nmin_syms = 20\nsilence_ms = 800\npin_mcs = 5\n"));
  const auto& p = c.link.ladder_cfg.probe;
  CHECK(!p.enable); CHECK(p.rung_offset == 2); CHECK(p.clean_bodies == 180);
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
  // clean_bodies is bounded [10, 100000]: fewer than 10 bodies is no
  // evidence at all, and 9 must reject.
  threw = false;
  try { maburgs::load_config(write_tmp("[link.probe]\nclean_bodies = 9\n")); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

// probe.clean_ms went with the ms streak (probe per AU, 2026-09-16): the
// streak is clean_bodies now, and the old key fails boot like any other
// unknown key -- a config still carrying it would silently mean a
// different confidence at a different frame rate.
TEST(probe_clean_ms_key_fails_boot) {
  bool threw = false;
  try { maburgs::load_config(write_tmp("[link.probe]\nclean_ms = 3000\n")); }
  catch (const std::exception& e) {
    threw = std::string(e.what()).find("clean_ms") != std::string::npos;
  }
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
  // radio.cards is auto-scan by design. link.key (Task 4's optional inline
  // overlay) is read by presence (gs/src/config.cpp), so its absence --
  // the bundle deliberately leaves it unset in favour of key_file -- must
  // NOT register as a defaulted key (fix round 1, Task 4 review): it would
  // otherwise print "link.key=" under every plain boot's defaulted-key
  // list, diluting the real DEFAULT-key warning right next to it.
  CHECK(defaulted.size() == 1);
  CHECK(!defaulted.empty() && defaulted[0] == "radio.cards=(auto-scan)");
}

// spec 2026-09-25-nhm-airtime §6; default blocked_pct is 50, not the spec's
// 30 -- hw spike findings (docs/nhm-airtime-spike-findings-2026-09-25.md).
// Under radio.scan since 2026-10-04: every measurer reads them.
TEST(scan_busy_keys_parse_and_validate) {
  auto c = maburgs::load_config(write_tmp("[radio.scan]\nbusy_dbm = -86\nblocked_pct = 40\n"));
  CHECK(c.radio.scan.busy.busy_dbm == -86 && c.radio.scan.busy.blocked_pct == 40.0);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[radio.scan]\nbusy_dbm = -82\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("radio.scan.busy_dbm") != std::string::npos; }
  CHECK(threw);   // -82 is not an NHM bucket edge
}

// Task 11 (b): the floor on the recovered-symbols impaired term (bench
// session 0232: 97 % of recovered-only impaired windows had <= 8).
// Revert (drop the key from check_keys / the parse): the first load throws
// "unknown key" and the default read is not 8.
TEST(hop_verdict_recovered_min_key) {
  auto c = maburgs::load_config(write_tmp("[hop.verdict]\nrecovered_min = 0\n"));
  CHECK(c.hop.verdict.recovered_min == 0);
  auto d = maburgs::load_config(write_tmp("[hop.verdict]\nrecovered_min = 25\n"));
  CHECK(d.hop.verdict.recovered_min == 25);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[hop.verdict]\nrecovered_min = 1001\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("hop.verdict") != std::string::npos; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[hop.verdict]\nrecovered_min = -1\n")); }
  catch (const std::runtime_error&) { threw = true; }
  CHECK(threw);
  CHECK(maburgs::HopVerdictCfg{}.recovered_min == 8);
}

// Task 12 (e): the AU-rate starved term. Revert (drop the key from
// check_keys / the parse): the first load throws "unknown key".
TEST(hop_verdict_starved_frac_key) {
  auto c = maburgs::load_config(write_tmp("[hop.verdict]\nstarved_frac = 0\n"));
  CHECK(c.hop.verdict.starved_frac == 0.0);
  auto d = maburgs::load_config(write_tmp("[hop.verdict]\nstarved_frac = 0.5\n"));
  CHECK(d.hop.verdict.starved_frac == 0.5);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[hop.verdict]\nstarved_frac = 1.5\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("hop.verdict") != std::string::npos; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[hop.verdict]\nstarved_frac = -0.1\n")); }
  catch (const std::runtime_error&) { threw = true; }
  CHECK(threw);
  CHECK(maburgs::HopVerdictCfg{}.starved_frac == 0.25);
}

// Task 12 (f): the confirm extension while the op reads blocked.
// Revert (drop the key from check_keys / the parse): the first load throws
// "unknown key".
TEST(hop_confirm_extend_ms_key) {
  auto c = maburgs::load_config(write_tmp("[hop]\nconfirm_extend_ms = 0\n"));
  CHECK(c.hop.confirm_extend_ms == 0);
  auto d = maburgs::load_config(write_tmp("[hop]\nconfirm_extend_ms = 30000\n"));
  CHECK(d.hop.confirm_extend_ms == 30000);
  bool threw = false;
  try { maburgs::load_config(write_tmp("[hop]\nconfirm_extend_ms = 30001\n")); }
  catch (const std::runtime_error& e) { threw = std::string(e.what()).find("hop") != std::string::npos; }
  CHECK(threw);
  threw = false;
  try { maburgs::load_config(write_tmp("[hop]\nconfirm_extend_ms = -1\n")); }
  catch (const std::runtime_error&) { threw = true; }
  CHECK(threw);
  CHECK(maburgs::HopCfg{}.confirm_extend_ms == 3000);
}

// ---- Task 4: link.key_file + link.key overlay (spec 2026-10-01-link-pairing §2) ----

TEST(link_key_file_missing_uses_default_and_says_so) {
  auto path = write_tmp("[link]\nkey_file = \"" + std::string(MABUR_TEST_SCRATCH_DIR) +
                        "/gs_absent.key\"\n");
  auto cfg = maburgs::load_config(path);
  CHECK(cfg.link.key_is_default);
  CHECK(cfg.link.key == mabur::kDefaultLinkKey);
  CHECK(cfg.link.key_source == "default");
}

TEST(link_key_file_present_is_loaded_and_bad_fails_boot) {
  const std::string kf = std::string(MABUR_TEST_SCRATCH_DIR) + "/gs_cfg.key";
  { std::ofstream o(kf); o << "# key\n3f9a1c77e04b5d2290ab6ef1c8d34e5a\n"; }
  auto path = write_tmp("[link]\nkey_file = \"" + kf + "\"\n");
  auto cfg = maburgs::load_config(path);
  CHECK(!cfg.link.key_is_default);
  CHECK(mabur::key_to_hex(cfg.link.key) == "3f9a1c77e04b5d2290ab6ef1c8d34e5a");
  CHECK(cfg.link.key_source == kf);
  { std::ofstream o(kf); o << "garbage\n"; }
  const std::string msg = what_of([&] { (void)maburgs::load_config(path); });
  CHECK(msg.find("link.key_file") != std::string::npos);
  CHECK(msg.find(kf) != std::string::npos);
}

TEST(link_vtx_id_is_an_unknown_key_now) {
  auto path = write_tmp("[link]\nvtx_id = 1\n");
  const std::string msg = what_of([&] { (void)maburgs::load_config(path); });
  CHECK(msg.find("link.vtx_id") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
}

TEST(link_key_inline_overrides_key_file) {
  auto path = write_tmp("[link]\nkey_file = \"/nonexistent/x.key\"\nkey = \"3F9A1C77E04B5D2290AB6EF1C8D34E5A\"\n");
  auto cfg = maburgs::load_config(path);
  CHECK(!cfg.link.key_is_default);
  CHECK(mabur::key_to_hex(cfg.link.key) == "3f9a1c77e04b5d2290ab6ef1c8d34e5a");
  CHECK(cfg.link.key_source == "link.key");
  auto bad = write_tmp("[link]\nkey = \"zz\"\n");
  const std::string msg = what_of([&] { (void)maburgs::load_config(bad); });
  CHECK(msg.find("link.key") != std::string::npos);
  CHECK(msg.find("32 hex") != std::string::npos);
}

static std::string write_tmp_at(const std::string& path, const std::string& text) {
  std::ofstream f(path);
  f << text;
  return path;
}
static const char* kBundle = MABUR_GS_BUNDLE_DIR "/maburgs.default.toml";

// Overlay: arrays replace wholesale (the page's ladder is the ladder).
TEST(overlay_replaces_ladder_wholesale) {
  const auto base = maburgs::load_config(kBundle);
  const auto ov = write_tmp_at("/tmp/maburgs_test_overlay.toml",
      "[link]\nmax_mcs = 7\n[[link.ladder]]\nmcs = 2\nbw = 20\n"
      "overhead_base = 0.6\noverhead_enh = 0.3\n");
  const auto c = maburgs::load_config(kBundle, nullptr, ov);
  REQUIRE(c.link.ladder_cfg.ladder.size() == 1);
  CHECK(c.link.ladder_cfg.ladder[0].mcs == 2);
  CHECK(c.link.ladder_cfg.ladder[0].bw == 20);
  CHECK(c.link.ladder_cfg.ladder[0].overhead_base > 0.599 &&
        c.link.ladder_cfg.ladder[0].overhead_base < 0.601);
  // A key the overlay did not name keeps the FILE's value, not the struct default.
  CHECK(c.link.ladder_cfg.down_util == base.link.ladder_cfg.down_util);
  CHECK(c.radio.channels == base.radio.channels);
}

// Overlay: tables merge key-by-key.
TEST(overlay_merges_tables_keywise) {
  const auto base = maburgs::load_config(kBundle);
  const auto ov = write_tmp_at("/tmp/maburgs_test_overlay.toml", "[link]\nstatic_mcs = 3\n");
  const auto c = maburgs::load_config(kBundle, nullptr, ov);
  CHECK(c.link.static_mcs == 3);
  CHECK(c.link.ladder_cfg.ladder.size() == base.link.ladder_cfg.ladder.size());
}

// Overlay: strict keys still apply.
TEST(overlay_unknown_key_fails) {
  const auto ov = write_tmp_at("/tmp/maburgs_test_overlay.toml", "[link]\nbogus = 1\n");
  std::string msg;
  try { maburgs::load_config(kBundle, nullptr, ov); } catch (const std::exception& e) { msg = e.what(); }
  CHECK(msg.find("link.bogus") != std::string::npos);
  CHECK(msg.find("unknown key") != std::string::npos);
}

// Overlay: a bad rung fails with the same field/why a file rung would.
TEST(overlay_bad_rung_fails_like_file) {
  const auto ov = write_tmp_at("/tmp/maburgs_test_overlay.toml",
      "[[link.ladder]]\nmcs = 0\nbw = 40\noverhead_base = 0.2\noverhead_enh = 0.3\n");
  std::string msg;
  try { maburgs::load_config(kBundle, nullptr, ov); } catch (const std::exception& e) { msg = e.what(); }
  CHECK(msg.find("link.ladder[0].overhead_base: must be >= overhead_enh") != std::string::npos);
}

// Overlay: missing file is a config error, not a crash.
TEST(overlay_missing_file_fails) {
  bool threw = false;
  try { maburgs::load_config(kBundle, nullptr, "/tmp/definitely_not_here_overlay.toml"); }
  catch (const std::exception&) { threw = true; }
  CHECK(threw);
}

// radio.relays (spec 2026-10-02-maburgs-remote-card §3): CPE510 relays are
// RemoteCards appended after the USB cards, "ipv4:port" each.
TEST(radio_relays_default_empty_and_parse) {
  auto none = maburgs::load_config(write_tmp(""));
  CHECK(none.radio.relays.empty());
  auto two = maburgs::load_config(write_tmp(
      "[radio]\nrelays = [\"10.83.11.1:8310\", \"10.83.11.2:8310\"]\n"));
  REQUIRE(two.radio.relays.size() == 2);
  CHECK(two.radio.relays[0] == "10.83.11.1:8310");
  CHECK(two.radio.relays[1] == "10.83.11.2:8310");
}

TEST(radio_relays_entries_are_validated) {
  auto w = what_of([] { maburgs::load_config(write_tmp("[radio]\nrelays = [\"10.83.11.1\"]\n")); });
  CHECK(w.find("radio.relays[0]") != std::string::npos && w.find("port") != std::string::npos);
  w = what_of([] { maburgs::load_config(write_tmp("[radio]\nrelays = [\"10.83.11.1:0\"]\n")); });
  CHECK(w.find("radio.relays[0]") != std::string::npos);
  w = what_of([] { maburgs::load_config(write_tmp("[radio]\nrelays = [\"10.0.0.1:8310\", \"10.0.0.1:8310\"]\n")); });
  CHECK(w.find("radio.relays[1]") != std::string::npos && w.find("duplicate") != std::string::npos);
  // A hostname is refused: the UDP transport resolves nothing (getaddrinfo
  // on the core thread every 2 s reopen would stall video on a dead resolver).
  w = what_of([] { maburgs::load_config(write_tmp("[radio]\nrelays = [\"cpe.local:8310\"]\n")); });
  CHECK(w.find("radio.relays[0]") != std::string::npos &&
        w.find("dotted IPv4") != std::string::npos);
  w = what_of([] { maburgs::load_config(write_tmp("[radio]\nrelays = [\"10.83.11:8310\"]\n")); });
  CHECK(w.find("dotted IPv4") != std::string::npos);
  w = what_of([] { maburgs::load_config(write_tmp("[radio]\nrelays = [8310]\n")); });
  CHECK(w.find("radio.relays[0]") != std::string::npos);
}

TEST(tx_card_may_name_a_relay_after_the_explicit_cards) {
  auto cfg = maburgs::load_config(write_tmp(
      "[radio]\ntx_card = 1\nrelays = [\"10.83.11.1:8310\"]\n[[radio.cards]]\nusb_pid = 34842\n"));
  CHECK(cfg.radio.tx_card == 1);   // card 0 = the USB entry, card 1 = the relay
  auto w = what_of([] { maburgs::load_config(write_tmp(
      "[radio]\ntx_card = 2\nrelays = [\"10.83.11.1:8310\"]\n[[radio.cards]]\nusb_pid = 34842\n")); });
  CHECK(w.find("radio.tx_card") != std::string::npos);
}
