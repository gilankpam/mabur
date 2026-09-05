#include <cmath>
#include <cstdio>
#include "mtest.h"
#include "telemetry.h"
#include "mabur/rc_proto.h"

TEST(make_telem_maps_and_saturates) {
  mabur::TelemInputs in;
  in.state = 2; in.failsafe_shed = true; in.radio_rx_ok = true; in.probe_on = true;
  in.congestion_shed = true;
  in.roi_qp = -24;
  in.generation = 7; in.mode = mabur::rc::PhyMode::HT; in.mcs = 5; in.bw = 20;
  in.applied_ov_base = 0.25;
  in.applied_ov_enh = 0.4;
  in.rcf_age_ms = 700000;            // saturates u16
  in.enc_bytes = 5ull << 30;         // 5 GiB -> kbytes fits u32
  in.ring_drops = 1 << 20;           // saturates u16
  in.usb_fail = 1 << 20;             // saturates u16
  in.txq_depth = 300;                // saturates u8
  in.uplink.has = true;
  in.uplink.rssi[0] = 51.4; in.uplink.rssi[1] = 52.0;
  in.uplink.snr[0] = 21.2; in.uplink.snr[1] = 22.0;
  in.soc_temp_c = 61; in.load1 = 0.72;
  in.idr_disagree = 3; in.enhance_disagree = 70000;
  in.rcf_seq_echo = 0x4711;
  in.rcf_seq_echo_valid = true;
  in.pts_at_build_us = 0x0011223344556677ull;
  const auto t = mabur::make_telem(9, in);
  CHECK(t.tlm_seq == 9);
  CHECK(t.state == 2);
  // failsafe_shed | radio_rx_ok | probe_on | rcf_seq_echo_valid |
  // congestion_shed — bit3 is the GS's only way to tell "aging against this
  // seq" from "aging against a DISC/failsafe rebase where the echoed seq is
  // stale"; bit4 is the TxQueue-pressure / USB-failure shed
  // (RcAgent::run_congestion_guard), distinct from bit0's failsafe shed so
  // a bench can count congestion sheds and flightreport can attribute an
  // enh gap to congestion rather than RF.
  CHECK(t.flags == 0x1F);
  // roi_qp is the ROI override RcAgent commanded (signed). It was exported
  // as an unsigned `qp` until 2026-09-03; there is no encoder QP on the wire.
  CHECK(t.roi_qp == -24);
  CHECK(t.applied_profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 5, 20));
  // applied_ov_base/enh map straight through — the commanded per-stream
  // pair, or the debug-HTTP override when armed (main.cpp).
  CHECK(std::abs(t.applied_ov_base - 0.25) < 1e-9);
  CHECK(std::abs(t.applied_ov_enh - 0.4) < 1e-9);
  CHECK(t.rcf_age_ms == 65535);
  CHECK(t.enc_kbytes == (5ull << 30) / 1024);
  CHECK(t.ring_drops == 65535);
  CHECK(t.usb_fail == 65535);
  CHECK(t.txq_depth == 255);
  CHECK(t.up_rssi[0] == 51);         // rounded raw
  CHECK(t.up_snr[1] == 22);
  CHECK(t.soc_temp_c == 61);
  CHECK(t.load_x100 == 72);
  CHECK(t.idr_disagree == 3);
  CHECK(t.enhance_disagree == 65535);  // saturates to u16
  // link-rtt: seq echo + pts pass through at full width, no saturation —
  // pts_at_build is a timestamp, not a gauge.
  CHECK(t.rcf_seq_echo == 0x4711);
  CHECK(t.pts_at_build == 0x0011223344556677ull);
}

TEST(uplink_track_ema_and_thread_snapshot) {
  mabur::UplinkTrack u;
  CHECK(!u.snap().has);
  const uint8_t r1[2] = {50, 60}; const int8_t s1[2] = {20, 30};
  u.on_rc_frame(r1, s1);
  CHECK(u.snap().rssi[1] == 60.0);   // seeded
  const uint8_t r2[2] = {60, 50}; const int8_t s2[2] = {30, 20};
  u.on_rc_frame(r2, s2);
  CHECK(u.snap().rssi[1] > 58.9 && u.snap().rssi[1] < 59.1);  // 0.9*60+0.1*50
}

TEST(sys_readers_fail_soft) {
  CHECK(mabur::read_soc_temp_c("/nonexistent") == -128);
  CHECK(mabur::read_soc_temp_c_sigmastar("/nonexistent") == -128);
  CHECK(mabur::read_load1("/nonexistent") == 0.0);
}

TEST(soc_temp_formats) {
  // Standard zone: millidegrees. SigmaStar cpufreq: "Temp=NN" already in C.
  {
    FILE* f = std::fopen("/tmp/mabur_test_thermal", "w");
    std::fprintf(f, "53000\n"); std::fclose(f);
    CHECK(mabur::read_soc_temp_c("/tmp/mabur_test_thermal") == 53);
  }
  {
    FILE* f = std::fopen("/tmp/mabur_test_sstar", "w");
    std::fprintf(f, "Temp=53\n"); std::fclose(f);
    CHECK(mabur::read_soc_temp_c_sigmastar("/tmp/mabur_test_sstar") == 53);
    // wrong format for each reader -> unavailable, not garbage
    CHECK(mabur::read_soc_temp_c_sigmastar("/tmp/mabur_test_thermal") == -128);
  }
}


TEST(vanish_counters_saturate_and_round_trip) {
  // venc-ring vanish detection (docs/venc-ring-vanish-findings-2026-08-12.md):
  // the pipeline's u64 counters ride the wire as saturating u16.
  mabur::TelemInputs in;
  in.vanished_base = 70000;
  in.vanished_enh = 80000;
  in.self_idr_refused = 90000;
  auto t = mabur::make_telem(1, in);
  CHECK(t.vanished_base == 65535);
  CHECK(t.vanished_enh == 65535);
  CHECK(t.self_idr_refused == 65535);

  in.vanished_base = 3;
  in.vanished_enh = 5;
  in.self_idr_refused = 2;
  auto wire = mabur::rc::pack_telem(mabur::make_telem(2, in));
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK(back->vanished_base == 3);
  CHECK(back->vanished_enh == 5);
  CHECK(back->self_idr_refused == 2);
}

TEST(venc_ring_stats_clamp_and_round_trip) {
  // venc_get_stats() -> wire (spec 2026-08-28 venc-foldin, Task B6):
  // full_drops is a lifetime u64 riding as a saturating u16; fill_pct is
  // CLAMPED to the documented 0..100, not merely saturated to u8.
  // REVERT CHECK: drop the std::clamp in make_telem and the 100/0 cases below
  // report 200 and 255 respectively.
  mabur::TelemInputs in;
  in.venc_full_drops = 70000;
  in.venc_ring_fill_pct = 200;
  auto t = mabur::make_telem(1, in);
  CHECK(t.venc_full_drops == 65535);
  CHECK(t.venc_ring_fill_pct == 100);

  in.venc_ring_fill_pct = -5;
  CHECK(mabur::make_telem(1, in).venc_ring_fill_pct == 0);

  in.venc_full_drops = 12;
  in.venc_ring_fill_pct = 73;
  auto wire = mabur::rc::pack_telem(mabur::make_telem(2, in));
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK(back->venc_full_drops == 12);
  CHECK(back->venc_ring_fill_pct == 73);
}

// Air clock (spec 2026-09-06 §4.6): window-max backlog + drop counter ride
// the wire as saturating u16; flags bit5 = at least one enh AU dropped by
// the air clock in this telemetry window (distinct from bit0 failsafe and
// bit4 congestion so the bench can tell the three sheds apart).
TEST(air_clock_fields_saturate_and_round_trip) {
  mabur::TelemInputs in;
  in.air_shed = true;
  in.air_backlog_max_ms = 70000;
  in.air_shed_drops = 80000;
  auto t = mabur::make_telem(1, in);
  CHECK((t.flags & 0x20) != 0);
  CHECK(t.air_backlog_max_ms == 65535);
  CHECK(t.air_shed_drops == 65535);

  in.air_shed = false;
  in.air_backlog_max_ms = 37;
  in.air_shed_drops = 12;
  auto wire = mabur::rc::pack_telem(mabur::make_telem(2, in));
  CHECK(wire.size() == 87 + 2);   // TELEM_LEN + crc16
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK((back->flags & 0x20) == 0);
  CHECK(back->air_backlog_max_ms == 37);
  CHECK(back->air_shed_drops == 12);
  CHECK(back->venc_ring_fill_pct == 0);   // the field before ours still parses
}

MTEST_MAIN
