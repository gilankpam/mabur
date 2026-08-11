#include <cstdio>
#include "mtest.h"
#include "telemetry.h"
#include "mabur/rc_proto.h"

TEST(make_telem_maps_and_saturates) {
  mabur::TelemInputs in;
  in.state = 2; in.failsafe_shed = true; in.radio_rx_ok = true; in.probing = true;
  in.generation = 7; in.mode = mabur::rc::PhyMode::HT; in.mcs = 5; in.bw = 20;
  in.applied_ov = 0.25; in.applied_off_qdb = -4; in.derate_qdb = 2;
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
  const auto t = mabur::make_telem(9, in);
  CHECK(t.tlm_seq == 9);
  CHECK(t.state == 2);
  CHECK(t.flags == 0x07);  // failsafe_shed | radio_rx_ok | probing
  CHECK(t.applied_profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 5, 20));
  CHECK(t.applied_ov_x100 == 25);
  CHECK(t.applied_off_qdb == 60);    // bias-64
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

TEST(idr_grants_saturates_and_round_trips) {
  mabur::TelemInputs in;
  in.idr_grants = 70000;
  auto t = mabur::make_telem(1, in);
  CHECK(t.idr_grants == 65535);

  in.idr_grants = 4;
  auto wire = mabur::rc::pack_telem(mabur::make_telem(2, in));
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK(back->idr_grants == 4);
}

MTEST_MAIN
