#include <cmath>
#include <cstdio>
#include "mtest.h"
#include "telemetry.h"
#include "mabur/rc_proto.h"

TEST(make_telem_maps_and_saturates) {
  mabur::TelemInputs in;
  in.state = 2; in.failsafe_shed = true;
  in.congestion_shed = true;
  in.low_power = true;
  in.rcf_age_ms = 700000;            // saturates u16
  in.usb_fail = 1 << 20;             // saturates u16
  in.txq_drops = 1ull << 33;         // saturates u32
  in.txq_wait_max_ms = 70000;        // saturates u16
  in.cmd_kbps = 90000;               // saturates u16
  in.uplink.has = true;
  in.uplink.rssi[0] = 51.4; in.uplink.rssi[1] = 52.0;
  in.uplink.snr[0] = 21.2; in.uplink.snr[1] = 22.0;
  in.soc_temp_c = 61; in.cpu_pct = 14.267;
  in.rcf_seq_echo = 0x4711;
  in.rcf_seq_echo_valid = true;
  in.pts_at_build_us = 0x0011223344556677ull;
  in.rx_crcfail = 70000;             // saturates u16
  in.rx_own = 6; in.rx_foreign = 7;
  in.rec_status = 0x16;
  const auto t = mabur::make_telem(9, in);
  CHECK(t.tlm_seq == 9);
  CHECK(t.state == 2);
  // failsafe_shed | rcf_seq_echo_valid | congestion_shed — bit3 is the GS's only way to tell "aging against this
  // seq" from "aging against a DISC/failsafe rebase where the echoed seq is
  // stale"; bit4 is the TxQueue-pressure / USB-failure shed
  // (RcAgent::run_congestion_guard), distinct from bit0's failsafe shed so
  // a bench can count congestion sheds and flightreport can attribute an
  // enh gap to congestion rather than RF.
  // Bits 1/2/5 (radio_rx_ok / probe_on / air_shed) are gone since
  // 2026-09-30 and must stay clear.
  CHECK(t.flags == 0x99);  // | low_power (bit7, spec 2026-09-20)
  CHECK(t.rcf_age_ms == 65535);
  CHECK(t.usb_fail == 65535);
  CHECK(t.txq_drops == 0xFFFFFFFFu);
  CHECK(t.txq_wait_max_ms == 65535);
  CHECK(t.cmd_kbps == 65535);
  CHECK(t.up_rssi[0] == 51);         // rounded raw
  CHECK(t.up_snr[1] == 22);
  CHECK(t.soc_temp_c == 61);
  CHECK(t.cpu_busy_x100 == 1427);
  // link-rtt: seq echo + pts pass through at full width, no saturation —
  // pts_at_build is a timestamp, not a gauge.
  CHECK(t.rcf_seq_echo == 0x4711);
  CHECK(t.pts_at_build == 0x0011223344556677ull);
  // RX frame split per telemetry period (cca-on 2026-09-23): saturating u16s.
  CHECK(t.rx_crcfail == 65535);
  CHECK(t.rx_own == 6);
  CHECK(t.rx_foreign == 7);
  CHECK(t.rec_status == 0x16);
}

TEST(low_power_flag_round_trips) {
  mabur::TelemInputs in;
  in.low_power = true;
  auto t = mabur::make_telem(1, in);
  CHECK((t.flags & 0x80) != 0);
  auto wire = mabur::rc::pack_telem(t);
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK((back->flags & 0x80) != 0);
  in.low_power = false;
  CHECK((mabur::make_telem(2, in).flags & 0x80) == 0);
}

// Link pairing (spec 2026-10-01 §8): a control frame that failed its tag
// since the last Telem sets flags bit1, so the GS can show it.
TEST(auth_reject_is_flags_bit1) {
  mabur::TelemInputs in;
  in.auth_reject = true;
  CHECK((mabur::make_telem(1, in).flags & 0x02) != 0);
  in.auth_reject = false;
  CHECK((mabur::make_telem(2, in).flags & 0x02) == 0);
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
  mabur::CpuBusySampler cpu;
  CHECK(!cpu.sample("/nonexistent"));
}

// drone.sys.load was /proc/loadavg[0], which on the SigmaStar image counts
// the SDK's parked D-state workers and read a flat ~13 idle or pegged
// (docs/dq-spike-findings-2026-08-31.md). Replaced 2026-09-21 by the CPU
// busy fraction of the tick, from a /proc/stat delta: bench read loadavg
// 12.08 against 14 % busy over the same 2 s.
TEST(cpu_busy_is_a_proc_stat_delta_not_loadavg) {
  const char* p = "/tmp/mabur_test_stat";
  auto write = [&](const char* line) {
    FILE* f = std::fopen(p, "w");
    std::fprintf(f, "%s\nintr 1 2 3\n", line); std::fclose(f);
  };
  mabur::CpuBusySampler cpu;
  // The live drone's two reads, 2 s apart: user+16, system+38, idle+329.
  write("cpu  8968 0 14081 126891 18 0 394 0 0 0");
  CHECK(!cpu.sample(p));  // no baseline yet
  write("cpu  8984 0 14119 127220 18 0 394 0 0 0");
  auto v = cpu.sample(p);
  REQUIRE(v);
  CHECK(std::abs(*v - 100.0 * 54.0 / 383.0) < 1e-9);  // 14.1 %, not 12.08
  // iowait counts as idle; irq/softirq/steal count as busy.
  write("cpu  8984 0 14119 127220 118 5 404 1 0 0");
  v = cpu.sample(p);
  REQUIRE(v);
  CHECK(std::abs(*v - 100.0 * 16.0 / 116.0) < 1e-9);
  // No time passed: nothing to divide by -> unavailable, not NaN/garbage.
  CHECK(!cpu.sample(p));
  // A vanished/garbled file drops the baseline: the next good read is a
  // fresh first sample, never a delta against stale counters.
  write("cpuX garbage");
  CHECK(!cpu.sample(p));
  write("cpu  9000 0 14200 127500 118 5 404 1 0 0");
  CHECK(!cpu.sample(p));
  write("cpu  9010 0 14210 127580 118 5 404 1 0 0");
  v = cpu.sample(p);
  REQUIRE(v);
  CHECK(std::abs(*v - 100.0 * 20.0 / 100.0) < 1e-9);
  // Wire: unavailable is 65535, never 0 -- 0 is a real idle reading.
  mabur::TelemInputs in;
  CHECK(mabur::make_telem(1, in).cpu_busy_x100 == 65535);
  in.cpu_pct = 0.0;
  CHECK(mabur::make_telem(1, in).cpu_busy_x100 == 0);
  in.cpu_pct = 100.0;
  CHECK(mabur::make_telem(1, in).cpu_busy_x100 == 10000);
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


MTEST_MAIN
