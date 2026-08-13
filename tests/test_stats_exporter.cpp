#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "json.hpp"
#include "mabur/profile.h"
#include "mabur/uep_encoder.h"
#include "mtest.h"
#include "stats_exporter.h"
using namespace maburgs;
using nlohmann::json;

namespace {
StatsInput base_input() {
  StatsInput in;
  in.vtx_id = 1;
  in.in_session = true;
  in.tx_card = 0;
  in.op.mcs = 5; in.op.bw = 20; in.op.overhead = 0.25; in.op.snr_req = 18.5;
  in.deadline_ms = 60;
  in.residual_loss = 0.012;
  in.layer_delivery_pct = {100, 100, 97, 91};
  StatsCardIn c;
  c.up = true; c.frames = 1000; c.crc_fail = 12;
  c.seq_expected = 1000; c.seq_received = 996; c.rx_bytes = 1'000'000;
  c.last_frame_us = 999'000;
  c.self_frames = 100; c.foreign = 50;
  c.classes[1].frames = 900; c.classes[1].has_ema = true;  // s1
  c.classes[1].rssi_ema = 59.9; c.classes[1].rssi_a_ema = 59.1; c.classes[1].rssi_b_ema = 57.7;
  c.classes[1].snr_ema = 27.1; c.classes[1].snr_a_ema = 26.0; c.classes[1].snr_b_ema = 24.5;
  c.classes[1].evm_has = true; c.classes[1].evm_a_has = true; c.classes[1].evm_b_has = true;
  c.classes[1].evm_ema = -48.0; c.classes[1].evm_a_ema = -48.0; c.classes[1].evm_b_ema = -44.2;
  c.classes[5].frames = 10; c.classes[5].has_ema = true;  // ctrl
  c.classes[5].rssi_ema = 62.8;
  c.classes[5].snr_ema = 25.0;
  c.tx_fail = 2;
  in.cards.push_back(c);
  in.streams[0].bodies = 500;
  in.streams[0].syms_recovered = 40;
  in.streams[0].symbols_in = 4000;
  in.frames_clean = 100; in.frames_truncated = 1;
  in.ring_published = 5000; in.ring_dropped_oversize = 1; in.ring_bytes = 4'000'000;
  return in;
}

struct Capture {
  std::vector<std::string> sent;
  StatsExporter::SendFn fn() {
    return [this](const std::string& s) { sent.push_back(s); return true; };
  }
  json last() const { return json::parse(sent.back()); }
};
}  // namespace

TEST(first_emission_immediate_with_null_rates) {
  Capture cap;
  StatsExporter ex(0xDEADBEEF, 500, cap.fn());
  CHECK(ex.poll(1000, base_input()));
  REQUIRE(cap.sent.size() == 1);
  const json j = cap.last();
  CHECK(j["v"] == 1);
  CHECK(j["session"] == 0xDEADBEEF);
  CHECK(j["seq"] == 0);
  CHECK(j["t_ms"] == 1000);
  CHECK(j["link"]["video"]["fps"].is_null());        // no window yet
  CHECK(j["cards"][0]["rx_mbps"].is_null());
  CHECK(j["cards"][0]["loss_pct"].is_null());
  CHECK(j["cards"][0]["foreign_pps"].is_null());
  CHECK(j["cards"][0]["self_pps"].is_null());
  // gauges are live even on the first datagram
  CHECK(j["link"]["vtx_id"] == 1);
  CHECK(j["link"]["state"] == "session");
  CHECK(j["link"]["op"]["mcs"] == 5);
  CHECK(j["link"]["deadline_ms"] == 60);
  CHECK(j["cards"][0]["frames"] == 1000);
}

TEST(interval_gate_and_seq) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  CHECK(ex.poll(1000, base_input()));
  CHECK(!ex.poll(1400, base_input()));   // 400 ms < interval
  CHECK(ex.poll(1500, base_input()));    // due
  CHECK(cap.sent.size() == 2);
  CHECK(cap.last()["seq"] == 1);
}

TEST(rates_use_measured_window) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  ex.poll(1000, in);
  in.cards[0].rx_bytes += 250'000;   // +2 Mbit over 1 s -> 2.0 Mbps
  in.cards[0].frames += 500;         // 500 pps
  in.cards[0].seq_expected += 100;
  in.cards[0].seq_received += 98;    // 2% loss
  in.streams[0].syms_recovered += 12;
  in.ring_bytes += 125'000;          // 1.0 Mbps video
  ex.poll(2000, in);                 // 1000 ms window (2x nominal: measured wins)
  const json j = cap.last();
  CHECK(j["cards"][0]["rx_mbps"].get<double>() > 1.99 && j["cards"][0]["rx_mbps"].get<double>() < 2.01);
  CHECK(j["cards"][0]["pps"].get<double>() > 499 && j["cards"][0]["pps"].get<double>() < 501);
  CHECK(j["cards"][0]["loss_pct"].get<double>() > 1.99 && j["cards"][0]["loss_pct"].get<double>() < 2.01);
  CHECK(j["link"]["streams"][0]["recovered_s"].get<double>() > 11.9 && j["link"]["streams"][0]["recovered_s"].get<double>() < 12.1);
  CHECK(j["link"]["video"]["mbps"].get<double>() > 0.99 && j["link"]["video"]["mbps"].get<double>() < 1.01);
}

TEST(recovered_arrived_exported_with_rate) {
  // Repair-vs-arrival race counter (schema-additive under v:1): cumulative on
  // every datagram, windowed rate once a measured window exists.
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  in.streams[0].syms_recovered_arrived = 30;
  ex.poll(1000, in);
  json j = cap.last();
  CHECK(j["link"]["streams"][0]["recovered_arrived"] == 30);
  CHECK(j["link"]["streams"][0]["recovered_arrived_s"].is_null());
  in.streams[0].syms_recovered_arrived += 9;
  ex.poll(2000, in);  // 1 s window -> 9.0/s
  j = cap.last();
  CHECK(j["link"]["streams"][0]["recovered_arrived"] == 39);
  CHECK(j["link"]["streams"][0]["recovered_arrived_s"].get<double>() > 8.9 &&
        j["link"]["streams"][0]["recovered_arrived_s"].get<double>() < 9.1);
}

TEST(loss_pct_null_when_no_expected_and_clamp_negative) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  ex.poll(1000, in);
  ex.poll(1500, in);                 // identical counters: zero deltas
  json j = cap.last();
  CHECK(j["cards"][0]["loss_pct"].is_null());   // delta expected == 0
  in.cards[0].rx_bytes -= 1000;                 // impossible regression
  ex.poll(2000, in);
  j = cap.last();
  CHECK(j["cards"][0]["rx_mbps"].get<double>() == 0.0);  // clamped, not negative
}

TEST(fps_and_jitter_from_on_frame) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  ex.poll(1000, base_input());
  // 60 fps cadence with one 4 ms wobble: intervals 16,16,20 -> D = 0,4
  ex.on_frame(1100); ex.on_frame(1116); ex.on_frame(1132); ex.on_frame(1152);
  ex.poll(1500, base_input());
  json j = cap.last();
  CHECK(j["link"]["video"]["fps"].get<double>() == 8.0);         // 4 frames / 0.5 s
  // J: 0 +(0-0)/16 = 0, then +(4-0)/16 = 0.25
  CHECK(j["link"]["video"]["jitter_ms"].get<double>() > 0.24 && j["link"]["video"]["jitter_ms"].get<double>() < 0.26);
  // >1 s frame gap resets jitter
  ex.on_frame(3000);
  ex.poll(3100, base_input());
  CHECK(cap.last()["link"]["video"]["jitter_ms"].get<double>() == 0.0);
}

TEST(fec_rows_sticky_and_idle_omitted) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();     // only stream 0 has bodies
  ex.poll(1000, in);
  json j = cap.last();
  REQUIRE(j["link"]["streams"].size() == 1);
  CHECK(j["link"]["streams"][0]["stream"] == 0);
  in.streams[2].bodies = 5;         // stream 2 wakes up
  ex.poll(1500, in);
  CHECK(cap.last()["link"]["streams"].size() == 2);
  in.streams[2].bodies = 5;         // no new bodies, but sticky
  ex.poll(2000, in);
  CHECK(cap.last()["link"]["streams"].size() == 2);
}

TEST(null_gauges_before_data) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  in.cards[0].classes[1].has_ema = false;   // s1 has traffic but no ema yet
  in.cards[0].last_frame_us = 0;
  in.residual_loss.reset();
  ex.poll(1000, in);
  const json j = cap.last();
  CHECK(j["cards"][0]["classes"]["s1"]["rssi"].is_null());
  CHECK(j["cards"][0]["classes"]["s1"]["snr_a"].is_null());
  CHECK(j["cards"][0]["last_frame_age_ms"].is_null());
  CHECK(j["link"]["residual_loss"].is_null());
}

TEST(rssi_converted_to_dbm) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  ex.poll(1000, base_input());   // rssi_ema 59.9 raw (class s1)
  const json j = cap.last();
  CHECK(j["cards"][0]["classes"]["s1"]["rssi"].get<double>() > -50.2 &&
        j["cards"][0]["classes"]["s1"]["rssi"].get<double>() < -50.0);
  // snr_ema 27.1 is raw HALF-dB (devourer units) -> 13.55 dB exported.
  CHECK(j["cards"][0]["classes"]["s1"]["snr"].get<double>() > 13.5 &&
        j["cards"][0]["classes"]["s1"]["snr"].get<double>() < 13.6);
}

TEST(evm_exported_in_db_half_db_raw) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  ex.poll(1000, base_input());
  // evm_ema -48 raw half-dB -> -24.0 dB; per-chain likewise.
  const json j = cap.last();
  CHECK(j["cards"][0]["classes"]["s1"]["evm"].get<double>() == -24.0);
  CHECK(j["cards"][0]["classes"]["s1"]["evm_a"].get<double>() == -24.0);
  CHECK(j["cards"][0]["classes"]["s1"]["evm_b"].get<double>() > -22.2 &&
        j["cards"][0]["classes"]["s1"]["evm_b"].get<double>() < -22.0);
}

TEST(evm_null_until_sampled_independent_of_snr) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  in.cards[0].classes[1].evm_has = false;      // snr has_ema stays true
  in.cards[0].classes[1].evm_b_has = false;
  ex.poll(1000, in);
  const json j = cap.last();
  CHECK(j["cards"][0]["classes"]["s1"]["evm"].is_null());
  CHECK(j["cards"][0]["classes"]["s1"]["evm_b"].is_null());
  CHECK(!j["cards"][0]["classes"]["s1"]["evm_a"].is_null());  // A sampled
  CHECK(!j["cards"][0]["classes"]["s1"]["snr"].is_null());    // untouched
}

// devourer's RxAtrib.snr is HALF-dB (LinkHealth.h:49, and RxQuality divides
// by 2). radio_frontend.cpp copies it through untouched, so the exporter is
// the last place it can be corrected -- and the `snr` key already claims dB.
TEST(snr_is_exported_in_dB_not_half_dB) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  StatsClassIn& s0 = in.cards[0].classes[0];  // RfClass s0
  s0.frames = 100;
  s0.has_ema = true;
  s0.rssi_ema = 52.0;   // raw PWDB byte (rssi = raw - 110 dBm) -> -58.0 dBm
  s0.snr_ema = 70.0;    // raw half-dB -> 35.0 dB
  s0.snr_a_ema = 68.0;  // -> 34.0 dB
  s0.snr_b_ema = 72.0;  // -> 36.0 dB
  ex.poll(1000, in);
  const json j = cap.last();
  const json& c = j["cards"][0]["classes"]["s0"];
  CHECK(c["snr"].get<double>() > 34.99 && c["snr"].get<double>() < 35.01);
  CHECK(c["snr_a"].get<double>() > 33.99 && c["snr_a"].get<double>() < 34.01);
  CHECK(c["snr_b"].get<double>() > 35.99 && c["snr_b"].get<double>() < 36.01);
  // RSSI is already dBm and must NOT be touched.
  CHECK(c["rssi"].get<double>() > -58.01 && c["rssi"].get<double>() < -57.99);
}

TEST(send_failure_counted_never_thrown) {
  int calls = 0;
  StatsExporter ex(1, 500, [&](const std::string&) { ++calls; return false; });
  CHECK(!ex.poll(1000, base_input()));   // emitted but send failed -> false
  ex.poll(1500, base_input());
  CHECK(calls == 2);
  CHECK(ex.send_failed() == 2);
}

TEST(tx_and_injection_rates) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  ex.poll(1000, in);
  json j = cap.last();
  CHECK(j["cards"][0]["tx_pps"].is_null());     // first emission
  CHECK(j["cards"][0]["inj_pps"].is_null());
  CHECK(j["cards"][0]["tx_fail"] == 2);         // cumulative, live immediately
  in.cards[0].tx_frames += 10;                  // 20/s over 0.5 s
  in.cards[0].seq_expected += 750;              // drone injected 1500/s
  in.cards[0].seq_received += 748;
  ex.poll(1500, in);
  j = cap.last();
  CHECK(j["cards"][0]["tx_pps"].get<double>() > 19.9 && j["cards"][0]["tx_pps"].get<double>() < 20.1);
  CHECK(j["cards"][0]["inj_pps"].get<double>() > 1499 && j["cards"][0]["inj_pps"].get<double>() < 1501);
}

TEST(stream_rung_phy_and_injection_estimates) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();                  // op: HT mcs5 bw20
  in.streams[1].bodies = 100;                    // activate s1's stream row
  const auto ladder = mabur::rc::ladder_from(mabur::rc::PhyMode::HT, 5, 20);
  ex.poll(1000, in);
  json j = cap.last();
  const json& s0 = j["link"]["streams"][0];
  CHECK(s0["rung_mcs"] == ladder[0].mcs);
  CHECK(s0["rung_ldpc"] == ladder[0].ldpc);
  CHECK(s0["rung_stbc"] == ladder[0].stbc);
  const double want_phy = mabur::rc::phy_rate_mbps(ladder[0]);
  CHECK(s0["phy_mbps"].get<double>() > want_phy - 1e-9 && s0["phy_mbps"].get<double>() < want_phy + 1e-9);
  CHECK(s0["inj_kbps"].is_null());               // first emission
  CHECK(j["link"]["air_pct"].is_null());
  // Window: card0 hears 1 Mbps of s1 with 20% loss -> injected est 1.25 Mbps.
  in.cards[0].classes[1].bytes += 62'500;
  in.cards[0].seq_expected += 500;
  in.cards[0].seq_received += 400;               // 20% card loss this window
  ex.poll(1500, in);
  j = cap.last();
  const double inj = j["link"]["streams"][1]["inj_kbps"].get<double>();
  CHECK(inj > 1240.0 && inj < 1260.0);           // 1000 kbps / 0.8
  const double air = j["link"]["air_pct"].get<double>();
  const double want_air = 100.0 * (1.25 / mabur::rc::phy_rate_mbps(ladder[1]));
  CHECK(air > want_air - 0.1 && air < want_air + 0.1);
}

TEST(class_mbps_windowed) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  ex.poll(1000, in);
  CHECK(cap.last()["cards"][0]["classes"]["s1"]["mbps"].is_null());  // first emission
  in.cards[0].classes[1].bytes += 62'500;   // +0.5 Mbit over 0.5 s -> 1.0 Mbps
  ex.poll(1500, in);
  const double mbps = cap.last()["cards"][0]["classes"]["s1"]["mbps"].get<double>();
  CHECK(mbps > 0.99 && mbps < 1.01);
}

TEST(class_entries_sticky_and_rates) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  ex.poll(1000, in);
  json j = cap.last();
  REQUIRE(j["cards"][0]["classes"].contains("s1"));
  REQUIRE(j["cards"][0]["classes"].contains("ctrl"));
  CHECK(!j["cards"][0]["classes"].contains("s0"));   // never seen -> absent
  CHECK(j["cards"][0]["classes"]["s1"]["pps"].is_null());  // first emission
  in.cards[0].classes[1].frames += 450;              // 900 pps over 0.5 s
  in.cards[0].self_frames += 10;                     // 20/s
  in.cards[0].foreign += 2;                          // 4/s
  ex.poll(1500, in);
  j = cap.last();
  CHECK(j["cards"][0]["classes"]["s1"]["pps"].get<double>() > 899 &&
        j["cards"][0]["classes"]["s1"]["pps"].get<double>() < 901);
  CHECK(j["cards"][0]["self_pps"].get<double>() > 19.9 && j["cards"][0]["self_pps"].get<double>() < 20.1);
  CHECK(j["cards"][0]["foreign_pps"].get<double>() > 3.9 && j["cards"][0]["foreign_pps"].get<double>() < 4.1);
  in.cards[0].classes[1].frames += 0;                // s1 silent this window
  ex.poll(2000, in);
  CHECK(cap.last()["cards"][0]["classes"].contains("s1"));  // sticky
}

TEST(stream_rows_carry_effective_overhead) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();                      // op.overhead = 0.25
  ex.poll(1000, in);
  const json j = cap.last();
  const double ov0 = j["link"]["streams"][0]["ov"].get<double>();
  const double want = mabur::uep_layer_overhead(0, 0.25);
  CHECK(ov0 > want - 1e-9 && ov0 < want + 1e-9);
  CHECK(j["link"]["vtx_id"] == 1);
}
TEST(drone_section_null_then_rates) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  ex.poll(1000, in);
  CHECK(cap.last()["drone"].is_null());
  mabur::rc::Telem t;
  t.tlm_seq = 1; t.state = 2; t.enc_frames = 1000; t.enc_kbytes = 1000;
  t.rcf_rx = 100; t.radio_sent = 5000; t.up_rssi[1] = 52; t.soc_temp_c = 61;
  t.idr_disagree = 1; t.enhance_disagree = 2;
  t.flags = 0x04;  // probing set, failsafe_shed/radio_rx_ok clear
  in.telem = t; in.telem_rx_ms = 1400;
  ex.poll(1500, in);
  json j = cap.last();
  CHECK(j["drone"]["state"] == "linked");
  CHECK(j["drone"]["tlm_age_ms"] == 100);
  CHECK(j["drone"]["enc"]["fps"].is_null());        // one snapshot only
  CHECK(j["drone"]["enc"]["idr_disagree"] == 1);
  CHECK(j["drone"]["enc"]["enhance_disagree"] == 2);
  CHECK(j["drone"]["uplink"]["rssi_b"].get<double>() > -58.1 &&
        j["drone"]["uplink"]["rssi_b"].get<double>() < -57.9);
  CHECK(j["drone"]["failsafe_shed"] == false);
  CHECK(j["drone"]["radio_rx_ok"] == false);
  CHECK(j["drone"]["probing"] == true);
  t.tlm_seq = 2; t.enc_frames = 1060; t.enc_kbytes = 2125;
  t.rcf_rx = 120; t.radio_sent = 6460;
  t.flags = 0;  // probe over -- bit clears
  in.telem = t; in.telem_rx_ms = 2400;               // 1000 ms later
  ex.poll(2500, in);
  j = cap.last();
  CHECK(j["drone"]["enc"]["fps"].get<double>() > 59.9 && j["drone"]["enc"]["fps"].get<double>() < 60.1);
  CHECK(j["drone"]["enc"]["mbps"].get<double>() > 9.1 && j["drone"]["enc"]["mbps"].get<double>() < 9.3);
  CHECK(j["drone"]["rcf"]["rx_pps"].get<double>() > 19.9 && j["drone"]["rcf"]["rx_pps"].get<double>() < 20.1);
  CHECK(j["drone"]["radio"]["sent_pps"].get<double>() > 1459 && j["drone"]["radio"]["sent_pps"].get<double>() < 1461);
  CHECK(j["drone"]["probing"] == false);
  // same tlm_seq again: rates keep the last computed window, age grows
  ex.poll(3000, in);
  CHECK(cap.last()["drone"]["tlm_age_ms"] == 600);
}

// A maburd restart resets tlm_seq/generation/every cumulative counter back
// toward 0. Two normal snapshots establish a rate window; a third snapshot
// whose tlm_seq/generation/counters are all LOWER than the second (the
// restart) must null every telem rate for that poll instead of computing a
// ~4e9-scale garbage delta — and the snapshot after THAT (a fresh, distinct
// pair with the restart as its new baseline) must produce sane rates again.
TEST(telem_restart_nulls_rates_then_recovers) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();

  mabur::rc::Telem t;
  t.tlm_seq = 1; t.generation = 1; t.state = 2;
  t.enc_frames = 1000; t.enc_kbytes = 1000;
  t.rcf_rx = 100; t.radio_sent = 5000;
  in.telem = t; in.telem_rx_ms = 1000;
  ex.poll(1000, in);
  CHECK(cap.last()["drone"]["enc"]["fps"].is_null());   // one snapshot only

  t.tlm_seq = 2; t.enc_frames = 1060; t.enc_kbytes = 2125;
  t.rcf_rx = 120; t.radio_sent = 6460;
  in.telem = t; in.telem_rx_ms = 2000;                  // 1000 ms later
  ex.poll(2500, in);
  json j = cap.last();
  CHECK(j["drone"]["enc"]["fps"].get<double>() > 59.9 && j["drone"]["enc"]["fps"].get<double>() < 60.1);
  CHECK(j["drone"]["enc"]["mbps"].get<double>() > 9.1 && j["drone"]["enc"]["mbps"].get<double>() < 9.3);
  CHECK(j["drone"]["rcf"]["rx_pps"].get<double>() > 19.9 && j["drone"]["rcf"]["rx_pps"].get<double>() < 20.1);
  CHECK(j["drone"]["radio"]["sent_pps"].get<double>() > 1459 && j["drone"]["radio"]["sent_pps"].get<double>() < 1461);

  // Restart: tlm_seq goes backwards (1 < 2), generation regresses (0 < 1),
  // and every cumulative counter drops back near 0.
  t.tlm_seq = 1; t.generation = 0;
  t.enc_frames = 5; t.enc_kbytes = 2;
  t.rcf_rx = 1; t.radio_sent = 10;
  in.telem = t; in.telem_rx_ms = 3000;
  ex.poll(3500, in);
  j = cap.last();
  CHECK(j["drone"]["tlm_seq"] == 1);
  CHECK(j["drone"]["gen"] == 0);
  CHECK(j["drone"]["enc"]["fps"].is_null());            // no garbage rate
  CHECK(j["drone"]["enc"]["mbps"].is_null());
  CHECK(j["drone"]["rcf"]["rx_pps"].is_null());
  CHECK(j["drone"]["radio"]["sent_pps"].is_null());

  // Next distinct snapshot after the restart: a clean pair, sane rates.
  t.tlm_seq = 2; t.enc_frames = 65; t.enc_kbytes = 1002;
  t.rcf_rx = 21; t.radio_sent = 1510;
  in.telem = t; in.telem_rx_ms = 4000;                  // 1000 ms after the restart snapshot
  ex.poll(4500, in);
  j = cap.last();
  CHECK(j["drone"]["enc"]["fps"].get<double>() > 59.9 && j["drone"]["enc"]["fps"].get<double>() < 60.1);
  CHECK(j["drone"]["enc"]["mbps"].get<double>() > 8.1 && j["drone"]["enc"]["mbps"].get<double>() < 8.3);
  CHECK(j["drone"]["rcf"]["rx_pps"].get<double>() > 19.9 && j["drone"]["rcf"]["rx_pps"].get<double>() < 20.1);
  CHECK(j["drone"]["radio"]["sent_pps"].get<double>() > 1499 && j["drone"]["radio"]["sent_pps"].get<double>() < 1501);
}

// Deaf-radio case: the wire's all-zero uplink default (never heard an RC
// frame back from the drone) must render as null, not as a plausible-looking
// -110.0 dBm / 0 dB SNR.
TEST(ctl_null_in_pin_mode) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();          // in.ctl left nullopt (pin mode)
  ex.poll(1000, in);
  CHECK(cap.last()["link"]["ctl"].is_null());
}

TEST(ctl_block_shape_and_values) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  StatsCtlIn ci;
  ci.rung_idx = 3; ci.rung_mcs = 5; ci.rung_ov = 0.25;
  ci.util = 0.08; ci.pre_fec_loss = 0.035; ci.budget = 0.43;
  ci.probation_ms_left = 0;
  ci.penalized = {{5, 8200}};
  ci.demotes_residual = 0; ci.demotes_util = 3; ci.promotes = 4;
  ci.probation_fails = 1; ci.starved_drops = 0; ci.timeout_drops = 1;
  ci.last_event_t_ms = 39243748; ci.last_event_from = 4; ci.last_event_to = 3;
  ci.last_event_reason = "util"; ci.last_event_u = 0.65;
  ci.util3 = 0.07;
  ci.probes_started = 3; ci.probes_ok = 2; ci.probe_fails = 1; ci.probe_aborts = 0;
  ci.demotes_s3_residual = 1; ci.demotes_s3_util = 0;
  ci.last_event_snr_db = 27.5;
  ci.last_event_evm_db = -20.5;
  ci.last_probe_t_ms = 1234; ci.last_probe_rung = 3;
  ci.last_probe_outcome = "fail"; ci.last_probe_snr_db = 24.0;
  ci.last_probe_evm_db = -21.0;
  ci.last_probe_u_pred = 0.9; ci.last_probe_dur_ms = 600;
  in.ctl = ci;
  ex.poll(1000, in);
  const json ctl = cap.last()["link"]["ctl"];
  CHECK(ctl["rung"]["idx"] == 3);
  CHECK(ctl["rung"]["mcs"] == 5);
  CHECK(ctl["rung"]["ov"].get<double>() > 0.249 && ctl["rung"]["ov"].get<double>() < 0.251);
  CHECK(ctl["util"].get<double>() > 0.079 && ctl["util"].get<double>() < 0.081);
  CHECK(ctl["pre_fec_loss"].get<double>() > 0.034 && ctl["pre_fec_loss"].get<double>() < 0.036);
  CHECK(ctl["budget"].get<double>() > 0.429 && ctl["budget"].get<double>() < 0.431);
  CHECK(ctl["probation_ms_left"] == 0);
  REQUIRE(ctl["penalized"].size() == 1);
  CHECK(ctl["penalized"][0]["rung"] == 5);
  CHECK(ctl["penalized"][0]["ms_left"] == 8200);
  CHECK(ctl["counters"]["demotes_residual"] == 0);
  CHECK(ctl["counters"]["demotes_util"] == 3);
  CHECK(ctl["counters"]["promotes"] == 4);
  CHECK(ctl["counters"]["probation_fails"] == 1);
  CHECK(ctl["counters"]["starved_drops"] == 0);
  CHECK(ctl["counters"]["timeout_drops"] == 1);
  CHECK(ctl["last_event"]["t_ms"] == 39243748);
  CHECK(ctl["last_event"]["from"] == 4);
  CHECK(ctl["last_event"]["to"] == 3);
  CHECK(ctl["last_event"]["reason"] == "util");
  CHECK(ctl["last_event"]["u"].get<double>() > 0.649 && ctl["last_event"]["u"].get<double>() < 0.651);
  CHECK(ctl["last_event"]["snr"].get<double>() > 27.49 && ctl["last_event"]["snr"].get<double>() < 27.51);
  CHECK(ctl["last_event"]["evm"].get<double>() > -20.51 && ctl["last_event"]["evm"].get<double>() < -20.49);
  CHECK(ctl["util3"].get<double>() > 0.069 && ctl["util3"].get<double>() < 0.071);
  CHECK(ctl["counters"]["probes_started"] == 3);
  CHECK(ctl["counters"]["probes_ok"] == 2);
  CHECK(ctl["counters"]["probe_fails"] == 1);
  CHECK(ctl["counters"]["probe_aborts"] == 0);
  CHECK(ctl["counters"]["demotes_s3_residual"] == 1);
  CHECK(ctl["counters"]["demotes_s3_util"] == 0);
  REQUIRE(!ctl["last_probe"].is_null());
  CHECK(ctl["last_probe"]["t_ms"] == 1234);
  CHECK(ctl["last_probe"]["rung"] == 3);
  CHECK(ctl["last_probe"]["outcome"] == "fail");
  CHECK(ctl["last_probe"]["snr"].get<double>() > 23.99 && ctl["last_probe"]["snr"].get<double>() < 24.01);
  CHECK(ctl["last_probe"]["evm"].get<double>() > -21.01 && ctl["last_probe"]["evm"].get<double>() < -20.99);
  CHECK(ctl["last_probe"]["u_pred"] == 0.9);
  CHECK(ctl["last_probe"]["dur_ms"] == 600);
}

// last_probe_t_ms == 0 (never probed) must serialize as a null last_probe,
// not an object with zeroed fields -- a consumer would otherwise mistake it
// for a real probe that started at t=0.
TEST(ctl_last_probe_null_when_never_probed) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  StatsCtlIn ci;
  ci.last_probe_t_ms = 0;
  in.ctl = ci;
  ex.poll(1000, in);
  const json ctl = cap.last()["link"]["ctl"];
  CHECK(ctl["last_probe"].is_null());
}

// NaN SNR (no reading known this window) must serialize as JSON null on both
// last_event.snr and last_probe.snr -- never a bare `nan` token, which is not
// valid JSON and breaks every jq-based consumer.
TEST(ctl_snr_nan_is_json_null) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  StatsCtlIn ci;
  ci.last_event_snr_db = std::nan("");
  ci.last_event_evm_db = std::nan("");
  ci.last_probe_t_ms = 500;  // non-zero so last_probe is emitted
  ci.last_probe_snr_db = std::nan("");
  ci.last_probe_evm_db = std::nan("");
  in.ctl = ci;
  ex.poll(1000, in);
  const json ctl = cap.last()["link"]["ctl"];
  CHECK(ctl["last_event"]["snr"].is_null());
  CHECK(ctl["last_event"]["evm"].is_null());
  REQUIRE(!ctl["last_probe"].is_null());
  CHECK(ctl["last_probe"]["snr"].is_null());
  CHECK(ctl["last_probe"]["evm"].is_null());
}

// util3 and last_probe.u_pred (and last_event.u for s3 reasons) can carry a
// 1e9 division-zero-guard sentinel from the controller (unreachable in
// practice, see LadderController::update()). The exporter must clamp it to a
// sane ceiling rather than putting a near-billion float on the wire.
TEST(ctl_util_sentinel_is_clamped) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  StatsCtlIn ci;
  ci.util3 = 1e9;
  ci.last_event_u = 1e9;
  ci.last_probe_t_ms = 700;
  ci.last_probe_u_pred = 1e9;
  in.ctl = ci;
  ex.poll(1000, in);
  const json ctl = cap.last()["link"]["ctl"];
  CHECK(ctl["util3"].get<double>() <= 1e3);
  CHECK(ctl["last_event"]["u"].get<double>() <= 1e3);
  REQUIRE(!ctl["last_probe"].is_null());
  CHECK(ctl["last_probe"]["u_pred"].get<double>() <= 1e3);
}

TEST(ctl_default_event_is_none_with_zeros) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  in.ctl = StatsCtlIn{};              // default-constructed: no event yet
  ex.poll(1000, in);
  const json ctl = cap.last()["link"]["ctl"];
  CHECK(ctl["last_event"]["reason"] == "none");
  CHECK(ctl["last_event"]["t_ms"] == 0);
  CHECK(ctl["last_event"]["from"] == 0);
  CHECK(ctl["last_event"]["to"] == 0);
  CHECK(ctl["last_event"]["u"].get<double>() == 0.0);
  CHECK(ctl["penalized"].empty());
  CHECK(ctl["util3"].get<double>() == 0.0);
  CHECK(ctl["last_probe"].is_null());
  CHECK(ctl["counters"]["probes_started"] == 0);
}

TEST(ctl_ladder_and_thresholds) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  StatsCtlIn ci;
  ci.ladder = {{0, 1.0}, {2, 0.5}, {7, 0.1}};
  ci.down_util = 0.6;
  ci.up_util = 0.15;
  in.ctl = ci;
  CHECK(ex.poll(1000, in));
  const json ctl = cap.last()["link"]["ctl"];
  REQUIRE(ctl["ladder"].size() == 3);
  CHECK(ctl["ladder"][0]["mcs"] == 0);
  CHECK(ctl["ladder"][0]["ov"].get<double>() > 0.999 && ctl["ladder"][0]["ov"].get<double>() < 1.001);
  CHECK(ctl["ladder"][1]["mcs"] == 2);
  CHECK(ctl["ladder"][2]["mcs"] == 7);
  CHECK(ctl["ladder"][2]["ov"].get<double>() > 0.099 && ctl["ladder"][2]["ov"].get<double>() < 0.101);
  CHECK(ctl["down_util"].get<double>() > 0.599 && ctl["down_util"].get<double>() < 0.601);
  CHECK(ctl["up_util"].get<double>() > 0.149 && ctl["up_util"].get<double>() < 0.151);
}

TEST(uplink_nulled_when_both_chains_raw_zero) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  mabur::rc::Telem t;  // default-constructed: up_rssi/up_snr all zero
  in.telem = t; in.telem_rx_ms = 900;
  ex.poll(1000, in);
  const json j = cap.last();
  CHECK(j["drone"]["uplink"]["rssi_a"].is_null());
  CHECK(j["drone"]["uplink"]["rssi_b"].is_null());
  CHECK(j["drone"]["uplink"]["snr_a"].is_null());
  CHECK(j["drone"]["uplink"]["snr_b"].is_null());
}

// The drone's own receiver reads the uplink through the same devourer
// RxAtrib.snr, and telemetry.cpp forwards it raw, so drone.uplink.snr_* had
// the identical half-dB bug cards[].classes[].snr was fixed for. Until this
// landed, ONE datagram carried true dB under one key name and half-dB under
// a near-identical one.
TEST(uplink_snr_is_exported_in_dB_not_half_dB) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  mabur::rc::Telem t;
  t.up_rssi[0] = 52;  t.up_rssi[1] = 47;   // raw byte - 110 -> -58 / -63 dBm
  t.up_snr[0] = 65;   t.up_snr[1] = 60;    // raw half-dB -> 32.5 / 30.0 dB
  in.telem = t; in.telem_rx_ms = 900;
  ex.poll(1000, in);
  // last() returns by value; binding a reference to a SUBOBJECT of that
  // temporary would not extend its lifetime, so hold the whole thing.
  const json j = cap.last();
  const json& u = j["drone"]["uplink"];
  // 32.5 is the point of halving HERE rather than on the drone: the wire
  // field is an int8_t the drone lround()s, so a drone-side halving could
  // only ever yield whole dB.
  CHECK(u["snr_a"].get<double>() > 32.49 && u["snr_a"].get<double>() < 32.51);
  CHECK(u["snr_b"].get<double>() > 29.99 && u["snr_b"].get<double>() < 30.01);
  // Uplink RSSI is already dBm and must NOT be touched.
  CHECK(u["rssi_a"].get<double>() > -58.01 && u["rssi_a"].get<double>() < -57.99);
  CHECK(u["rssi_b"].get<double>() > -63.01 && u["rssi_b"].get<double>() < -62.99);
}
// Runtime TX power control was deleted on 2026-08-12, and with it three
// sideport keys. Nothing else pins their absence: every other assertion here
// checks a key that IS emitted, so re-adding `offset_qdb` to link.op or
// drone.applied would sail through the suite while silently un-doing a
// documented schema removal (CLAUDE.md records it as the one exception to
// the additive-only v:1 rule). Checked against the exporter's real output,
// with a telem snapshot present so drone.applied/drone.sys actually exist —
// against a null drone section these `contains` checks would pass vacuously.
// sys.thermal_delta is asserted PRESENT in the same breath: the sensor and
// its telemetry deliberately survived; only the actuator died.
TEST(removed_power_keys_absent_thermal_delta_kept) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  mabur::rc::Telem t;
  t.tlm_seq = 1; t.state = 2; t.thermal_delta = 3;
  in.telem = t; in.telem_rx_ms = 900;
  ex.poll(1000, in);
  const json j = cap.last();
  REQUIRE(j["link"]["op"].is_object());
  CHECK(!j["link"]["op"].contains("offset_qdb"));
  CHECK(j["link"]["op"]["mcs"] == 5);          // the object is still populated
  REQUIRE(j["drone"].is_object());
  REQUIRE(j["drone"]["applied"].is_object());
  CHECK(!j["drone"]["applied"].contains("offset_qdb"));
  CHECK(!j["drone"]["applied"].contains("derate_qdb"));
  REQUIRE(j["drone"]["sys"].is_object());
  CHECK(j["drone"]["sys"]["thermal_delta"] == 3);
}

TEST(exporter_link_rungs_array) {
  std::string sent;
  maburgs::StatsExporter ex(1, 500,
                             [&](const std::string& s) { sent = s; return true; });
  maburgs::StatsInput in;
  in.ctl.emplace();
  maburgs::StatsRungIn rg;
  rg.mcs = 5;
  rg.ov = 0.25;
  rg.u = 0.0625;
  rg.n = 42;
  rg.age_s = 3.5;
  rg.dwell_s = 120.0;
  rg.visits = 2;
  rg.exits_bad = 1;
  rg.probe_u = 1e9;  // sentinel -> clamped to 1e3 in JSON
  rg.probe_n = 3;
  rg.probe_age_s = -1.0;
  rg.evm_db = std::numeric_limits<double>::quiet_NaN();     // -> null
  rg.evm_sd_db = std::numeric_limits<double>::quiet_NaN();  // -> null
  in.ctl->rungs.push_back(rg);
  ex.poll(500, in);
  ex.poll(1100, in);  // first poll is gated; second emits
  REQUIRE(!sent.empty());
  auto j = nlohmann::json::parse(sent);
  const auto& rungs = j["link"]["rungs"];
  REQUIRE(rungs.is_array());
  REQUIRE(rungs.size() == 1);
  CHECK(rungs[0]["i"] == 0);
  CHECK(rungs[0]["mcs"] == 5);
  CHECK(rungs[0]["n"] == 42);
  CHECK(rungs[0]["evm"].is_null());
  CHECK(rungs[0]["evm_sd"].is_null());
  CHECK(rungs[0]["probe_u"] == 1e3);
  CHECK(rungs[0]["probe_age_s"] == -1.0);
  CHECK(rungs[0]["exits_bad"] == 1);

  // Pin mode (ctl nullopt): no rungs key at all.
  maburgs::StatsInput pin;
  ex.poll(1700, pin);
  auto jp = nlohmann::json::parse(sent);
  CHECK(!jp["link"].contains("rungs"));
}

// drone.enc.{vanished_base,vanished_enh,self_idr_refused}: the venc-ring
// vanish counters (docs/venc-ring-vanish-findings-2026-08-12.md), additive
// under v:1.
// REVERT CHECK: fails if any of the three keys is dropped from the enc block.
TEST(vanish_counters_exported) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  mabur::rc::Telem t;
  t.vanished_base = 2;
  t.vanished_enh = 5;
  t.self_idr_refused = 1;
  in.telem = t;
  ex.poll(1000, in);
  const json enc = cap.last()["drone"]["enc"];
  CHECK(enc["vanished_base"] == 2);
  CHECK(enc["vanished_enh"] == 5);
  CHECK(enc["self_idr_refused"] == 1);
}

MTEST_MAIN
