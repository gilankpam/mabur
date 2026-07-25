#include <string>
#include <vector>
#include "json.hpp"
#include "mtest.h"
#include "stats_exporter.h"
using namespace maburgs;
using nlohmann::json;

namespace {
StatsInput base_input() {
  StatsInput in;
  in.in_session = true;
  in.tx_card = 0;
  in.op.mcs = 5; in.op.bw = 20; in.op.overhead = 0.25; in.op.snr_req = 18.5;
  in.deadline_ms = 60;
  in.residual_loss = 0.012;
  in.layer_delivery_pct = {100, 100, 97, 91};
  StatsCardIn c;
  c.up = true; c.frames = 1000; c.crc_fail = 12;
  c.seq_expected = 1000; c.seq_received = 996; c.rx_bytes = 1'000'000;
  c.has_ema = true;
  c.rssi_ema = 59.9; c.rssi_a_ema = 59.1; c.rssi_b_ema = 57.7;
  c.snr_ema = 27.1; c.snr_a_ema = 26.0; c.snr_b_ema = 24.5;
  c.last_frame_us = 999'000;
  in.cards.push_back(c);
  in.streams[0].bodies = 500;
  in.streams[0].syms_recovered = 40;
  in.streams[0].symbols_in = 4000;
  in.frames_clean = 100; in.frames_truncated = 1;
  in.rtp_ok = 5000; in.udp_sent = 5001; in.udp_bytes = 4'000'000;
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
  CHECK(j["video"]["fps"].is_null());        // no window yet
  CHECK(j["cards"][0]["rx_mbps"].is_null());
  CHECK(j["cards"][0]["loss_pct"].is_null());
  // gauges are live even on the first datagram
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
  in.udp_bytes += 125'000;           // 1.0 Mbps video
  ex.poll(2000, in);                 // 1000 ms window (2x nominal: measured wins)
  const json j = cap.last();
  CHECK(j["cards"][0]["rx_mbps"].get<double>() > 1.99 && j["cards"][0]["rx_mbps"].get<double>() < 2.01);
  CHECK(j["cards"][0]["pps"].get<double>() > 499 && j["cards"][0]["pps"].get<double>() < 501);
  CHECK(j["cards"][0]["loss_pct"].get<double>() > 1.99 && j["cards"][0]["loss_pct"].get<double>() < 2.01);
  CHECK(j["fec"][0]["recovered_s"].get<double>() > 11.9 && j["fec"][0]["recovered_s"].get<double>() < 12.1);
  CHECK(j["video"]["mbps"].get<double>() > 0.99 && j["video"]["mbps"].get<double>() < 1.01);
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
  CHECK(j["video"]["fps"].get<double>() == 8.0);         // 4 frames / 0.5 s
  // J: 0 +(0-0)/16 = 0, then +(4-0)/16 = 0.25
  CHECK(j["video"]["jitter_ms"].get<double>() > 0.24 && j["video"]["jitter_ms"].get<double>() < 0.26);
  // >1 s frame gap resets jitter
  ex.on_frame(3000);
  ex.poll(3100, base_input());
  CHECK(cap.last()["video"]["jitter_ms"].get<double>() == 0.0);
}

TEST(fec_rows_sticky_and_idle_omitted) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();     // only stream 0 has bodies
  ex.poll(1000, in);
  json j = cap.last();
  REQUIRE(j["fec"].size() == 1);
  CHECK(j["fec"][0]["stream"] == 0);
  in.streams[2].bodies = 5;         // stream 2 wakes up
  ex.poll(1500, in);
  CHECK(cap.last()["fec"].size() == 2);
  in.streams[2].bodies = 5;         // no new bodies, but sticky
  ex.poll(2000, in);
  CHECK(cap.last()["fec"].size() == 2);
}

TEST(null_gauges_before_data) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  StatsInput in = base_input();
  in.cards[0].has_ema = false;
  in.cards[0].last_frame_us = 0;
  in.residual_loss.reset();
  ex.poll(1000, in);
  const json j = cap.last();
  CHECK(j["cards"][0]["rssi"].is_null());
  CHECK(j["cards"][0]["snr_a"].is_null());
  CHECK(j["cards"][0]["last_frame_age_ms"].is_null());
  CHECK(j["link"]["residual_loss"].is_null());
}

TEST(rssi_converted_to_dbm) {
  Capture cap;
  StatsExporter ex(1, 500, cap.fn());
  ex.poll(1000, base_input());   // rssi_ema 59.9 raw
  const json j = cap.last();
  CHECK(j["cards"][0]["rssi"].get<double>() > -50.2 && j["cards"][0]["rssi"].get<double>() < -50.0);
  CHECK(j["cards"][0]["snr"].get<double>() > 27.0 && j["cards"][0]["snr"].get<double>() < 27.2);  // snr NOT shifted
}

TEST(send_failure_counted_never_thrown) {
  int calls = 0;
  StatsExporter ex(1, 500, [&](const std::string&) { ++calls; return false; });
  CHECK(!ex.poll(1000, base_input()));   // emitted but send failed -> false
  ex.poll(1500, base_input());
  CHECK(calls == 2);
  CHECK(ex.send_failed() == 2);
}
MTEST_MAIN
