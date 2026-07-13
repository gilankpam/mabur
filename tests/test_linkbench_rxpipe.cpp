#include "mtest.h"
#include "bench_wire.h"
#include "rx_pipeline.h"
#include "tx_pipeline.h"
using namespace linkbench;

TEST(seq_fwd_delta12_wraps) {
  CHECK(seq_fwd_delta12(5, 6) == 1);
  CHECK(seq_fwd_delta12(4095, 0) == 1);
  CHECK(seq_fwd_delta12(4090, 5) == 11);
  CHECK(seq_fwd_delta12(6, 5) == 4095);  // backwards = huge forward = reorder
}

// Feed a clean TX→RX pass and check every counter.
TEST(rxpipe_clean_pass_counts_everything) {
  FecParams p;
  p.bpb = 12;
  TxPipeline tx(p);
  RxPipeline rx(p);
  std::vector<std::vector<uint8_t>> bodies;
  const uint32_t npkt = 8 * 50;  // 50 full blocks
  for (uint32_t s = 0; s < npkt; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), bodies);
  }
  // RsEncoder seals symbols lazily — flush() completes the final block (see rs_encoder.cpp add_packet)
  tx.flush(bodies);
  const uint8_t rssi[2] = {50, 52};
  const int8_t snr[2] = {25, 26};
  uint16_t mseq = 0;
  for (auto& b : bodies)
    rx.on_body(b.data(), b.size(), mseq++, true, rssi, snr, 1000);
  auto s = rx.snapshot();
  CHECK(s.frames == bodies.size());
  CHECK(s.crc_bad == 0);
  CHECK(s.mac_lost == 0);
  CHECK(s.blocks_decoded == 50);
  CHECK(s.blocks_unrec == 0);
  CHECK(s.pkts == npkt);
  CHECK(s.pkts_expected == npkt);
  CHECK(s.pattern_bad == 0);
  CHECK(s.good_bytes == npkt * 62ull);
  CHECK(s.sig_frames == bodies.size());
  // rxsnr arrives in the vendor's half-dB s(8,1) format; ingestion converts
  // to dB (raw 25/26 → 12.5/13.0 per frame).
  CHECK(s.snr_sum[0] == 12.5 * static_cast<double>(s.sig_frames));
  CHECK(s.snr_sum[1] == 13.0 * static_cast<double>(s.sig_frames));
}

TEST(rxpipe_mac_gap_counts_lost_and_ignores_reorder) {
  FecParams p;
  RxPipeline rx(p);
  // Minimal valid-looking body isn't needed for mac accounting — but
  // on_body filters on stream id first, so build one real body.
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> bodies;
  for (uint32_t s = 0; s < 8 * 4; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), bodies);
  }
  tx.flush(bodies);
  REQUIRE(bodies.size() >= 3);
  const uint8_t rssi[2] = {50, 50};
  const int8_t snr[2] = {20, 20};
  rx.on_body(bodies[0].data(), bodies[0].size(), 10, true, rssi, snr, 1);
  rx.on_body(bodies[1].data(), bodies[1].size(), 14, true, rssi, snr, 2);  // gap of 3
  rx.on_body(bodies[2].data(), bodies[2].size(), 13, true, rssi, snr, 3);  // reorder
  auto s = rx.snapshot();
  CHECK(s.mac_lost == 3);
  CHECK(s.frames == 3);
}

TEST(rxpipe_non_bench_stream_ignored) {
  FecParams p;
  RxPipeline rx(p);
  std::vector<uint8_t> junk(100, 0x55);  // no SBI magic
  const uint8_t rssi[2] = {50, 50};
  const int8_t snr[2] = {20, 20};
  rx.on_body(junk.data(), junk.size(), 1, true, rssi, snr, 1);
  CHECK(rx.snapshot().frames == 0);
}

TEST(rxpipe_expire_counts_unrecoverable) {
  FecParams p;
  p.bpb = 12;
  TxPipeline tx(p);
  RxPipeline rx(p);
  std::vector<std::vector<uint8_t>> bodies;
  for (uint32_t s = 0; s < 8; ++s) {  // one block → one body at bpb=12
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), bodies);
  }
  tx.flush(bodies);
  REQUIRE(bodies.size() == 1);
  // Truncate the body so only 3 sub-blocks survive (< k=8): undecodable.
  auto& b = bodies[0];
  const uint8_t rssi[2] = {50, 50};
  const int8_t snr[2] = {20, 20};
  rx.on_body(b.data(), 7 + 3 * (2 + 75), 0, true, rssi, snr, 1000);
  rx.expire(2000);  // > 300 ms age
  auto s = rx.snapshot();
  CHECK(s.blocks_unrec == 1);
  CHECK(s.pkts == 0);
}

TEST(rxpipe_snapshot_delta_subtracts) {
  RxSnapshot a, b;
  a.frames = 10; a.pkts = 100; a.rssi_sum[0] = 500; a.sig_frames = 10;
  b.frames = 25; b.pkts = 260; b.rssi_sum[0] = 1300; b.sig_frames = 25;
  auto d = snapshot_delta(b, a);
  CHECK(d.frames == 15);
  CHECK(d.pkts == 160);
  CHECK(d.rssi_sum[0] == 800.0);
  CHECK(d.sig_frames == 15);
}

TEST(format_line_and_json_render) {
  RxSnapshot d;
  d.frames = 1352; d.air_bytes = 1227500; d.mac_lost = 29;
  d.blocks_decoded = 1733; d.blocks_unrec = 2;
  d.pkts = 15987; d.pkts_expected = 16132; d.good_bytes = 991194;
  d.rssi_sum[0] = 1352 * 52.0; d.rssi_sum[1] = 1352 * 49.0;
  d.snr_sum[0] = 1352 * 28.0; d.snr_sum[1] = 1352 * 26.0;
  d.sig_frames = 1352;
  auto line = format_line(5, d);
  CHECK(line.find("air 9.82M") != std::string::npos);
  CHECK(line.find("good 7.93M") != std::string::npos);
  CHECK(line.find("frm 1352") != std::string::npos);
  CHECK(line.find("unrec 2") != std::string::npos);
  CHECK(line.find("rssi -58/-61") != std::string::npos);
  CHECK(line.find("snr 28/26") != std::string::npos);
  auto js = format_json(5, d);
  CHECK(js.find("\"t\":5") != std::string::npos);
  CHECK(js.find("\"frames\":1352") != std::string::npos);
  CHECK(js.find("\"blocks_unrec\":2") != std::string::npos);
}

MTEST_MAIN
