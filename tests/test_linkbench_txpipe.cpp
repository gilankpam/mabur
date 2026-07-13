#include "mtest.h"
#include "bench_wire.h"
#include "tx_pipeline.h"
#include "mabur/sbi.h"
using namespace linkbench;

// One RS block completes after 9 packets of max size (one packet per symbol).
// RsEncoder seals a symbol lazily when the next packet doesn't fit, so the
// 9th packet completes the 8-symbol block. n envelopes then need n/bpb bodies
// (bpb 12 = n → exactly one).
TEST(txpipe_emits_one_body_per_block_at_bpb_n) {
  FecParams p;  // k=8 ov=0.5 ss=64 → n=12
  p.bpb = 12;
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  for (uint32_t s = 0; s < 9; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  REQUIRE(out.size() == 1);
  CHECK(tx.blocks_encoded() == 1);
  // 7-byte SBI header + 12 × (2-byte crc + 75-byte envelope)
  CHECK(out[0].size() == 7u + 12u * (2u + 75u));
  CHECK(mabur::sbi_peek_stream_id(out[0].data(), out[0].size()) == kBenchStreamId);
}

// With interleave depth 12 the packer holds bodies back until 12 blocks are
// pending, then emits 12 bodies each carrying one envelope from each block.
// 89 packets encode 11 blocks (packet 88 completes block 11, with 0-87 sealing
// blocks 0-10). Packet 96 completes block 12.
TEST(txpipe_interleave_defers_then_emits_rounds) {
  FecParams p;
  p.bpb = 12;
  p.interleave = 12;
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  for (uint32_t s = 0; s < 89; ++s) {  // 89 packets = 11 full blocks: below depth
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  CHECK(out.empty());
  for (uint32_t s = 89; s < 97; ++s) {  // 8 more packets complete the 12th block
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  CHECK(out.size() == 12);
}

// flush() drains a partial block and (with interleave) sub-depth rounds as
// short bodies — nothing may remain buffered. 20 packets seal 19 symbols: 2
// full blocks (16 symbols) + partial 3rd block (3 sealed). flush() pads to k=8
// and encodes the partial block.
TEST(txpipe_flush_drains_everything) {
  FecParams p;
  p.bpb = 12;
  p.interleave = 12;
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  for (uint32_t s = 0; s < 20; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  tx.flush(out);
  REQUIRE(!out.empty());
  size_t envs = 0;
  for (auto& b : out) {
    auto r = mabur::sbi_unpack(b.data(), b.size(), 75);
    CHECK(r.header_ok);
    envs += r.survivors.size();
  }
  CHECK(envs == 3u * 12u);  // 3 blocks (last one padded) × n envelopes
}

TEST(txpipe_oversize_packet_counted_not_crashed) {
  FecParams p;
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  std::vector<uint8_t> big(200, 0xAB);  // > max_packet_size (62)
  tx.add_packet(big.data(), big.size(), out);
  CHECK(out.empty());
  CHECK(tx.oversize_drops() == 1);
}

MTEST_MAIN
