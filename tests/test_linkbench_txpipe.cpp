#include "mtest.h"
#include "bench_wire.h"
#include "tx_pipeline.h"
#include "mabur/sbi.h"
using namespace linkbench;

// SwEncoder seals a symbol LAZILY when the next packet doesn't fit — the
// Nth max-size packet doesn't complete a symbol until the (N+1)th packet
// arrives and finds no room. So N packets of max size yield N-1 source
// envelopes; the Nth stays buffered in current_symbol_ until flush() (or the
// next packet) seals it (common/src/sw_encoder.cpp add_packet/seal_current).
TEST(txpipe_lazy_seal_yields_n_minus_1_sources) {
  FecParams p;  // overhead=0.5 symbol_size=64 window=128 → max_packet_size 62
  p.overhead = 0.0;  // isolate source counting from credited repairs
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  const uint32_t n = 9;
  for (uint32_t s = 0; s < n; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  CHECK(tx.sources_sent() == n - 1);
  CHECK(tx.repairs_sent() == 0);
}

// flush() seals whatever partial symbol is pending (the final source) and,
// since a seal happened since the last repair, emits exactly one tail
// repair — even at overhead 0 (SwEncoder::flush: tail_repair_pending_ is
// set by every seal_current() and cleared only by an actual repair).
TEST(txpipe_flush_yields_final_source_and_tail_repair) {
  FecParams p;
  p.overhead = 0.0;
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  const uint32_t n = 9;
  for (uint32_t s = 0; s < n; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  CHECK(tx.sources_sent() == n - 1);
  tx.flush(out);
  CHECK(tx.sources_sent() == n);       // the 9th packet's symbol sealed
  CHECK(tx.repairs_sent() == 1);       // one tail repair, despite overhead 0
}

// overhead 0.5 credits one repair every 2 seals: 8 seals -> exactly 4
// repairs. Reach exactly 8 seals via flush() (9 packets: 8 lazy-sealed in
// the loop, the 9th sealed by flush) — the 8th seal's credited repair
// clears tail_repair_pending_, but flush() itself performs a 9th seal (the
// pending packet), which re-arms the flag and adds its own tail repair. A
// SECOND, idle flush() (nothing pending) must then add nothing more.
TEST(txpipe_overhead_half_credits_repair_every_two_seals) {
  FecParams p;
  CHECK(p.overhead == 0.5);
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  // 9 packets of max size -> 8 seals (lazy seal: Nth packet seals symbol
  // N-1), leaving the 9th packet's symbol pending.
  const uint32_t n = 9;
  for (uint32_t s = 0; s < n; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  CHECK(tx.sources_sent() == 8);
  CHECK(tx.repairs_sent() == 4);
  tx.flush(out);
  CHECK(tx.sources_sent() == 9);       // flush sealed the 9th packet's symbol
  CHECK(tx.repairs_sent() == 5);       // flush's own tail repair for that seal
  tx.flush(out);                       // idle: nothing pending, no new seal
  CHECK(tx.sources_sent() == 9);
  CHECK(tx.repairs_sent() == 5);       // tail_repair_pending_ stays cleared
}

// Bodies form once bpb envelopes accumulate (SbiPacker), independent of the
// source/repair split — a run producing bpb-worth of envelopes yields
// exactly one body.
TEST(txpipe_body_forms_when_bpb_envelopes_accumulate) {
  FecParams p;
  p.bpb = 12;
  p.overhead = 0.0;  // envelope count == source count, easy to hit bpb exactly
  TxPipeline tx(p);
  std::vector<std::vector<uint8_t>> out;
  // 13 packets -> 12 sources (lazy seal), landing exactly on bpb=12.
  for (uint32_t s = 0; s < 13; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), out);
  }
  REQUIRE(out.size() == 1);
  CHECK(tx.sources_sent() == 12);
  // 11-byte SBI header + 12 × (2-byte crc + envelope_len bytes)
  CHECK(out[0].size() == 11u + 12u * (2u + static_cast<size_t>(p.envelope_len())));
  CHECK(mabur::sbi_peek_stream_id(out[0].data(), out[0].size()) == kBenchStreamId);
}

TEST(txpipe_flush_drains_everything) {
  FecParams p;
  p.bpb = 12;
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
    auto r = mabur::sbi_unpack(b.data(), b.size(), p.envelope_len());
    CHECK(r.header_ok);
    envs += r.survivors.size();
  }
  CHECK(envs == tx.sources_sent() + tx.repairs_sent());
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
