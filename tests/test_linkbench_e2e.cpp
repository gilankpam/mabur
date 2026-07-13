// Host end-to-end: TxPipeline → simulated lossy channel → RxPipeline, no
// USB. Loss below the FEC budget must be invisible post-FEC; loss above it
// must surface as unrecoverable blocks; a config mismatch must surface as
// sym_badcfg. Body-drop geometry matters: without interleaving one lost
// body erases bpb consecutive symbols (the truncation-chain failure mode);
// with depth=n each body carries one symbol per block.
#include "mtest.h"
#include "bench_wire.h"
#include "rx_pipeline.h"
#include "tx_pipeline.h"
#include <functional>
using namespace linkbench;

namespace {
struct Channel {
  RxPipeline* rx;
  uint16_t mseq = 0;
  uint64_t now_ms = 0;
  uint64_t delivered = 0, dropped = 0;
  void deliver(const std::vector<std::vector<uint8_t>>& bodies,
               const std::function<bool(size_t idx)>& drop) {
    const uint8_t rssi[2] = {50, 52};
    const int8_t snr[2] = {25, 26};
    for (size_t i = 0; i < bodies.size(); ++i) {
      const uint16_t s = mseq;
      mseq = static_cast<uint16_t>((mseq + 1) & 0x0FFF);
      now_ms += 1;
      if (drop(i)) { ++dropped; continue; }
      ++delivered;
      rx->on_body(bodies[i].data(), bodies[i].size(), s, true, rssi, snr, now_ms);
    }
  }
};

std::vector<std::vector<uint8_t>> encode_stream(TxPipeline& tx, uint32_t npkt) {
  std::vector<std::vector<uint8_t>> bodies;
  for (uint32_t s = 0; s < npkt; ++s) {
    auto pkt = build_bench_packet(s, 62);
    tx.add_packet(pkt.data(), pkt.size(), bodies);
  }
  tx.flush(bodies);
  return bodies;
}
}  // namespace

TEST(e2e_clean_channel_zero_loss) {
  FecParams p;
  p.bpb = 12;
  TxPipeline tx(p);
  RxPipeline rx(p);
  Channel ch{&rx};
  const uint32_t npkt = 8 * 200;
  ch.deliver(encode_stream(tx, npkt), [](size_t) { return false; });
  rx.expire(ch.now_ms + 10000);
  auto s = rx.snapshot();
  CHECK(s.pkts == npkt);
  CHECK(s.pkts_expected == npkt);
  CHECK(s.blocks_unrec == 0);
  CHECK(s.mac_lost == 0);
  CHECK(s.pattern_bad == 0);
}

// Interleaved at depth n=12, dropping every 6th body costs each block ~2 of
// its 12 symbols — well under the 4-repair budget: zero post-FEC loss, while
// the pre-FEC counters see every dropped frame.
TEST(e2e_loss_below_budget_recovers_fully) {
  FecParams p;
  p.bpb = 12;
  p.interleave = 12;
  TxPipeline tx(p);
  RxPipeline rx(p);
  Channel ch{&rx};
  const uint32_t npkt = 8 * 240;  // 240 blocks = 20 interleave windows
  ch.deliver(encode_stream(tx, npkt), [](size_t i) { return i % 6 == 5; });
  rx.expire(ch.now_ms + 10000);
  auto s = rx.snapshot();
  CHECK(s.pkts == npkt);
  CHECK(s.blocks_unrec == 0);
  // The last body (239, 239%6==5) is dropped; a trailing drop leaves no
  // following frame to reveal its gap, so one drop is unobservable.
  CHECK(s.mac_lost == ch.dropped - 1);
  CHECK(s.pattern_bad == 0);
}

// Dropping every other body loses ~6 of 12 symbols per block — over budget:
// unrecoverable blocks and post-FEC packet loss must both show.
TEST(e2e_loss_above_budget_shows_unrecoverable) {
  FecParams p;
  p.bpb = 12;
  p.interleave = 12;
  TxPipeline tx(p);
  RxPipeline rx(p);
  Channel ch{&rx};
  const uint32_t npkt = 8 * 240;
  ch.deliver(encode_stream(tx, npkt), [](size_t i) { return i % 2 == 1; });
  rx.expire(ch.now_ms + 10000);
  auto s = rx.snapshot();
  CHECK(s.blocks_unrec > 0);
  CHECK(s.pkts < npkt);
  CHECK(s.pattern_bad == 0);  // whatever decodes must decode correctly
}

// Same drop pattern WITHOUT interleaving: a lost body erases 12 consecutive
// symbols (a whole block at this geometry) — the e2e must show interleaving
// out-recovering the flat packing under bursty body loss.
TEST(e2e_interleave_beats_flat_packing_on_body_loss) {
  const uint32_t npkt = 8 * 240;
  auto run = [&](int interleave) {
    FecParams p;
    p.bpb = 12;
    p.interleave = interleave;
    TxPipeline tx(p);
    RxPipeline rx(p);
    Channel ch{&rx};
    ch.deliver(encode_stream(tx, npkt), [](size_t i) { return i % 6 == 5; });
    rx.expire(ch.now_ms + 10000);
    return rx.snapshot();
  };
  auto flat = run(0);
  auto il = run(12);
  CHECK(il.pkts == npkt);
  CHECK(flat.pkts < il.pkts);
  // Flat packing puts a whole block in one body: a dropped body's block
  // never reaches the decoder at all, so it is INVISIBLE — not counted
  // unrecoverable. 40 dropped bodies = 40 whole blocks = 320 packets gone.
  CHECK(flat.blocks_unrec == 0);
  CHECK(flat.pkts == npkt - 8u * 40u);
}

// RX configured with a different k: every envelope is dropped as bad-cfg —
// the "mismatched FEC flags" failure is visible, not silent.
TEST(e2e_config_mismatch_surfaces_as_badcfg) {
  FecParams ptx;
  ptx.bpb = 12;
  FecParams prx = ptx;
  prx.k = 10;
  TxPipeline tx(ptx);
  RxPipeline rx(prx);
  Channel ch{&rx};
  ch.deliver(encode_stream(tx, 8 * 20), [](size_t) { return false; });
  auto s = rx.snapshot();
  CHECK(s.sym_badcfg > 0);
  CHECK(s.pkts == 0);
}

MTEST_MAIN
