// Host end-to-end: TxPipeline → simulated lossy channel → RxPipeline, no
// USB. Loss below the FEC budget must be invisible post-FEC; loss above it
// must surface as abandoned symbols; a symbol_size mismatch must surface as
// visible pre-FEC failure (sub-block CRC, not sym_badcfg — see the two
// config-mismatch tests below for why). Window width matters for burst
// loss: a wide window's repairs span more of the burst than a narrow one —
// time diversity bought by overlapping repair windows, not by delaying
// sources (see common/include/mabur/sw_encoder.h).
#include "mtest.h"
#include "bench_wire.h"
#include "rx_pipeline.h"
#include "tx_pipeline.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
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
  CHECK(s.syms_abandoned == 0);
  CHECK(s.mac_lost == 0);
  CHECK(s.pattern_bad == 0);
}

// window=128 at the default overhead 0.5 comfortably covers a scattered
// every-6th-body loss: each dropped body costs one symbol out of a
// 128-wide repair span, well under budget. Drops stop 20 bodies before the
// end so every hole still has a full repair window's worth of stream after
// it — a drop inside the last window has nowhere left to draw a covering
// repair from and legitimately never resolves (that's a stream-tail
// truncation artifact, not a decode failure; see e2e_loss_above_budget for
// the actual "irrecoverable" contract).
TEST(e2e_loss_below_budget_recovers_fully) {
  FecParams p;
  p.bpb = 12;
  p.window = 128;
  TxPipeline tx(p);
  RxPipeline rx(p);
  Channel ch{&rx};
  const uint32_t npkt = 8 * 240;
  auto bodies = encode_stream(tx, npkt);
  const size_t tail_guard = 20;
  ch.deliver(bodies, [&](size_t i) {
    return i % 6 == 5 && i < bodies.size() - tail_guard;
  });
  rx.expire(ch.now_ms + 10000);
  auto s = rx.snapshot();
  CHECK(s.pkts == npkt);
  CHECK(s.syms_abandoned == 0);
  CHECK(s.mac_lost == ch.dropped);
  CHECK(s.pattern_bad == 0);
}

// Dropping every other body overwhelms the overhead-0.5 repair budget
// regardless of window width: over half the symbols in every span are gone,
// so some seqs age past the horizon unrecovered.
TEST(e2e_loss_above_budget_shows_unrecoverable) {
  FecParams p;
  p.bpb = 12;
  p.window = 128;
  TxPipeline tx(p);
  RxPipeline rx(p);
  Channel ch{&rx};
  const uint32_t npkt = 8 * 240;
  ch.deliver(encode_stream(tx, npkt), [](size_t i) { return i % 2 == 1; });
  rx.expire(ch.now_ms + 10000);
  auto s = rx.snapshot();
  CHECK(s.syms_abandoned > 0);
  CHECK(s.pkts < npkt);
  CHECK(s.pattern_bad == 0);  // whatever decodes must decode correctly
}

// Same burst pattern, two window widths: a wide window's repairs span more
// of a multi-body burst than a narrow one, so window 128 out-recovers
// window 16 (mirrors test_uep_sw.cpp's wider_window_survives_longer_bursts).
TEST(e2e_wide_window_beats_narrow_on_burst_loss) {
  const uint32_t npkt = 8 * 240;
  auto run = [&](int window) {
    FecParams p;
    p.bpb = 12;
    p.window = window;
    TxPipeline tx(p);
    RxPipeline rx(p);
    Channel ch{&rx};
    // 4-body bursts every ~50 bodies: a burst erases several consecutive
    // symbols, which a wide repair window can still span.
    int burst_left = 0;
    ch.deliver(encode_stream(tx, npkt), [&](size_t i) {
      if (burst_left > 0) { --burst_left; return true; }
      if (i % 50 == 49) { burst_left = 3; return true; }
      return false;
    });
    rx.expire(ch.now_ms + 10000);
    return rx.snapshot();
  };
  auto w16 = run(16);
  auto w128 = run(128);
  CHECK(w128.pkts == npkt);
  CHECK(w16.pkts < w128.pkts);
}

// RX configured with a different symbol_size: block_payload = envelope_len
// is derived from symbol_size on BOTH ends of the SBI framing (unlike the
// old RS scheme's k, which didn't affect envelope_len). A symbol_size
// mismatch therefore desyncs SbiPacker/sbi_unpack itself — sub-blocks fail
// their CRC16 before any envelope reaches SwDecoder, so nothing decodes and
// the pre-FEC counters (not sym_badcfg) show the mismatch. See
// sw_decoder_flags_symbol_size_mismatch below for the field SwDecoder does
// guard directly.
TEST(e2e_config_mismatch_surfaces_as_subblock_crc_failures) {
  FecParams ptx;
  ptx.bpb = 12;
  FecParams prx = ptx;
  prx.symbol_size = 128;
  TxPipeline tx(ptx);
  RxPipeline rx(prx);
  Channel ch{&rx};
  ch.deliver(encode_stream(tx, 8 * 20), [](size_t) { return false; });
  auto s = rx.snapshot();
  CHECK(s.sub_crc_fail > 0);
  CHECK(s.sym_badcfg == 0);  // never reached: SBI framing rejected it first
  CHECK(s.pkts == 0);
}

// SwDecoder's own bad-cfg guard (add_symbol: h.symbol_size != cfg_.
// symbol_size, or envelope length mismatch) fires on an envelope whose WIRE
// header disagrees with local config while still matching RX's SBI
// block_payload length byte-for-byte — the scenario a stale/rolling deploy
// could produce. Bypasses SbiPacker/TxPipeline to hit SwDecoder::add_symbol
// directly, since going through matched-length framing is the only way to
// reach this guard (see the test above).
TEST(sw_decoder_flags_symbol_size_mismatch) {
  FecParams p;
  RxPipeline rx(p);
  mabur::sw::SwHeader h;
  h.repair = false;
  h.symbol_size = static_cast<uint16_t>(p.symbol_size + 8);  // wire disagrees
  h.seq = 0;
  std::vector<uint8_t> env;
  mabur::sw::pack_header(env, h);
  env.insert(env.end(), static_cast<size_t>(p.symbol_size), 0xAB);  // RX-length pad
  mabur::SbiPacker packer(p.envelope_len(), 1, kBenchStreamId);
  auto bodies = packer.add(env.data(), env.size());
  for (auto& b : packer.flush()) bodies.push_back(std::move(b));
  const uint8_t rssi[2] = {50, 52};
  const int8_t snr[2] = {25, 26};
  for (auto& b : bodies) rx.on_body(b.data(), b.size(), 0, true, rssi, snr, 1);
  auto s = rx.snapshot();
  CHECK(s.sym_badcfg > 0);
  CHECK(s.pkts == 0);
}

MTEST_MAIN
