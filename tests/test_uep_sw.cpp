// Loss-injection matrix for the sliding-window FEC through the full UEP
// pipeline (UepEncoder -> lossy channel -> UepDecoder). Replaces the
// interleaver matrix (test_uep_interleave.cpp): time diversity now comes
// from overlapping repair windows, so sources never wait — the checks here
// pin delivery under the same random/burst loss the interleaver was built
// for (bench 2026-07-13, docs/handover-video-delivery.md §2).
#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
#include "mtest.h"

using namespace mabur;

namespace {

struct SimResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t bodies = 0, bodies_dropped = 0;
};

std::array<UepLayerCfg, 4> layers_for(int symbol_size, int bpb_base, int window) {
  std::array<UepLayerCfg, 4> layers{};
  const int bpb[4] = {bpb_base, bpb_base, bpb_base * 2, bpb_base * 2};
  for (int s = 0; s < 4; ++s) {
    layers[static_cast<size_t>(s)].fec =
        SwConfig{symbol_size, window, kUepRefOverhead[s]};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  return layers;
}

SimResult run_sim(int symbol_size, int bpb_base, int window, int n_pkts,
                  double drop_pct, int burst_len) {
  auto layers = layers_for(symbol_size, bpb_base, window);
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers, /*decode_deadline_ms=*/200);

  std::mt19937 drop_rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  int burst_left = 0;
  SimResult r;
  auto keep = [&]() {
    ++r.bodies;
    if (burst_left > 0) { --burst_left; ++r.bodies_dropped; return false; }
    if (u(drop_rng) * 100.0 < drop_pct) {
      ++r.bodies_dropped;
      if (burst_len > 1) burst_left = burst_len - 1;
      return false;
    }
    return true;
  };

  std::mt19937 rng(42);
  uint64_t now = 1000;
  auto consume = [&](std::vector<UepBody>& bodies) {
    for (auto& b : bodies)
      if (keep())
        r.delivered += dec.add_body(b.body.data(), b.body.size(), now).size();
  };
  for (int i = 0; i < n_pkts; ++i) {
    const size_t paylen = (i % 7 == 6) ? 300 : 1388;
    std::vector<uint8_t> pkt(12 + paylen);
    pkt[0] = 0x80; pkt[1] = 0x60;
    pkt[2] = static_cast<uint8_t>(i >> 8);
    pkt[3] = static_cast<uint8_t>(i & 0xFF);
    pkt[12] = 49 << 1; pkt[13] = 1; pkt[14] = 1;  // HEVC FU -> stream 1 (T0)
    for (size_t b = 15; b < pkt.size(); ++b)
      pkt[b] = static_cast<uint8_t>(rng());
    ++r.sent;
    auto bodies = enc.add_rtp(pkt.data(), pkt.size(), now);
    consume(bodies);
    if (i % 30 == 29) {
      now += 15;
      auto flushed = enc.poll(now);
      consume(flushed);
    }
  }
  now += 100;
  auto tail = enc.flush_all();
  consume(tail);
  dec.poll(now + 5000);
  return r;
}

double pct(const SimResult& r) {
  return r.sent ? 100.0 * static_cast<double>(r.delivered) /
                      static_cast<double>(r.sent)
                : 0.0;
}

}  // namespace

TEST(lossless_delivers_everything) {
  auto r = run_sim(164, 8, 128, 5000, 0.0, 1);
  CHECK(r.delivered == r.sent);
}

TEST(survives_random_frame_loss) {
  // The bench geometry (symbol 164 / bpb 8) at 5% random body loss — the
  // scenario that killed non-interleaved block FEC at ~91%.
  auto r = run_sim(164, 8, 128, 20000, 5.0, 1);
  CHECK(pct(r) >= 99.5);
}

TEST(survives_heavier_random_loss) {
  auto r = run_sim(164, 8, 128, 20000, 10.0, 1);
  CHECK(pct(r) >= 99.0);
}

TEST(survives_burst_loss) {
  // 1% loss events x 4-body bursts (T0: 32 consecutive symbols lost;
  // window 128 at overhead 0.75 tolerates bursts up to ~54).
  auto r = run_sim(164, 8, 128, 20000, 1.0, 4);
  CHECK(pct(r) >= 99.0);
}

TEST(wider_window_survives_longer_bursts) {
  // 6-body bursts = 48 consecutive T0 symbols: window 32 cannot span the
  // hole (budget ~13), window 128 can (~54).
  auto w32 = run_sim(164, 8, 32, 20000, 1.0, 6);
  auto w128 = run_sim(164, 8, 128, 20000, 1.0, 6);
  CHECK(pct(w128) >= 99.0);
  CHECK(pct(w128) > pct(w32));
}

TEST(alternate_geometry) {
  auto r = run_sim(340, 4, 128, 20000, 5.0, 1);
  CHECK(pct(r) >= 99.0);
}

MTEST_MAIN
