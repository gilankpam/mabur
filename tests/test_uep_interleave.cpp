// Loss-injection matrix for the encoder-side symbol interleaver
// (bench 2026-07-13: blocks_per_body packs one RS block into ~2 air frames,
// so one lost frame kills the block — docs/handover-video-delivery.md §2).
// Drives the real UepEncoder → lossy channel → real UepDecoder, mirroring
// tools/bench/fec_geometry_sim.cpp, and pins the delivery contract:
// interleave-off reproduces the on-air failure, interleave-on survives it.
#include <array>
#include <cstdint>
#include <random>
#include <set>
#include <vector>

#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
#include "mtest.h"

using namespace mabur;

namespace {

struct SimResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t bodies = 0, bodies_dropped = 0;
  bool mixed_block_bodies_ok = true;  // full bodies carry distinct block_ids
};

std::array<UepLayerCfg, 4> layers_for(int symbol_size, int bpb_base,
                                      int depth = 0) {
  std::array<UepLayerCfg, 4> layers{};
  const int bpb[4] = {bpb_base, bpb_base, bpb_base * 2, bpb_base * 2};
  for (int s = 0; s < 4; ++s) {
    layers[s].fec = RsConfig{8, symbol_size, kUepRefOverhead[s]};
    layers[s].blocks_per_body = bpb[s];
    layers[s].interleave_depth = depth;
  }
  return layers;
}

// Full bodies must carry blocks_per_body sub-blocks of DISTINCT block_ids —
// the whole point of the interleaver. Envelope: 11B header with block_id
// (u16 LE) at offset 7; body: 7B SBI header then per sub-block 2B crc + env.
bool body_blocks_distinct(const std::vector<uint8_t>& body, int symbol_size,
                          int blocks_per_body) {
  const size_t stride = 2 + 11 + static_cast<size_t>(symbol_size);
  if (body.size() != 7 + stride * static_cast<size_t>(blocks_per_body))
    return true;  // short flush body; distinctness not required
  std::set<uint16_t> ids;
  for (size_t off = 7; off + stride <= body.size(); off += stride) {
    const uint8_t* env = body.data() + off + 2;
    ids.insert(static_cast<uint16_t>(env[7] | (env[8] << 8)));
  }
  return ids.size() == static_cast<size_t>(blocks_per_body);
}

SimResult run_sim(int symbol_size, int bpb_base, bool interleave, int n_pkts,
                  double drop_pct, int burst_len, bool check_mixing = false,
                  int depth = 0) {
  auto layers = layers_for(symbol_size, bpb_base, depth);
  UepEncoder enc(layers, /*flush_ms=*/15, interleave);
  UepDecoder dec(layers, /*block_max_age_ms=*/2000);

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
    for (auto& b : bodies) {
      if (check_mixing &&
          !body_blocks_distinct(b.body, symbol_size,
                                layers[b.stream_id].blocks_per_body))
        r.mixed_block_bodies_ok = false;
      if (keep())
        r.delivered += dec.add_body(b.body.data(), b.body.size(), now).size();
    }
  };
  for (int i = 0; i < n_pkts; ++i) {
    const size_t paylen = (i % 7 == 6) ? 300 : 1388;
    std::vector<uint8_t> pkt(12 + paylen);
    pkt[0] = 0x80; pkt[1] = 0x60;
    pkt[2] = static_cast<uint8_t>(i >> 8);
    pkt[3] = static_cast<uint8_t>(i & 0xFF);
    // HEVC FU (type 49), mid-fragment of a trail NAL → stream 1 (T0)
    pkt[12] = 49 << 1; pkt[13] = 1; pkt[14] = 1;
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

TEST(lossless_parity_both_modes) {
  // Zero loss: both framings deliver every packet (pipeline correctness,
  // including the interleaver's drain-on-flush path).
  auto off = run_sim(164, 8, false, 5000, 0.0, 1);
  auto on = run_sim(164, 8, true, 5000, 0.0, 1, /*check_mixing=*/true);
  CHECK(off.delivered == off.sent);
  CHECK(on.delivered == on.sent);
  CHECK(on.mixed_block_bodies_ok);
}

TEST(interleave_survives_random_frame_loss) {
  // The bench geometry (symbol 164 / bpb 8, 1400B bodies) at 5% random body
  // loss: non-interleaved collapses (~91%, matches the on-air tap);
  // interleaved spreads each block over n bodies and stays ~lossless.
  auto off = run_sim(164, 8, false, 20000, 5.0, 1);
  auto on = run_sim(164, 8, true, 20000, 5.0, 1);
  CHECK(pct(off) < 97.0);  // documents the failure being fixed
  CHECK(pct(on) >= 99.5);
}

TEST(interleave_survives_heavier_loss) {
  auto on10 = run_sim(164, 8, true, 20000, 10.0, 1);
  CHECK(pct(on10) >= 99.0);
}

TEST(interleave_survives_burst_loss) {
  // 1% loss events × 4-frame bursts: consecutive air frames now carry
  // different blocks' symbols, so a burst costs each block few symbols.
  auto off = run_sim(164, 8, false, 20000, 1.0, 4);
  auto on = run_sim(164, 8, true, 20000, 1.0, 4);
  CHECK(pct(on) >= 99.0);
  CHECK(pct(on) >= pct(off));
}

TEST(deeper_window_survives_long_bursts) {
  // 1% loss events × 12-frame bursts: at depth 8 a burst erases most of a
  // 14-symbol span; at depth 32 (block spans 56 bodies) it costs each block
  // at most ~3 symbols. Bodies must still carry distinct blocks even when
  // depth is not tied to blocks_per_body.
  auto d8 = run_sim(164, 8, true, 20000, 1.0, 12, /*check_mixing=*/true, 8);
  auto d32 = run_sim(164, 8, true, 20000, 1.0, 12, /*check_mixing=*/true, 32);
  CHECK(d8.mixed_block_bodies_ok);
  CHECK(d32.mixed_block_bodies_ok);
  CHECK(pct(d32) >= 99.5);
  CHECK(pct(d32) > pct(d8));
  // Lossless with a non-multiple depth: alignment/flush must still deliver
  // everything with distinct-block bodies.
  auto odd = run_sim(164, 8, true, 5000, 0.0, 1, /*check_mixing=*/true, 12);
  CHECK(odd.delivered == odd.sent);
  CHECK(odd.mixed_block_bodies_ok);
}

TEST(interleave_alternate_geometry) {
  // Repo-default-ish small symbols: property holds independent of geometry.
  auto on = run_sim(340, 4, true, 20000, 5.0, 1, /*check_mixing=*/true);
  CHECK(on.mixed_block_bodies_ok);
  CHECK(pct(on) >= 99.0);
}

MTEST_MAIN
