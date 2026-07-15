// Encoder-capacity microbench for the maburd hot path: feeds a realistic
// 60 fps / ~8.7 Mbps synthetic HEVC-FU RTP stream through UepEncoder
// (classify -> fragment -> sliding-window FEC -> SBI pack) as fast as the
// CPU allows and reports sustainable packets/s. Built to compare FEC
// geometries ON the SSC338Q after the per-layer big-symbol config pinned the
// waybeam ring full (2026-07-15): capacity below waybeam's ~800 pkt/s means
// ring overflow -> seq-invisible slice-tail aborts no transport counter sees.
//   encbench <scalar|perlayer> [sim_seconds]
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "mabur/gf256.h"
#include "mabur/uep_encoder.h"

using namespace mabur;

static std::vector<uint8_t> make_fu(int inner, bool start, bool end,
                                    size_t paylen, uint16_t seq) {
  std::vector<uint8_t> p(12 + paylen, 0xA5);
  p[0] = 0x80;
  p[1] = 96;
  p[2] = static_cast<uint8_t>(seq >> 8);
  p[3] = static_cast<uint8_t>(seq & 0xFF);
  p[12] = 49 << 1;                    // FU
  p[13] = 1;                          // tid 0
  p[14] = static_cast<uint8_t>((start ? 0x80 : 0) | (end ? 0x40 : 0) | inner);
  return p;
}

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "perlayer";
  const int sim_seconds = argc > 2 ? std::atoi(argv[2]) : 10;
  const double cmd_ov = 0.38;  // matches op=... ov0.38 on the bench link

  std::array<UepLayerCfg, 4> layers;
  const bool perlayer = mode != "scalar";
  const int syms[4] = {164, perlayer ? 1312 : 164, perlayer ? 1312 : 164,
                       perlayer ? 1312 : 164};
  const int bpb[4] = {perlayer ? 4 : 8, perlayer ? 1 : 8, perlayer ? 1 : 8,
                      perlayer ? 1 : 8};
  for (int s = 0; s < 4; ++s) {
    layers[static_cast<size_t>(s)].fec =
        SwConfig{syms[s], 64, uep_layer_overhead(s, cmd_ov)};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  UepEncoder uep(layers, 25);

  // 60 fps, 13 x 1400B FU packets per frame (~8.7 Mbps), one IDR frame/s.
  uint64_t now = 1;
  uint64_t bodies_n[4] = {0, 0, 0, 0}, bodies_b[4] = {0, 0, 0, 0};
  size_t pkts = 0, in_bytes = 0;
  uint16_t seq = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < sim_seconds; ++s) {
    for (int f = 0; f < 60; ++f) {
      const int inner = (f == 0) ? 19 : 1;  // IDR_W_RADL vs TRAIL_R
      for (int k = 0; k < 13; ++k) {
        const auto p = make_fu(inner, k == 0, k == 12, 1400, seq++);
        auto out = uep.add_rtp(p.data(), p.size(), now);
        for (auto& b : out) {
          bodies_n[b.stream_id]++;
          bodies_b[b.stream_id] += b.body.size();
        }
        ++pkts;
        in_bytes += p.size();
      }
      now += 16;
      auto out = uep.poll(now);
      for (auto& b : out) {
        bodies_n[b.stream_id]++;
        bodies_b[b.stream_id] += b.body.size();
      }
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double el = std::chrono::duration<double>(t1 - t0).count();

  uint64_t tb = 0, tn = 0;
  for (int s = 0; s < 4; ++s) {
    tb += bodies_b[s];
    tn += bodies_n[s];
  }
  std::printf("mode=%s gf=%s sim=%ds wall=%.2fs speedup=%.2fx\n", mode.c_str(),
              gf::backend(), sim_seconds, el, sim_seconds / el);
  std::printf("pkts=%zu (%.0f pkt/s capacity) in=%.2f Mbps -> air=%.2f Mbps "
              "(x%.2f inflation)\n",
              pkts, pkts / el, in_bytes * 8.0 / sim_seconds / 1e6,
              tb * 8.0 / sim_seconds / 1e6,
              static_cast<double>(tb) / static_cast<double>(in_bytes));
  for (int s = 0; s < 4; ++s)
    std::printf("  s%d bodies=%llu bytes=%llu\n", s,
                static_cast<unsigned long long>(bodies_n[s]),
                static_cast<unsigned long long>(bodies_b[s]));
  return 0;
}
