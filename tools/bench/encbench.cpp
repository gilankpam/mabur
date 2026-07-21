// Encoder-capacity microbench for the maburd hot path: feeds a synthetic
// 60 fps HEVC-FU RTP stream through UepEncoder (classify -> fragment ->
// sliding-window FEC -> SBI pack) as fast as the CPU allows and reports the
// sustainable single-thread throughput. Built to compare FEC geometries ON
// the SSC338Q after the per-layer big-symbol config pinned the waybeam ring
// full (2026-07-15): capacity below the commanded feed means ring overflow ->
// seq-invisible slice-tail aborts no transport counter sees.
//
//   encbench sweep [sim_seconds]              # overhead + bitrate sweep (default)
//   encbench <scalar|perlayer> [cmd_ov] [pkts_per_frame] [sim_seconds]
//
// The sweep answers one question: is the drone drain ceiling a fixed AIR-BYTE
// rate, or a FEC-parity-work rate that scales with overhead? If sustainable
// air Mbps is ~constant across the overhead column, it's an air-byte ceiling;
// if it falls as overhead rises, the limiter is single-core FEC (GF256 repair
// generation), and "air Mbps" is a misleading way to express the budget.
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

struct Metrics {
  double wall;        // seconds of CPU wall time to encode the whole sim
  double sim;         // simulated stream seconds
  uint64_t in_bytes;  // source (video) bytes fed in
  uint64_t air_bytes; // FEC body bytes emitted (on-air payload)
  size_t pkts;
};

// Feed a `sim_seconds` long, 60 fps stream of `ppf` FU packets/frame (1400 B
// payload each) through a fresh UepEncoder at the given geometry, flat out.
static Metrics run_point(bool perlayer, double cmd_ov, int ppf, int sim_seconds) {
  std::array<UepLayerCfg, 4> layers;
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

  uint64_t now = 1;
  uint64_t air_bytes = 0;
  size_t pkts = 0, in_bytes = 0;
  uint16_t seq = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < sim_seconds; ++s) {
    for (int f = 0; f < 60; ++f) {
      const int inner = (f == 0) ? 19 : 1;  // IDR_W_RADL vs TRAIL_R
      for (int k = 0; k < ppf; ++k) {
        const auto p = make_fu(inner, k == 0, k == ppf - 1, 1400, seq++);
        auto out = uep.add_rtp(p.data(), p.size(), now);
        for (auto& b : out) air_bytes += b.body.size();
        ++pkts;
        in_bytes += p.size();
      }
      now += 16;
      auto out = uep.poll(now);
      for (auto& b : out) air_bytes += b.body.size();
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  Metrics m;
  m.wall = std::chrono::duration<double>(t1 - t0).count();
  m.sim = sim_seconds;
  m.in_bytes = in_bytes;
  m.air_bytes = air_bytes;
  m.pkts = pkts;
  return m;
}

// video Mbps of a 60 fps / ppf-packet-per-frame / 1400 B-payload feed
static double feed_video_mbps(int ppf) {
  return ppf * 1400.0 * 8.0 * 60.0 / 1e6;
}

static void print_row(double ov, int ppf, const Metrics& m) {
  const double speedup = m.sim / m.wall;                 // x realtime
  const double in_mbps = m.in_bytes * 8.0 / m.sim / 1e6; // feed video Mbps (1x)
  const double air_mbps = m.air_bytes * 8.0 / m.sim / 1e6;
  const double sust_video = in_mbps * speedup;   // max sustainable video Mbps
  const double sust_air = air_mbps * speedup;    // max sustainable air Mbps
  const double ns_src = m.wall * 1e9 / m.in_bytes;
  const double ns_air = m.wall * 1e9 / m.air_bytes;
  std::printf("%5.3f %5d %7.2f %7.2f %8.2f %10.2f %9.2f %8.1f %8.1f\n", ov, ppf,
              in_mbps, air_mbps, speedup, sust_video, sust_air, ns_src, ns_air);
}

static void header() {
  std::printf("%5s %5s %7s %7s %8s %10s %9s %8s %8s\n", "ov", "ppf", "vid1x",
              "air1x", "speedup", "SUST_vid", "SUST_air", "ns/src", "ns/air");
}

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "sweep";

  if (mode == "sweep") {
    const int sim = argc > 2 ? std::atoi(argv[2]) : 8;
    std::printf("# gf=%s  geometry=scalar-164 win64 bpb8  sim=%ds/point\n",
                gf::backend(), sim);
    std::printf("# SUST_air = air1x * speedup = max sustainable on-air Mbps for "
                "this single thread.\n");
    std::printf("# If SUST_air is flat across ov -> air-byte ceiling.  If it "
                "falls with ov -> FEC-parity (overhead) ceiling.\n\n");

    std::printf("## Overhead axis (fixed ~8.7 Mbps video feed, ppf=13):\n");
    header();
    for (double ov : {0.05, 0.10, 0.175, 0.25, 0.375, 0.50}) {
      Metrics m = run_point(false, ov, 13, sim);
      print_row(ov, 13, m);
    }

    std::printf("\n## Bitrate axis (fixed ov=0.10, vary video feed):\n");
    header();
    for (int ppf : {8, 13, 16, 20, 24}) {
      Metrics m = run_point(false, 0.10, ppf, sim);
      print_row(0.10, ppf, m);
      (void)feed_video_mbps;
    }
    return 0;
  }

  // single-point mode (back-compat + targeted probes)
  const bool perlayer = mode != "scalar";
  const double cmd_ov = argc > 2 ? std::atof(argv[2]) : 0.38;
  const int ppf = argc > 3 ? std::atoi(argv[3]) : 13;
  const int sim = argc > 4 ? std::atoi(argv[4]) : 10;
  Metrics m = run_point(perlayer, cmd_ov, ppf, sim);
  std::printf("mode=%s gf=%s ov=%.3f ppf=%d sim=%ds wall=%.2fs\n", mode.c_str(),
              gf::backend(), cmd_ov, ppf, sim, m.wall);
  header();
  print_row(cmd_ov, ppf, m);
  return 0;
}
