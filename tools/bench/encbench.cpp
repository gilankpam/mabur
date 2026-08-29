// Encoder-capacity microbench for the maburd hot path: feeds a synthetic
// 60 fps whole-frame Annex-B stream through UepEncoder (fragment ->
// sliding-window FEC -> SBI pack, sealed per frame) as fast as the CPU allows
// and reports the
// sustainable single-thread throughput. Built to compare FEC geometries ON
// the SSC338Q after the per-layer big-symbol config pinned the waybeam ring
// full (2026-07-15): capacity below the commanded feed means ring overflow ->
// seq-invisible slice-tail aborts no transport counter sees.
//
//   encbench sweep [sim_seconds]              # overhead + bitrate sweep (default)
//   encbench <scalar|perlayer> [ov] [kb_per_frame_x1400] [sim_seconds]
//
// ov is the literal air overhead (source:repair ratio actually put on air),
// not a scaled command value — Task 3 (airtime-balance-uep) deleted the
// uep_layer_overhead ladder translation this tool used to route through.
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

#include "mabur/frame_wire.h"
#include "mabur/gf256.h"
#include "mabur/uep_encoder.h"

using namespace mabur;

// One frame unit as maburd sends it: FrameHdr + a single Annex-B NAL of the
// given type (19 = IDR_W_RADL, 1 = TRAIL_R at tid 0).
static std::vector<uint8_t> make_frame(int nal_type, size_t paylen, uint16_t id) {
  std::vector<uint8_t> p(framewire::kFrameHdrLen + paylen, 0xA5);
  framewire::FrameHdr h;
  h.frame_id = id;
  h.flags = nal_type == 19 ? framewire::kFlagIdr : 0;
  h.pts_us = id * 16667u;
  framewire::pack_frame_hdr(h, p.data());
  const size_t off = framewire::kFrameHdrLen;
  p[off] = 0x00; p[off + 1] = 0x00; p[off + 2] = 0x00; p[off + 3] = 0x01;
  p[off + 4] = static_cast<uint8_t>(nal_type << 1);
  p[off + 5] = 1;                     // tid 0
  return p;
}

struct Metrics {
  double wall;        // seconds of CPU wall time to encode the whole sim
  double sim;         // simulated stream seconds
  uint64_t in_bytes;  // source (video) bytes fed in
  uint64_t air_bytes; // FEC body bytes emitted (on-air payload)
  size_t frames;      // frames fed
};

// Feed a `sim_seconds` long, 60 fps stream of whole frames (`ppf` x 1400 B of
// Annex-B each) through a fresh UepEncoder at the given geometry, flat out.
// A scalar geometry (all layers identical) may override symbol size, window
// and blocks_per_body via ss/win/bpb_n; pass 0s for the 164/64/8 default.
static Metrics run_point(bool perlayer, double ov, int ppf, int sim_seconds,
                         int ss = 0, int win = 0, int bpb_n = 0) {
  std::array<UepLayerCfg, 2> layers;
  const int s0 = ss ? ss : 164;
  const int w0 = win ? win : 64;
  const int b0 = bpb_n ? bpb_n : 8;
  const int syms[2] = {perlayer ? 164 : s0, perlayer ? 1312 : s0};
  const int bpb[2] = {perlayer ? 4 : b0, perlayer ? 1 : b0};
  for (int s = 0; s < 2; ++s) {
    // ov is the literal air overhead now (no uep_layer_overhead ladder
    // scaling): both layers run at exactly the --overhead value passed in.
    layers[static_cast<size_t>(s)].fec =
        SwConfig{syms[s], perlayer ? 64 : w0, ov};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  UepEncoder uep(layers, 25);

  uint64_t now = 1;
  uint64_t air_bytes = 0;
  size_t frames = 0, in_bytes = 0;
  uint16_t id = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < sim_seconds; ++s) {
    for (int f = 0; f < 60; ++f) {
      const int nal = (f == 0) ? 19 : 1;   // IDR_W_RADL vs TRAIL_R
      const int sid = (f == 0) ? 0 : 1;    // critical vs T0
      const auto p = make_frame(nal, static_cast<size_t>(ppf) * 1400, id++);
      auto out = uep.add_frame(sid, p.data(), p.size(), now);
      for (auto& b : out) air_bytes += b.body.size();
      ++frames;
      in_bytes += p.size();
      now += 16;
      auto polled = uep.poll(now);
      for (auto& b : polled) air_bytes += b.body.size();
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  Metrics m;
  m.wall = std::chrono::duration<double>(t1 - t0).count();
  m.sim = sim_seconds;
  m.in_bytes = in_bytes;
  m.air_bytes = air_bytes;
  m.frames = frames;
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
    // Literal air overhead now (no uep_layer_overhead ladder scaling) — the
    // ×2 rule this migration applies everywhere: same axis coverage as the
    // pre-literal {0.05..0.50} cmd-overhead sweep.
    for (double ov : {0.10, 0.20, 0.35, 0.50, 0.75, 1.00}) {
      Metrics m = run_point(false, ov, 13, sim);
      print_row(ov, 13, m);
    }

    std::printf("\n## Bitrate axis (fixed ov=0.20, vary video feed):\n");
    header();
    for (int ppf : {8, 13, 16, 20, 24}) {
      Metrics m = run_point(false, 0.20, ppf, sim);
      print_row(0.20, ppf, m);
      (void)feed_video_mbps;
    }
    return 0;
  }

  if (mode == "symsweep") {
    // Symbol-size axis: does the parity ceiling lift with bigger symbols?
    // Two ladders at fixed ov/ppf:
    //  - same-SPAN: window rows scale down with symbol size (w*ss = 10496 B
    //    held constant) -> tests "small symbols force more rows for the same
    //    protection span" (MAC work per src byte = ov * window_rows).
    //  - same-ROWS: w64 at every size -> isolates per-call kernel efficiency
    //    (span grows with ss; MAC work per src byte identical in theory).
    const double ov = argc > 2 ? std::atof(argv[2]) : 0.5;  // literal (was cmd 0.25)
    const int ppf = argc > 3 ? std::atoi(argv[3]) : 13;
    const int sim = argc > 4 ? std::atoi(argv[4]) : 8;
    struct Geo { int ss, win, bpb; };
    std::printf("# gf=%s  symsweep ov=%.3f ppf=%d sim=%ds/point\n\n",
                gf::backend(), ov, ppf, sim);
    std::printf("## Same protection span (win*ss = 10496 B):\n");
    std::printf("%5s %4s %4s ", "ss", "win", "bpb");
    header();
    for (Geo g : {Geo{164, 64, 8}, Geo{328, 32, 4}, Geo{656, 16, 2},
                  Geo{1312, 8, 1}}) {
      Metrics m = run_point(false, ov, ppf, sim, g.ss, g.win, g.bpb);
      std::printf("%5d %4d %4d ", g.ss, g.win, g.bpb);
      print_row(ov, ppf, m);
    }
    std::printf("\n## Same window rows (w64; span grows with ss):\n");
    std::printf("%5s %4s %4s ", "ss", "win", "bpb");
    header();
    for (Geo g : {Geo{164, 64, 8}, Geo{328, 64, 4}, Geo{656, 64, 2},
                  Geo{1312, 64, 1}}) {
      Metrics m = run_point(false, ov, ppf, sim, g.ss, g.win, g.bpb);
      std::printf("%5d %4d %4d ", g.ss, g.win, g.bpb);
      print_row(ov, ppf, m);
    }
    return 0;
  }

  // single-point mode (back-compat + targeted probes)
  const bool perlayer = mode != "scalar";
  // ov is the literal air overhead now (was cmd 0.38 pre-literal).
  const double ov = argc > 2 ? std::atof(argv[2]) : 0.76;
  const int ppf = argc > 3 ? std::atoi(argv[3]) : 13;
  const int sim = argc > 4 ? std::atoi(argv[4]) : 10;
  Metrics m = run_point(perlayer, ov, ppf, sim);
  std::printf("mode=%s gf=%s ov=%.3f ppf=%d sim=%ds wall=%.2fs\n", mode.c_str(),
              gf::backend(), ov, ppf, sim, m.wall);
  header();
  print_row(ov, ppf, m);
  return 0;
}
