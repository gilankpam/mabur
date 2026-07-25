// SW-FEC optimization A/B harness: pins the CURRENT mabur encode cost as the
// baseline (common/ linked unmodified) and measures every candidate
// optimization against it on the same tables. Nothing in mabur changes until
// a candidate wins here on the SSC338Q.
//
//   fecbench [all|verify|kernel|repair|encoder] [sim_seconds]
//
//   verify   candidates are byte-exact (kernels) / same-envelope-set
//            (repair generators, order-insensitive for multicore) vs baseline
//   kernel   raw lincomb MB/s, L1-hot single symbol (upper bound; the 272
//            MB/s number from gf_bench is this table's ss=164 row)
//   repair   the number that matters: make_repair-shaped work — stream a
//            window of distinct heap symbols into an accumulator, one symbol
//            replaced per repair (seal cadence), per geometry
//   encoder  end-to-end UepEncoder SUST_air at deployed geometries (same
//            columns as tools/bench/encbench.cpp so results are comparable)
//
// Build: ./build.sh   Run on target: ./run_drone.sh
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "candidates.h"
#include "mabur/fec_worker.h"
#include "mabur/frame_wire.h"
#include "mabur/gf256.h"
#include "mabur/uep_encoder.h"
#include "mt_encoder.h"

#ifndef FECBENCH_SHA
#define FECBENCH_SHA "unknown"
#endif

using namespace fecbench;

static double now_s() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// ---------------------------------------------------------------- verify --

static uint8_t g_exp[512], g_log[256];
static void build_ref_tables() {
  int x = 1;
  for (int i = 0; i < 255; ++i) {
    g_exp[i] = (uint8_t)x;
    g_log[x] = (uint8_t)i;
    x <<= 1;
    if (x & 0x100) x ^= 0x11D;
  }
  for (int i = 255; i < 512; ++i) g_exp[i] = g_exp[i - 255];
}
static void ref_lincomb(uint8_t* acc, const uint8_t* sym, uint8_t c, size_t n) {
  if (!c) return;
  for (size_t i = 0; i < n; ++i)
    if (sym[i]) acc[i] ^= g_exp[g_log[c] + g_log[sym[i]]];
}

static int run_verify() {
  build_ref_tables();
  std::mt19937 rng(1);
  int failures = 0;

  for (const auto& k : kernel_candidates()) {
    bool ok = true;
    for (int it = 0; it < 2000 && ok; ++it) {
      size_t n = 1 + rng() % 1400;  // odd tails included
      std::vector<uint8_t> a(n), b(n), sym(n);
      for (auto& v : sym) v = (uint8_t)rng();
      for (size_t i = 0; i < n; ++i) a[i] = b[i] = (uint8_t)rng();
      uint8_t c = (uint8_t)rng();
      k.fn(a.data(), sym.data(), c, n);
      ref_lincomb(b.data(), sym.data(), c, n);
      ok = std::memcmp(a.data(), b.data(), n) == 0;
    }
    std::printf("verify kernel %-24s %s\n", k.name, ok ? "OK" : "FAIL");
    failures += !ok;
  }

  // Drive baseline + candidate through an identical seal/repair sequence
  // (window fill, wrap, partial windows, multi-repair calls) and require the
  // same envelope SET at every repair point — order-insensitive within one
  // call so multicore candidates verify too.
  const auto& base = repair_candidates()[0];
  for (const auto& r : repair_candidates()) {
    bool ok = true;
    for (int trial = 0; trial < 6 && ok; ++trial) {
      const int ss = (trial & 1) ? 1312 : 164;
      const int w = (trial < 2) ? 2 + (int)(rng() % 6) : 32 + (int)(rng() % 97);
      auto ga = base.create(ss, w);
      auto gb = r.create(ss, w);
      uint32_t key_a = 7u * (uint32_t)trial, key_b = key_a;
      uint32_t seq = rng();
      std::vector<uint8_t> sym((size_t)ss);
      const int seals = w + w / 2 + 3;  // exercises pre-full AND wrapped
      for (int s = 0; s < seals && ok; ++s) {
        for (auto& v : sym) v = (uint8_t)rng();
        ga->on_seal(sym.data());
        gb->on_seal(sym.data());
        ++seq;
        if ((s % 3) == 2) {
          const int nrep = 1 + (int)(rng() % 3);
          std::vector<std::vector<uint8_t>> ea, eb;
          ga->make_repairs(seq, &key_a, nrep, &ea);
          gb->make_repairs(seq, &key_b, nrep, &eb);
          std::sort(ea.begin(), ea.end());
          std::sort(eb.begin(), eb.end());
          ok = (key_a == key_b) && (ea == eb);
        }
      }
    }
    std::printf("verify repair %-24s %s\n", r.name, ok ? "OK" : "FAIL");
    failures += !ok;
  }
  return failures;
}

// ---------------------------------------------------------------- kernel --

static void run_kernel_bench() {
  std::printf("\n## kernel: L1-hot lincomb MB/s (single reused symbol)\n");
  std::printf("%-24s %10s %10s\n", "candidate", "ss=164", "ss=1312");
  std::mt19937 rng(2);
  for (const auto& k : kernel_candidates()) {
    double mbps[2] = {0, 0};
    const int sizes[2] = {164, 1312};
    for (int gi = 0; gi < 2; ++gi) {
      const size_t ss = (size_t)sizes[gi];
      std::vector<uint8_t> acc(ss), sym(ss);
      for (auto& v : sym) v = (uint8_t)rng();
      // calibrate to ~1s
      long iters = 50000;
      for (;;) {
        double t0 = now_s();
        for (long i = 0; i < iters; ++i)
          k.fn(acc.data(), sym.data(), (uint8_t)(1 + (i & 0xFE)), ss);
        double dt = now_s() - t0;
        if (dt > 0.5) {
          mbps[gi] = (double)iters * (double)ss / dt / 1e6;
          break;
        }
        iters *= 4;
      }
    }
    std::printf("%-24s %10.1f %10.1f\n", k.name, mbps[0], mbps[1]);
  }
}

// ---------------------------------------------------------------- repair --

struct Geometry {
  int ss, window;
};
static const Geometry kGeoms[] = {{164, 64}, {164, 128}, {1312, 64}, {1312, 128}};

// make_repair-shaped load at the production cadence: one seal (the
// candidate's own storage maintenance — deque alloc vs ring memcpy is part
// of what's being compared) followed by one full repair, per iteration.
static void run_repair_bench() {
  std::printf("\n## repair: seal+repair throughput at production cadence\n");
  std::printf("%-24s %-11s %12s %12s %12s\n", "candidate", "geometry",
              "repairs/s", "srcMB/s", "us/repair");
  for (const auto& r : repair_candidates()) {
    for (const auto& g : kGeoms) {
      std::mt19937 rng(3);
      auto gen = r.create(g.ss, g.window);
      // symbol pool rotated through the window, fill cost outside the loop
      std::vector<std::vector<uint8_t>> pool((size_t)g.window + 3);
      for (auto& s : pool) {
        s.resize((size_t)g.ss);
        for (auto& v : s) v = (uint8_t)rng();
      }
      uint32_t key = 0, seq = 0;
      size_t pi = 0;
      for (int i = 0; i < g.window; ++i) {  // pre-fill window
        gen->on_seal(pool[pi++ % pool.size()].data());
        ++seq;
      }
      long iters = 200;
      double dt = 0;
      for (;;) {
        std::vector<std::vector<uint8_t>> out;
        out.reserve(64);
        double t0 = now_s();
        for (long i = 0; i < iters; ++i) {
          gen->on_seal(pool[pi++ % pool.size()].data());
          ++seq;
          gen->make_repairs(seq, &key, 1, &out);
          if ((i & 0x3F) == 0x3F) out.clear();  // bound memory
        }
        dt = now_s() - t0;
        if (dt > 1.0) break;
        iters *= 4;
      }
      const double rps = iters / dt;
      const double src_mbps = rps * g.window * g.ss / 1e6;
      char geo[32];
      std::snprintf(geo, sizeof geo, "%dx w%d", g.ss, g.window);
      std::printf("%-24s %-11s %12.0f %12.1f %12.1f\n", r.name, geo, rps,
                  src_mbps, 1e6 / rps);
    }
  }
}

// --------------------------------------------------------------- encoder --
// End-to-end feed, source-identical to tools/bench/encbench.cpp run_point so
// SUST_air numbers line up across the two tools. Three engines:
//   base  the real UepEncoder (mabur code, unmodified)
//   copy  UepEncoderT<SwStock> — the bench replica hosting the stock engine
//         (sanity: must perform within noise of base, proving the replica
//         is faithful before trusting the mt row)
//   mt2   UepEncoderT<SwEncoderMt> — flat ring + row-split spin worker

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
  p[off] = 0; p[off + 1] = 0; p[off + 2] = 0; p[off + 3] = 1;
  p[off + 4] = (uint8_t)(nal_type << 1);
  p[off + 5] = 1;  // tid 0
  return p;
}

static std::array<UepLayerCfg, 4> make_layers(bool perlayer, double cmd_ov) {
  std::array<UepLayerCfg, 4> layers;
  const int syms[4] = {164, perlayer ? 1312 : 164, perlayer ? 1312 : 164,
                       perlayer ? 1312 : 164};
  const int bpb[4] = {perlayer ? 4 : 8, perlayer ? 1 : 8, perlayer ? 1 : 8,
                      perlayer ? 1 : 8};
  for (int s = 0; s < 4; ++s) {
    layers[(size_t)s].fec = SwConfig{syms[s], 64, uep_layer_overhead(s, cmd_ov)};
    layers[(size_t)s].blocks_per_body = bpb[s];
  }
  return layers;
}

// Feeds the synthetic stream; returns {wall_s, air_bytes, in_bytes}. When
// bodies != nullptr also captures every emitted body for byte-comparison.
template <class Enc>
static void feed_stream(Enc& uep, int ppf, int sim_seconds, double* wall,
                        uint64_t* air_bytes, uint64_t* in_bytes,
                        std::vector<std::vector<uint8_t>>* bodies = nullptr) {
  uint64_t now = 1;
  *air_bytes = 0;
  *in_bytes = 0;
  uint16_t id = 0;
  auto sink = [&](std::vector<UepBody> out) {
    for (auto& b : out) {
      *air_bytes += b.body.size();
      if (bodies) {
        b.body.insert(b.body.begin(), b.stream_id);  // tag for comparison
        bodies->push_back(std::move(b.body));
      }
    }
  };
  const double t0 = now_s();
  for (int s = 0; s < sim_seconds; ++s) {
    for (int f = 0; f < 60; ++f) {
      const int nal = (f == 0) ? 19 : 1;  // IDR_W_RADL vs TRAIL_R
      const int sid = (f == 0) ? 0 : 1;   // critical vs T0
      const auto p = make_frame(nal, (size_t)ppf * 1400, id++);
      sink(uep.add_frame(sid, p.data(), p.size(), now));
      *in_bytes += p.size();
      now += 16;
      sink(uep.poll(now));
    }
  }
  *wall = now_s() - t0;
}

// The mt engine inside the full composition must reproduce the stock body
// stream byte-for-byte, in order (its fork-join is synchronous). Random
// initial seqs make the REAL UepEncoder incomparable, so both sides run the
// bench replica with pinned seqs; the replica's own faithfulness is covered
// by the copy-vs-base perf row.
static int verify_encoder() {
  static const std::array<uint32_t, 4> kSeqs = {11, 2222, 333333, 44444444};
  int failures = 0;
  for (int perlayer = 0; perlayer < 2; ++perlayer) {
    for (double ov : {0.10, 0.375}) {
      auto layers = make_layers(perlayer != 0, ov);
      UepEncoderT<SwStock> stock(layers, 25, kSeqs, nullptr);
      SpinWorker worker(1);
      UepEncoderT<SwEncoderMt> mt(layers, 25, kSeqs, &worker);
      double w;
      uint64_t ab1, ib1, ab2, ib2;
      std::vector<std::vector<uint8_t>> b1, b2;
      feed_stream(stock, 13, 1, &w, &ab1, &ib1, &b1);
      feed_stream(mt, 13, 1, &w, &ab2, &ib2, &b2);
      const bool ok = b1 == b2;
      failures += !ok;
      std::printf("verify encoder %s ov=%.3f          %s\n",
                  perlayer ? "perlayer" : "scalar  ", ov, ok ? "OK" : "FAIL");
    }
  }
  return failures;
}

// The async engine relaxes emission ORDER (repairs trail sources), so it is
// verified at the engine level against stock SwEncoder: identical packet
// feed with flushes at identical points must yield the identical envelope
// SET — every envelope byte-exact, order free.
static int verify_async_engine() {
  int failures = 0;
  std::mt19937 rng(7);
  for (int gi = 0; gi < 2; ++gi) {
    const int ss = gi ? 1312 : 164;
    SwConfig cfg{ss, 64, 0.75};
    SwStock stock(cfg, 123456u + (uint32_t)gi, nullptr);
    AsyncFecWorker worker;
    SwEncoderMtAsync async(cfg, 123456u + (uint32_t)gi, &worker);
    std::vector<std::vector<uint8_t>> ea, eb;
    auto sink = [](std::vector<std::vector<uint8_t>> envs,
                   std::vector<std::vector<uint8_t>>& into) {
      for (auto& e : envs) into.push_back(std::move(e));
    };
    for (int i = 0; i < 3000; ++i) {
      const size_t len = 1 + rng() % (size_t)cfg.max_packet_size();
      std::vector<uint8_t> p(len);
      for (auto& v : p) v = (uint8_t)rng();
      sink(stock.add_packet(p.data(), p.size()), ea);
      sink(async.add_packet(p.data(), p.size()), eb);
      if (i % 97 == 96) {
        sink(stock.flush(), ea);
        sink(async.flush(), eb);
      }
    }
    sink(stock.flush(), ea);
    sink(async.flush(), eb);
    std::sort(ea.begin(), ea.end());
    std::sort(eb.begin(), eb.end());
    const bool ok = ea == eb;
    failures += !ok;
    std::printf("verify async-engine ss=%-4d            %s\n", ss,
                ok ? "OK" : "FAIL");
  }
  return failures;
}

static void print_enc_row(const char* eng, const char* mode, double ov,
                          double wall, int sim, uint64_t air_bytes,
                          uint64_t in_bytes) {
  const double speedup = sim / wall;
  const double in_mbps = in_bytes * 8.0 / sim / 1e6;
  const double air_mbps = air_bytes * 8.0 / sim / 1e6;
  std::printf("%-5s %-8s %5.3f %7.2f %7.2f %8.2f %10.2f %9.2f\n", eng, mode,
              ov, in_mbps, air_mbps, speedup, in_mbps * speedup,
              air_mbps * speedup);
}

static void run_encoder_bench(int sim) {
  static const std::array<uint32_t, 4> kSeqs = {11, 2222, 333333, 44444444};
  std::printf("\n## encoder: end-to-end base vs copy vs mt2 (%ds/point, ppf=13)\n",
              sim);
  std::printf("%-5s %-8s %5s %7s %7s %8s %10s %9s\n", "eng", "mode", "ov",
              "vid1x", "air1x", "speedup", "SUST_vid", "SUST_air");
  for (double ov : {0.10, 0.375}) {
    for (int perlayer = 0; perlayer < 2; ++perlayer) {
      const char* mode = perlayer ? "perlayer" : "scalar";
      auto layers = make_layers(perlayer != 0, ov);
      double wall;
      uint64_t air, in;
      {
        UepEncoder uep(layers, 25);
        feed_stream(uep, 13, sim, &wall, &air, &in);
        print_enc_row("base", mode, ov, wall, sim, air, in);
      }
      {
        UepEncoderT<SwStock> uep(layers, 25, kSeqs, nullptr);
        feed_stream(uep, 13, sim, &wall, &air, &in);
        print_enc_row("copy", mode, ov, wall, sim, air, in);
      }
      {
        SpinWorker worker(1);
        UepEncoderT<SwEncoderMt> uep(layers, 25, kSeqs, &worker);
        feed_stream(uep, 13, sim, &wall, &air, &in);
        print_enc_row("mt2j", mode, ov, wall, sim, air, in);
      }
      {
        AsyncFecWorker worker(1);
        UepEncoderT<SwEncoderMtAsync> uep(layers, 25, kSeqs, &worker);
        feed_stream(uep, 13, sim, &wall, &air, &in);
        print_enc_row("mt2a", mode, ov, wall, sim, air, in);
      }
      {
        // The PRODUCTION classes (common/ SwEncoder+FecWorker through the
        // real UepEncoder). Must reproduce mt2a within noise (~5%) — a
        // bigger gap means the productionization lost something.
        FecWorker worker(1);
        UepEncoder uep(layers, 25, &worker);
        feed_stream(uep, 13, sim, &wall, &air, &in);
        print_enc_row("prod", mode, ov, wall, sim, air, in);
      }
    }
  }
}

// ------------------------------------------------------------------ main --

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "all";
  const int sim = argc > 2 ? std::atoi(argv[2]) : 6;

  // Main thread on cpu0, SpinWorker on cpu1: removes scheduler-migration
  // noise from every row (single- and dual-core alike).
  pin_to_cpu(0);

  std::printf("# fecbench  gf=%s  mabur=%s  %zu-bit\n", gf::backend(),
              FECBENCH_SHA, sizeof(void*) * 8);

  int failures = 0;
  if (mode == "all" || mode == "verify")
    failures = run_verify() + verify_encoder() + verify_async_engine();
  if (failures) {
    std::printf("verification FAILED — benchmarks of wrong code are noise\n");
    return 1;
  }
  if (mode == "all" || mode == "kernel") run_kernel_bench();
  if (mode == "all" || mode == "repair") run_repair_bench();
  if (mode == "all" || mode == "encoder") run_encoder_bench(sim);
  return 0;
}
