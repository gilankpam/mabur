// On-target GF256 lincomb verify + throughput bench, plus an end-to-end
// UepEncoder packets/second measurement at the bench geometry. Verifies the
// NEON vtbl path byte-for-byte against a local scalar reference (the same
// math test_gf256's golden vectors pin on the host).
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "mabur/gf256.h"
#include "mabur/frame_wire.h"
#include "mabur/uep_encoder.h"

using namespace mabur;

static uint8_t g_exp[512], g_log[256];
static void build_tables() {
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

int main() {
  build_tables();
  std::mt19937 rng(1);

  // verify: random lengths (odd tails), coeffs, data
  for (int it = 0; it < 2000; ++it) {
    size_t n = 1 + rng() % 300;
    std::vector<uint8_t> a(n), b(n), sym(n);
    for (auto& v : sym) v = (uint8_t)rng();
    for (size_t i = 0; i < n; ++i) a[i] = b[i] = (uint8_t)rng();
    uint8_t c = (uint8_t)rng();
    gf::lincomb(a.data(), sym.data(), c, n);
    ref_lincomb(b.data(), sym.data(), c, n);
    if (std::memcmp(a.data(), b.data(), n) != 0) {
      std::printf("VERIFY FAIL at it=%d n=%zu c=%u\n", it, n, c);
      return 1;
    }
  }
  std::printf("verify: OK\n");

  // verify lincomb_rows (the repair kernel) against n accumulated
  // reference lincombs: random row counts, odd tails, coeffs incl. 0.
  for (int it = 0; it < 2000; ++it) {
    const int n = (int)(rng() % 40);
    const size_t len = 1 + rng() % 400;
    std::vector<std::vector<uint8_t>> rows((size_t)n, std::vector<uint8_t>(len));
    std::vector<const uint8_t*> ptrs;
    std::vector<uint8_t> coeffs;
    for (auto& r : rows) {
      for (auto& v : r) v = (uint8_t)rng();
      ptrs.push_back(r.data());
      coeffs.push_back((uint8_t)(it % 7 == 0 ? 0 : rng()));
    }
    std::vector<uint8_t> want(len, 0), got(len, 0xA5);
    for (int k = 0; k < n; ++k) ref_lincomb(want.data(), ptrs[(size_t)k], coeffs[(size_t)k], len);
    gf::lincomb_rows(got.data(), ptrs.data(), coeffs.data(), n, len);
    if (got != want) {
      std::printf("VERIFY_ROWS FAIL at it=%d n=%d len=%zu\n", it, n, len);
      return 1;
    }
  }
  std::printf("verify_rows: OK\n");

  // repair-shaped throughput: one 32-row window at symbol size 332 (the
  // flight geometry), per-row lincomb vs lincomb_rows, unpadded (332,
  // 12 B scalar tail per row) and padded to the ring stride (336, what
  // SwEncoder::build_repair passes since 2026-09-22). MB/s counts row
  // bytes read; us/repair is the number the fec_worker gauge reports.
  // "hot": the same 32 rows every repair (L1-resident). "cold": a
  // 544-row ring (window + kSlackRows, 183 kB > L1) with the window
  // sliding one row per repair, as the encoder's ring does in situ.
  for (int cold = 0; cold < 2; ++cold)
  for (size_t len : {(size_t)332, (size_t)336}) {
    const int n = 32;
    const size_t nrows = cold ? 544 : 32;
    std::vector<std::vector<uint8_t>> rows(nrows, std::vector<uint8_t>(len + 16));
    for (auto& r : rows) for (auto& v : r) v = (uint8_t)rng();
    std::vector<uint8_t> coeffs;
    for (int k = 0; k < n; ++k) coeffs.push_back((uint8_t)(1 + rng() % 255));
    std::vector<const uint8_t*> ptrs((size_t)n);
    auto window = [&](long i) {
      for (int k = 0; k < n; ++k) ptrs[(size_t)k] = rows[((size_t)i + (size_t)k) % nrows].data();
    };
    std::vector<uint8_t> out(len + 16);
    const long iters = 20000;
    auto t0 = std::chrono::steady_clock::now();
    for (long i = 0; i < iters; ++i) {
      window(i);
      std::memset(out.data(), 0, len);
      for (int k = 0; k < n; ++k) gf::lincomb(out.data(), ptrs[(size_t)k], coeffs[(size_t)k], len);
    }
    double dt_row = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    t0 = std::chrono::steady_clock::now();
    for (long i = 0; i < iters; ++i) {
      window(i);
      gf::lincomb_rows(out.data(), ptrs.data(), coeffs.data(), n, len);
    }
    double dt_rows = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double bytes = (double)iters * n * len;
    std::printf("repair 32x%zu %s: per-row lincomb %.1f MB/s %.1f us/repair | lincomb_rows %.1f MB/s %.1f us/repair\n",
                len, cold ? "cold" : "hot ", bytes / dt_row / 1e6, dt_row / iters * 1e6,
                bytes / dt_rows / 1e6, dt_rows / iters * 1e6);
  }

  // raw lincomb throughput
  {
    std::vector<uint8_t> acc(164), sym(164);
    for (auto& v : sym) v = (uint8_t)rng();
    auto t0 = std::chrono::steady_clock::now();
    const long iters = 400000;
    for (long i = 0; i < iters; ++i)
      gf::lincomb(acc.data(), sym.data(), (uint8_t)(1 + (i & 0xFE)), 164);
    auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("lincomb: %.1f MB/s\n", iters * 164.0 / dt / 1e6);
  }

  // end-to-end encoder frames/s at bench geometry (symbol 164, bpb 8,
  // window 128, overhead 0.75 literal — Task 3 deleted the kUepRefOverhead
  // ladder; 0.75 reproduces the exact pre-Task-3 value byte-for-byte
  // (kUepRefOverhead was flat 0.50 at every index post-2026-08-29-flatten,
  // and this file already fed the *1.5-scaled result straight into
  // SwConfig as the actually-applied overhead, so no x2 rule applies here
  // — it was never a cmd value routed through uep_layer_overhead).
  // 14000 B frames at 60 fps ~ 6.7 Mbps of video, the shape maburd ingests.
  {
    std::array<UepLayerCfg, 2> layers{};
    for (int s = 0; s < 2; ++s) {
      layers[s].fec = SwConfig{164, 128, 0.75};
      layers[s].blocks_per_body = 8;
    }
    UepEncoder enc(layers, 15);
    const size_t kFrameBytes = 14000;
    std::vector<uint8_t> unit(framewire::kFrameHdrLen + kFrameBytes);
    const size_t off = framewire::kFrameHdrLen;
    unit[off] = 0; unit[off + 1] = 0; unit[off + 2] = 0; unit[off + 3] = 1;
    unit[off + 4] = 1 << 1;  // TRAIL_R
    unit[off + 5] = 1;       // tid 0
    for (size_t i = off + 6; i < unit.size(); ++i) unit[i] = (uint8_t)rng();
    const int nframes = 2000;
    uint64_t now = 1000, bodies = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < nframes; ++i) {
      framewire::FrameHdr h;
      h.frame_id = (uint16_t)i;
      h.pts_us = (uint32_t)i * 16667u;
      framewire::pack_frame_hdr(h, unit.data());
      bodies += enc.add_frame(1, unit.data(), unit.size(), now).size();
      now += 16;
    }
    auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("encoder: %.0f frames/s (%.1f Mbps video), %llu bodies\n",
                nframes / dt, nframes / dt * (double)kFrameBytes * 8 / 1e6,
                (unsigned long long)bodies);
  }
  return 0;
}
