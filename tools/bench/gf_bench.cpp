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

  // end-to-end encoder pkt/s at bench geometry (symbol 164, bpb 8, window 128,
  // overhead ladder at cmd 0.375 equivalent: use ref ladder scaled 1.5)
  {
    std::array<UepLayerCfg, 4> layers{};
    for (int s = 0; s < 4; ++s) {
      layers[s].fec = SwConfig{164, 128, kUepRefOverhead[s] * 1.5};
      layers[s].blocks_per_body = 8;
    }
    UepEncoder enc(layers, 15);
    std::vector<uint8_t> pkt(12 + 1388);
    pkt[0] = 0x80; pkt[1] = 0x60;
    pkt[12] = 49 << 1; pkt[13] = 1; pkt[14] = 1;
    for (size_t i = 15; i < pkt.size(); ++i) pkt[i] = (uint8_t)rng();
    const int npkt = 20000;
    uint64_t now = 1000, bodies = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < npkt; ++i) {
      pkt[2] = (uint8_t)(i >> 8); pkt[3] = (uint8_t)i;
      bodies += enc.add_rtp(pkt.data(), pkt.size(), now).size();
      if (i % 20 == 19) ++now;
    }
    auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("encoder: %.0f pkt/s (%.1f Mbps video), %llu bodies\n",
                npkt / dt, npkt / dt * 1400 * 8 / 1e6,
                (unsigned long long)bodies);
  }
  return 0;
}
