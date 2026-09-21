// CRC-16/CCITT-FALSE microbench: the production mabur::crc16_ccitt against a
// bit-serial reference, over the FEC envelope size the drone's hot thread
// CRCs ~200 times per frame (2026-09-21 bitrate-ceiling findings). Prints
// ns/byte and the per-frame cost at a given air-bytes-per-frame figure.
//
//   crc_bench [block_bytes=332] [air_kb_per_frame=72]
//
// Host build: tests CMake target `crc_bench`. Drone: cross-build like
// encbench (docs/per-layer-symbol-debug-handoff.md) with common/src/crc16.cpp.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "mabur/crc16.h"

namespace {
uint16_t crc16_bitwise(const uint8_t* data, size_t len, uint16_t init = 0xFFFF) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

template <typename F>
double ns_per_byte(F fn, const std::vector<std::vector<uint8_t>>& blocks, double seconds) {
  using clock = std::chrono::steady_clock;
  volatile uint16_t sink = 0;
  size_t bytes = 0;
  const auto t0 = clock::now();
  while (std::chrono::duration<double>(clock::now() - t0).count() < seconds) {
    for (auto& b : blocks) {
      sink = static_cast<uint16_t>(sink ^ fn(b.data(), b.size()));
      bytes += b.size();
    }
  }
  const double ns = std::chrono::duration<double, std::nano>(clock::now() - t0).count();
  return ns / static_cast<double>(bytes);
}
}  // namespace

int main(int argc, char** argv) {
  const size_t block = argc > 1 ? static_cast<size_t>(std::atoi(argv[1])) : 332;
  const double air_kb = argc > 2 ? std::atof(argv[2]) : 72.0;
  std::vector<std::vector<uint8_t>> blocks(64, std::vector<uint8_t>(block));
  uint32_t x = 0x9E3779B9;
  for (auto& b : blocks)
    for (auto& v : b) { x = x * 1664525u + 1013904223u; v = static_cast<uint8_t>(x >> 24); }
  // Same answer or the bench is meaningless.
  for (auto& b : blocks)
    if (mabur::crc16_ccitt(b.data(), b.size()) != crc16_bitwise(b.data(), b.size())) {
      std::fprintf(stderr, "MISMATCH between production and reference CRC\n");
      return 1;
    }
  const double ref = ns_per_byte([](const uint8_t* d, size_t n) { return crc16_bitwise(d, n); }, blocks, 1.0);
  const double prod = ns_per_byte([](const uint8_t* d, size_t n) { return mabur::crc16_ccitt(d, n); }, blocks, 1.0);
  const double bytes_per_frame = air_kb * 1024.0;
  std::printf("block %zu B, %.0f kB air/frame\n", block, air_kb);
  std::printf("bitwise   %6.2f ns/B  -> %6.2f ms/frame\n", ref, ref * bytes_per_frame / 1e6);
  std::printf("table     %6.2f ns/B  -> %6.2f ms/frame\n", prod, prod * bytes_per_frame / 1e6);
  std::printf("speedup   %.1fx\n", ref / prod);
  return 0;
}
