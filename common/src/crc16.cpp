#include "mabur/crc16.h"

#include <array>

namespace mabur {
namespace {
// CRC-16/CCITT-FALSE, poly 0x1021, MSB-first, slicing-by-N: kTable[0] is the
// classic one-lookup-per-byte table, built at compile time from the same
// bit-serial recurrence the reference (tests/test_crc16.cpp, devourer's
// Python) uses; kTable[k][b] is the CRC of byte b followed by k zero bytes.
// N bytes then cost N independent lookups instead of N chained ones (the
// chained load-to-use is what bounds the byte loop on the Cortex-A7).
// 8 slices (4 KB of tables) measured 5.1 ns/B on the SSC338Q against 10.2
// for the single table; 2 and 4 slices landed at 7.1 and 6.5.
constexpr int kN = 8;
using Tables = std::array<std::array<uint16_t, 256>, kN>;
constexpr Tables make_tables() {
  Tables t{};
  for (int i = 0; i < 256; ++i) {
    uint16_t crc = static_cast<uint16_t>(i << 8);
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    t[0][static_cast<size_t>(i)] = crc;
  }
  for (int k = 1; k < kN; ++k)
    for (size_t i = 0; i < 256; ++i) {
      const uint16_t p = t[static_cast<size_t>(k - 1)][i];
      t[static_cast<size_t>(k)][i] = static_cast<uint16_t>((p << 8) ^ t[0][p >> 8]);
    }
  return t;
}
constexpr Tables kTable = make_tables();
}  // namespace

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init) {
  uint16_t crc = init;
  size_t i = 0;
  for (; i + kN <= len; i += kN) {
    // The state folds into the first two bytes; byte j has kN-1-j bytes after it.
    uint16_t x = kTable[kN - 1][(crc >> 8) ^ data[i]] ^
                 kTable[kN - 2][(crc & 0xFF) ^ data[i + 1]];
    for (int j = 2; j < kN; ++j) x ^= kTable[kN - 1 - j][data[i + j]];
    crc = x;
  }
  for (; i < len; ++i)
    crc = static_cast<uint16_t>((crc << 8) ^ kTable[0][((crc >> 8) ^ data[i]) & 0xFF]);
  return crc;
}

}  // namespace mabur
