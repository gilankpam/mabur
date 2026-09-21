#include "mabur/crc16.h"

#include <array>

namespace mabur {
namespace {
// CRC-16/CCITT-FALSE, poly 0x1021, MSB-first, one table lookup per byte.
// The 256-entry table is built at compile time from the same bit-serial
// recurrence the reference (tests/test_crc16.cpp, devourer's Python) uses,
// so the two stay byte-identical by construction. 512 B, read-only.
constexpr std::array<uint16_t, 256> make_table() {
  std::array<uint16_t, 256> t{};
  for (int i = 0; i < 256; ++i) {
    uint16_t crc = static_cast<uint16_t>(i << 8);
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    t[static_cast<size_t>(i)] = crc;
  }
  return t;
}
constexpr std::array<uint16_t, 256> kTable = make_table();
}  // namespace

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i)
    crc = static_cast<uint16_t>((crc << 8) ^ kTable[((crc >> 8) ^ data[i]) & 0xFF]);
  return crc;
}

}  // namespace mabur
