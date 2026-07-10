#pragma once
#include <cstddef>
#include <cstdint>
namespace mabur {
// CRC-16-CCITT-FALSE (poly 0x1021, init 0xFFFF) — byte-identical to
// devourer fec_subblock.crc16_ccitt.
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);
}
