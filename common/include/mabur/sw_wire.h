#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur::sw {

// Sliding-window FEC wire envelope: ONE fixed-size 14-byte header for both
// source and repair symbols — SbiPacker/sbi_unpack partition bodies by a
// config-authoritative fixed block_payload, so the two types must be the
// same length. Sources carry window_len = 0, repair_key = 0.
//
// <HBHIBI> little-endian:
//   MAGIC=0xF541(u16), flags(u8: bit0 = repair), symbol_size(u16),
//   seq(u32: source's own seq | repair's window_start),
//   window_len(u8), repair_key(u32)
constexpr uint16_t kSwMagic = 0xF541;
constexpr size_t kSwHeaderLen = 14;
constexpr uint8_t kFlagRepair = 0x01;
constexpr int kMaxWindow = 255;  // window_len is u8

struct SwHeader {
  bool repair = false;
  uint16_t symbol_size = 0;
  uint32_t seq = 0;
  uint8_t window_len = 0;
  uint32_t repair_key = 0;
};

void pack_header(std::vector<uint8_t>& out, const SwHeader& h);

// False on short/bad-magic/unknown-flags input or type-inconsistent fields
// (repair with window_len 0, source with nonzero window_len/repair_key).
bool parse_header(const uint8_t* p, size_t len, SwHeader* h);

// FROZEN coefficient contract (pinned by tests/vectors/sw.json against
// tools/pyref/sw_fec.py): splitmix64 seeded with repair_key, one draw per
// window position, low byte mapped to [1,255] via (b % 255) + 1. Coefficient
// i scales source symbol (window_start + i). Never zero, so every repair
// covers its full window.
void repair_coeffs(uint32_t repair_key, int window_len, uint8_t* out);

}  // namespace mabur::sw
