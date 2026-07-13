#include "mabur/sw_wire.h"

namespace mabur::sw {
namespace {
uint64_t splitmix64(uint64_t& s) {
  uint64_t z = (s += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}
void put16(std::vector<uint8_t>& o, uint16_t v) {
  o.push_back(static_cast<uint8_t>(v & 0xFF));
  o.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}
void put32(std::vector<uint8_t>& o, uint32_t v) {
  for (int i = 0; i < 4; ++i) o.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}
uint16_t rd16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace

void pack_header(std::vector<uint8_t>& out, const SwHeader& h) {
  put16(out, kSwMagic);
  out.push_back(h.repair ? kFlagRepair : 0);
  put16(out, h.symbol_size);
  put32(out, h.seq);
  out.push_back(h.window_len);
  put32(out, h.repair_key);
}

bool parse_header(const uint8_t* p, size_t len, SwHeader* h) {
  if (len < kSwHeaderLen || rd16(p) != kSwMagic) return false;
  const uint8_t flags = p[2];
  if (flags & static_cast<uint8_t>(~kFlagRepair)) return false;
  h->repair = (flags & kFlagRepair) != 0;
  h->symbol_size = rd16(p + 3);
  h->seq = rd32(p + 5);
  h->window_len = p[9];
  h->repair_key = rd32(p + 10);
  if (h->repair) {
    if (h->window_len == 0) return false;
  } else {
    if (h->window_len != 0 || h->repair_key != 0) return false;
  }
  return true;
}

void repair_coeffs(uint32_t repair_key, int window_len, uint8_t* out) {
  uint64_t s = repair_key;
  for (int i = 0; i < window_len; ++i)
    out[i] = static_cast<uint8_t>((splitmix64(s) & 0xFF) % 255 + 1);
}

}  // namespace mabur::sw
