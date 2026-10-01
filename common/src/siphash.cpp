#include "mabur/siphash.h"

namespace mabur {
namespace {
inline uint64_t rotl(uint64_t x, int b) { return (x << b) | (x >> (64 - b)); }
inline uint64_t u8to64_le(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
  return v;
}
inline void sipround(uint64_t& v0, uint64_t& v1, uint64_t& v2, uint64_t& v3) {
  v0 += v1; v1 = rotl(v1, 13); v1 ^= v0; v0 = rotl(v0, 32);
  v2 += v3; v3 = rotl(v3, 16); v3 ^= v2;
  v0 += v3; v3 = rotl(v3, 21); v3 ^= v0;
  v2 += v1; v1 = rotl(v1, 17); v1 ^= v2; v2 = rotl(v2, 32);
}
}  // namespace

uint64_t siphash24(const std::array<uint8_t, 16>& key, const uint8_t* in, size_t len) {
  const uint64_t k0 = u8to64_le(key.data());
  const uint64_t k1 = u8to64_le(key.data() + 8);
  uint64_t v0 = 0x736f6d6570736575ULL ^ k0;
  uint64_t v1 = 0x646f72616e646f6dULL ^ k1;
  uint64_t v2 = 0x6c7967656e657261ULL ^ k0;
  uint64_t v3 = 0x7465646279746573ULL ^ k1;
  const uint8_t* end = in + (len & ~static_cast<size_t>(7));
  uint64_t b = static_cast<uint64_t>(len) << 56;
  for (; in != end; in += 8) {
    const uint64_t m = u8to64_le(in);
    v3 ^= m;
    sipround(v0, v1, v2, v3);
    sipround(v0, v1, v2, v3);
    v0 ^= m;
  }
  const size_t left = len & 7;
  for (size_t i = 0; i < left; ++i) b |= static_cast<uint64_t>(in[i]) << (8 * i);
  v3 ^= b;
  sipround(v0, v1, v2, v3);
  sipround(v0, v1, v2, v3);
  v0 ^= b;
  v2 ^= 0xff;
  for (int i = 0; i < 4; ++i) sipround(v0, v1, v2, v3);
  return v0 ^ v1 ^ v2 ^ v3;
}
}  // namespace mabur
