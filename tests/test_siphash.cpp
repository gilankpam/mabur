#include "mtest.h"
#include "mabur/siphash.h"
using namespace mabur;

static std::array<uint8_t, 16> ref_key() {
  std::array<uint8_t, 16> k{};
  for (int i = 0; i < 16; ++i) k[static_cast<size_t>(i)] = static_cast<uint8_t>(i);
  return k;
}

// Reference vectors from the SipHash paper / reference vectors.h: key
// 00..0f, message 00..n-1. Output as the u64 the bytes spell little-endian.
TEST(siphash24_reference_vectors) {
  const auto k = ref_key();
  uint8_t msg[15];
  for (int i = 0; i < 15; ++i) msg[i] = static_cast<uint8_t>(i);
  CHECK(siphash24(k, msg, 0) == 0x726fdb47dd0e0e31ULL);
  CHECK(siphash24(k, msg, 1) == 0x74f839c593dc67fdULL);
  CHECK(siphash24(k, msg, 15) == 0xa129ca6149be45e5ULL);
}

TEST(siphash24_key_and_message_sensitivity) {
  auto k = ref_key();
  const uint8_t m[4] = {1, 2, 3, 4};
  const uint64_t a = siphash24(k, m, 4);
  k[0] ^= 1;
  CHECK(siphash24(k, m, 4) != a);
  const uint8_t m2[4] = {1, 2, 3, 5};
  CHECK(siphash24(ref_key(), m2, 4) != a);
  CHECK(siphash24(ref_key(), m, 3) != a);
}
MTEST_MAIN
