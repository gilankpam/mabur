// Host tests for drone/venc/rec_frame.h: the record drain's per-NAL
// start-code -> length-prefix rewrite (VTX recorder CPU diet, 2026-09-26).
#include <cstring>
#include <vector>

#include "mtest.h"
#include "rec_frame.h"

namespace {
std::vector<uint8_t> slice(std::initializer_list<uint8_t> sc, uint8_t type,
                           std::initializer_list<uint8_t> body) {
  std::vector<uint8_t> v(sc);
  v.push_back(static_cast<uint8_t>(type << 1));
  v.push_back(0x01);
  v.insert(v.end(), body);
  return v;
}
}  // namespace

TEST(four_byte_start_code_becomes_big_endian_length) {
  const auto s = slice({0, 0, 0, 1}, 19, {0xAA, 0xBB, 0xCC});
  uint8_t out[32] = {};
  int type = -1;
  const size_t w = rec_put_nal(out, sizeof(out), s.data(), s.size(), &type);
  REQUIRE(w == 4 + 5);
  CHECK(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 5);
  CHECK(std::memcmp(out + 4, s.data() + 4, 5) == 0);
  CHECK(type == 19);
}

TEST(three_byte_start_code_grows_by_one) {
  const auto s = slice({0, 0, 1}, 1, {0x01, 0x02});
  uint8_t out[32] = {};
  int type = -1;
  const size_t w = rec_put_nal(out, sizeof(out), s.data(), s.size(), &type);
  REQUIRE(w == 4 + 4);                     // 3-byte code in, 4-byte length out
  CHECK(out[3] == 4);
  CHECK(std::memcmp(out + 4, s.data() + 3, 4) == 0);
  CHECK(type == 1);
}

TEST(long_nal_length_uses_all_four_bytes) {
  std::vector<uint8_t> s = {0, 0, 0, 1, static_cast<uint8_t>(1 << 1), 0x01};
  s.resize(4 + 0x012345, 0x77);
  std::vector<uint8_t> out(s.size() + 8);
  int type = -1;
  REQUIRE(rec_put_nal(out.data(), out.size(), s.data(), s.size(), &type) == 4 + 0x012345);
  CHECK(out[0] == 0x00 && out[1] == 0x01 && out[2] == 0x23 && out[3] == 0x45);
}

TEST(rejects_what_it_cannot_rewrite) {
  uint8_t out[32];
  int type = -1;
  const auto no_sc = slice({0, 1, 0}, 1, {0x01});           // not a start code
  CHECK(rec_put_nal(out, sizeof(out), no_sc.data(), no_sc.size(), &type) == 0);
  const uint8_t only_sc[] = {0, 0, 0, 1};                    // nothing after it
  CHECK(rec_put_nal(out, sizeof(out), only_sc, sizeof(only_sc), &type) == 0);
  const auto s = slice({0, 0, 0, 1}, 1, {0x01, 0x02, 0x03});
  CHECK(rec_put_nal(out, 8, s.data(), s.size(), &type) == 0);  // 4 + 5 > 8
  CHECK(rec_put_nal(out, sizeof(out), s.data(), 2, &type) == 0);
}

MTEST_MAIN
