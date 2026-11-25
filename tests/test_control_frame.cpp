// control_frame.h: the 802.11 management header and the radiotap + header +
// body assembly every control-path frame goes through.
#include <vector>
#include "mtest.h"
#include "control_frame.h"
using namespace mabur;

TEST(dot11_header_layout) {
  const auto h = build_dot11_header(0x123);
  REQUIRE(h.size() == 24);
  CHECK(h[0] == 0x40);
  CHECK(h[1] == 0 && h[2] == 0 && h[3] == 0);
  for (int i = 4; i < 10; ++i) CHECK(h[i] == 0xff);
  for (int i = 0; i < 6; ++i) CHECK(h[10 + i] == kCanonicalSa[i]);
  for (int i = 0; i < 6; ++i) CHECK(h[16 + i] == kCanonicalSa[i]);
  CHECK(h[22] == 0x30);
  CHECK(h[23] == 0x12);
}

TEST(control_frame_is_radiotap_header_body) {
  const std::vector<uint8_t> rt = {1, 2, 3}, body = {9, 8};
  const auto f = build_control_frame(rt, 5, body.data(), 2);
  REQUIRE(f.size() == 3 + 24 + 2);
  for (int i = 0; i < 3; ++i) CHECK(f[i] == rt[i]);
  const auto h = build_dot11_header(5);
  for (int i = 0; i < 24; ++i) CHECK(f[3 + i] == h[i]);
  CHECK(f[27] == 9);
  CHECK(f[28] == 8);
}

MTEST_MAIN
