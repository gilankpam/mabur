#include "mtest.h"
#include "radio_frontend.h"
using namespace maburgs;

TEST(control_frame_structure) {
  const uint8_t body[4] = {'R', 'C', 0x01, 0x02};
  auto f = build_control_frame(0x123, body, sizeof(body));
  // radiotap: version 0, LE length at [2:4]
  REQUIRE(f.size() > 24 + 4);
  CHECK(f[0] == 0);
  const size_t rl = static_cast<size_t>(f[2] | (f[3] << 8));
  REQUIRE(f.size() == rl + 24 + 4);
  const uint8_t* d11 = f.data() + rl;
  CHECK(d11[0] == 0x40);                            // probe-req
  for (int i = 4; i < 10; ++i) CHECK(d11[i] == 0xFF);  // broadcast DA
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  for (int i = 0; i < 6; ++i) { CHECK(d11[10 + i] == sa[i]); CHECK(d11[16 + i] == sa[i]); }
  const uint16_t seq_ctl = static_cast<uint16_t>(d11[22] | (d11[23] << 8));
  CHECK((seq_ctl >> 4) == 0x123);
  CHECK(f[rl + 24] == 'R');
  // Same seq -> identical bytes (radiotap is cached/constant per process).
  CHECK(build_control_frame(0x123, body, sizeof(body)) == f);
}
MTEST_MAIN
