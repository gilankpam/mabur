// relay_wire (gs/src/relay_wire.h): mabur's side of the mabur-relay v3
// contract. Vectors are the relay repo's tests/test_wire.c goldens.
#include <cstring>
#include "mtest.h"
#include "relay_wire.h"
using namespace maburgs::relay;

TEST(frame_golden) {
  const uint8_t b[] = {0x4D, 0x52, 0x03, 0x01, 0x04, 0x03, 0x02, 0x01, 136, 2, 0x15, 4,
                       0xDB, 0xD4, 0xA1, 0xA1, 0xD4, 0xC3, 0xB2, 0xA1, 0x88, 0x01};
  FrameMeta m;
  const uint8_t* d = nullptr;
  size_t dl = 0;
  REQUIRE(msg_type(b, sizeof b) == kFrame);
  REQUIRE(parse_frame(b, sizeof b, m, d, dl));
  CHECK(m.seq == 0x01020304u && m.rx_channel == 136 && m.sec == 2);
  CHECK(m.flags == 0x15 && m.mcs == 4);
  CHECK(m.rssi[0] == -37 && m.rssi[1] == -44 && m.noise[0] == -95 && m.noise[1] == -95);
  CHECK(m.tsf_lo == 0xA1B2C3D4u);
  CHECK(d == b + 20 && dl == 2);
  CHECK(!parse_frame(b, 19, m, d, dl));
}

TEST(tune_golden) {
  const auto t = pack_tune(0xBEEF, 149, 1);
  const uint8_t want[] = {0x4D, 0x52, 0x03, 0x03, 0xEF, 0xBE, 149, 1};
  REQUIRE(t.size() == sizeof want);
  CHECK(std::memcmp(t.data(), want, sizeof want) == 0);
  const auto h = pack_hello();
  const uint8_t hw[] = {0x4D, 0x52, 0x03, 0x02};
  CHECK(h.size() == 4 && std::memcmp(h.data(), hw, 4) == 0);
}

TEST(status_golden) {
  uint8_t b[kStatusLen] = {0x4D, 0x52, 0x03, 0x04, 7, 0, 0, 136, 2, 1, 1, 0x44, 0x33, 0x22, 0x11};
  const uint8_t tail[] = {0xD0, 0xC0, 0xB0, 0xA0, 0x04, 0x03, 0x02, 0x01,
                          0x06, 0x00, 0x00, 0x00, 0x10, 0x00, 0xF0, 0xE0};
  std::memcpy(b + kStatusLen - 16, tail, 16);
  Status s;
  REQUIRE(parse_status(b, sizeof b, s));
  CHECK(s.tune_id == 7 && s.state == 0 && s.channel == 136 && s.sec == 2);
  CHECK(s.owner == 1 && s.you_own == 1 && s.rx == 0x11223344u);
  CHECK(s.uptime_s == 0xA0B0C0D0u && s.tx == 0x01020304u && s.tx_fail == 6);
  CHECK(s.tx_refused == 0xE0F00010u);
  CHECK(!parse_status(b, kStatusLen - 1, s));
}

TEST(tx_golden) {
  uint8_t dot11[24];
  for (int i = 0; i < 24; ++i) dot11[i] = static_cast<uint8_t>(0x40 + i);
  const auto t = pack_tx(0, kTxLdpc | kTxStbc, dot11, sizeof dot11);
  const uint8_t head[] = {0x4D, 0x52, 0x03, 0x05, 0x00, 0x03};
  REQUIRE(t.size() == kTxHdrLen + 24);
  CHECK(std::memcmp(t.data(), head, sizeof head) == 0);
  CHECK(std::memcmp(t.data() + kTxHdrLen, dot11, 24) == 0);
}

TEST(msg_type_rejects) {
  uint8_t b[] = {0x4D, 0x52, 0x02, 0x04};
  CHECK(msg_type(b, 4) == -1);   // v2 is gone
  b[2] = 0x03;
  CHECK(msg_type(b, 3) == -1);
  b[0] = 0x00;
  CHECK(msg_type(b, 4) == -1);
}

MTEST_MAIN
