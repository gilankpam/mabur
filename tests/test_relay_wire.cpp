// relay_wire (gs/src/relay_wire.h): mabur's side of the mabur-relay v4
// contract. Vectors are the relay repo's tests/test_wire.c goldens.
#include <cstring>
#include "mtest.h"
#include "relay_wire.h"
using namespace maburgs::relay;

TEST(frame_golden) {
  const uint8_t b[] = {0x4D, 0x52, 0x04, 0x01, 0x04, 0x03, 0x02, 0x01, 136, 2, 0x15, 4,
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
  const uint8_t want[] = {0x4D, 0x52, 0x04, 0x03, 0xEF, 0xBE, 149, 1};
  REQUIRE(t.size() == sizeof want);
  CHECK(std::memcmp(t.data(), want, sizeof want) == 0);
  const auto h = pack_hello();
  const uint8_t hw[] = {0x4D, 0x52, 0x04, 0x02};
  CHECK(h.size() == 4 && std::memcmp(h.data(), hw, 4) == 0);
}

TEST(status_golden) {
  uint8_t b[kStatusLen] = {0x4D, 0x52, 0x04, 0x04, 7, 0, 0, 136, 2, 1, 1, 0x44, 0x33, 0x22, 0x11};
  const uint8_t tail[] = {0xD0, 0xC0, 0xB0, 0xA0, 0x04, 0x03, 0x02, 0x01,
                          0x06, 0x00, 0x00, 0x00, 0x10, 0x00, 0xF0, 0xE0};
  std::memcpy(b + 31, tail, 16);   // uptime_s, tx, tx_fail, tx_refused -- fixed offsets, not the tail
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
  const uint8_t head[] = {0x4D, 0x52, 0x04, 0x05, 0x00, 0x03};
  REQUIRE(t.size() == kTxHdrLen + 24);
  CHECK(std::memcmp(t.data(), head, sizeof head) == 0);
  CHECK(std::memcmp(t.data() + kTxHdrLen, dot11, 24) == 0);
}

TEST(msg_type_rejects) {
  uint8_t b[] = {0x4D, 0x52, 0x03, 0x04};
  CHECK(msg_type(b, 4) == -1);   // v3 is gone
  b[2] = 0x04;
  CHECK(msg_type(b, 3) == -1);
  b[0] = 0x00;
  CHECK(msg_type(b, 4) == -1);
}

TEST(survey_golden_matches_the_relay) {
  const std::vector<uint8_t> b = {0x4D, 0x52, 0x04, 0x06, 112, 2, 0x02, 0x01,
      0xE8, 0x03, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x0D, 0x0C, 0x0B, 0x0A, 0x07, 0x00, 0x00, 0x00};
  Survey s;
  REQUIRE(parse_survey(b.data(), b.size(), s));
  CHECK(s.channel == 112 && s.sec == 2 && s.gen == 0x0102);
  CHECK(s.active_ms == 1000 && s.busy_ms == 300 && s.rx_ms == 100 && s.tx_ms == 0);
  CHECK(s.ofdm_err == 0x0A0B0C0Du && s.foreign == 7);
  CHECK(!parse_survey(b.data(), b.size() - 1, s));
}

TEST(scan_golden_matches_the_relay) {
  const auto b = pack_scan(0xBEEF, 2, 20, {40, 64, 144});
  const std::vector<uint8_t> want = {0x4D, 0x52, 0x04, 0x07, 0xEF, 0xBE, 2, 20, 3, 40, 64, 144};
  CHECK(b == want);
}

TEST(scan_result_golden_matches_the_relay) {
  const std::vector<uint8_t> b = {0x4D, 0x52, 0x04, 0x08, 0xEF, 0xBE, 0, 112, 2, 1,
                                  64, 1, 1, 0x14, 0x00, 0x0E, 0x00, 0x01, 0x00,
                                  0x03, 0x00, 0x02, 0x01};
  maburgs::SweepResult r;
  REQUIRE(parse_scan_result(b.data(), b.size(), r));
  CHECK(r.scan_id == 0xBEEF && r.status == 0 && r.back_channel == 112 && r.back_sec == 2);
  REQUIRE(r.entries.size() == 1);
  const auto& e = r.entries[0];
  CHECK(e.ch == 64 && e.pass == 1 && e.valid);
  CHECK(e.active_ms == 20 && e.busy_ms == 14 && e.rx_ms == 1 && e.foreign == 3 && e.ofdm_err == 0x0102);
  CHECK(!parse_scan_result(b.data(), b.size() - 1, r));
}

TEST(status_v4_carries_tx_scan_drop) {
  std::vector<uint8_t> b(kStatusLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 4; b[3] = kStatus;
  b[47] = 0x0D; b[48] = 0x0C; b[49] = 0x0B; b[50] = 0x0A;
  Status s;
  REQUIRE(parse_status(b.data(), b.size(), s));
  CHECK(s.tx_scan_drop == 0x0A0B0C0Du);
  b[2] = 3;
  CHECK(!parse_status(b.data(), b.size(), s));      // v3 is gone
}

MTEST_MAIN
