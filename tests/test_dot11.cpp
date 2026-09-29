// fill_rx_body (gs/src/dot11.h): the one dot11 -> RxBody conversion shared
// by RadioFrontend and the web GS glue.
#include <cstring>
#include "dot11.h"
#include "mtest.h"
using namespace maburgs;

namespace {
std::vector<uint8_t> frame(uint8_t fc, bool canonical, uint16_t seq, size_t body_len) {
  const size_t hdr = fc == 0x88 ? 26 : 24;
  std::vector<uint8_t> f(hdr + body_len, 0);
  f[0] = fc;
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  if (canonical) std::memcpy(f.data() + 10, sa, 6);
  const uint16_t sc = static_cast<uint16_t>(seq << 4);
  f[22] = sc & 0xff;
  f[23] = sc >> 8;
  for (size_t i = 0; i < body_len; ++i) f[hdr + i] = static_cast<uint8_t>(i + 1);
  return f;
}
}  // namespace

TEST(qos_data_body_and_fields) {
  auto f = frame(0x88, true, 0x123, 5);
  RxMeta m;
  m.data_rate = 0x0C + 4;  // jaguar DESC_RATE MCS4
  m.rssi[0] = 60; m.snr[1] = 30; m.evm[0] = -40; m.physt = true; m.tsfl = 77;
  mabur::node::RxBody b;
  REQUIRE(fill_rx_body(f.data(), f.size(), m, b) == RxVerdict::Body);
  CHECK(b.body.size() == 5 && b.body[0] == 1);
  CHECK(b.mac_seq == 0x123);
  CHECK(b.mcs == 4);
  CHECK(b.crc_ok);
  CHECK(b.rssi[0] == 60 && b.snr[1] == 30 && b.evm[0] == -40);
  CHECK(b.phy_valid && b.tsfl == 77);
}

TEST(kestrel_rate_code_maps_to_mcs) {
  auto f = frame(0x88, true, 1, 1);
  RxMeta m;
  m.data_rate = 0x80 + 6;
  mabur::node::RxBody b;
  REQUIRE(fill_rx_body(f.data(), f.size(), m, b) == RxVerdict::Body);
  CHECK(b.mcs == 6);
  m.data_rate = 0x04;  // legacy OFDM: unknown
  REQUIRE(fill_rx_body(f.data(), f.size(), m, b) == RxVerdict::Body);
  CHECK(b.mcs == 255);
}

TEST(foreign_sa_dropped_only_when_crc_good) {
  auto f = frame(0x88, false, 1, 4);
  RxMeta m;
  mabur::node::RxBody b;
  CHECK(fill_rx_body(f.data(), f.size(), m, b) == RxVerdict::Foreign);
  m.crc_err = true;  // a corrupt SA proves nothing: pass for SBI salvage
  CHECK(fill_rx_body(f.data(), f.size(), m, b) == RxVerdict::Body);
  CHECK(!b.crc_ok);
}

TEST(short_frame_rejected) {
  auto f = frame(0x40, true, 1, 0);  // header only, no body byte
  RxMeta m;
  mabur::node::RxBody b;
  CHECK(fill_rx_body(f.data(), f.size(), m, b) == RxVerdict::Short);
}

MTEST_MAIN
