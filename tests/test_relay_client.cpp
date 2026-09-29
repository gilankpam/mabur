// RelayClient (gs/src/relay_client.h): the pure mabur-relay v3 client.
#include <cstring>
#include "dot11.h"
#include "mtest.h"
#include "relay_client.h"
using namespace maburgs;
using namespace maburgs::relay;

namespace {
struct Sink {
  std::vector<std::vector<uint8_t>> sent;
  RelayClient::SendFn fn() { return [this](const std::vector<uint8_t>& m) { sent.push_back(m); }; }
  int count(Type t) const {
    int c = 0;
    for (auto& m : sent) c += msg_type(m.data(), m.size()) == t;
    return c;
  }
};

std::vector<uint8_t> status(uint8_t state, uint8_t ch, uint8_t sec, uint8_t you_own) {
  std::vector<uint8_t> b(kStatusLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 3; b[3] = kStatus;
  b[6] = state; b[7] = ch; b[8] = sec; b[9] = 1 /* owner kind: UDP */; b[10] = you_own;
  return b;
}

std::vector<uint8_t> frame(uint32_t seq, uint8_t rx_ch, uint8_t flags, uint8_t mcs, int8_t r0,
                           int8_t r1, int8_t n0, int8_t n1) {
  std::vector<uint8_t> b(kFrameHdrLen, 0);
  b[0] = 0x4D; b[1] = 0x52; b[2] = 3; b[3] = kFrame;
  for (int i = 0; i < 4; ++i) b[4 + i] = static_cast<uint8_t>(seq >> (8 * i));
  b[8] = rx_ch; b[9] = 2; b[10] = flags; b[11] = mcs;
  b[12] = static_cast<uint8_t>(r0); b[13] = static_cast<uint8_t>(r1);
  b[14] = static_cast<uint8_t>(n0); b[15] = static_cast<uint8_t>(n1);
  b[16] = 0x78; b[17] = 0x56; b[18] = 0x34; b[19] = 0x12;
  // QoS-Data, canonical SA, seq 0x123, 3 body bytes.
  std::vector<uint8_t> d(26 + 3, 0);
  d[0] = 0x88;
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  std::memcpy(d.data() + 10, sa, 6);
  d[22] = 0x30; d[23] = 0x12;
  d[26] = 0xAA; d[27] = 0xBB; d[28] = 0xCC;
  b.insert(b.end(), d.begin(), d.end());
  return b;
}
}  // namespace

TEST(start_sends_hello_then_tune) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  REQUIRE(s.sent.size() == 2);
  CHECK(msg_type(s.sent[0].data(), 4) == kHello);
  CHECK(msg_type(s.sent[1].data(), 8) == kTune);
  CHECK(s.sent[1][6] == 136 && s.sent[1][7] == 2);
}

TEST(hello_every_500ms) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  auto st = status(0, 136, 2, 1);
  mabur::node::RxBody b;
  c.on_message(st.data(), st.size(), 10, b);
  for (uint64_t t = 0; t <= 2000; t += 50) c.tick(t);
  CHECK(s.count(kHello) == 5);   // t=0 (start), 500, 1000, 1500, 2000
  CHECK(s.count(kTune) == 1);    // owned + tuned: no retry
  CHECK(c.owned_and_tuned());
}

TEST(tune_retried_while_refused_then_owned) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  auto no = status(3, 132, 0, 0);
  mabur::node::RxBody b;
  c.on_message(no.data(), no.size(), 20, b);
  for (uint64_t t = 0; t <= 1000; t += 50) c.tick(t);
  CHECK(s.count(kTune) == 3);    // start, 500, 1000
  CHECK(!c.owned_and_tuned() && !c.refused(1000));
  auto yes = status(0, 136, 2, 1);
  c.on_message(yes.data(), yes.size(), 1100, b);
  CHECK(c.owned_and_tuned());
  for (uint64_t t = 1100; t <= 3000; t += 50) c.tick(t);
  CHECK(s.count(kTune) == 3);
}

TEST(refused_after_window) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  auto no = status(3, 132, 0, 0);
  mabur::node::RxBody b;
  for (uint64_t t = 0; t <= 2600; t += 100) {
    c.on_message(no.data(), no.size(), t, b);
    c.tick(t);
  }
  CHECK(c.refused(2600));
  CHECK(!c.lost(2600));
  CHECK(s.count(kTune) == 6);    // 0,500,...,2500: the window ends retries
}

TEST(lost_without_status) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  CHECK(!c.lost(1999));
  CHECK(c.lost(2001));
  auto st = status(0, 136, 2, 1);
  mabur::node::RxBody b;
  c.on_message(st.data(), st.size(), 2100, b);
  CHECK(!c.lost(4000));
  CHECK(c.lost(4101));
}

TEST(frame_maps_to_rxbody) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  auto f = frame(7, 136, kFlagPhyValid, 4, -40, -50, -95, -95);
  mabur::node::RxBody b;
  REQUIRE(c.on_message(f.data(), f.size(), 5, b) == RelayClient::Rx::Body);
  CHECK(b.body.size() == 3 && b.body[0] == 0xAA);
  CHECK(b.mac_seq == 0x123 && b.mcs == 4 && b.crc_ok && b.phy_valid);
  CHECK(b.rssi[0] == 70 && b.rssi[1] == 60);     // dBm + 110
  CHECK(b.snr[0] == 110 && b.snr[1] == 90);      // (rssi - noise) dB, half-dB raw
  CHECK(b.evm[0] == 0 && b.evm[1] == 0);
  CHECK(b.tsfl == 0x12345678u && b.rx_channel == 136);
}

TEST(frame_mapping_absent_chain_and_retune) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  auto f = frame(1, 0, kFlagBadFcs, kMcsNone, -30, kDbmAbsent, -95, kDbmAbsent);
  mabur::node::RxBody b;
  REQUIRE(c.on_message(f.data(), f.size(), 5, b) == RelayClient::Rx::Body);
  CHECK(!b.crc_ok && !b.phy_valid);
  CHECK(b.mcs == 255);
  CHECK(b.rssi[1] == 0 && b.snr[1] == 0);
  CHECK(b.rx_channel == 0);
  // Out-of-range dBm clamps rather than wrapping.
  auto g = frame(2, 136, kFlagPhyValid, 0, 120, -128 + 1, -95, -95);
  REQUIRE(c.on_message(g.data(), g.size(), 6, b) == RelayClient::Rx::Body);
  CHECK(b.rssi[0] == 230 && b.rssi[1] == 1);
  CHECK(b.snr[0] == 127);
}

TEST(seq_gaps_counted) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  c.start(0);
  mabur::node::RxBody b;
  for (uint32_t q : {10u, 11u, 14u, 15u}) {
    auto f = frame(q, 136, 0, 4, -40, -40, -95, -95);
    c.on_message(f.data(), f.size(), 1, b);
  }
  CHECK(c.frames() == 4 && c.seq_gaps() == 2);
}

TEST(garbage_counted_not_fatal) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  const uint8_t junk[] = {1, 2, 3, 4, 5};
  mabur::node::RxBody b;
  CHECK(c.on_message(junk, sizeof junk, 1, b) == RelayClient::Rx::None);
  CHECK(c.bad_msgs() == 1);
}

TEST(send_control_strips_radiotap) {
  Sink s;
  RelayClient c(136, 2, s.fn());
  const uint8_t body[] = {9, 8, 7};
  const auto f = build_control_frame(0x42, body, sizeof body);
  const size_t rt = f[2] | (f[3] << 8);
  REQUIRE(c.send_control(f));
  REQUIRE(s.sent.size() == 1);
  const auto& m = s.sent[0];
  CHECK(msg_type(m.data(), m.size()) == kTx);
  CHECK(m[4] == 0 && m[5] == (kTxLdpc | kTxStbc));
  REQUIRE(m.size() == kTxHdrLen + f.size() - rt);
  CHECK(std::memcmp(m.data() + kTxHdrLen, f.data() + rt, f.size() - rt) == 0);
  CHECK(c.tx_sent() == 1);
  const uint8_t bad[] = {0, 0, 200, 0};   // it_len past the end
  CHECK(!c.send_control(std::vector<uint8_t>(bad, bad + 4)));
}

MTEST_MAIN
