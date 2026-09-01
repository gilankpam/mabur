#include "mtest.h"
#include "radio_frontend.h"
#include <cstring>
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

TEST(sa_canonical_filters_by_source_address) {
  uint8_t hdr[24] = {0};
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  std::memcpy(hdr + 10, sa, 6);
  CHECK(maburgs::sa_canonical(hdr, sizeof(hdr)));
  hdr[15] = 0x01;                                  // foreign SA
  CHECK(!maburgs::sa_canonical(hdr, sizeof(hdr)));
  CHECK(!maburgs::sa_canonical(hdr, 15));          // too short to hold an SA
  // The frames we ourselves build must pass their own filter (SA lives after
  // the radiotap header build_control_frame prepends).
  const uint8_t body[3] = {1, 2, 3};
  auto frame = maburgs::build_control_frame(7, body, sizeof(body));
  const size_t rt = frame.size() - 24 - sizeof(body);  // radiotap length
  CHECK(maburgs::sa_canonical(frame.data() + rt, frame.size() - rt));
}

TEST(dot11_body_offset_keys_on_frame_control) {
  uint8_t probe[25] = {0};  // 24-byte header + 1 body byte
  probe[0] = 0x40;
  CHECK(maburgs::dot11_body_offset(probe, sizeof(probe)) == 24);

  uint8_t qos[27] = {0};    // 26-byte header + 1 body byte
  qos[0] = 0x88;
  CHECK(maburgs::dot11_body_offset(qos, sizeof(qos)) == 26);

  // Too short to hold header + 1 body byte -> 0 (reject).
  CHECK(maburgs::dot11_body_offset(probe, 24) == 0);  // probe hdr, no body
  CHECK(maburgs::dot11_body_offset(qos, 26) == 0);    // qos hdr, no body
  CHECK(maburgs::dot11_body_offset(qos, 10) == 0);    // truncated garbage

  // Unknown FC types parse at the legacy 24-byte offset (today's behavior
  // for anything the SA filter passes).
  uint8_t other[25] = {0};
  other[0] = 0x08;  // plain Data, non-QoS
  CHECK(maburgs::dot11_body_offset(other, sizeof(other)) == 24);
}

TEST(dot11_body_offset_seq_position_shared_by_both_layouts) {
  // seq_ctl lives at bytes 22-23 in BOTH layouts — the parser reads it
  // unconditionally, so pin that assumption here.
  uint8_t qos[28] = {0};
  qos[0] = 0x88;
  qos[22] = 0x30; qos[23] = 0x12;  // seq_ctl = 0x1230 -> seq 0x123
  const uint16_t seq_ctl = static_cast<uint16_t>(qos[22] | (qos[23] << 8));
  CHECK((seq_ctl >> 4) == 0x123);
  CHECK(maburgs::dot11_body_offset(qos, sizeof(qos)) == 26);
}
MTEST_MAIN
