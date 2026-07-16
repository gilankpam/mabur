#include "mtest.h"
#include "sweep_wire.h"

using namespace txagcbench;

TEST(round_trip) {
  auto p = build_sweep_payload(37, 2, 0xBEEF, 0);
  CHECK(p.size() == kSweepPayloadLen);
  SweepInfo si;
  CHECK(parse_sweep_payload(p.data(), p.size(), &si));
  CHECK(si.idx == 37);
  CHECK(si.pass == 2);
  CHECK(si.seq == 0xBEEF);
  CHECK(si.mcs == 0);
}

TEST(rejects_bad_magic) {
  auto p = build_sweep_payload(5, 1, 1, 0);
  p[0] = 'X';
  SweepInfo si;
  CHECK(!parse_sweep_payload(p.data(), p.size(), &si));
}

TEST(rejects_wrong_length) {
  auto p = build_sweep_payload(5, 1, 1, 0);
  SweepInfo si;
  CHECK(!parse_sweep_payload(p.data(), p.size() - 1, &si));
}

TEST(rejects_corrupt_fill) {
  auto p = build_sweep_payload(5, 1, 1, 0);
  p[40] ^= 0xFF;
  SweepInfo si;
  CHECK(!parse_sweep_payload(p.data(), p.size(), &si));
}

TEST(rejects_idx_over_63) {
  auto p = build_sweep_payload(64, 1, 1, 0);
  SweepInfo si;
  CHECK(!parse_sweep_payload(p.data(), p.size(), &si));
}

TEST(dot11_header_canonical) {
  auto h = build_dot11_header(7);
  CHECK(h.size() == kDot11HeaderLen);
  CHECK(h[0] == 0x40);                       // probe-req
  CHECK(h[10] == 0x57 && h[15] == 0x00);     // canonical SA
  CHECK(h[22] == static_cast<uint8_t>(7 << 4));  // seq_ctl low byte
}

MTEST_MAIN
