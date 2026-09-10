#include "mtest.h"
#include "mabur/cal_wire.h"

using namespace mabur::cal;

TEST(round_trip) {
  auto p = build_cal_payload(/*rate=*/5, /*idx=*/37, kPhaseFine, /*seq=*/0xBEEF);
  CHECK(p.size() == kCalPayloadLen);
  CalFrameInfo f;
  CHECK(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.rate == 5);
  CHECK(f.idx == 37);
  CHECK(f.phase == kPhaseFine);
  CHECK(f.seq == 0xBEEF);
}

TEST(accepts_trailing_fcs) {
  // devourer hands up the frame WITH its 4-byte FCS; RadioFrontend strips
  // only the dot11 header, so the body arrives 4 bytes long.
  auto p = build_cal_payload(0, 91, kPhaseCoarse, 1);
  p.resize(kCalPayloadLen + 4, 0xAA);
  CalFrameInfo f;
  CHECK(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.idx == 91);
}

TEST(rejects_bad_magic) {
  auto p = build_cal_payload(0, 5, kPhaseCoarse, 1);
  p[0] = 'X';
  CalFrameInfo f;
  CHECK(!parse_cal_payload(p.data(), p.size(), &f));
}

TEST(rejects_wrong_length) {
  auto p = build_cal_payload(0, 5, kPhaseCoarse, 1);
  CalFrameInfo f;
  CHECK(!parse_cal_payload(p.data(), p.size() - 1, &f));
  p.resize(kCalPayloadLen + 3);
  CHECK(!parse_cal_payload(p.data(), p.size(), &f));
}

TEST(rejects_corrupt_fill) {
  // keep_corrupted is ON: a corrupt frame reaches us. Its idx byte may be
  // garbage, so it must never be attributed to a cell.
  auto p = build_cal_payload(3, 60, kPhaseCoarse, 1);
  p[40] ^= 0xFF;
  CalFrameInfo f;
  CHECK(!parse_cal_payload(p.data(), p.size(), &f));
}

TEST(rejects_out_of_range_fields) {
  auto p = build_cal_payload(3, 60, kPhaseCoarse, 1);
  p[4] = 8;  // rate > 7
  CalFrameInfo f;
  CHECK(!parse_cal_payload(p.data(), p.size(), &f));

  auto q = build_cal_payload(3, 60, kPhaseCoarse, 1);
  q[6] = 9;  // phase not in {1,2,3}
  CHECK(!parse_cal_payload(q.data(), q.size(), &f));
}

TEST(accepts_full_7bit_idx) {
  // Jaguar3 TXAGC is 7-bit: 127 is the top legal index.
  auto p = build_cal_payload(0, 127, kPhaseVerify, 9);
  CalFrameInfo f;
  CHECK(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.idx == 127);
}

TEST(fill_depends_on_idx) {
  // Two payloads differing only in idx must differ in their fill, so a
  // corrupt frame cannot silently pass another cell's fill check.
  auto a = build_cal_payload(0, 10, kPhaseCoarse, 1);
  auto b = build_cal_payload(0, 11, kPhaseCoarse, 1);
  CHECK(a[kCalPayloadLen - 1] != b[kCalPayloadLen - 1]);
}

MTEST_MAIN
