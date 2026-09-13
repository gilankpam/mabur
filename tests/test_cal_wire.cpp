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
  auto p = build_cal_payload(0, 40, kPhaseCoarse, 1);
  p.resize(kCalPayloadLen + 4, 0xAA);
  CalFrameInfo f;
  CHECK(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.idx == 40);
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

TEST(accepts_negative_relative_idx) {
  // idx is a SIGNED index relative to the chip's anchor (spec 2026-09-13):
  // the coarse sweep starts at -40.
  auto p = build_cal_payload(0, -40, kPhaseCoarse, 9);
  CalFrameInfo f;
  REQUIRE(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.idx == -40);
}

TEST(rejects_idx_outside_the_diff_field_range) {
  // The chip's per-rate diff field is [-64, 63]; anything else in the idx
  // byte is corruption that survived the FCS, not a cell.
  auto lo = build_cal_payload(0, -64, kPhaseCoarse, 1);
  auto hi = build_cal_payload(0, 63, kPhaseCoarse, 1);
  CalFrameInfo f;
  CHECK(parse_cal_payload(lo.data(), lo.size(), &f));
  CHECK(parse_cal_payload(hi.data(), hi.size(), &f));
  auto bad = build_cal_payload(0, 63, kPhaseCoarse, 1);
  bad[5] = static_cast<uint8_t>(100);  // +100: outside [-64,63]
  bad[9] = static_cast<uint8_t>(0x5A ^ 100);  // keep the fill consistent
  for (size_t i = 9; i < kCalPayloadLen; ++i) bad[i] = bad[9];
  CHECK(!parse_cal_payload(bad.data(), bad.size(), &f));
}

TEST(fill_depends_on_idx) {
  // Two payloads differing only in idx must differ in their fill, so a
  // corrupt frame cannot silently pass another cell's fill check.
  auto a = build_cal_payload(0, 10, kPhaseCoarse, 1);
  auto b = build_cal_payload(0, 11, kPhaseCoarse, 1);
  CHECK(a[kCalPayloadLen - 1] != b[kCalPayloadLen - 1]);
}

MTEST_MAIN
