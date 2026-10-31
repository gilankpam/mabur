#include "mtest.h"
#include "mabur/cal_wire.h"

using namespace mabur::cal;

TEST(round_trips_every_field) {
  auto p = build_cal_payload(Kind::Power, /*rate=*/5, /*idx=*/-41,
                             /*bw=*/40, /*flags=*/3, /*seq=*/0x12345);
  CHECK(p.size() == kCalPayloadLen);
  CalFrameInfo f;
  CHECK(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.kind == Kind::Power);
  CHECK(f.rate == 5);
  CHECK(f.idx == -41);
  CHECK(f.bw == 40);
  CHECK(f.flags == 3);
  CHECK(f.seq == 0x12345);
}

TEST(capacity_frame_is_body_sized) {
  auto p = build_cal_payload(Kind::Capacity, 3, 0, 20, 2, 7, 1405);
  CHECK(p.size() == 1405);
  CalFrameInfo f;
  CHECK(parse_cal_payload(p.data(), p.size(), &f));
  CHECK(f.seq == 7);
  CHECK(f.kind == Kind::Capacity);
  CHECK(f.len == 1405);
}

TEST(fcs_tail_is_tolerated_on_both_sizes) {
  // devourer hands up the frame WITH its 4-byte FCS; RadioFrontend strips
  // only the dot11 header, so the body may arrive 4 bytes long, whatever
  // size the payload itself was built at.
  auto small = build_cal_payload(Kind::Verify, 0, 40, 20, 0, 1);
  small.resize(small.size() + 4, 0xAA);
  CalFrameInfo f;
  CHECK(parse_cal_payload(small.data(), small.size(), &f));
  CHECK(f.idx == 40);
  CHECK(f.len == kCalPayloadLen);  // the payload, the FCS left out

  auto big = build_cal_payload(Kind::Capacity, 3, 0, 20, 2, 7, 1405);
  big.resize(big.size() + 4, 0xAA);
  CHECK(parse_cal_payload(big.data(), big.size(), &f));
  CHECK(f.seq == 7);
  CHECK(f.len == 1405);
}

TEST(rejects_bad_kind_bw_flags) {
  auto base = build_cal_payload(Kind::Power, 0, 5, 20, 0, 1);
  CalFrameInfo f;

  auto bad_kind = base;
  bad_kind[4] = 4;  // kind not in {1,2,3}
  CHECK(!parse_cal_payload(bad_kind.data(), bad_kind.size(), &f));

  auto bad_bw = base;
  bad_bw[7] = 30;  // bw not in {20,40}
  CHECK(!parse_cal_payload(bad_bw.data(), bad_bw.size(), &f));

  auto bad_flags = base;
  bad_flags[8] = 4;  // bit above kFlagLdpc|kFlagStbc
  CHECK(!parse_cal_payload(bad_flags.data(), bad_flags.size(), &f));
}

TEST(one_flipped_fill_byte_is_refused) {
  // keep_corrupted is ON: a corrupt frame reaches us. Its idx byte may be
  // garbage, so it must never be attributed to a cell.
  auto p = build_cal_payload(Kind::Power, 3, 60, 20, 0, 1);
  p[40] ^= 0xFF;
  CalFrameInfo f;
  CHECK(!parse_cal_payload(p.data(), p.size(), &f));
}

MTEST_MAIN
