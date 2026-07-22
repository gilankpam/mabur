// tests/test_frame_wire.cpp
#include "mtest.h"
#include "mabur/frame_wire.h"
#include "mabur/rc_proto.h"
using namespace mabur::framewire;

TEST(frame_hdr_roundtrip) {
  FrameHdr h;
  h.frame_id = 0xBEEF;
  h.flags = kFlagIdr | kFlagDiscont;
  h.codec = kCodecH265;
  h.pts_us = 0xDEADBEEF;
  uint8_t buf[kFrameHdrLen];
  pack_frame_hdr(h, buf);
  auto p = parse_frame_hdr(buf, sizeof buf);
  REQUIRE(p.has_value());
  CHECK(p->frame_id == 0xBEEF);
  CHECK(p->flags == (kFlagIdr | kFlagDiscont));
  CHECK(p->codec == kCodecH265);
  CHECK(p->pts_us == 0xDEADBEEF);
}

TEST(frame_hdr_wire_layout_little_endian) {
  FrameHdr h;
  h.frame_id = 0x0201;
  h.flags = 0x01;
  h.codec = 0x01;
  h.pts_us = 0x04030201;
  uint8_t buf[kFrameHdrLen];
  pack_frame_hdr(h, buf);
  const uint8_t expect[8] = {0x01, 0x02, 0x01, 0x01, 0x01, 0x02, 0x03, 0x04};
  for (int i = 0; i < 8; ++i) CHECK(buf[i] == expect[i]);
}

TEST(frame_hdr_short_buffer_rejected) {
  uint8_t buf[7] = {0};
  CHECK(!parse_frame_hdr(buf, sizeof buf).has_value());
  CHECK(!parse_frame_hdr(nullptr, 0).has_value());
}

TEST(cap_frame_wire_bit_defined) {
  CHECK(mabur::rc::CAP_FRAME_WIRE == 0x0001);
}

MTEST_MAIN
