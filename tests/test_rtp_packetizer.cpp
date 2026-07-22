#include <cstring>
#include <vector>
#include "mtest.h"
#include "rtp_packetizer.h"
#include "mabur/frame_wire.h"
using namespace maburgs;
using mabur::framewire::FrameHdr;

namespace {
struct Rtp {
  uint8_t pt; bool marker; uint16_t seq; uint32_t ts; uint32_t ssrc;
  std::vector<uint8_t> payload;
};
Rtp parse(const std::vector<uint8_t>& p) {
  Rtp r;
  r.marker = (p[1] & 0x80) != 0;
  r.pt = p[1] & 0x7F;
  r.seq = static_cast<uint16_t>((p[2] << 8) | p[3]);
  r.ts = (static_cast<uint32_t>(p[4]) << 24) | (p[5] << 16) | (p[6] << 8) | p[7];
  r.ssrc = (static_cast<uint32_t>(p[8]) << 24) | (p[9] << 16) | (p[10] << 8) | p[11];
  r.payload.assign(p.begin() + 12, p.end());
  return r;
}
std::vector<uint8_t> nal(uint8_t type, size_t total_len) {
  std::vector<uint8_t> v = {0x00, 0x00, 0x00, 0x01,
                            static_cast<uint8_t>(type << 1), 0x01};
  for (size_t i = v.size(); i < 4 + total_len; ++i)
    v.push_back(static_cast<uint8_t>(i));
  return v;
}
FrameHdr hdr(uint16_t id, uint32_t pts, uint8_t flags = 0) {
  FrameHdr h; h.frame_id = id; h.pts_us = pts; h.flags = flags; return h;
}
}  // namespace

TEST(small_nal_single_packet_with_marker) {
  std::vector<Rtp> out;
  RtpPacketizer pk({97, 0x4D414252, 1400, 16667},
                   [&](const std::vector<uint8_t>& p) { out.push_back(parse(p)); });
  auto f = nal(19, 100);
  pk.begin_frame(hdr(0, 0));
  pk.data(f.data(), f.size());
  pk.end_frame(true);
  REQUIRE(out.size() == 1);
  CHECK(out[0].pt == 97);
  CHECK(out[0].marker);
  CHECK(out[0].ssrc == 0x4D414252u);
  // payload = the bare NAL (start code stripped)
  CHECK(out[0].payload == std::vector<uint8_t>(f.begin() + 4, f.end()));
}

TEST(large_nal_fu_fragments) {
  std::vector<Rtp> out;
  RtpPacketizer pk({97, 1, 200, 16667},
                   [&](const std::vector<uint8_t>& p) { out.push_back(parse(p)); });
  auto f = nal(1, 1000);
  pk.begin_frame(hdr(0, 0));
  pk.data(f.data(), f.size());
  pk.end_frame(true);
  REQUIRE(out.size() >= 5);
  // FU: PayloadHdr type 49, FU hdr carries S on first, E+marker on last.
  for (auto& r : out) CHECK(((r.payload[0] >> 1) & 0x3F) == 49);
  CHECK((out.front().payload[2] & 0x80) != 0);            // S
  CHECK((out.back().payload[2] & 0x40) != 0);             // E
  CHECK(out.back().marker);
  for (size_t i = 0; i + 1 < out.size(); ++i) {
    CHECK(!out[i].marker);
    CHECK(static_cast<uint16_t>(out[i].seq + 1) == out[i + 1].seq);
  }
  // Reassemble: FU payloads carry NAL bytes from offset 2.
  std::vector<uint8_t> got = {f[4], f[5]};
  for (auto& r : out) got.insert(got.end(), r.payload.begin() + 3, r.payload.end());
  CHECK(got == std::vector<uint8_t>(f.begin() + 4, f.end()));
  CHECK((out.front().payload[2] & 0x3F) == 1);            // FuType = orig type
}

TEST(byte_at_a_time_equals_bulk) {
  auto run = [](bool bytewise) {
    std::vector<std::vector<uint8_t>> out;
    RtpPacketizer pk({97, 1, 200, 16667},
                     [&](const std::vector<uint8_t>& p) { out.push_back(p); });
    auto f = nal(1, 700);
    auto g = nal(39, 50);
    std::vector<uint8_t> all(f);
    all.insert(all.end(), g.begin(), g.end());
    pk.begin_frame(hdr(0, 0));
    if (bytewise)
      for (auto b : all) pk.data(&b, 1);
    else
      pk.data(all.data(), all.size());
    pk.end_frame(true);
    return out;
  };
  CHECK(run(true) == run(false));
}

TEST(timestamp_90khz_and_wrap) {
  std::vector<Rtp> out;
  RtpPacketizer pk({97, 1, 1400, 16667},
                   [&](const std::vector<uint8_t>& p) { out.push_back(parse(p)); });
  auto f = nal(1, 50);
  // Two frames straddling the u32-µs pts wrap: 2^32-10000 then +16667 (wraps).
  pk.begin_frame(hdr(0, 4294957296u));
  pk.data(f.data(), f.size());
  pk.end_frame(true);
  pk.begin_frame(hdr(1, static_cast<uint32_t>(4294957296u + 16667u)));
  pk.data(f.data(), f.size());
  pk.end_frame(true);
  REQUIRE(out.size() == 2);
  // Delta must be 16667 µs * 9/100 = 1500 ticks despite the wrap.
  CHECK(static_cast<uint32_t>(out[1].ts - out[0].ts) == 1500u);
}

TEST(truncated_frame_no_marker_no_e_bit) {
  std::vector<Rtp> out;
  RtpPacketizer pk({97, 1, 200, 16667},
                   [&](const std::vector<uint8_t>& p) { out.push_back(parse(p)); });
  auto f = nal(1, 1000);
  pk.begin_frame(hdr(0, 0));
  pk.data(f.data(), f.size() - 300);  // frame cut mid-NAL
  pk.end_frame(false);
  REQUIRE(!out.empty());
  for (auto& r : out) {
    CHECK(!r.marker);
    CHECK((r.payload[2] & 0x40) == 0);  // no fake E bit
  }
}

MTEST_MAIN
