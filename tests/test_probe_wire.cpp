#include "mtest.h"
#include "mabur/probe_wire.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
using namespace mabur;
using namespace mabur::probe;

static constexpr int kBpb = 4;
static constexpr int kPayload = static_cast<int>(sw::kSwHeaderLen) + 332;  // 348

TEST(probe_body_is_video_geometry_long) {
  ProbeHdr h{0x01020304, 0x05, 0x0607};
  auto b = build_probe_body(h, kBpb, kPayload);
  CHECK(b.size() == 1403);
  CHECK(b.size() == probe_body_len(kBpb, kPayload));
  CHECK(sbi_peek_stream_id(b.data(), b.size()) == kProbeStreamId);
}

TEST(probe_header_round_trips_and_repeats_in_every_block) {
  ProbeHdr h{42, 0x24, 999};
  auto b = build_probe_body(h, kBpb, kPayload);
  for (int i = 0; i < kBpb; ++i) {
    const uint8_t* blk = b.data() + SBI_HDR_LEN + i * (2 + kPayload) + 2;
    ProbeHdr got;
    REQUIRE(parse_hdr(blk, kPayload, &got));
    CHECK(got.seq == 42); CHECK(got.profile == 0x24); CHECK(got.enh_fid == 999);
  }
  ProbeRx rx;
  REQUIRE(parse_probe_body(b.data(), b.size(), kPayload, &rx));
  CHECK(rx.n_blocks == kBpb); CHECK(rx.n_ok == kBpb); CHECK(rx.survivors == 0xF);
  CHECK(rx.hdr.seq == 42);
}

TEST(killed_block_zero_leaves_others_attributable) {
  auto b = build_probe_body(ProbeHdr{7, 0x03, 1}, kBpb, kPayload);
  b[SBI_HDR_LEN + 2 + 3] ^= 0xFF;  // corrupt a payload byte of block 0
  ProbeRx rx;
  REQUIRE(parse_probe_body(b.data(), b.size(), kPayload, &rx));
  CHECK(rx.n_ok == 3); CHECK((rx.survivors & 1u) == 0); CHECK(rx.hdr.seq == 7);
}

TEST(all_blocks_dead_is_unparseable) {
  auto b = build_probe_body(ProbeHdr{7, 0x03, 1}, kBpb, kPayload);
  for (int i = 0; i < kBpb; ++i) b[SBI_HDR_LEN + i * (2 + kPayload) + 2 + 5] ^= 0x55;
  ProbeRx rx;
  CHECK(!parse_probe_body(b.data(), b.size(), kPayload, &rx));
}

TEST(disagreeing_blocks_are_rejected) {
  auto a = build_probe_body(ProbeHdr{1, 0x03, 1}, kBpb, kPayload);
  auto c = build_probe_body(ProbeHdr{2, 0x03, 1}, kBpb, kPayload);
  // splice block 1 (crc+payload) of c into a
  const size_t off = SBI_HDR_LEN + 1 * (2 + kPayload);
  std::copy(c.begin() + off, c.begin() + off + 2 + kPayload, a.begin() + off);
  ProbeRx rx;
  CHECK(!parse_probe_body(a.data(), a.size(), kPayload, &rx));
}

TEST(fill_is_deterministic_per_seq) {
  auto a = build_probe_body(ProbeHdr{9, 1, 1}, kBpb, kPayload);
  auto b = build_probe_body(ProbeHdr{9, 1, 1}, kBpb, kPayload);
  auto c = build_probe_body(ProbeHdr{10, 1, 1}, kBpb, kPayload);
  CHECK(a == b); CHECK(a != c);
}

TEST(short_or_wrong_stream_body_rejected) {
  ProbeRx rx;
  std::vector<uint8_t> junk(20, 0);
  CHECK(!parse_probe_body(junk.data(), junk.size(), kPayload, &rx));
}

MTEST_MAIN
