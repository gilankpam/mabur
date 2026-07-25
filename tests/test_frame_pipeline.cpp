// FramePipeline: the per-frame ingest step maburd's hot thread and its
// --dry-run replay share (classify, protect up on the producer IDR flag, stamp
// the FrameHdr in place, hand the unit to the UEP encoder).
#include <cstring>
#include <vector>

#include "frame_pipeline.h"
#include "mabur/frame_wire.h"
#include "mtest.h"

using namespace mabur;

namespace {

std::array<UepLayerCfg, 4> layers() {
  std::array<UepLayerCfg, 4> l{};
  for (auto& c : l) {
    c.fec.symbol_size = 164;
    c.fec.window = 32;
    c.fec.overhead = 0.5;
    c.blocks_per_body = 4;
  }
  return l;
}

// A ring buffer as FrameSource::read fills it: 8 bytes of meta room followed by
// the Annex-B payload. Returns the buffer; payload length is nal_payload + 6.
std::vector<uint8_t> ring_buf(uint8_t nal_type, uint8_t tid, size_t nal_payload) {
  std::vector<uint8_t> buf(VENC_FRAME_META_SIZE, 0);
  for (uint8_t b : {0x00, 0x00, 0x00, 0x01}) buf.push_back(b);
  buf.push_back(static_cast<uint8_t>(nal_type << 1));
  buf.push_back(static_cast<uint8_t>(tid + 1));
  buf.resize(buf.size() + nal_payload, 0x5A);
  return buf;
}

VencFrameMeta meta_of(uint32_t pts, bool idr) {
  VencFrameMeta m{};
  m.pts = pts;
  m.codec = VENC_FRAME_CODEC_H265;
  m.flags = idr ? VENC_FRAME_FLAG_IDR : 0;
  return m;
}

size_t payload_len(const std::vector<uint8_t>& buf) {
  return buf.size() - VENC_FRAME_META_SIZE;
}

}  // namespace

TEST(frame_pipeline_stamps_hdr_over_meta_in_place) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  auto buf = ring_buf(/*nal_type=*/1, /*tid=*/0, 900);
  const std::vector<uint8_t> annexb(buf.begin() + VENC_FRAME_META_SIZE, buf.end());

  pipe.encode(enc, buf.data(), payload_len(buf), meta_of(123456, false), 1);

  auto h = framewire::parse_frame_hdr(buf.data(), buf.size());
  REQUIRE(h.has_value());
  CHECK(h->frame_id == 0);
  CHECK(h->pts_us == 123456);
  CHECK(h->codec == framewire::kCodecH265);
  // Annex-B bytes after the header are untouched — the unit is contiguous.
  CHECK(std::memcmp(buf.data() + framewire::kFrameHdrLen, annexb.data(),
                    annexb.size()) == 0);
}

TEST(frame_pipeline_frame_id_increments_per_frame) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  for (uint16_t i = 0; i < 3; ++i) {
    auto buf = ring_buf(1, 0, 500);
    pipe.encode(enc, buf.data(), payload_len(buf), meta_of(i * 16667, false), 1);
    auto h = framewire::parse_frame_hdr(buf.data(), buf.size());
    REQUIRE(h.has_value());
    CHECK(h->frame_id == i);
  }
  CHECK(pipe.next_frame_id() == 3);
}

TEST(frame_pipeline_first_frame_is_a_discontinuity) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;

  auto first = ring_buf(1, 0, 500);
  pipe.encode(enc, first.data(), payload_len(first), meta_of(0, false), 1);
  auto h0 = framewire::parse_frame_hdr(first.data(), first.size());
  REQUIRE(h0.has_value());
  CHECK((h0->flags & framewire::kFlagDiscont) != 0);

  auto second = ring_buf(1, 0, 500);
  pipe.encode(enc, second.data(), payload_len(second), meta_of(16667, false), 2);
  auto h1 = framewire::parse_frame_hdr(second.data(), second.size());
  REQUIRE(h1.has_value());
  CHECK((h1->flags & framewire::kFlagDiscont) == 0);
}

TEST(frame_pipeline_mark_discontinuity_flags_the_next_frame_only) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  auto warmup = ring_buf(1, 0, 500);
  pipe.encode(enc, warmup.data(), payload_len(warmup), meta_of(0, false), 1);

  pipe.mark_discontinuity();  // producer restart: joined a new ring mid-GOP
  auto rejoin = ring_buf(1, 0, 500);
  pipe.encode(enc, rejoin.data(), payload_len(rejoin), meta_of(99, false), 2);
  auto h = framewire::parse_frame_hdr(rejoin.data(), rejoin.size());
  REQUIRE(h.has_value());
  CHECK((h->flags & framewire::kFlagDiscont) != 0);

  auto after = ring_buf(1, 0, 500);
  pipe.encode(enc, after.data(), payload_len(after), meta_of(116, false), 3);
  auto h2 = framewire::parse_frame_hdr(after.data(), after.size());
  REQUIRE(h2.has_value());
  CHECK((h2->flags & framewire::kFlagDiscont) == 0);
}

TEST(frame_pipeline_routes_by_temporal_id) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  const int want[3] = {1, 2, 3};  // tid 0,1,2 -> streams 1,2,3
  for (int i = 0; i < 3; ++i) {
    auto buf = ring_buf(1, static_cast<uint8_t>(i), 500);
    auto bodies = pipe.encode(enc, buf.data(), payload_len(buf), meta_of(0, false), 1);
    REQUIRE(!bodies.empty());
    CHECK(bodies[0].stream_id == want[i]);
  }
  CHECK(pipe.idr_disagreements() == 0);
}

TEST(frame_pipeline_idr_frame_is_critical_and_flagged) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  auto buf = ring_buf(/*nal_type=*/19, /*tid=*/0, 2000);  // IDR_W_RADL
  auto bodies = pipe.encode(enc, buf.data(), payload_len(buf), meta_of(7, true), 1);
  REQUIRE(!bodies.empty());
  CHECK(bodies[0].stream_id == 0);
  auto h = framewire::parse_frame_hdr(buf.data(), buf.size());
  REQUIRE(h.has_value());
  CHECK((h->flags & framewire::kFlagIdr) != 0);
  // Scan and producer flag agree: no disagreement booked.
  CHECK(pipe.idr_disagreements() == 0);
}

TEST(frame_pipeline_producer_idr_flag_protects_up_and_counts_disagreement) {
  // Producer says IDR, the Annex-B scan says tid-1 slice: trust the union
  // (stream 0) and count the disagreement as the bug signal it is.
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  auto buf = ring_buf(/*nal_type=*/1, /*tid=*/1, 800);
  auto bodies = pipe.encode(enc, buf.data(), payload_len(buf), meta_of(0, true), 1);
  REQUIRE(!bodies.empty());
  CHECK(bodies[0].stream_id == 0);
  CHECK(pipe.idr_disagreements() == 1);
}

MTEST_MAIN
