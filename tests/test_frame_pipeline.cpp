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

VencFrameMeta meta_flags(uint32_t pts, uint8_t flags) {
  VencFrameMeta m{};
  m.pts = pts;
  m.codec = VENC_FRAME_CODEC_H265;
  m.flags = flags;
  return m;
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

// The discont flag rides on EVERY frame for ~1 s, not exactly one: the first
// frame after a restart is sent into a link that just came back up, so a
// one-shot flag is systematically lost and the GS's emit cursor wedges above
// the new epoch's ids (docs/gs-frame-stall-after-drone-restart-handoff.md).
namespace {
bool discont_at(UepEncoder& enc, FramePipeline& pipe, uint64_t now_ms) {
  auto buf = ring_buf(1, 0, 500);
  pipe.encode(enc, buf.data(), payload_len(buf), meta_of(0, false), now_ms);
  auto h = framewire::parse_frame_hdr(buf.data(), buf.size());
  REQUIRE(h.has_value());
  return (h->flags & framewire::kFlagDiscont) != 0;
}
}  // namespace

TEST(frame_pipeline_discont_sticks_for_a_second_after_start) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  CHECK(discont_at(enc, pipe, 1));     // first frame
  CHECK(discont_at(enc, pipe, 500));   // still inside the sticky window
  CHECK(!discont_at(enc, pipe, 1100)); // window expired
}

TEST(frame_pipeline_mark_discontinuity_restarts_the_sticky_window) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  CHECK(discont_at(enc, pipe, 1));
  CHECK(!discont_at(enc, pipe, 2000));  // start window long gone

  pipe.mark_discontinuity();  // producer restart: joined a new ring mid-GOP
  CHECK(discont_at(enc, pipe, 2001));
  CHECK(discont_at(enc, pipe, 2900));   // sticky: window anchors at the mark
  CHECK(!discont_at(enc, pipe, 3100));
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
  // TRAIL_R tid 2 routes to sid 3 (devourer-tid routing) without being
  // flagged as an enhance disagreement: the agreement check keys on real
  // TRAIL_N-ness, not on sid == 3.
  CHECK(pipe.enhance_disagreements() == 0);
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

TEST(frame_pipeline_enhance_needs_flag_and_scan_agreement) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;

  // Agree: TRAIL_N (type 0) + ENHANCE flag -> sid 3.
  auto b1 = ring_buf(/*nal_type=*/0, /*tid=*/0, 900);
  auto out1 = pipe.encode(enc, b1.data(), payload_len(b1),
                          meta_flags(1000, VENC_FRAME_FLAG_ENHANCE), 1);
  REQUIRE(!out1.empty());
  CHECK(out1[0].stream_id == 3);
  CHECK(pipe.enhance_disagreements() == 0);

  // Scan-only: TRAIL_N without the flag -> base (1) + disagree.
  auto b2 = ring_buf(0, 0, 900);
  auto out2 = pipe.encode(enc, b2.data(), payload_len(b2), meta_flags(2000, 0), 2);
  REQUIRE(!out2.empty());
  CHECK(out2[0].stream_id == 1);
  CHECK(pipe.enhance_disagreements() == 1);

  // Flag-only: TRAIL_R (type 1) with the flag -> base (1) + disagree.
  auto b3 = ring_buf(1, 0, 900);
  auto out3 = pipe.encode(enc, b3.data(), payload_len(b3),
                          meta_flags(3000, VENC_FRAME_FLAG_ENHANCE), 3);
  REQUIRE(!out3.empty());
  CHECK(out3[0].stream_id == 1);
  CHECK(pipe.enhance_disagreements() == 2);

  // Flag on an IDR (producer bug): critical wins -> sid 0 + disagree.
  auto b4 = ring_buf(19, 0, 900);
  auto out4 = pipe.encode(enc, b4.data(), payload_len(b4),
                          meta_flags(4000, VENC_FRAME_FLAG_IDR | VENC_FRAME_FLAG_ENHANCE), 4);
  REQUIRE(!out4.empty());
  CHECK(out4[0].stream_id == 0);
  CHECK(pipe.enhance_disagreements() == 3);

  // TRAIL_R tid 2 (devourer-tid routing, NOT enhance): sid 3 with no flag
  // is not a disagreement, because the scan side is frame_is_trail_n, not
  // sid == 3.
  auto b5 = ring_buf(1, 2, 900);
  auto out5 = pipe.encode(enc, b5.data(), payload_len(b5), meta_flags(5000, 0), 5);
  REQUIRE(!out5.empty());
  CHECK(out5[0].stream_id == 3);
  CHECK(pipe.enhance_disagreements() == 3);
}

TEST(frame_pipeline_shed_frames_consume_no_frame_id) {
  UepEncoder enc(layers(), 15);
  enc.set_shed(3, true);
  FramePipeline pipe;

  // Shed enhance frame: no bodies, no id consumed, drop booked, discont
  // latch NOT consumed (the window must anchor on a frame that ships).
  auto b1 = ring_buf(0, 0, 900);
  auto out1 = pipe.encode(enc, b1.data(), payload_len(b1),
                          meta_flags(1000, VENC_FRAME_FLAG_ENHANCE), 1);
  CHECK(out1.empty());
  CHECK(pipe.next_frame_id() == 0);
  CHECK(enc.dropped(3) == 1);

  // Next base frame gets id 0 and carries the discont flag.
  auto b2 = ring_buf(1, 0, 900);
  auto out2 = pipe.encode(enc, b2.data(), payload_len(b2), meta_flags(2000, 0), 2);
  REQUIRE(!out2.empty());
  CHECK(pipe.next_frame_id() == 1);
  auto h = framewire::parse_frame_hdr(b2.data(), b2.size());
  REQUIRE(h.has_value());
  CHECK(h->frame_id == 0);
  CHECK((h->flags & framewire::kFlagDiscont) != 0);
}

MTEST_MAIN
