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
  const int want[3] = {0, 0, 0};  // tid 0,1,2 -> all base (sid 0) in 2-stream space
  for (int i = 0; i < 3; ++i) {
    auto buf = ring_buf(1, static_cast<uint8_t>(i), 500);
    auto bodies = pipe.encode(enc, buf.data(), payload_len(buf), meta_of(0, false), 1);
    REQUIRE(!bodies.empty());
    CHECK(bodies[0].stream_id == want[i]);
  }
  // TRAIL_R (type 1) routes to sid 0 (base) regardless of tid.
  // The agreement check keys on real TRAIL_N-ness (frame_is_trail_n), not on sid value.
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
  // Producer says IDR, the Annex-B scan says TRAIL_N enhance (type 0):
  // trust the union (stream 0) and count the disagreement as the bug signal it is.
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;
  auto buf = ring_buf(/*nal_type=*/0, /*tid=*/0, 800);
  auto bodies = pipe.encode(enc, buf.data(), payload_len(buf), meta_of(0, true), 1);
  REQUIRE(!bodies.empty());
  CHECK(bodies[0].stream_id == 0);
  CHECK(pipe.idr_disagreements() == 1);
}

TEST(frame_pipeline_enhance_needs_flag_and_scan_agreement) {
  UepEncoder enc(layers(), 15);
  FramePipeline pipe;

  // Agree: TRAIL_N (type 0) + ENHANCE flag -> sid 1 (enhance).
  auto b1 = ring_buf(/*nal_type=*/0, /*tid=*/0, 900);
  auto out1 = pipe.encode(enc, b1.data(), payload_len(b1),
                          meta_flags(1000, VENC_FRAME_FLAG_ENHANCE), 1);
  REQUIRE(!out1.empty());
  CHECK(out1[0].stream_id == 1);
  CHECK(pipe.enhance_disagreements() == 0);

  // Scan-only: TRAIL_N without the flag -> protect up to base (0) + disagree.
  auto b2 = ring_buf(0, 0, 900);
  auto out2 = pipe.encode(enc, b2.data(), payload_len(b2), meta_flags(2000, 0), 2);
  REQUIRE(!out2.empty());
  CHECK(out2[0].stream_id == 0);
  CHECK(pipe.enhance_disagreements() == 1);

  // Flag-only: TRAIL_R (type 1) with the flag -> protect up to base (0) + disagree.
  auto b3 = ring_buf(1, 0, 900);
  auto out3 = pipe.encode(enc, b3.data(), payload_len(b3),
                          meta_flags(3000, VENC_FRAME_FLAG_ENHANCE), 3);
  REQUIRE(!out3.empty());
  CHECK(out3[0].stream_id == 0);
  CHECK(pipe.enhance_disagreements() == 2);

  // Flag on an IDR (producer bug): critical wins -> sid 0 + disagree.
  auto b4 = ring_buf(19, 0, 900);
  auto out4 = pipe.encode(enc, b4.data(), payload_len(b4),
                          meta_flags(4000, VENC_FRAME_FLAG_IDR | VENC_FRAME_FLAG_ENHANCE), 4);
  REQUIRE(!out4.empty());
  CHECK(out4[0].stream_id == 0);
  CHECK(pipe.enhance_disagreements() == 3);

  // TRAIL_R tid 2 (type 1): sid 0 with no flag is not a disagreement,
  // because the scan side is frame_is_trail_n (which returns false for type 1),
  // not sid == 0.
  auto b5 = ring_buf(1, 2, 900);
  auto out5 = pipe.encode(enc, b5.data(), payload_len(b5), meta_flags(5000, 0), 5);
  REQUIRE(!out5.empty());
  CHECK(out5[0].stream_id == 0);
  CHECK(pipe.enhance_disagreements() == 3);
}

TEST(frame_pipeline_shed_frames_consume_no_frame_id) {
  UepEncoder enc(layers(), 15);
  enc.set_shed(1, true);  // shed enhance stream (sid 1)
  FramePipeline pipe;

  // Shed enhance frame: no bodies, no id consumed, drop booked, discont
  // latch NOT consumed (the window must anchor on a frame that ships).
  auto b1 = ring_buf(0, 0, 900);
  auto out1 = pipe.encode(enc, b1.data(), payload_len(b1),
                          meta_flags(1000, VENC_FRAME_FLAG_ENHANCE), 1);
  CHECK(out1.empty());
  CHECK(pipe.next_frame_id() == 0);
  CHECK(enc.dropped(1) == 1);

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

// ── venc-ring vanish detection (docs/venc-ring-vanish-findings-2026-08-12.md)
//
// Encoded frames can vanish between waybeam's encoder and maburd's ring read
// with no counter anywhere: the wire sequence closes seamlessly over the hole.
// The only read-side evidence is the pts step: consecutive ring reads step one
// frame period; a ~2x jump means a frame vanished upstream. FramePipeline
// derives the period from observed deltas (never hardcodes 60 fps), classifies
// the vanished frame from the NEIGHBOURS' producer ENHANCE flags (strict
// base/enhance alternation, capture-proven), and on a base-class vanish
// latches a self-IDR request — guarded against the re-seed loop (the IDR
// burst itself is the likely vanish trigger).

namespace {

constexpr uint32_t kStepUs = 16667;  // 60 fps

struct VanishFeeder {
  UepEncoder enc{layers(), 15};
  FramePipeline pipe;

  // flags select the frame kind: IDR -> type 19, ENHANCE -> TRAIL_N (scan and
  // producer flag agree, so no disagreement side effects), else TRAIL_R base.
  void feed(uint32_t pts, uint8_t flags, uint64_t now_ms) {
    const bool idr = (flags & VENC_FRAME_FLAG_IDR) != 0;
    const bool enh = (flags & VENC_FRAME_FLAG_ENHANCE) != 0;
    auto buf = ring_buf(idr ? 19 : (enh ? 0 : 1), 0, 400);
    pipe.encode(enc, buf.data(), payload_len(buf), meta_flags(pts, flags), now_ms);
  }

  // Even slots base, odd slots enhance — the capture-proven alternation.
  void warm_up(uint32_t n = 6, uint32_t pts0 = 0) {
    for (uint32_t i = 0; i < n; ++i)
      feed(pts0 + i * kStepUs, (i % 2) ? VENC_FRAME_FLAG_ENHANCE : 0, 1000 + i * 17);
  }
};

}  // namespace

TEST(frame_pipeline_pts_hole_books_vanished_base_and_latches_self_idr) {
  VanishFeeder f;
  f.warm_up();
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(!f.pipe.self_idr_pending());

  // Slot 6 (base) vanishes in the ring; the next read is slot 7 (enhance).
  // prev (slot 5) and cur are both enhance -> the vanished one was base.
  f.feed(7 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1200);
  CHECK(f.pipe.vanished_base() == 1);
  CHECK(f.pipe.vanished_enhance() == 0);
  CHECK(f.pipe.self_idr_pending());
  CHECK(f.pipe.self_idr_refused() == 0);

  // A double hole: slots 8 (base) and 9 (enhance) both vanish; next read is
  // slot 10 (base). Classes alternate out from the previous frame's flag.
  f.feed(10 * kStepUs, 0, 1260);
  CHECK(f.pipe.vanished_base() == 2);
  CHECK(f.pipe.vanished_enhance() == 1);
}

TEST(frame_pipeline_enhance_vanish_is_counted_but_requests_nothing) {
  VanishFeeder f;
  f.warm_up(7);
  // Slot 7 (enhance) vanishes; next read is slot 8 (base). Harmless
  // (TRAIL_N, nothing references it): counted, but no self-IDR latched.
  f.feed(8 * kStepUs, 0, 1200);
  CHECK(f.pipe.vanished_enhance() == 1);
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(!f.pipe.self_idr_pending());
  CHECK(f.pipe.self_idr_refused() == 0);
}

TEST(frame_pipeline_pts_wrap_is_not_a_hole_and_holes_across_wrap_fire) {
  // pts is a 32-bit µs counter (wraps every ~71.6 min). Unsigned deltas make
  // the wrap itself invisible, and a real hole spanning the wrap still fires.
  VanishFeeder f;
  const uint32_t pts0 = 0xFFFFFFF0u - 3 * kStepUs;  // wraps mid-warm-up
  f.warm_up(6, pts0);
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(f.pipe.vanished_enhance() == 0);

  f.feed(pts0 + 7 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1200);  // slot 6 vanished
  CHECK(f.pipe.vanished_base() == 1);
}

TEST(frame_pipeline_shed_leaves_no_pts_hole) {
  // Shed happens in maburd AFTER the ring read: encode() observes every
  // frame's meta (pts) before drop_if_shed, so read-side pts stays continuous
  // across shed drops and shedding must never book a vanish.
  VanishFeeder f;
  f.enc.set_shed(1, true);  // shed enhance stream (sid 1)
  for (uint32_t i = 0; i < 12; ++i)
    f.feed(i * kStepUs, (i % 2) ? VENC_FRAME_FLAG_ENHANCE : 0, 1000 + i * 17);
  CHECK(f.enc.dropped(1) > 0);  // sheds actually happened
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(f.pipe.vanished_enhance() == 0);
}

TEST(frame_pipeline_vanish_right_after_idr_is_refused_not_requested) {
  // Re-seed loop guard: the IDR's own ~10x frame-size burst is the likely
  // vanish trigger, so a base vanish detected within kSelfIdrGuardMs of the
  // last IDR read must NOT self-request (it would re-seed at cooldown rate,
  // forever) — it is counted as refused so the loop stays visible.
  VanishFeeder f;
  f.warm_up();
  f.feed(6 * kStepUs, VENC_FRAME_FLAG_IDR, 1102);            // IDR in a base slot
  f.feed(7 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1119);
  // Slot 8 (base) vanishes, detected 48 ms after the IDR read: refused.
  f.feed(9 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1150);
  CHECK(f.pipe.vanished_base() == 1);
  CHECK(f.pipe.self_idr_refused() == 1);
  CHECK(!f.pipe.self_idr_pending());

  // Same shape well past the guard window: latches the request.
  f.feed(10 * kStepUs, 0, 1700);
  f.feed(11 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1717);
  f.feed(13 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1750);  // slot 12 (base) vanished
  CHECK(f.pipe.vanished_base() == 2);
  CHECK(f.pipe.self_idr_pending());
  CHECK(f.pipe.self_idr_refused() == 1);
}

TEST(frame_pipeline_idr_arrival_clears_self_idr_pending) {
  // The pending latch is a LEVEL reconciled by the healing event itself: any
  // IDR passing through (granted, or a natural one) resets the decoder's DPB
  // downstream, so the request is moot the moment one ships.
  VanishFeeder f;
  f.warm_up();
  f.feed(7 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1200);  // slot 6 (base) vanished
  CHECK(f.pipe.self_idr_pending());
  f.feed(8 * kStepUs, VENC_FRAME_FLAG_IDR, 1220);      // the granted IDR arrives
  CHECK(!f.pipe.self_idr_pending());
}

TEST(frame_pipeline_discontinuity_resets_vanish_tracking) {
  // mark_discontinuity (producer restart / link-up) means the pts stream may
  // re-base: the tracker must re-anchor and re-confirm its period instead of
  // booking phantom vanishes against the old epoch.
  VanishFeeder f;
  f.warm_up();
  f.pipe.mark_discontinuity();
  f.feed(0x40000000u, 0, 3000);  // new epoch: seeds, never counts
  // Hole-shaped delta, but the period is unconfirmed after the reset.
  f.feed(0x40000000u + 2 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 3020);
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(f.pipe.vanished_enhance() == 0);
  CHECK(!f.pipe.self_idr_pending());
}

TEST(frame_pipeline_no_vanish_before_period_confirmed) {
  VanishFeeder f;
  f.feed(0, 0, 1000);
  f.feed(2 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1017);  // hole-shaped 1st delta
  f.feed(3 * kStepUs, 0, 1034);
  f.feed(4 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1051);
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(f.pipe.vanished_enhance() == 0);
}

TEST(frame_pipeline_pts_resync_jump_is_not_counted_as_vanish) {
  // A giant forward jump (>= 1 s) is an encoder clock discontinuity, not a
  // plausible ring vanish (the ring holds ~133 ms): re-anchor, count nothing.
  VanishFeeder f;
  f.warm_up();
  f.feed(5 * kStepUs + 5000000u, 0, 1200);  // +5 s jump
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(f.pipe.vanished_enhance() == 0);
  // The stream after the jump re-confirms and detection works again.
  for (uint32_t i = 1; i < 7; ++i)
    f.feed(5 * kStepUs + 5000000u + i * kStepUs,
           (i % 2) ? VENC_FRAME_FLAG_ENHANCE : 0, 1200 + i * 17);
  f.feed(5 * kStepUs + 5000000u + 8 * kStepUs, 0,
         1400);  // slot 7 (enhance) vanished post-resync
  CHECK(f.pipe.vanished_base() + f.pipe.vanished_enhance() == 1);
}

MTEST_MAIN

TEST(frame_pipeline_reset_vanish_counters_zeros_all_counters) {
  // Boot-window churn books counts before the link exists (flight finding
  // 2026-08-13: ~8-9 pre-link counts per boot); main zeroes at the FIRST
  // link-establish so telemetry reports in-flight vanishes only.
  VanishFeeder f;
  f.warm_up(6);
  f.feed(6 * kStepUs, VENC_FRAME_FLAG_IDR, 1102);   // IDR read: arms the guard
  f.feed(7 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1119);
  // Slot 8 (base) vanishes inside the guard window -> refused, not latched.
  f.feed(9 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1136);
  // Slots 10 (base) + 11 (enhance) vanish outside the guard.
  f.feed(12 * kStepUs, 0, 1700);
  CHECK(f.pipe.vanished_base() == 2);
  CHECK(f.pipe.vanished_enhance() == 1);
  CHECK(f.pipe.self_idr_refused() == 1);

  f.pipe.reset_vanish_counters();
  CHECK(f.pipe.vanished_base() == 0);
  CHECK(f.pipe.vanished_enhance() == 0);
  CHECK(f.pipe.self_idr_refused() == 0);
}

TEST(frame_pipeline_reset_vanish_counters_preserves_detection) {
  // Reset must touch COUNTERS only — not the period tracker or prev-pts
  // anchor (unlike mark_discontinuity). A hole on the very next read still
  // books, so an implementation that resets by re-anchoring fails here.
  VanishFeeder f;
  f.warm_up(6);
  f.feed(7 * kStepUs, VENC_FRAME_FLAG_ENHANCE, 1200);  // slot 6 base vanished
  CHECK(f.pipe.vanished_base() == 1);

  f.pipe.reset_vanish_counters();
  f.feed(8 * kStepUs, 0, 1260);   // slot 8 (base), contiguous — no hole
  f.feed(10 * kStepUs, 0, 1277);  // slot 9 (enhance) vanished post-reset
  CHECK(f.pipe.vanished_enhance() == 1);
  CHECK(f.pipe.vanished_base() == 0);
}
