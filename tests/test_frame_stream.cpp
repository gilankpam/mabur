#include <cstring>
#include <string>
#include <vector>
#include "mtest.h"
#include "frame_stream.h"
#include "mabur/frag.h"
#include "mabur/frame_wire.h"
using namespace maburgs;
using mabur::framewire::FrameHdr;

namespace {
struct Capture {
  struct Ev { char kind; FrameHdr hdr; std::vector<uint8_t> bytes; bool complete; };
  std::vector<Ev> evs;
  std::vector<uint8_t> cur;
  FrameStream::Callbacks cbs() {
    return {
        [this](const FrameHdr& h) { evs.push_back({'B', h, {}, false}); cur.clear(); },
        [this](const uint8_t* d, size_t n) { cur.insert(cur.end(), d, d + n); },
        [this](bool c) { evs.push_back({'E', {}, cur, c}); cur.clear(); }};
  }
};

// Fragments one frame unit (FrameHdr+payload) with a wide Fragmenter and
// returns the raw fragment packets as UepDecoder wide mode would emit them.
std::vector<std::vector<uint8_t>> frag_frame(mabur::Fragmenter& f, uint16_t frame_id,
                                             const std::vector<uint8_t>& payload,
                                             uint8_t flags = 0) {
  std::vector<uint8_t> unit(mabur::framewire::kFrameHdrLen + payload.size());
  FrameHdr h; h.frame_id = frame_id; h.flags = flags; h.pts_us = 16667u * frame_id;
  mabur::framewire::pack_frame_hdr(h, unit.data());
  std::memcpy(unit.data() + 8, payload.data(), payload.size());
  return f.fragment(unit.data(), unit.size(), 158);
}
}  // namespace

TEST(in_order_frames_emit_clean) {
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter frag(true);
  std::vector<uint8_t> pay(1000, 0xAB);
  for (uint16_t id = 0; id < 3; ++id)
    for (auto& p : frag_frame(frag, id, pay))
      fs.push_fragment(1, p.data(), p.size(), 10 + id);
  REQUIRE(cap.evs.size() == 6);  // B E B E B E
  for (size_t i = 0; i < 6; i += 2) {
    CHECK(cap.evs[i].kind == 'B');
    CHECK(cap.evs[i].hdr.frame_id == i / 2);
    CHECK(cap.evs[i + 1].complete);
    CHECK(cap.evs[i + 1].bytes == pay);  // FrameHdr stripped
  }
  CHECK(fs.frames_clean() == 3);
}

TEST(mid_stream_reorder_holds_for_gap_frame) {
  // Stream established with frame 0. Frame 2 (layer 2) then decodes fully
  // BEFORE any fragment of frame 1 (layer 1): emission must hold, then go
  // 1 -> 2 once frame 1 arrives.
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter fa(true), fb(true);
  std::vector<uint8_t> p0(500, 0x00), p1(500, 0x11), p2(500, 0x22);
  for (auto& p : frag_frame(fa, 0, p0)) fs.push_fragment(1, p.data(), p.size(), 10);
  REQUIRE(cap.evs.size() == 2);  // frame 0 emitted clean
  for (auto& p : frag_frame(fb, 2, p2)) fs.push_fragment(2, p.data(), p.size(), 11);
  CHECK(cap.evs.size() == 2);    // holds: frame 1 is the gap, not stale yet
  for (auto& p : frag_frame(fa, 1, p1)) fs.push_fragment(1, p.data(), p.size(), 12);
  REQUIRE(cap.evs.size() == 6);
  CHECK(cap.evs[2].hdr.frame_id == 1);
  CHECK(cap.evs[3].bytes == p1);
  CHECK(cap.evs[4].hdr.frame_id == 2);
  CHECK(cap.evs[5].bytes == p2);
}

TEST(late_frame_after_advance_is_dropped) {
  // Cold start emits the first-known head immediately (zero start latency);
  // an earlier frame decoding later is late -> dropped, never emitted.
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter fa(true), fb(true);
  std::vector<uint8_t> p0(500, 0x00), p1(500, 0x11);
  for (auto& p : frag_frame(fb, 1, p1)) fs.push_fragment(2, p.data(), p.size(), 10);
  REQUIRE(cap.evs.size() == 2);  // frame 1 emitted at cold start
  for (auto& p : frag_frame(fa, 0, p0)) fs.push_fragment(1, p.data(), p.size(), 12);
  fs.poll(13);
  CHECK(cap.evs.size() == 2);    // frame 0 never emitted
  CHECK(fs.frames_dropped() >= 1);
}

TEST(gap_timeout_truncates_prefix) {
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter frag(true);
  std::vector<uint8_t> pay(2000, 0xCD);
  auto frags = frag_frame(frag, 0, pay);
  REQUIRE(frags.size() >= 4);
  // Deliver all but fragment 2 — contiguous prefix is frags 0..1.
  for (size_t i = 0; i < frags.size(); ++i)
    if (i != 2) fs.push_fragment(1, frags[i].data(), frags[i].size(), 10);
  fs.poll(20);
  CHECK(cap.evs.size() == 1);  // began, streaming prefix, gap holds it open
  fs.poll(70);                 // 10 + 50ms timeout passed
  REQUIRE(cap.evs.size() == 2);
  CHECK(cap.evs[1].kind == 'E');
  CHECK(!cap.evs[1].complete);
  // Prefix = frag0 payload (minus 8-byte hdr) + frag1 payload = 2*158 - 8.
  CHECK(cap.evs[1].bytes.size() == 2 * 158 - 8);
  CHECK(fs.frames_truncated() == 1);
}

TEST(lookahead_forces_advance) {
  Capture cap;
  FrameStream fs({5000, 3}, cap.cbs());  // huge timeout: only lookahead fires
  // ONE fragmenter per layer: FRAG seq must advance across frames, or the
  // per-(sid,fseq) slots collide.
  mabur::Fragmenter frag(true);
  std::vector<uint8_t> pay(400, 0x77);   // 408-byte unit -> 3 fragments
  auto f0 = frag_frame(frag, 0, pay);
  REQUIRE(f0.size() == 3);
  // Frame 0: only fragment 0 arrives (incomplete forever).
  fs.push_fragment(1, f0[0].data(), f0[0].size(), 10);
  for (uint16_t id = 1; id <= 3; ++id)
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), 11);
  fs.poll(12);
  // Frame 3 is 3 ahead of head-of-line 0 => frame 0 force-truncated,
  // then 1..3 emit clean.
  CHECK(fs.frames_truncated() == 1);
  CHECK(fs.frames_clean() == 3);
}

TEST(discontinuity_resets_ordering) {
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter frag(true);
  std::vector<uint8_t> pay(100, 0x42);
  for (auto& p : frag_frame(frag, 100, pay)) fs.push_fragment(1, p.data(), p.size(), 10);
  // Producer restarted: id jumps backward with the discont flag set.
  for (auto& p : frag_frame(frag, 3, pay, mabur::framewire::kFlagDiscont))
    fs.push_fragment(1, p.data(), p.size(), 20);
  CHECK(fs.frames_clean() == 2);  // both emitted despite the backward jump
}

TEST(reset_closes_in_flight_frame_truncated) {
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter frag(true);
  std::vector<uint8_t> pay(2000, 0xEF);
  auto frags = frag_frame(frag, 0, pay);
  REQUIRE(frags.size() >= 4);
  // Deliver only fragment 0: the frame begins and streams its prefix, but
  // never completes.
  fs.push_fragment(1, frags[0].data(), frags[0].size(), 10);
  REQUIRE(cap.evs.size() == 1);
  CHECK(cap.evs[0].kind == 'B');
  fs.reset();
  REQUIRE(cap.evs.size() == 2);
  CHECK(cap.evs[1].kind == 'E');
  CHECK(!cap.evs[1].complete);
  // reset() must not touch FrameStream's own accounting.
  CHECK(fs.frames_clean() == 0);
  CHECK(fs.frames_truncated() == 0);
}

MTEST_MAIN
