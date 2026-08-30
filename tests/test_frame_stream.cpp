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
  struct Ev { char kind; FrameHdr hdr; std::vector<uint8_t> bytes; bool complete; uint8_t sid = 0; };
  std::vector<Ev> evs;
  std::vector<uint8_t> cur;
  FrameStream::Callbacks cbs() {
    return {
        [this](const FrameHdr& h, uint8_t sid) { evs.push_back({'B', h, {}, false, sid}); cur.clear(); },
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
  mabur::Fragmenter frag;
  std::vector<uint8_t> pay(1000, 0xAB);
  for (uint16_t id = 0; id < 3; ++id)
    for (auto& p : frag_frame(frag, id, pay))
      fs.push_fragment(3, p.data(), p.size(), 10 + id);
  REQUIRE(cap.evs.size() == 6);  // B E B E B E
  for (size_t i = 0; i < 6; i += 2) {
    CHECK(cap.evs[i].kind == 'B');
    CHECK(cap.evs[i].hdr.frame_id == i / 2);
    CHECK(cap.evs[i].sid == 3);  // begin_frame carries the fragment's stream id
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
  mabur::Fragmenter fa, fb;
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
  mabur::Fragmenter fa, fb;
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
  mabur::Fragmenter frag;
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
  mabur::Fragmenter frag;
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
  mabur::Fragmenter frag;
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
  mabur::Fragmenter frag;
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

TEST(waybeam_restart_discont_continues_clean) {
  // waybeam (drone video producer) restarts mid-stream: the ring is recreated
  // and maburd's FrameSource reattaches, setting kFlagDiscont on ONE frame —
  // but maburd itself keeps running, so frame_id does NOT reset; it keeps
  // climbing from wherever it was. Establish the stream at a nonzero starting
  // frame_id (5000) so next_emit_id64_ has nonzero low-16 bits: that's what
  // exposes the re-base bug (next_emit_id64_ + 0x20000 inherits those bits).
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter frag;
  std::vector<uint8_t> pay(100, 0x42);
  for (uint16_t id = 5000; id <= 5002; ++id)
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), 10);
  REQUIRE(fs.frames_clean() == 3);
  // Producer restart: id continues climbing (NOT reset) but discont flag set.
  for (auto& p : frag_frame(frag, 5003, pay, mabur::framewire::kFlagDiscont))
    fs.push_fragment(1, p.data(), p.size(), 20);
  for (uint16_t id = 5004; id <= 5006; ++id)
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), 21);
  CHECK(fs.frames_clean() == 7);  // all seven frames emit, none wedged/dropped
  // Confirm real emission (not just a counter), by checking payload bytes.
  REQUIRE(cap.evs.size() == 14);  // B E per frame x 7
  for (size_t i = 0; i < 14; i += 2) {
    CHECK(cap.evs[i].kind == 'B');
    CHECK(cap.evs[i + 1].complete);
    CHECK(cap.evs[i + 1].bytes == pay);
  }
}

TEST(discont_run_rebases_once_and_holds_order) {
  // Sticky discont (drone side) sets the flag on EVERY frame for ~1 s after a
  // restart. Only the first flagged arrival may re-base; the rest must
  // delta-track so ordering inside the run survives out-of-order decode, and
  // a flagged frame that did NOT re-base must not skip the gap-hold.
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter fa, fb;
  std::vector<uint8_t> p0(100, 0x00), p1(100, 0x11), p2(100, 0x22);
  for (auto& p : frag_frame(fa, 3000, p0)) fs.push_fragment(1, p.data(), p.size(), 10);
  REQUIRE(fs.frames_clean() == 1);
  // Restart: new epoch, ids 0..2 all flagged. Frame 1 (layer 1) decodes late:
  // arrival order 0, 2, 1.
  for (auto& p : frag_frame(fa, 0, p0, mabur::framewire::kFlagDiscont))
    fs.push_fragment(1, p.data(), p.size(), 20);
  for (auto& p : frag_frame(fb, 2, p2, mabur::framewire::kFlagDiscont))
    fs.push_fragment(2, p.data(), p.size(), 21);
  for (auto& p : frag_frame(fa, 1, p1, mabur::framewire::kFlagDiscont))
    fs.push_fragment(1, p.data(), p.size(), 22);
  // All four frames emitted, in id order — frame 2 held for frame 1.
  REQUIRE(cap.evs.size() == 8);
  CHECK(cap.evs[2].hdr.frame_id == 0);
  CHECK(cap.evs[4].hdr.frame_id == 1);
  CHECK(cap.evs[6].hdr.frame_id == 2);
  CHECK(fs.frames_clean() == 4);
  CHECK(fs.frames_dropped() == 0);
}

TEST(waybeam_restart_discont_no_phantom_drops) {
  // Same restart scenario: the discont re-base must not book a synthetic
  // ~0x20000 id jump as real "dropped" frames.
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  mabur::Fragmenter frag;
  std::vector<uint8_t> pay(100, 0x42);
  for (uint16_t id = 5000; id <= 5002; ++id)
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), 10);
  for (auto& p : frag_frame(frag, 5003, pay, mabur::framewire::kFlagDiscont))
    fs.push_fragment(1, p.data(), p.size(), 20);
  for (uint16_t id = 5004; id <= 5006; ++id)
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), 21);
  CHECK(fs.frames_dropped() < 100);  // not a huge synthetic ~131072 jump
}

TEST(lost_discont_restart_recovers_via_stall_watchdog) {
  // The 2026-07-25 rig outage: maburd restarts, every frame of its discont
  // window is lost at radio bring-up, and the new epoch's ids unwrap BELOW
  // the surviving emit cursor via the signed-16 delta. Every arriving frame
  // is then evicted as "late" — no emission for up to 18 min. The watchdog
  // must notice frames arriving with nothing emitted and reset ordering.
  Capture cap;
  FrameStream fs({50, 8, 500}, cap.cbs());  // stall_reset_ms = 500
  mabur::Fragmenter frag;
  std::vector<uint8_t> pay(100, 0x42);
  // Establish the stream ~3000 frames into the old epoch (a ~50 s session).
  for (uint16_t id = 3000; id <= 3002; ++id)
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), 10);
  REQUIRE(fs.frames_clean() == 3);
  // Restart: ids resume near 0 with NO discont flag, 60 fps.
  uint64_t t = 100;
  uint16_t id = 1;
  for (; id <= 10; ++id, t += 16)  // well under stall_reset_ms: still stalled
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), t);
  CHECK(cap.evs.size() == 6);      // nothing emitted since the restart
  CHECK(fs.frames_dropped() >= 10);
  for (; id <= 60; ++id, t += 16)  // keep arriving past the watchdog window
    for (auto& p : frag_frame(frag, id, pay)) fs.push_fragment(1, p.data(), p.size(), t);
  CHECK(fs.stall_resets() == 1);
  CHECK(fs.frames_clean() > 13);   // emission resumed and kept going
}

TEST(per_sid_gap_timeout_mid_frame) {
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  fs.set_gap_timeout(1, 100);  // rate-aware stretch on the enh stream only
  mabur::Fragmenter frag;
  std::vector<uint8_t> pay(2000, 0xCD);
  auto frags = frag_frame(frag, 0, pay);
  REQUIRE(frags.size() >= 4);
  for (size_t i = 0; i < frags.size(); ++i)
    if (i != 2) fs.push_fragment(1, frags[i].data(), frags[i].size(), 10);
  fs.poll(70);   // past the old 50ms — must still be waiting
  CHECK(fs.frames_truncated() == 0);
  fs.poll(115);  // past 10 + 100ms
  CHECK(fs.frames_truncated() == 1);
}

TEST(whole_frame_gap_waits_for_the_max_sid_timeout) {
  Capture cap;
  FrameStream fs({50, 8}, cap.cbs());
  fs.set_gap_timeout(1, 150);  // the MISSING frame's sid is unknown — the
                               // gap must wait for the slowest stream
  mabur::Fragmenter frag;
  std::vector<uint8_t> pay(400, 0x77);
  for (auto& p : frag_frame(frag, 0, pay)) fs.push_fragment(0, p.data(), p.size(), 10);
  // Frame 1 never arrives; frame 2 arrives complete on sid 0 (timeout 50).
  for (auto& p : frag_frame(frag, 2, pay)) fs.push_fragment(0, p.data(), p.size(), 20);
  fs.poll(90);   // 20 + 50ms passed, but max(sid timeouts) = 150
  CHECK(fs.frames_dropped() == 0);
  fs.poll(180);  // 20 + 150ms passed
  CHECK(fs.frames_dropped() == 1);
  CHECK(fs.frames_clean() == 2);
}

MTEST_MAIN
