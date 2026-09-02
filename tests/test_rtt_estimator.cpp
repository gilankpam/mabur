// link-rtt (2026-09-02): GS-side control-path RTT + pts-offset estimator.
// The drone echoes WHICH RCF rcf_age_ms ages against (seq identity) plus
// its pts-domain clock at telem build; this module matches the echo
// against recorded send times and produces:
//   rtt   = (telem_rx − rcf_send) − rcf_age        [control-path RTT]
//   offset = pts_at_build − telem_rx + rtt/2       [pts − GS-mono clock]
// The offset reported is the one from the lowest-RTT sample in a sliding
// window (PTP-style min-filter: queueing inflates rtt and offset together,
// so the least-queued sample is the least-biased offset).
#include <cmath>
#include "mtest.h"
#include "rtt_estimator.h"

using maburgs::RttEstimator;

TEST(rtt_happy_path_sample) {
  RttEstimator e;
  CHECK(!e.has_rtt());
  e.on_rcf_sent(10, 1'000'000);
  // Heard 5ms before telem build; telem lands 20ms after our send.
  CHECK(e.on_telem(10, /*valid=*/true, /*age_ms=*/5, /*pts=*/0,
                   /*rx_us=*/1'020'000));
  REQUIRE(e.has_rtt());
  CHECK(std::abs(e.rtt_ms() - 15.0) < 1e-9);
  CHECK(std::abs(e.rtt_min_ms() - 15.0) < 1e-9);
  CHECK(e.samples() == 1);
}

TEST(rtt_rejects_unmatched_and_invalid) {
  RttEstimator e;
  e.on_rcf_sent(10, 1'000'000);
  CHECK(!e.on_telem(11, true, 5, 0, 1'020'000));   // unknown seq
  CHECK(!e.on_telem(10, false, 5, 0, 1'020'000));  // echo invalid (flags bit3)
  CHECK(!e.on_telem(10, true, 1000, 0, 2'500'000));  // stale: age >= 1s
  CHECK(!e.has_rtt());
}

TEST(rtt_negative_clamp_and_discard) {
  RttEstimator e;
  // age quantization (1ms floor-truncated on the drone) can push the
  // subtraction slightly negative: clamp to 0 within -1ms, discard worse.
  e.on_rcf_sent(20, 1'000'000);
  CHECK(e.on_telem(20, true, 5, 0, 1'004'200));  // rtt = -800us -> 0
  CHECK(std::abs(e.rtt_ms() - 0.0) < 1e-9);
  e.on_rcf_sent(21, 2'000'000);
  CHECK(!e.on_telem(21, true, 7, 0, 2'004'200));  // rtt = -2800us -> discard
  CHECK(e.samples() == 1);
}

TEST(rtt_implausible_discarded) {
  RttEstimator e;
  e.on_rcf_sent(30, 1'000'000);
  // >3s "RTT" means the ring entry and the echo no longer describe the
  // same exchange (u16 seq reuse after wrap) — corrupt, not a sample.
  CHECK(!e.on_telem(30, true, 5, 0, 5'000'000));
}

TEST(rtt_ewma_and_min_track_separately) {
  RttEstimator e;
  e.on_rcf_sent(1, 1'000'000);
  CHECK(e.on_telem(1, true, 0, 0, 1'020'000));  // 20ms
  e.on_rcf_sent(2, 2'000'000);
  CHECK(e.on_telem(2, true, 0, 0, 2'010'000));  // 10ms
  CHECK(e.rtt_ms() > 10.0 && e.rtt_ms() < 20.0);  // EWMA between
  CHECK(std::abs(e.rtt_min_ms() - 10.0) < 1e-9);  // min ratchets
  CHECK(e.samples() == 2);
}

TEST(offset_from_min_rtt_sample) {
  RttEstimator e;
  CHECK(!e.has_offset());
  // Sample A: rtt 20ms, pts clock reads 500s at build.
  e.on_rcf_sent(1, 1'000'000);
  CHECK(e.on_telem(1, true, 0, 500'000'000, 1'020'000));
  REQUIRE(e.has_offset());
  // off = 500e6 - 1'020'000 + 10'000
  CHECK(e.pts_off_us() == 500'000'000 - 1'020'000 + 10'000);
  // Sample B: LOWER rtt (10ms) -> its offset wins even though it came later.
  e.on_rcf_sent(2, 2'000'000);
  CHECK(e.on_telem(2, true, 0, 501'000'000, 2'010'000));
  CHECK(e.pts_off_us() == 501'000'000 - 2'010'000 + 5'000);
  // Sample C: HIGHER rtt (30ms) -> min-filter keeps B's offset.
  e.on_rcf_sent(3, 3'000'000);
  CHECK(e.on_telem(3, true, 0, 502'000'000, 3'030'000));
  CHECK(e.pts_off_us() == 501'000'000 - 2'010'000 + 5'000);
}

TEST(offset_skipped_when_pts_unavailable) {
  RttEstimator e;
  e.on_rcf_sent(1, 1'000'000);
  // pts_at_build 0 = drone sentinel for "MI clock unreadable": the RTT
  // sample is still good, the offset must not be.
  CHECK(e.on_telem(1, true, 0, 0, 1'020'000));
  CHECK(e.has_rtt());
  CHECK(!e.has_offset());
}

TEST(send_ring_evicts_oldest) {
  RttEstimator e;
  e.on_rcf_sent(100, 1'000'000);
  for (uint16_t i = 0; i < 64; ++i)  // ring capacity 64: seq 100 evicted
    e.on_rcf_sent(static_cast<uint16_t>(200 + i), 2'000'000 + i);
  CHECK(!e.on_telem(100, true, 5, 0, 1'020'000));
  CHECK(e.on_telem(200, true, 0, 0, 2'010'000));
}

TEST(min_rtt_window_slides) {
  RttEstimator e;
  // An early lucky sample must eventually age out of the offset filter so
  // a drifting clock cannot pin the offset to a minutes-old reading.
  e.on_rcf_sent(1, 1'000'000);
  CHECK(e.on_telem(1, true, 0, 500'000'000, 1'005'000));  // 5ms, the lucky one
  const int64_t lucky = e.pts_off_us();
  // 32 more samples at 10ms rtt push the lucky one out of the window.
  for (uint16_t i = 0; i < 32; ++i) {
    const uint64_t t = 2'000'000 + 100'000ull * i;
    e.on_rcf_sent(static_cast<uint16_t>(10 + i), t);
    CHECK(e.on_telem(static_cast<uint16_t>(10 + i), true, 0,
                     600'000'000 + 100'000ull * i, t + 10'000));
  }
  CHECK(e.pts_off_us() != lucky);
}

MTEST_MAIN
