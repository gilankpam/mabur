#include "../drone/src/peak_rate.h"
#include "mtest.h"

using mabur::PeakRate;

// Feeds a 60 fps stream sampled on a 5 ms grid the way the agent loop
// does: frames land every 16.67 ms, so a 100 ms wall window holds 6 or 7
// of them depending on alignment. kbps is per-frame size x 60 fps.
struct Feed {
  uint64_t bytes = 0, frames = 0;
  double next_frame_ms = 0.0;
  void advance_to(uint64_t now_ms, uint64_t frame_bytes) {
    while (next_frame_ms <= static_cast<double>(now_ms)) {
      bytes += frame_bytes;
      frames += 1;
      next_frame_ms += 1000.0 / 60.0;
    }
  }
};

// A flat 16 Mbit/s stream (33333 B/frame) must read 16000 kbit/s at every
// alignment — NOT 16000 x 7/6 whenever a window happens to hold 7 frames.
// That phantom +17 % is exactly what the first deploy printed on a static
// scene (enc_pk100 17.2-18.6 Mb/s against 16.384 programmed).
TEST(peak_rate_flat_stream_has_no_alignment_phantom) {
  PeakRate pk(100, 60);
  Feed f;
  for (uint64_t now = 0; now <= 1000; now += 5) {
    f.advance_to(now, 33333);
    pk.sample(now, f.bytes, f.frames);
  }
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 15950 && peak <= 16050);
}

// A 100 ms burst at twice the frame size inside a flat second: the peak
// reports the burst window, and take() resets so the next second starts
// clean.
TEST(peak_rate_reports_the_burst_window_then_resets) {
  PeakRate pk(100, 60);
  Feed f;
  for (uint64_t now = 0; now <= 1000; now += 5) {
    const bool burst = now >= 400 && now < 500;
    f.advance_to(now, burst ? 66666 : 33333);
    pk.sample(now, f.bytes, f.frames);
  }
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 28000 && peak <= 32100);  // window straddles the burst edges
  CHECK(pk.take_peak_kbps() == 0);
  for (uint64_t now = 1005; now <= 2000; now += 5) {
    f.advance_to(now, 33333);
    pk.sample(now, f.bytes, f.frames);
  }
  const uint32_t peak2 = pk.take_peak_kbps();
  CHECK(peak2 >= 15950 && peak2 <= 16050);
}

// Sparse wakes (a stalled loop) close a longer window; frame normalisation
// still yields the per-frame-size rate.
TEST(peak_rate_sparse_samples_still_frame_normalised) {
  PeakRate pk(100, 60);
  pk.sample(0, 0, 0);
  pk.sample(250, 15 * 33333, 15);  // 15 frames at 33333 B
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 15950 && peak <= 16050);
}

// A counter reset (encoder restart re-seeds the counters) re-opens the
// window silently instead of printing a ~2^64 rate.
TEST(peak_rate_counter_going_backwards_reseeds) {
  PeakRate pk(100, 60);
  pk.sample(0, 1000000, 600);
  pk.sample(50, 1010000, 603);
  pk.sample(120, 5000, 1);  // went backwards
  CHECK(pk.take_peak_kbps() == 0);
  pk.sample(230, 5000 + 6 * 33333, 7);
  const uint32_t peak = pk.take_peak_kbps();
  CHECK(peak >= 15950 && peak <= 16050);
}

// No frames inside a window (encoder stalled) reads 0, not a division by
// zero.
TEST(peak_rate_no_frames_reads_zero) {
  PeakRate pk(100, 60);
  pk.sample(0, 100, 5);
  pk.sample(150, 100, 5);
  CHECK(pk.take_peak_kbps() == 0);
}

MTEST_MAIN
