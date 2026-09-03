#pragma once
#include <cstdint>

namespace mabur {

// Peak short-window encoder rate from the monotonic (bytes, frames)
// counters, for the drone `stats:` line (2026-09-03,
// docs/handover-venc-overshoot-2026-09-03.md).
//
// The 1 Hz Telem enc_kbytes delta averages away exactly the thing that
// fills the TxQueue: flight-0011's scene-change bursts read 17-22 Mb/s
// over a second against a 16 Mb/s command, but a 154-body queue at rung 5
// (~1.7 Mbit) could be a 100 ms spike at 17 Mb/s excess or a 1 s trickle
// at 1.7 — the telemetry cannot tell. The agent loop already wakes every
// link.rc_drain_ms (5 ms) and reads both counters for free, so it feeds
// every wake here; a window closes once >= window_ms has elapsed since it
// opened and its rate competes for the max. take_peak_kbps() returns the
// busiest closed window since the last take and resets — printed as
// `enc_pk100=` in decimal kbit/s, the same unit as cmd_kbps.
//
// The rate is bytes over FRAMES x nominal frame period, not bytes over
// wall time: a 100 ms wall window at 60 fps holds 6 or 7 frames depending
// on alignment, which read as a phantom +17 % on a perfectly flat stream
// (measured 1.05-1.13x on the static bench scene, first deploy). Frame
// normalisation makes the peak a size-driven quantity — which is the one
// the encoder overshoot question is about; frame bunching was ruled out
// separately (commit interval max 19.9 ms on the bench).
//
// A counter that goes backwards (encoder re-seed) re-opens the window
// without closing it, so a restart never prints a ~2^64 rate. A window
// with no frames in it closes at 0.
class PeakRate {
 public:
  PeakRate(uint64_t window_ms, uint32_t nominal_fps)
      : window_ms_(window_ms > 0 ? window_ms : 1),
        frame_us_(nominal_fps > 0 ? 1000000ull / nominal_fps : 1000000ull / 60) {}

  void sample(uint64_t now_ms, uint64_t bytes, uint64_t frames) {
    if (!open_ || bytes < bytes0_ || frames < frames0_) {
      start_ms_ = now_ms;
      bytes0_ = bytes;
      frames0_ = frames;
      open_ = true;
      return;
    }
    if (now_ms - start_ms_ < window_ms_) return;
    const uint64_t df = frames - frames0_;
    if (df > 0) {
      // bits / (frames * frame_us) = bits/us = Mbit/s; x1000 -> kbit/s
      const uint64_t kbps = (bytes - bytes0_) * 8ull * 1000ull / (df * frame_us_);
      if (kbps > peak_kbps_) peak_kbps_ = kbps;
    }
    start_ms_ = now_ms;
    bytes0_ = bytes;
    frames0_ = frames;
  }

  // kbit/s (decimal) of the busiest window closed since the last call;
  // 0 when none closed. Saturates at UINT32_MAX.
  uint32_t take_peak_kbps() {
    const uint64_t p = peak_kbps_;
    peak_kbps_ = 0;
    return p > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(p);
  }

 private:
  uint64_t window_ms_;
  uint64_t frame_us_;
  uint64_t start_ms_ = 0;
  uint64_t bytes0_ = 0;
  uint64_t frames0_ = 0;
  uint64_t peak_kbps_ = 0;
  bool open_ = false;
};

}  // namespace mabur
