#pragma once
#include <cstdint>

namespace mabur {

// Peak short-window byte rate of a monotonic counter, for the drone
// `stats:` line (2026-09-03, docs/handover-venc-overshoot-2026-09-03.md).
//
// The 1 Hz Telem enc_kbytes delta averages away exactly the thing that
// fills the TxQueue: flight-0011's scene-change bursts read 17-22 Mb/s
// over a second against a 16 Mb/s command, but a 154-body queue at rung 5
// (~1.7 Mbit) could be a 100 ms spike at 17 Mb/s excess or a 1 s trickle
// at 1.7 — the telemetry cannot tell. The agent loop already wakes every
// link.rc_drain_ms (5 ms) and can read enc_bytes_total for free, so it
// feeds every wake here; a window closes once >= window_ms has elapsed
// since it opened and its average rate (over the REAL span, not the
// nominal window) competes for the max. take_peak_kbps() returns the
// busiest closed window since the last take and resets — the stats line
// prints it as `enc_pk100=` in decimal kbit/s, the same unit as cmd_kbps,
// so a ratio against the command is a straight division.
//
// A counter that goes backwards (encoder re-seed) re-opens the window
// without closing it, so a restart never prints a ~2^64 rate.
class PeakRate {
 public:
  explicit PeakRate(uint64_t window_ms = 100)
      : window_ms_(window_ms > 0 ? window_ms : 1) {}

  void sample(uint64_t now_ms, uint64_t bytes) {
    if (!open_ || bytes < bytes0_) {
      start_ms_ = now_ms;
      bytes0_ = bytes;
      open_ = true;
      return;
    }
    const uint64_t span = now_ms - start_ms_;
    if (span < window_ms_) return;
    const uint64_t kbps = (bytes - bytes0_) * 8ull / span;  // bits/ms == kbit/s
    if (kbps > peak_kbps_) peak_kbps_ = kbps;
    start_ms_ = now_ms;
    bytes0_ = bytes;
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
  uint64_t start_ms_ = 0;
  uint64_t bytes0_ = 0;
  uint64_t peak_kbps_ = 0;
  bool open_ = false;
};

}  // namespace mabur
