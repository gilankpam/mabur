#ifndef MABUR_GS_RTT_ESTIMATOR_H_
#define MABUR_GS_RTT_ESTIMATOR_H_

#include <cstdint>

namespace maburgs {

// Control-path RTT + pts-offset estimator (link-rtt, 2026-09-02).
//
// Clock-sync-free: both timestamps in every subtraction are GS-local
// CLOCK_MONOTONIC µs; the drone contributes only a duration (rcf_age_ms)
// and a raw pts-domain clock reading that is only ever differenced.
//
//   rtt    = (telem_rx − rcf_send) − rcf_age
//   offset = pts_at_build − telem_rx + rtt/2      (pts − GS-mono)
//
// The seq echo makes the send-time match exact (repeats are 10 ms apart —
// closer than the RTT being measured, so fuzzy matching was rejected).
// The offset is the one from the lowest-RTT sample in a sliding window of
// the last kOffWindow samples (PTP-style min-filter): queueing inflates
// rtt and offset together, so the least-queued sample carries the least
// asymmetry bias, and the window keeps a minutes-old lucky sample from
// pinning the offset against clock drift (~ppm, ms/min class).
//
// Labeled control-path RTT everywhere it surfaces: the telem reply queues
// behind video on the drone's half-duplex TX, so samples read high under
// saturation — honest congestion signal, not PHY RTT.
//
// Single-threaded by design: on_rcf_sent and on_telem are both called from
// the maburgs main loop.
class RttEstimator {
 public:
  // Record one RCF emission (step() sends AND repeat-burst copies — each
  // repeat carries a fresh seq, so every emission is matchable).
  void on_rcf_sent(uint16_t seq, uint64_t t_us);

  // One T_TELEM arrival. echo_valid is Telem flags bit3 — false whenever
  // the drone's seq window was reset (DISC re-establish, failsafe rebase)
  // and rcf_age_ms references no RCF. pts_at_build_us 0 means the MI clock
  // was unreadable: the RTT sample still counts, the offset must not.
  // Returns true iff an RTT sample was taken.
  bool on_telem(uint16_t seq_echo, bool echo_valid, uint16_t age_ms,
                uint64_t pts_at_build_us, uint64_t rx_us);

  bool has_rtt() const { return n_ > 0; }
  double rtt_ms() const { return rtt_ewma_us_ / 1000.0; }      // EWMA
  double rtt_min_ms() const { return rtt_min_us_ / 1000.0; }   // session min
  uint32_t samples() const { return n_; }

  bool has_offset() const { return off_n_ > 0; }
  int64_t pts_off_us() const;  // offset of the min-rtt sample in the window

 private:
  static constexpr int kRingCap = 64;     // ~1-3 s of sends at RCF rates
  static constexpr int kOffWindow = 32;   // ~30 s of telem at 1 Hz
  static constexpr int64_t kNegClampUs = -1000;   // age is ms-quantized
  static constexpr int64_t kMaxPlausibleUs = 3'000'000;  // seq-reuse guard
  static constexpr uint16_t kMaxAgeMs = 1000;  // stale echo = old news
  static constexpr double kEwmaAlpha = 0.2;

  struct Sent {
    uint16_t seq = 0;
    uint64_t t_us = 0;
    bool used = false;
  };
  Sent ring_[kRingCap];
  int ring_next_ = 0;

  double rtt_ewma_us_ = 0.0;
  double rtt_min_us_ = 0.0;
  uint32_t n_ = 0;

  struct OffSample {
    int64_t rtt_us = 0;
    int64_t off_us = 0;
  };
  OffSample off_win_[kOffWindow];
  int off_next_ = 0;
  int off_n_ = 0;
};

}  // namespace maburgs

#endif  // MABUR_GS_RTT_ESTIMATOR_H_
