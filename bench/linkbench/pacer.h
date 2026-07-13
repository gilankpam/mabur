#pragma once
// Token-bucket pacer for linkbench-tx: paces APPLICATION payload bytes to
// the target bitrate (the iperf-comparable number; FEC/framing expansion
// rides on top). Caller drives it with a monotonic microsecond clock.
#include <cstddef>
#include <cstdint>

namespace linkbench {

class TokenBucket {
 public:
  TokenBucket(double bytes_per_sec, double max_burst_bytes)
      : rate_(bytes_per_sec), burst_(max_burst_bytes) {}

  // Accrues tokens for the elapsed time since the previous advance(). The
  // first call only establishes the epoch. Non-monotonic input is ignored
  // (steady_clock shouldn't produce it, but a bench must not misbehave if
  // the caller feeds a stale timestamp).
  void advance(uint64_t now_us) {
    if (!has_epoch_) { last_us_ = now_us; has_epoch_ = true; return; }
    if (now_us <= last_us_) return;
    tokens_ += rate_ * static_cast<double>(now_us - last_us_) / 1e6;
    if (tokens_ > burst_) tokens_ = burst_;
    last_us_ = now_us;
  }

  bool spend(size_t bytes) {
    if (tokens_ < static_cast<double>(bytes)) return false;
    tokens_ -= static_cast<double>(bytes);
    return true;
  }

  double tokens() const { return tokens_; }

 private:
  double rate_;
  double burst_;
  double tokens_ = 0.0;
  uint64_t last_us_ = 0;
  bool has_epoch_ = false;
};

}  // namespace linkbench
