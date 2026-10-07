#pragma once
#include <algorithm>
#include <cstdint>

namespace mabur {

// Air cap for NACK re-sends (spec 2026-10-05 fec-nack §4.3), in symbols.
// The handler refills it at nack.air_pct of the base layer's delivered
// capacity and takes one token per re-sent symbol (two for a repeat-flagged,
// doubled send). A take beyond the tokens is REFUSED whole -- never queued,
// never partially honoured -- so a NACK storm cannot eat the video's air.
// The first refill() only seeds the clock; tokens start full (depth). The
// depth is re-set on every refill from the current op (set_depth).
// Single-threaded: owned by the RX thread's T_NACK handler.
class TokenBucket {
 public:
  explicit TokenBucket(double depth) : depth_(depth), tokens_(depth) {}
  // Adds elapsed * rate_per_s, clamped to depth.
  void refill(uint64_t now_us, double rate_per_s) {
    if (have_t_ && now_us > t_us_)
      tokens_ = std::min(depth_, tokens_ + static_cast<double>(now_us - t_us_) * 1e-6 * rate_per_s);
    t_us_ = now_us;
    have_t_ = true;
  }
  // false (nothing taken) if n > tokens.
  bool take(double n) {
    if (n > tokens_) return false;
    tokens_ -= n;
    return true;
  }
  double tokens() const { return tokens_; }
  double depth() const { return depth_; }
  // nack.burst_ms of air at sym_per_s (the current op's delivered sid-0
  // rate), never below one symbol.
  static double depth_for(double sym_per_s, int burst_ms) {
    return std::max(1.0, sym_per_s * static_cast<double>(burst_ms) / 1000.0);
  }
  // Depth tracks the rung (nack.burst_ms of air at the current op): growing
  // it only raises the refill ceiling, shrinking it clamps what is held so
  // a demote to a slow rung cannot burst a fast rung's worth of symbols.
  void set_depth(double depth) {
    depth_ = depth;
    tokens_ = std::min(tokens_, depth_);
  }

 private:
  double depth_, tokens_;
  uint64_t t_us_ = 0;
  bool have_t_ = false;
};

}  // namespace mabur
