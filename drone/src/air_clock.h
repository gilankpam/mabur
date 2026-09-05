#pragma once
#include <cstddef>
#include <cstdint>

namespace mabur {

// Per-frame virtual air-serialization clock (spec 2026-09-06 air-clock §2).
//
// free_at_us_ is the moment the air becomes free given every body pushed to
// the TxQueue so far, priced at the applied op's PHY rate per layer:
//
//     air_us(body) = bytes * 8 / (rate_mbps * efficiency) + body_us
//     free_at      = max(free_at, now) + air_us(body)
//     backlog      = max(0, free_at - now)
//
// The max() clamp makes the model leaky: once the air catches up the
// backlog is 0 and any accumulated pricing error is gone (same floor the
// GS-side airdrain.py replay uses). Booked from ACTUAL emitted body bytes
// (SBI/frag framing and repair quantization included), never from
// len*(1+ov) -- docs/airtime-model.md §1 measured a ~2x gain error in the
// naive formula. `efficiency` and `body_us` are calibration knobs
// (config air_clock.*): nominal PHY rate ignores preamble/SIFS, the
// half-duplex RCF/telemetry slots, aggregation and USB pacing.
//
// No clock of its own: every call takes the caller's steady-clock µs, like
// RcAgent's now_ms contract, so tests drive it synthetically. Hot-thread
// only; not thread-safe.
class AirClock {
 public:
  static constexpr int kProbeSid = 2;   // sid 0 = base, 1 = enh, 2 = probe body

  void set_rates(double base_mbps, double enh_mbps, double probe_mbps,
                 double efficiency, uint32_t body_us) {
    us_per_byte_[0] = per_byte(base_mbps, efficiency);
    us_per_byte_[1] = per_byte(enh_mbps, efficiency);
    us_per_byte_[2] = per_byte(probe_mbps, efficiency);
    body_us_ = body_us;
  }

  // Books one pushed body. A sid with no rate (probe stream off, or an
  // out-of-range sid) books nothing: better an unbooked body than a body
  // priced at a rate the op never commanded.
  void book(uint64_t now_us, size_t bytes, int sid) {
    if (sid < 0 || sid > kProbeSid) return;
    const double upb = us_per_byte_[sid];
    if (upb <= 0.0) return;
    const uint64_t start = free_at_us_ > now_us ? free_at_us_ : now_us;
    const double cost = static_cast<double>(bytes) * upb +
                        static_cast<double>(body_us_);
    free_at_us_ = start + static_cast<uint64_t>(cost + 0.5);
  }

  uint32_t backlog_us(uint64_t now_us) const {
    if (free_at_us_ <= now_us) return 0;
    const uint64_t d = free_at_us_ - now_us;
    return d > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<uint32_t>(d);
  }

 private:
  static double per_byte(double mbps, double eff) {
    return (mbps > 0.0 && eff > 0.0) ? 8.0 / (mbps * eff) : 0.0;
  }
  double us_per_byte_[3] = {0.0, 0.0, 0.0};
  uint32_t body_us_ = 0;
  uint64_t free_at_us_ = 0;
};

}  // namespace mabur
