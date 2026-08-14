#pragma once
#include <cstdint>

namespace mabur {

// Wall-clock deadline gate for the agent loop's per-tick housekeeping
// (spec 2026-08-14 fade-demote §3b). The loop now wakes every
// link.rc_drain_ms to drain queued RCFs, but the tick-cadenced work
// (radio health polls, RcAgent::tick, watchdog, 1 Hz stats/telem checks)
// must keep its historical link.tick_ms period. Deadline form matches the
// legacy loop's shape (body, then sleep): next = now + period, so a wake
// interval >= period degenerates to firing every wake — the legacy loop.
//
// A non-positive period is clamped to 1 ms. config.cpp already bounds
// link.tick_ms to [1,1000], but the unclamped cast was a silent total
// failure — (uint64_t)(-1) is a ~1.8e19 ms period, so the gate fires once at
// construction and never again, taking the failsafe, the watchdog and the
// telemetry with it and logging nothing (review finding 2026-08-14).
class TickGate {
 public:
  TickGate(uint64_t now_ms, int period_ms)
      : next_ms_(now_ms),
        period_ms_(period_ms > 0 ? static_cast<uint64_t>(period_ms) : 1) {}
  bool due(uint64_t now_ms) {
    if (now_ms < next_ms_) return false;
    next_ms_ = now_ms + period_ms_;
    return true;
  }

 private:
  uint64_t next_ms_;
  uint64_t period_ms_;
};

}  // namespace mabur
