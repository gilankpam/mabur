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
class TickGate {
 public:
  TickGate(uint64_t now_ms, int period_ms)
      : next_ms_(now_ms), period_ms_(static_cast<uint64_t>(period_ms)) {}
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
