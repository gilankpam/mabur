// Stall detection for the agent thread. The hot thread's beat moves every
// ring-read iteration regardless of RF activity, so a static beat always
// means the pipeline thread is wedged. The RX beat only moves when a frame
// is received off the air, so its silence is the expected steady state
// whenever the agent is not LINKED (waiting for a beacon, or failsafe because
// the ground station went quiet); only a LINKED agent with a silent RX beat
// is a stuck receive path. A calibration session silences the GS by design,
// so while one is active the RX clock is pinned to now: the instant it ends
// the check gets a full fresh stale window instead of reading the session's
// whole length as staleness. Nothing is judged inside the startup grace.
// Pure logic: the caller supplies the clock, the beats and the flags, and
// aborts on a verdict.
#pragma once
#include <cstdint>

namespace mabur {

class LoopWatchdog {
 public:
  enum class Stall { None, Hot, Rx };

  explicit LoopWatchdog(uint64_t start_ms, uint64_t grace_ms = 10000, uint64_t stale_ms = 3000)
      : start_(start_ms), grace_(grace_ms), stale_(stale_ms),
        hot_change_(start_ms), rx_change_(start_ms) {}

  Stall check(uint64_t now_ms, uint64_t hot_beat, uint64_t rx_beat, bool linked, bool cal_active) {
    if (hot_beat != last_hot_) { last_hot_ = hot_beat; hot_change_ = now_ms; }
    if (rx_beat != last_rx_) { last_rx_ = rx_beat; rx_change_ = now_ms; }
    if (cal_active) rx_change_ = now_ms;
    if (now_ms - start_ <= grace_) return Stall::None;
    if (now_ms - hot_change_ > stale_) return Stall::Hot;
    if (linked && now_ms - rx_change_ > stale_) return Stall::Rx;
    return Stall::None;
  }

  uint64_t stale_ms() const { return stale_; }

 private:
  uint64_t start_, grace_, stale_;
  uint64_t last_hot_ = 0, last_rx_ = 0;
  uint64_t hot_change_, rx_change_;
};

}  // namespace mabur
