#pragma once
#include <cstdint>

namespace maburgs {

// Detects a drone (maburd) restart from T_TELEM's tlm_seq, which restarts
// from 0 at every maburd start. DiscAck carries no boot id and a fade or a
// re-rendezvous keeps the drone's counter climbing, so a large backwards
// step is the one signal that means "new flight". main.cpp rotates the
// debug-log session on it.
//
// - never fires on the first sample (nothing to compare against);
// - tolerates reorder/late frames up to kTolerance;
// - a 16-bit wrap (65535 -> 0) is a forward step, not a restart;
// - after a fire, holds off for kHoldoffMs: the old run's stragglers must
//   not split one restart into several flights.
class DroneRestartDetector {
 public:
  static constexpr int kTolerance = 100;
  static constexpr double kHoldoffMs = 10000.0;

  bool on_telem(uint16_t tlm_seq, double now_ms) {
    if (!have_) {
      have_ = true;
      last_ = tlm_seq;
      return false;
    }
    const int prev = last_;
    last_ = tlm_seq;
    const int fwd = (static_cast<int>(tlm_seq) - prev + 65536) % 65536;
    // A forward step of 65536 - k is a backwards step of k.
    const int back = fwd == 0 ? 0 : 65536 - fwd;
    if (fwd <= 32768) return false;      // climbing (wrap included)
    if (back <= kTolerance) return false;  // reorder / late frame
    if (fired_ && now_ms - fired_ms_ < kHoldoffMs) return false;
    fired_ = true;
    fired_ms_ = now_ms;
    return true;
  }

 private:
  bool have_ = false;
  int last_ = 0;
  bool fired_ = false;
  double fired_ms_ = 0.0;
};

}  // namespace maburgs
