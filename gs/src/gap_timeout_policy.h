#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace maburgs {

// Rate-aware FrameStream gap timeout (docs/dejitter-findings-2026-08-30.md
// §follow-up). A lost symbol can only be repaired while the TX sliding
// window still references it — a lifetime of w/send_rate SECONDS, which at
// low rungs exceeds the fixed 50 ms gap timeout (w=32 spans ~85 ms at
// 1 Mb/s/stream), so the GS was truncating frames whose repairs were still
// inbound. The policy stretches the per-stream timeout to the window's
// time-span:
//
//   timeout(sid) = clamp( w / seq_rate(sid) + margin,  floor,  cap )
//
// seq_rate is the SEND rate estimated from the decoder's newest-seq
// advance (robust to loss: any arriving symbol advances it), EWMA with a
// 2 s half-life. During a fade the rate decays and the timeout rides the
// cap — deliberate: while nothing arrives, waiting costs nothing and the
// post-fade repair burst can still heal the gap. cap == 0 disables the
// stretch entirely (fixed floor, the pre-policy behavior).
//
// w is self-calibrating: callers pass the decoder's observed repair
// window_len high-water mark (the GS config does not know the drone's
// fec.window, and this stays correct if per-rung TX windows ever exist).
// Until a repair has been seen (w == 0) a conservative default applies.
class GapTimeoutPolicy {
 public:
  GapTimeoutPolicy(int floor_ms, int cap_ms)
      : floor_ms_(floor_ms), cap_ms_(cap_ms) {}

  // Feed ~1 Hz per sid with the decoder's newest virtual seq. A backward
  // jump (decoder reset / session change) re-anchors without polluting the
  // rate; a forward reset-span jump spikes the rate high for a few
  // half-lives, which only ever shortens the timeout toward the floor.
  void update(int sid, uint64_t newest_seq, int repair_window,
              uint64_t now_ms) {
    if (sid < 0 || sid > 1) return;
    S& s = s_[sid];
    if (repair_window > 0) s.window = repair_window;
    if (!s.seeded || newest_seq < s.last_seq) {
      s.seeded = true;
      s.last_seq = newest_seq;
      s.last_ms = now_ms;
      return;
    }
    if (now_ms <= s.last_ms) return;
    const double dt_s = static_cast<double>(now_ms - s.last_ms) / 1000.0;
    const double inst = static_cast<double>(newest_seq - s.last_seq) / dt_s;
    const double alpha = 1.0 - std::pow(0.5, dt_s / kHalfLifeS);
    s.rate += alpha * (inst - s.rate);
    s.have_rate = true;
    s.last_seq = newest_seq;
    s.last_ms = now_ms;
  }

  int timeout_ms(int sid) const {
    if (cap_ms_ <= 0 || sid < 0 || sid > 1) return floor_ms_;
    const S& s = s_[sid];
    if (!s.have_rate) return floor_ms_;
    if (s.rate < kMinRate) return cap_ms_;  // silence: span is unbounded
    const double span_ms =
        1000.0 * static_cast<double>(s.window) / s.rate + kMarginMs;
    return std::clamp(static_cast<int>(span_ms), floor_ms_, cap_ms_);
  }

 private:
  static constexpr double kHalfLifeS = 2.0;
  static constexpr double kMinRate = 0.5;  // syms/s below which span ~= inf
  static constexpr double kMarginMs = 15.0;  // repair serialization + jitter
  static constexpr int kDefaultWindow = 32;  // prod fec.window, pre-calib

  struct S {
    bool seeded = false;
    bool have_rate = false;
    uint64_t last_seq = 0;
    uint64_t last_ms = 0;
    double rate = 0.0;      // syms/s EWMA
    int window = kDefaultWindow;  // observed TX window (repair hwm)
  };

  const int floor_ms_;
  const int cap_ms_;
  S s_[2];
};

}  // namespace maburgs
