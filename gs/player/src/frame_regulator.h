#ifndef MABUR_PLAYER_FRAME_REGULATOR_H_
#define MABUR_PLAYER_FRAME_REGULATOR_H_

#include <cstdint>

#include "video_backend.h"

namespace maburplay {

// Phase-aware display release (display.regulate_ms). Decoded frames are
// held until floor(pts) + D on the GS monotonic clock, so presentation
// follows the encoder's capture cadence instead of arrival jitter — the
// dejitter spike measured today's mailbox presenter at ~750–940 vsync
// repeat+skip pairs/min and a D of 12–16 ms removing ~70% of them for
// under 5 ms of added mean latency (docs/dejitter-findings-2026-08-30.md).
//
// Clock map: pts is the drone's 32-bit µs counter; it is unwrapped by
// deltas and anchored by the FASTEST observed frame (min of mono − pts),
// so D means "delay past the best-case delivery path" — exactly how the
// spike measured the completion-delay distribution. The floor snaps down
// instantly when a faster frame proves it and leaks upward at kLeakPpm to
// follow relative clock drift (measured ~20 ppm; the leak is 3× that).
//
// Ownership: offer() either passes the frame through (present it NOW —
// regulator disabled, frame already past its release, or a pts
// discontinuity re-seed) or holds exactly one frame, mailbox-style. A
// held frame displaced by a newer one, or orphaned by a discontinuity,
// comes back via *replaced and MUST be returned to the backend by the
// caller. release_due() surfaces the held frame once its time arrives.
//
// ⚠ Single-slot consequence (bench A/B 2026-08-30): once D approaches
// the spacing to the next burst-decoded frame, displacement rate climbs
// and the dropped frames judder more than the smoothing helps — 12 ms
// measured 2.94 ms present-jitter with 5 replacements, 16 ms measured
// 5.32 ms with 804. Do not raise D past ~14 without first giving this a
// 2-deep release queue.
class FrameRegulator {
 public:
  explicit FrameRegulator(int regulate_ms)
      : d_us_(static_cast<int64_t>(regulate_ms) * 1000) {}

  bool enabled() const { return d_us_ > 0; }

  // Returns true when f should be presented immediately. Returns false
  // when the regulator holds it. Either way, if *did_replace is set the
  // frame in *replaced was displaced and must be released by the caller.
  bool offer(const DmaFrame& f, uint64_t mono_us, DmaFrame* replaced,
             bool* did_replace);

  // True (and fills *out) once the held frame's release time has arrived.
  bool release_due(uint64_t mono_us, DmaFrame* out);

  bool holding() const { return holding_; }

  uint64_t held_count() const { return held_count_; }
  uint64_t late_count() const { return late_count_; }
  uint64_t replaced_count() const { return replaced_count_; }
  uint64_t discont_count() const { return discont_count_; }
  double hold_ema_ms() const { return hold_ema_ms_; }

 private:
  // A pts step beyond this is an encoder restart, not cadence (matches the
  // drone-side vanish detector's resync threshold family).
  static constexpr int64_t kResyncUs = 2'000'000;
  static constexpr int64_t kLeakPpm = 60;  // floor creep, µs per second

  void displace_into(DmaFrame* replaced, bool* did_replace);

  const int64_t d_us_;

  bool have_pts_ = false;
  uint32_t last_pts32_ = 0;
  uint64_t pts64_ = 0;
  bool base_valid_ = false;
  int64_t base_us_ = 0;  // min(mono − pts64), leaks upward

  bool holding_ = false;
  DmaFrame held_{};
  uint64_t release_us_ = 0;

  uint64_t held_count_ = 0;
  uint64_t late_count_ = 0;
  uint64_t replaced_count_ = 0;
  uint64_t discont_count_ = 0;
  double hold_ema_ms_ = 0.0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_FRAME_REGULATOR_H_
