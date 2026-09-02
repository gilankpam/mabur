#ifndef MABUR_PLAYER_FRAME_REGULATOR_H_
#define MABUR_PLAYER_FRAME_REGULATOR_H_

#include <cstdint>

#include "pts_anchor.h"
#include "vblank_estimator.h"
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
// discontinuity re-seed) or holds it in a kMaxHeld-deep queue. A held
// frame displaced by a newer one, or orphaned by a discontinuity, comes
// back via *out (up to kMaxHeld at once) and MUST be returned to the
// backend by the caller. release_due() surfaces held frames once their
// time arrives — call it in a loop, since more than one can be due in
// the same tick.
//
// Queue depth (bench A/B 2026-08-30, generalized 2026-09-02): the old
// single-slot mailbox meant that once D approached the spacing to the
// next burst-decoded frame, displacement rate climbed and the dropped
// frames judder more than the smoothing helps — 12 ms measured 2.94 ms
// present-jitter with 5 replacements, 16 ms measured 5.32 ms with 804.
// Multiple slots let the queue hold one frame per vsync target instead
// of collapsing every arrival onto a single held frame; in fallback mode
// (no vsync lock) all entries share target_v == 0, so the same-target
// displacement rule below reproduces the old single-slot mailbox
// exactly. Depth 4 (was 2) covers the post-stall backlog: the decode
// sink offers before the main loop drains overdue releases, so a stall
// spanning a release deadline plus a decode burst legitimately wants 3+
// distinct vsync targets in hold — at 120 Hz (8.3 ms periods) with
// D=12–16 that window doubles relative to the 60 Hz bench. A full queue
// still evicts the oldest (most-overdue) head: latency-first.
class FrameRegulator {
 public:
  FrameRegulator(int regulate_ms, bool vsync_lock = false,
                 int vsync_lead_ms = 3)
      : d_us_(static_cast<int64_t>(regulate_ms) * 1000),
        vsync_lock_(vsync_lock),
        lead_us_(static_cast<uint64_t>(vsync_lead_ms) * 1000) {}

  bool enabled() const { return d_us_ > 0; }

  static constexpr int kMaxHeld = 4;

  struct Displaced {
    DmaFrame f[kMaxHeld];
    int n = 0;
  };

  // Returns true when f should be presented immediately. Returns false
  // when the regulator holds it. Either way, any frame(s) displaced from
  // the queue are returned in *out and must be released by the caller.
  bool offer(const DmaFrame& f, uint64_t mono_us, Displaced* out);

  // True (and fills *out) once the earliest held frame's release time has
  // arrived. Call in a loop to drain — more than one entry can be due in
  // the same tick.
  bool release_due(uint64_t mono_us, DmaFrame* out);

  bool holding() const { return count_ > 0; }

  // Forwarded from the presenter's FlipSink (main thread, no locking).
  void on_flip(uint64_t flip_us, bool exact) { est_.on_flip(flip_us, exact); }

  // Reseed the vblank estimator with the chosen DRM mode's exact period
  // (mode_period_us). The regulator is constructed before the presenter
  // (its flip-sink closures capture it by reference), so the panel rate
  // arrives here post-init rather than through the constructor. Call
  // before the first flip: reseeding discards any warm-up so far.
  void set_panel_period(double period_us) {
    est_ = VblankEstimator(period_us);
  }

  uint64_t held_count() const { return held_count_; }
  uint64_t late_count() const { return late_count_; }
  uint64_t replaced_count() const { return replaced_count_; }
  uint64_t discont_count() const { return discont_count_; }
  double hold_ema_ms() const { return hold_ema_ms_; }
  uint64_t vsync_skips() const { return vsync_skips_; }
  uint64_t fallback_frames() const { return fallback_frames_; }
  bool servo_locked() const { return servo_now_; }

  // Chain-break heal (hw 2026-08-31): a single missed latch puts the
  // display into a stable one-vsync-late mode -- every present then
  // arrives while the previous flip is in flight, parks in the
  // presenter's mailbox, and latches a period late, indefinitely. The
  // main loop detects the chain from the presenter's engagement-counter
  // rate and calls this: every pending servo release slips one slot, so
  // one vblank goes unfilled (a single repeated frame), the flip
  // pipeline drains, and the phase re-aligns. No-op when the queue is
  // empty or holds only fallback (untargeted) frames.
  void heal_slip();
  uint64_t heals() const { return heals_; }

  // Earliest pending release (0 = nothing held). The main loop uses this
  // to schedule its heavy 1 Hz work (OSD compose, stats, statvfs) into
  // the gap between releases: a stall that overlaps a release deadline
  // makes the frame miss its latch and start a one-vsync-late chain
  // (hw 2026-08-31), so heavy work defers when a release is imminent.
  uint64_t next_release_us() const {
    return count_ ? held_[0].release_us : 0;
  }

 private:
  struct Held {
    DmaFrame f{};
    uint64_t release_us = 0;
    uint64_t target_v = 0;  // 0 = untargeted (fallback rule)
  };
  void displace(int idx, Displaced* out);

  const int64_t d_us_;
  const bool vsync_lock_;
  const uint64_t lead_us_;

  maburgs::PtsAnchor anchor_;
  VblankEstimator est_;
  Held held_[kMaxHeld];  // held_[0] releases first (sorted by release_us)
  int count_ = 0;
  bool servo_now_ = false;

  uint64_t held_count_ = 0;
  uint64_t late_count_ = 0;
  uint64_t replaced_count_ = 0;
  uint64_t discont_count_ = 0;
  double hold_ema_ms_ = 0.0;
  uint64_t vsync_skips_ = 0;
  uint64_t heals_ = 0;
  uint64_t fallback_frames_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_FRAME_REGULATOR_H_
