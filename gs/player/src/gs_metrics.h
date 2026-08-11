#ifndef MABUR_PLAYER_GS_METRICS_H_
#define MABUR_PLAYER_GS_METRICS_H_

#include <cstdint>

#include "gs_overlay.h"  // RecState

namespace maburplay {

// The player-measured half of the GS overlay's inputs, split out of
// main.cpp's loop so it can be tested with no ring, no decoder and no DRM
// device. It lives here rather than inline because both defects this code
// has already had -- a link outage reading as a recording FAULT, and a
// truncation storm doing the same -- are edges no host test could reach
// while the logic sat in the middle of a hardware-only main loop.

// AU-delivery jitter: an EMA of the change in inter-arrival interval.
//
// Measured at delivery and not at flip because the presenter is a mailbox
// on a 60 Hz vsync, so flip deltas are quantized to 16.67 ms multiples and
// a 3 ms arrival wobble is invisible in them.
class AuJitter {
 public:
  // A gap at least this long is a freeze, not jitter. 200 ms is ~12 frames
  // at 60 fps and ~6 at 30 -- far outside anything a healthy link produces,
  // and low enough to exclude the common 200-900 ms bad-link stutter, which
  // folded into the EMA would decay over the next ~16 AUs and go on reading
  // as jitter long after the event that caused it.
  static constexpr uint64_t kStallMs = 200;
  // 1/16 per sample: ~16 AUs to forget an event, a quarter second at 60 fps.
  static constexpr double kAlphaDiv = 16.0;

  void on_au(uint64_t now_ms);

  // Called on the caller's 1 Hz mark. Without it an outage leaves the last
  // EMA sitting on screen -- a frozen but entirely plausible number beside
  // an FPS that has fallen to zero.
  void on_tick(uint64_t now_ms);

  double ms() const { return ema_ms_; }

 private:
  uint64_t last_au_ms_ = 0;         // 0 = nothing has ever arrived
  double last_interval_ms_ = -1.0;  // <0 = no interval yet, or just left a stall
  double ema_ms_ = 0.0;
};

// The recording indicator's state machine.
class RecTracker {
 public:
  // Samples not advancing for this long WHILE THERE IS INPUT is a fault.
  static constexpr uint64_t kStallMs = 3000;

  struct Inputs {
    bool broken = false;     // the recorder could not be brought up at all
    bool open = false;       // a file exists / the recorder thread is running
    bool low_space = false;  // free space below the floor. NOT "statvfs failed":
                             // a failed syscall is not evidence of a full disk.
    uint64_t samples = 0;    // muxed samples (raw) or encoded frames (burned)
    // What would make `samples` advance if the recorder were healthy.
    // COMPLETE AUs on the raw path -- an incomplete AU is skipped whole, so
    // counting deliveries here would make a truncation storm (AUs arriving,
    // none of them recordable) read as a recorder fault. Decoded frames in
    // burned mode, which is what its encoder actually consumes.
    uint64_t feed = 0;
  };

  RecState update(const Inputs& in, uint64_t now_ms);

 private:
  uint64_t samples_ = 0, feed_ = 0;
  uint64_t stall_since_ms_ = 0;
  // The recording clock's origin: the first sample actually written, which
  // is the same instant in both modes. Anchoring it to "the recorder was
  // armed" instead would make burned mode over-report by however long the
  // link took to come up.
  uint64_t start_ms_ = 0;
  bool started_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_METRICS_H_
