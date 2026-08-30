#include "frame_regulator.h"

namespace maburplay {

void FrameRegulator::displace_into(DmaFrame* replaced, bool* did_replace) {
  if (!holding_) return;
  *replaced = held_;
  *did_replace = true;
  holding_ = false;
  ++replaced_count_;
}

bool FrameRegulator::offer(const DmaFrame& f, uint64_t mono_us,
                           DmaFrame* replaced, bool* did_replace) {
  *did_replace = false;
  if (d_us_ <= 0) return true;

  if (!have_pts_) {
    have_pts_ = true;
    pts64_ = f.pts_us;
    last_pts32_ = f.pts_us;
  } else {
    const int64_t d = static_cast<int32_t>(f.pts_us - last_pts32_);
    last_pts32_ = f.pts_us;
    if (d > kResyncUs || d < -kResyncUs) {
      // Encoder clock discontinuity (drone restart): the old timebase is
      // dead. Free any held old-timebase frame, show this one now, and let
      // the NEXT frame seed a fresh floor.
      pts64_ = f.pts_us;
      base_valid_ = false;
      displace_into(replaced, did_replace);
      ++discont_count_;
      return true;
    }
    pts64_ += static_cast<uint64_t>(d);
    // Upward creep so the floor follows drift when arrivals never dip
    // below it; a faster arrival snaps it back down below.
    if (base_valid_ && d > 0) base_us_ += d * kLeakPpm / 1'000'000;
  }

  const int64_t off = static_cast<int64_t>(mono_us) - static_cast<int64_t>(pts64_);
  if (!base_valid_ || off < base_us_) {
    base_us_ = off;
    base_valid_ = true;
  }
  const uint64_t release =
      static_cast<uint64_t>(base_us_ + static_cast<int64_t>(pts64_) + d_us_);

  // Mailbox: a newer frame always displaces an older held one — burst
  // decodes (base+enh back-to-back) land here between loop iterations.
  displace_into(replaced, did_replace);

  if (release <= mono_us) {
    ++late_count_;
    return true;  // already past its slot — never stack delay on the tail
  }

  held_ = f;
  release_us_ = release;
  holding_ = true;
  ++held_count_;
  const double hold_ms = static_cast<double>(release - mono_us) / 1000.0;
  hold_ema_ms_ += (hold_ms - hold_ema_ms_) / 16.0;
  return false;
}

bool FrameRegulator::release_due(uint64_t mono_us, DmaFrame* out) {
  if (!holding_ || mono_us < release_us_) return false;
  *out = held_;
  holding_ = false;
  return true;
}

}  // namespace maburplay
