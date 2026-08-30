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

  const auto obs = anchor_.observe(f.pts_us, mono_us);
  if (obs.discont) {
    // Encoder clock discontinuity (drone restart): the old timebase is
    // dead. Free any held old-timebase frame, show this one now, and let
    // the NEXT frame seed a fresh floor.
    displace_into(replaced, did_replace);
    ++discont_count_;
    return true;
  }

  const uint64_t release = anchor_.map_us(obs.pts64) + static_cast<uint64_t>(d_us_);

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
