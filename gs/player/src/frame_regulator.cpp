#include "frame_regulator.h"

namespace maburplay {

void FrameRegulator::displace(int idx, Displaced* out) {
  if (idx >= count_) return;
  if (out && out->n < 2) out->f[out->n++] = held_[idx].f;
  for (int i = idx; i + 1 < count_; ++i) held_[i] = held_[i + 1];
  --count_;
  ++replaced_count_;
}

bool FrameRegulator::offer(const DmaFrame& f, uint64_t mono_us,
                           Displaced* out) {
  out->n = 0;
  if (d_us_ <= 0) return true;

  const auto obs = anchor_.observe(f.pts_us, mono_us);
  if (obs.discont) {
    // Encoder clock discontinuity (drone restart): the old timebase is
    // dead. Free any held old-timebase frames, show this one now, and
    // let the NEXT frame seed a fresh floor.
    while (count_ > 0) displace(0, out);
    ++discont_count_;
    return true;
  }

  // Task 2: fallback rule only. Task 3 adds the servo branch here.
  const uint64_t release =
      anchor_.map_us(obs.pts64) + static_cast<uint64_t>(d_us_);
  const uint64_t target_v = 0;

  // Freshest-wins displacement: same target (fallback: 0 == 0, i.e. the
  // classic mailbox), else oldest-out when full.
  for (int i = 0; i < count_;) {
    if (held_[i].target_v == target_v) displace(i, out);
    else ++i;
  }
  if (count_ == 2) displace(0, out);

  if (release <= mono_us) {
    ++late_count_;
    return true;  // never stack delay on the tail
  }

  // Insert sorted by release_us (earliest at [0]).
  int pos = count_;
  while (pos > 0 && held_[pos - 1].release_us > release) {
    held_[pos] = held_[pos - 1];
    --pos;
  }
  held_[pos] = Held{f, release, target_v};
  ++count_;
  ++held_count_;
  const double hold_ms = static_cast<double>(release - mono_us) / 1000.0;
  hold_ema_ms_ += (hold_ms - hold_ema_ms_) / 16.0;
  return false;
}

bool FrameRegulator::release_due(uint64_t mono_us, DmaFrame* out) {
  if (count_ == 0 || mono_us < held_[0].release_us) return false;
  *out = held_[0].f;
  for (int i = 0; i + 1 < count_; ++i) held_[i] = held_[i + 1];
  --count_;
  return true;
}

}  // namespace maburplay
