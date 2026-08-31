#include "frame_regulator.h"

#include <utility>

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

  uint64_t release = 0;
  uint64_t target_v = 0;
  servo_now_ = vsync_lock_ && est_.valid(mono_us);
  if (servo_now_) {
    const auto r = est_.next_release(mono_us, lead_us_);
    const uint64_t clamp =
        static_cast<uint64_t>(2.0 * est_.period_us()) + lead_us_;
    if (r.release_us > mono_us + clamp) {
      // Implausible hold (corrupted phase): don't trust the grid for
      // this frame; the estimator self-heals on the next real flips.
      servo_now_ = false;
    } else {
      release = r.release_us;
      target_v = r.vblank_us;
      // Sequential-slot assignment (hw 2026-08-31): base+enh of a
      // capture pair complete together (FEC generation close), so their
      // decodes land within ~1 ms while their pts sit a full frame
      // apart. Arrival-time targeting alone aims both at the same vblank
      // and freshest-wins then drops the earlier of nearly every pair
      // (bench: ~14 skips/s, 1-in-4 frames). pts order is decode order,
      // so a frame whose natural slot is occupied by a held predecessor
      // takes the NEXT slot instead. Deliberately relative to the
      // frame's OWN natural slot, never chained off the queue's furthest
      // target -- natural advances with the clock, so targets are
      // bounded at natural+period and holds at ~2 periods by
      // construction (chaining would creep unboundedly under a
      // faster-than-panel source). If the next slot is occupied too
      // (deep burst), the newest frame claims it -- freshest wins.
      const uint64_t per = static_cast<uint64_t>(est_.period_us());
      const uint64_t hp = per / 2;
      const auto occupant = [&](uint64_t slot) {
        for (int i = 0; i < count_; ++i) {
          const uint64_t hv = held_[i].target_v;
          if (hv != 0 && (hv > slot ? hv - slot : slot - hv) < hp) return i;
        }
        return -1;
      };
      if (occupant(target_v) >= 0) {
        const uint64_t seq = target_v + per;
        const int later = occupant(seq);
        if (later >= 0) {
          ++vsync_skips_;
          displace(later, out);
        }
        target_v = seq;
        release = seq - lead_us_;
      }
    }
  }
  if (!servo_now_) {
    release = anchor_.map_us(obs.pts64) + static_cast<uint64_t>(d_us_);
    if (vsync_lock_) ++fallback_frames_;
  }

  // Fallback mailbox semantics: an untargeted (fallback) frame always
  // displaces an untargeted held one -- the classic single-slot rule.
  // Servo frames never match anything here: the sequential-slot block
  // above either found the natural slot free or already moved this frame
  // past (and displaced) the claimed one, so with count_ <= 2 no held
  // entry can remain within half a period of the final target. The
  // rounding tolerance that used to live in a same_slot matcher is now
  // occupant()'s < hp window. vsync_skips_ increments only on the
  // deep-burst claim above.
  if (target_v == 0) {
    for (int i = 0; i < count_;) {
      if (held_[i].target_v == 0) displace(i, out);
      else ++i;
    }
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

void FrameRegulator::heal_slip() {
  bool any = false;
  const uint64_t per = static_cast<uint64_t>(est_.period_us());
  for (int i = 0; i < count_; ++i) {
    if (held_[i].target_v == 0) continue;  // fallback frames keep their rule
    held_[i].target_v += per;
    held_[i].release_us += per;
    any = true;
  }
  // Only servo entries moved, so a mixed servo/fallback queue can come
  // out inverted -- restore the sorted-by-release invariant that
  // release_due() and next_release_us() depend on (found in review: an
  // inverted head released the fallback frame ~7 ms late, out of pts
  // order, and made next_release_us() misreport to the pump/idle logic).
  if (count_ == 2 && held_[0].release_us > held_[1].release_us)
    std::swap(held_[0], held_[1]);
  if (any) ++heals_;
}

bool FrameRegulator::release_due(uint64_t mono_us, DmaFrame* out) {
  if (count_ == 0 || mono_us < held_[0].release_us) return false;
  *out = held_[0].f;
  for (int i = 0; i + 1 < count_; ++i) held_[i] = held_[i + 1];
  --count_;
  return true;
}

}  // namespace maburplay
