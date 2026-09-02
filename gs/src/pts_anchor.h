#ifndef MABURGS_PTS_ANCHOR_H_
#define MABURGS_PTS_ANCHOR_H_

#include <cstdint>

namespace maburgs {

// Min-anchor pts→mono clock map (extracted 2026-08-30 from maburplay's
// FrameRegulator so daemon and player share one implementation; the
// fork-lineage stack independently converged on the same rolling-min
// bridge as its pts-bias fix). Anchors on the FASTEST observed frame:
// base = min(mono − pts64), snap-down instant, upward leak kLeakPpm.
//
// pts is the drone's 32-bit µs counter; it is unwrapped by signed deltas
// against the last observed pts32 (kResyncUs bounds a legit step — a
// bigger jump is an encoder clock discontinuity, e.g. a drone restart,
// not cadence). The floor snaps down instantly whenever a faster arrival
// proves a smaller (mono − pts64), and leaks upward at kLeakPpm to follow
// relative clock drift when arrivals never dip below it again.
class PtsAnchor {
 public:
  struct Obs {
    uint64_t pts64;
    bool discont;
  };

  // Feed every frame's pts32 + mono arrival. discont=true means this
  // sample re-based (|Δpts| > kResyncUs): state reset, this sample seeds
  // the new pts64 space, base becomes invalid until the NEXT sample.
  Obs observe(uint32_t pts32, uint64_t mono_us) {
    if (!have_pts_) {
      have_pts_ = true;
      pts64_ = pts32;
      last_pts32_ = pts32;
    } else {
      const int64_t d = static_cast<int32_t>(pts32 - last_pts32_);
      last_pts32_ = pts32;
      if (d > kResyncUs || d < -kResyncUs) {
        // Encoder clock discontinuity: the old timebase is dead. This
        // sample seeds a fresh pts64 space; base stays invalid until the
        // NEXT sample re-anchors it.
        pts64_ = pts32;
        base_valid_ = false;
        warm_ = 0;
        return Obs{pts64_, true};
      }
      pts64_ += static_cast<uint64_t>(d);
      // Upward creep so the floor follows drift when arrivals never dip
      // below it; a faster arrival snaps it back down below. Applied
      // BEFORE the min-update, same order as the original inline code.
      if (base_valid_ && d > 0) base_us_ += d * kLeakPpm / 1'000'000;
    }

    const int64_t off =
        static_cast<int64_t>(mono_us) - static_cast<int64_t>(pts64_);
    if (!base_valid_ || off < base_us_) {
      base_us_ = off;
      base_valid_ = true;
    }
    ++warm_;
    return Obs{pts64_, false};
  }

  bool base_valid() const { return base_valid_; }
  int64_t base_us() const { return base_us_; }

  // usable(): ≥ kWarmFrames samples since the last reset — the ONE
  // validity rule every consumer (OSD, sideport, lat lines) shares.
  bool usable() const { return base_valid_ && warm_ >= kWarmFrames; }

  uint64_t map_us(uint64_t pts64) const {
    return static_cast<uint64_t>(base_us_ + static_cast<int64_t>(pts64));
  }

  void reset() {
    have_pts_ = false;
    last_pts32_ = 0;
    pts64_ = 0;
    base_valid_ = false;
    base_us_ = 0;
    warm_ = 0;
  }

  // A pts step beyond this is an encoder restart, not cadence (matches the
  // drone-side vanish detector's resync threshold family).
  static constexpr int64_t kResyncUs = 2'000'000;
  static constexpr int64_t kLeakPpm = 60;  // floor creep, µs per second
  static constexpr uint32_t kWarmFrames = 32;
  // Plausibility cap on the enc+dq correction subtracted from an arrival
  // before it feeds observe() (the "enc-excess" air fix): the TxQueue
  // drop-oldest bound is ~150 ms, so a claimed correction beyond this is
  // wire garbage — the caller must SKIP the sample (segments withheld,
  // like an unknown t_first) rather than let it drag the snap-down floor,
  // which the 60 ppm leak would take effectively forever to recover.
  static constexpr int64_t kMaxAnchorAdjustUs = 300'000;

 private:
  bool have_pts_ = false;
  uint32_t last_pts32_ = 0;
  uint64_t pts64_ = 0;
  bool base_valid_ = false;
  int64_t base_us_ = 0;  // min(mono − pts64), leaks upward
  uint32_t warm_ = 0;
};

// link-rtt (2026-09-02): absolute network floor from an anchor base plus
// the telem-derived (pts − GS-mono) offset. base_us = min(mono − pts64) =
// min_transit − offset, so base + offset = min_transit — except the
// anchor's pts64 space seeds from the TRUNCATED 32-bit slot pts while the
// offset is measured in the full 64-bit MI domain, leaving the raw sum off
// by k·2^32 µs (~71.6 min per drone-uptime wrap at seed). The true floor
// is ms-class, so folding the sum back into ±2^31 µs recovers it exactly
// for any k. Shared by maburgs (sideport floor_ms) and maburplay (absolute
// LAT) so the two can never disagree on the wrap rule.
inline int64_t floor_us_from(int64_t anchor_base_us, int64_t pts_off_us) {
  const uint64_t sum = static_cast<uint64_t>(anchor_base_us) +
                       static_cast<uint64_t>(pts_off_us);
  return static_cast<int32_t>(static_cast<uint32_t>(sum));
}

}  // namespace maburgs

#endif  // MABURGS_PTS_ANCHOR_H_
