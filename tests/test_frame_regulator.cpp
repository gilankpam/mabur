#include <cstdint>

#include "frame_regulator.h"
#include "mtest.h"

using maburplay::DmaFrame;
using maburplay::FrameRegulator;

namespace {

DmaFrame frame(uint32_t pts_us) {
  DmaFrame f;
  f.pts_us = pts_us;
  f.opaque = reinterpret_cast<void*>(static_cast<uintptr_t>(pts_us | 1u));
  return f;
}

constexpr uint64_t kD = 12'000;         // regulate_ms 12 in µs
constexpr uint64_t kT0 = 5'000'000;     // arbitrary mono anchor
constexpr int64_t kFrame = 16'684;      // 59.94 fps pts step

}  // namespace

TEST(disabled_passes_through) {
  FrameRegulator reg(0);
  CHECK(!reg.enabled());
  FrameRegulator::Displaced disp;
  CHECK(reg.offer(frame(0), kT0, &disp));
  CHECK(disp.n == 0);
  CHECK(!reg.holding());
}

TEST(first_frame_seeds_floor_and_holds_exactly_d) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));  // held
  CHECK(disp.n == 0);
  CHECK(reg.holding());
  CHECK(!reg.release_due(kT0 + kD - 1, &out));
  CHECK(reg.release_due(kT0 + kD, &out));
  CHECK(out.opaque == frame(0).opaque);
  CHECK(!reg.holding());
  CHECK(reg.held_count() == 1);
}

TEST(steady_cadence_releases_on_pts_grid_not_arrival) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));
  CHECK(reg.release_due(kT0 + kD, &out));
  // Frame 1 arrives 5 ms jittery-late relative to the floor: it must still
  // release at floor + pts + D, i.e. only 7 ms after its own arrival.
  const uint64_t a1 = kT0 + kFrame + 5'000;
  CHECK(!reg.offer(frame(kFrame), a1, &disp));
  // (small slack: the drift leak may shift the floor by ~1 µs per frame)
  CHECK(!reg.release_due(kT0 + kFrame + kD - 1'000, &out));
  CHECK(reg.release_due(kT0 + kFrame + kD + 16, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
}

TEST(late_frame_presents_immediately) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));
  CHECK(reg.release_due(kT0 + kD, &out));
  // Arrival AFTER its own release point (floor + pts + D).
  const uint64_t a1 = kT0 + kFrame + kD + 1'000;
  CHECK(reg.offer(frame(kFrame), a1, &disp));  // present now
  CHECK(disp.n == 0);
  CHECK(!reg.holding());
  CHECK(reg.late_count() == 1);
}

TEST(faster_frame_snaps_floor_down) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  // Seed with a SLOW first frame (its arrival becomes the provisional floor).
  CHECK(!reg.offer(frame(0), kT0, &disp));
  CHECK(reg.release_due(kT0 + kD, &out));
  // Frame 1 arrives 3 ms faster than the provisional floor predicts: floor
  // snaps down and its release is exactly its own arrival + D.
  const uint64_t a1 = kT0 + kFrame - 3'000;
  CHECK(!reg.offer(frame(kFrame), a1, &disp));
  CHECK(!reg.release_due(a1 + kD - 1, &out));
  CHECK(reg.release_due(a1 + kD, &out));
}

TEST(floor_leaks_upward_to_track_drift) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  // Seed the floor, then feed frames arriving a constant 2 ms above it.
  // The leak (~1 µs per frame at 60 fps) must close that gap: after 3000
  // frames the floor has crept >= 2 ms, so hold time is back to ~D.
  CHECK(!reg.offer(frame(0), kT0, &disp));
  CHECK(reg.release_due(kT0 + kD, &out));
  uint64_t pts = 0, arrival = 0;
  for (int i = 1; i <= 3000; ++i) {
    pts = static_cast<uint64_t>(i) * kFrame;
    arrival = kT0 + pts + 2'000;
    if (!reg.offer(frame(static_cast<uint32_t>(pts)), arrival, &disp))
      CHECK(reg.release_due(arrival + kD, &out));  // drain
  }
  // Final frame: floor >= arrival-offset, so release is within [arr, arr+D].
  const uint64_t a = kT0 + 3001ull * kFrame + 2'000;
  const bool held = !reg.offer(frame(static_cast<uint32_t>(3001ull * kFrame)),
                               a, &disp);
  if (held) {
    CHECK(!reg.release_due(a - 1, &out));      // never before arrival
    CHECK(reg.release_due(a + kD, &out));      // and by arrival + D
  }
  // Leak must never make a frame release before it arrives (that would be
  // a late-classification instead).
  CHECK(reg.late_count() + reg.held_count() >= 3000);
}

TEST(newer_frame_replaces_held_mailbox_style) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));
  // Burst decode: frame 1 offered before frame 0's release.
  CHECK(!reg.offer(frame(kFrame), kT0 + 1'000, &disp));
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(0).opaque);  // caller must free this one
  CHECK(reg.replaced_count() == 1);
  CHECK(reg.release_due(kT0 + kFrame + kD, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
}

TEST(pts_wrap_is_invisible) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint32_t p0 = 0xFFFFF000u;  // wraps on the next step
  CHECK(!reg.offer(frame(p0), kT0, &disp));
  CHECK(reg.release_due(kT0 + kD, &out));
  const uint32_t p1 = p0 + static_cast<uint32_t>(kFrame);  // wrapped
  CHECK(!reg.offer(frame(p1), kT0 + kFrame, &disp));
  CHECK(!reg.release_due(kT0 + kFrame + kD - 1'000, &out));
  CHECK(reg.release_due(kT0 + kFrame + kD + 16, &out));
}

TEST(pts_discontinuity_resets_and_passes_through) {
  FrameRegulator reg(12);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));
  // Drone restart: pts jumps 5 s. The frame passes through immediately and
  // the held old-timebase frame comes back for release.
  CHECK(reg.offer(frame(5'000'000), kT0 + 2'000, &disp));
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(0).opaque);
  CHECK(!reg.holding());
  // Next frame re-seeds a fresh floor and is held for exactly D again.
  const uint64_t a = kT0 + 20'000;
  CHECK(!reg.offer(frame(5'000'000 + static_cast<uint32_t>(kFrame)), a,
                   &disp));
  CHECK(!reg.release_due(a + kD - 1, &out));
  CHECK(reg.release_due(a + kD, &out));
}

TEST(fallback_mode_displaces_like_single_slot_mailbox) {
  FrameRegulator reg(12);  // vsync_lock defaults false
  FrameRegulator::Displaced disp;
  CHECK(!reg.offer(frame(0), kT0, &disp));
  CHECK(disp.n == 0);
  // Second frame arrives while the first is held: displaced (v=0 == v=0).
  CHECK(!reg.offer(frame(kFrame), kT0 + 2'000, &disp));
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(0).opaque);
  CHECK(reg.replaced_count() == 1);
}

namespace {
constexpr uint64_t kVb = 16'667;    // panel period µs
constexpr uint64_t kLead = 3'000;   // vsync_lead_ms 3

// Warm a servo regulator: 8 exact flips on the kT0 grid.
void warm(FrameRegulator& reg) {
  for (int i = 0; i < 8; ++i) reg.on_flip(kT0 + i * kVb, true);
}
}  // namespace

TEST(servo_releases_at_vblank_minus_lead) {
  FrameRegulator reg(12, /*vsync_lock=*/true, /*vsync_lead_ms=*/3);
  warm(reg);  // phase = kT0 + 7*kVb
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  // Decoded 9 ms before the next vblank: plenty of lead margin.
  const uint64_t t_dec = phase + kVb - 9'000;
  CHECK(!reg.offer(frame(0), t_dec, &disp));
  CHECK(reg.servo_locked());
  const uint64_t release = phase + kVb - kLead;
  CHECK(!reg.release_due(release - 1, &out));
  CHECK(reg.release_due(release, &out));
  CHECK(reg.fallback_frames() == 0);
}

TEST(servo_inside_lead_window_targets_next_vblank) {
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  // Only 2 ms of margin left (< lead): must NOT gamble on this vblank.
  const uint64_t t_dec = phase + kVb - 2'000;
  CHECK(!reg.offer(frame(0), t_dec, &disp));
  CHECK(!reg.release_due(phase + kVb, &out));            // not this one
  CHECK(reg.release_due(phase + 2 * kVb - kLead, &out)); // the next
}

TEST(servo_same_target_keeps_freshest) {
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));
  // Second frame decodes 5 ms later, same catchable vblank: contention.
  CHECK(!reg.offer(frame(kFrame), t1 + 5'000, &disp));
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(0).opaque);  // older dropped
  CHECK(reg.vsync_skips() == 1);
  CHECK(reg.release_due(phase + kVb - kLead, &out));
  CHECK(out.opaque == frame(kFrame).opaque);   // freshest shown
}

TEST(servo_distinct_targets_hold_both) {
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), phase + kVb - 9'000, &disp));
  // Second frame decodes after vblank+1's lead cutoff: targets vblank+2.
  CHECK(!reg.offer(frame(kFrame), phase + kVb - 1'000, &disp));
  CHECK(disp.n == 0);
  CHECK(reg.holding());
  CHECK(reg.release_due(phase + kVb - kLead, &out));
  CHECK(out.opaque == frame(0).opaque);
  CHECK(reg.release_due(phase + 2 * kVb - kLead, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
  CHECK(reg.vsync_skips() == 0);
}

TEST(servo_falls_back_when_stale_and_recovers) {
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  // 40 periods after the last flip: stale -> fallback rule (floor + D).
  const uint64_t t_dec = phase + 40 * kVb;
  CHECK(!reg.offer(frame(0), t_dec, &disp));
  CHECK(!reg.servo_locked());
  CHECK(reg.fallback_frames() == 1);
  // Fallback release = anchored floor + 12 ms; this frame seeds the
  // floor at t_dec, so release = t_dec + 12 ms exactly.
  CHECK(!reg.release_due(t_dec + kD - 1, &out));
  CHECK(reg.release_due(t_dec + kD, &out));
  // Flips resume: servo re-engages (phase recency revalidates).
  for (int i = 0; i < 8; ++i) reg.on_flip(t_dec + 20'000 + i * kVb, true);
  CHECK(!reg.offer(frame(2 * kFrame), t_dec + 20'000 + 8 * kVb, &disp));
  CHECK(reg.servo_locked());
}

TEST(servo_safety_clamp_uses_fallback_rule) {
  FrameRegulator reg(12, true, 3);
  warm(reg);
  // Corrupt-phase stand-in: an estimator whose next release lands
  // > 2*period + lead away must NOT be trusted for this frame.
  // Constructed via a phase far in the future: feed one exact flip 10
  // periods ahead (k=10 updates phase), then offer a frame "now".
  const uint64_t phase = kT0 + 7 * kVb;
  reg.on_flip(phase + 10 * kVb, true);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t_dec = phase + 10 * kVb - 60'000;  // 60 ms before phase
  CHECK(!reg.offer(frame(0), t_dec, &disp));
  CHECK(reg.fallback_frames() == 1);  // clamp routed to fallback
  CHECK(reg.release_due(t_dec + kD, &out));
}

MTEST_MAIN
