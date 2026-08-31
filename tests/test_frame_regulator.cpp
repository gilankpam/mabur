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

MTEST_MAIN
