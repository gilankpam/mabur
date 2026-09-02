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

TEST(servo_burst_pair_takes_sequential_vblanks) {
  // base+enh of one capture pair complete together (FEC generation
  // close), so their decodes land ~1 ms apart with pts a full frame
  // apart. Arrival-time targeting alone would aim both at the same
  // vblank and drop the earlier frame (hw 2026-08-31: ~14 skips/s,
  // 1-in-4 frames). Sequential-slot assignment must instead push the
  // later-pts frame to the next grid slot: both display, in order.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));
  CHECK(!reg.offer(frame(kFrame), t1 + 1'000, &disp));
  CHECK(disp.n == 0);              // NOT displaced
  CHECK(reg.vsync_skips() == 0);
  CHECK(reg.release_due(phase + kVb - kLead, &out));
  CHECK(out.opaque == frame(0).opaque);          // first pair member
  CHECK(!reg.release_due(phase + 2 * kVb - kLead - 100, &out));
  CHECK(reg.release_due(phase + 2 * kVb - kLead + 100, &out));
  CHECK(out.opaque == frame(kFrame).opaque);     // second, next vblank
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

TEST(mixed_fallback_and_servo_targets_all_hold) {
  // Mixed fallback/servo queue: start stale (fallback frame held,
  // target_v==0), re-warm to a fresh grid, then hold TWO servo frames at
  // distinct targets. Under the old 2-deep queue the third frame evicted
  // the fallback head; at kMaxHeld=4 all three coexist and each releases
  // by its own rule (fallback at floor+D, servo at vblank-lead). The
  // full-queue eviction path is pinned by
  // full_queue_evicts_overdue_head_on_fifth_target.
  FrameRegulator reg(12, /*vsync_lock=*/true, /*vsync_lead_ms=*/3);
  warm(reg);
  const uint64_t phase0 = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;

  // 1) Stale offer: estimator hasn't flipped in 40 periods -> fallback
  // rule, held with target_v == 0.
  const uint64_t t_dec1 = phase0 + 40 * kVb;
  CHECK(!reg.offer(frame(0), t_dec1, &disp));
  CHECK(!reg.servo_locked());
  CHECK(reg.fallback_frames() == 1);

  // 2) Re-warm on a fresh grid (one exact flip actually revalidates --
  // finding 3 -- 8 mirrors the other re-warm test's cadence).
  const uint64_t phase1 = t_dec1 + 20'000 + 7 * kVb;
  for (int i = 0; i < 8; ++i) reg.on_flip(t_dec1 + 20'000 + i * kVb, true);

  // 3) Servo frame targeting vblank phase1+kVb: distinct from the held
  // fallback target (0) -> both held, count_ == 2.
  const uint64_t t_dec2 = phase1 + kVb - 9'000;
  CHECK(!reg.offer(frame(kFrame), t_dec2, &disp));
  CHECK(reg.servo_locked());
  CHECK(disp.n == 0);
  CHECK(reg.holding());

  // 4) Second servo frame decoded inside the FIRST target's lead window
  // (only 1 ms of margin, < the 3 ms lead): rolls to the next vblank
  // (phase1+2*kVb), matching neither held target (0 or phase1+kVb). Three
  // distinct targets, all held -- no eviction, no skip.
  const uint64_t t_dec3 = phase1 + kVb - 1'000;
  CHECK(!reg.offer(frame(2 * kFrame), t_dec3, &disp));
  CHECK(disp.n == 0);
  CHECK(reg.replaced_count() == 0);
  CHECK(reg.vsync_skips() == 0);
  CHECK(reg.holding());

  // Releases interleave by each frame's own rule: the fallback frame's
  // floor+D release (long overdue by phase1) heads the queue, then the
  // servo frames at their vblanks.
  DmaFrame o0, o1, o2;
  CHECK(reg.release_due(phase1 + kVb - kLead, &o0));
  CHECK(o0.opaque == frame(0).opaque);
  CHECK(reg.release_due(phase1 + kVb - kLead, &o1));
  CHECK(o1.opaque == frame(kFrame).opaque);
  CHECK(reg.release_due(phase1 + 2 * kVb - kLead, &o2));
  CHECK(o2.opaque == frame(2 * kFrame).opaque);
  CHECK(!reg.holding());
}

TEST(discont_flushes_both_held_slots) {
  // Same mixed fallback/servo construction to reach count_==2, then a
  // pts jump > PtsAnchor::kResyncUs (2 s): the discont path must flush
  // BOTH held frames regardless of which path (fallback vs servo) put
  // them there.
  FrameRegulator reg(12, /*vsync_lock=*/true, /*vsync_lead_ms=*/3);
  warm(reg);
  const uint64_t phase0 = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;

  const uint64_t t_dec1 = phase0 + 40 * kVb;
  CHECK(!reg.offer(frame(0), t_dec1, &disp));  // fallback frame held

  const uint64_t phase1 = t_dec1 + 20'000 + 7 * kVb;
  for (int i = 0; i < 8; ++i) reg.on_flip(t_dec1 + 20'000 + i * kVb, true);

  const uint64_t t_dec2 = phase1 + kVb - 9'000;
  CHECK(!reg.offer(frame(kFrame), t_dec2, &disp));  // servo frame held
  CHECK(reg.holding());

  const uint64_t t_dec3 = t_dec2 + 1'000;
  const uint32_t pts_jump = kFrame + 3'000'000u;  // > kResyncUs past last pts
  CHECK(reg.offer(frame(pts_jump), t_dec3, &disp));  // present now
  CHECK(disp.n == 2);
  CHECK(disp.f[0].opaque == frame(0).opaque);       // oldest (fallback) first
  CHECK(disp.f[1].opaque == frame(kFrame).opaque);  // then the servo frame
  CHECK(reg.discont_count() == 1);
  CHECK(!reg.holding());
}

namespace {
// One simulated minute of decode stream vs 60.000 Hz panel flips.
// Event-driven — exact times, no tick quantization. The estimator is
// warmed (8 exact flips) before the first decode so the servo owns every
// offer; the two arms differ only in source period, i.e. beat direction.
struct BeatSim {
  uint64_t offered = 0, released = 0, drained = 0;
  uint64_t skips = 0, replaced = 0, fallback = 0, late = 0;
};

BeatSim run_beat_sim(double src_period_us, double panel_period_us = 1e6 / 60.0) {
  FrameRegulator reg(12, true, 3);
  reg.set_panel_period(panel_period_us);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const double kPanel = panel_period_us;
  BeatSim s;
  double flip_t = static_cast<double>(kT0);
  for (int i = 0; i < 8; ++i) {
    reg.on_flip(static_cast<uint64_t>(flip_t), true);
    flip_t += kPanel;
  }
  double dec_t = flip_t + 8'000.0;  // arbitrary phase into the warm grid
  uint32_t pts = 0;
  const double t_end = static_cast<double>(kT0) + 60e6;
  while (dec_t < t_end || flip_t < t_end) {
    const double t = flip_t <= dec_t ? flip_t : dec_t;
    while (reg.release_due(static_cast<uint64_t>(t), &out)) ++s.released;
    if (flip_t <= dec_t) {
      reg.on_flip(static_cast<uint64_t>(flip_t), true);
      flip_t += kPanel;
    } else {
      pts += static_cast<uint32_t>(src_period_us);
      if (reg.offer(frame(pts), static_cast<uint64_t>(dec_t), &disp)) {
        // late passthrough: presented immediately, counted via late_count()
      }
      ++s.offered;
      dec_t += src_period_us;
    }
  }
  while (reg.release_due(~0ull, &out)) ++s.drained;
  s.skips = reg.vsync_skips();
  s.replaced = reg.replaced_count();
  s.fallback = reg.fallback_frames();
  s.late = reg.late_count();
  return s;
}
}  // namespace

TEST(beat_simulation_slow_source_never_skips) {
  // Real hardware direction: 59.939 fps sensor vs 60.000 Hz panel. Decode
  // spacing (16683.6) exceeds the panel period (16666.7), so two decodes
  // can never share a target vblank — wraps repeat a frame on the panel,
  // they never drop one. Freshest-wins must stay silent.
  const BeatSim s = run_beat_sim(1e6 / 59.939);
  CHECK(s.fallback == 0);  // servo never disengaged
  CHECK(s.skips == 0);     // no contention, deterministically
  CHECK(s.replaced == 0);  // and no evictions either
  CHECK(s.offered == s.released + s.replaced + s.late + s.drained);
}

TEST(beat_simulation_fast_source_drops_bounded_by_wraps) {
  // Inverted beat (source faster than panel): the source gains one frame
  // on the vblank grid per ~16.4 s wrap. Depending on the wrap's phase a
  // drop surfaces either as a deep-burst slot claim (vsync_skips: both
  // the natural and next slot occupied, newest takes the later one) or
  // as an oldest-out eviction once the backlog wants a third slot; both
  // increment replaced_count, the total-drops figure, bounded by the
  // wrap count (instrumented run: skips=2 replaced=2 -- claims, no
  // evictions -- so assert the total, not the mechanism).
  const BeatSim s = run_beat_sim(1e6 / 60.061);
  CHECK(s.fallback == 0);
  // Depending on wrap phase a drop surfaces either as a both-slots-
  // occupied claim (vsync_skips) or as an oldest-out eviction; both
  // increment replaced_count, which is the total-drops figure.
  CHECK(s.skips <= s.replaced);
  CHECK(s.replaced >= 1);
  CHECK(s.replaced <= 4);
  CHECK(s.offered == s.released + s.replaced + s.late + s.drained);
}

TEST(cross_rate_90fps_on_60hz_drops_one_in_three) {
  // The live bench case: 90 fps sensor on the 60 Hz GS panel. Three
  // decodes per two vblanks -- exactly one of every three frames must be
  // dropped (freshest-wins), never more, and the servo never disengages.
  const BeatSim s = run_beat_sim(1e6 / 90.0);
  CHECK(s.fallback == 0);
  CHECK(s.offered == s.released + s.replaced + s.late + s.drained);
  CHECK(static_cast<double>(s.replaced) >= 0.30 * s.offered);
  CHECK(static_cast<double>(s.replaced) <= 0.36 * s.offered);
}

TEST(matched_120fps_on_120hz_never_drops) {
  // Matched rates on a fast panel: one decode per vblank, no contention,
  // no drops -- the servo behaves exactly as 60-on-60 does today.
  const BeatSim s = run_beat_sim(1e6 / 120.0, 1e6 / 120.0);
  CHECK(s.fallback == 0);
  CHECK(s.skips == 0);
  CHECK(s.replaced == 0);
  CHECK(s.offered == s.released + s.replaced + s.late + s.drained);
}

TEST(cross_rate_120fps_on_60hz_drops_half) {
  // Double-rate source: every second frame is dropped, freshest-wins.
  const BeatSim s = run_beat_sim(1e6 / 120.0);
  CHECK(s.fallback == 0);
  CHECK(s.offered == s.released + s.replaced + s.late + s.drained);
  CHECK(static_cast<double>(s.replaced) >= 0.46 * s.offered);
  CHECK(static_cast<double>(s.replaced) <= 0.54 * s.offered);
}

MTEST_MAIN

TEST(panel_period_reseed_locks_120hz_grid) {
  // A 120 Hz panel's flips land 8333 µs apart -- under the default 60 Hz
  // seed the estimator rounds that delta to k=0 and DROPS every flip, so
  // the servo can never engage. main.cpp reseeds from the chosen DRM
  // mode's timings before the presenter produces its first flip.
  constexpr uint64_t kVb120 = 8'333;
  FrameRegulator reg(12, true, 3);
  reg.set_panel_period(8'333.3);
  for (int i = 0; i < 8; ++i) reg.on_flip(kT0 + i * kVb120, true);
  const uint64_t phase = kT0 + 7 * kVb120;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t_dec = phase + kVb120 - 5'000;
  CHECK(!reg.offer(frame(0), t_dec, &disp));
  CHECK(reg.servo_locked());
  const uint64_t release = phase + kVb120 - kLead;
  CHECK(!reg.release_due(release - 1, &out));
  CHECK(reg.release_due(release, &out));
  CHECK(reg.fallback_frames() == 0);
}

TEST(without_reseed_120hz_flips_never_warm) {
  // The defect the reseed fixes, pinned so it stays visible: same flip
  // stream, no set_panel_period -- every offer takes the fallback rule.
  constexpr uint64_t kVb120 = 8'333;
  FrameRegulator reg(12, true, 3);
  for (int i = 0; i < 8; ++i) reg.on_flip(kT0 + i * kVb120, true);
  FrameRegulator::Displaced disp;
  CHECK(!reg.offer(frame(0), kT0 + 8 * kVb120, &disp));
  CHECK(!reg.servo_locked());
  CHECK(reg.fallback_frames() == 1);
}

TEST(stall_backlog_holds_three_distinct_targets) {
  // Post-stall scenario (main loop order: decode sink OFFERS before the
  // loop drains release_due): f1@v1 and f2@v2 are held, the loop stalls
  // past v1's release, and f3 decodes during the catch-up -- its natural
  // slot v2 is occupied, so it takes v3. A 2-deep queue can only evict
  // f1, a frame already due for display; the deeper queue holds all
  // three and f1 merely releases late.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), phase + kVb - 9'000, &disp));        // @v1
  CHECK(!reg.offer(frame(kFrame), phase + kVb - 1'000, &disp));   // @v2
  CHECK(disp.n == 0);
  // Stall: v1's release (v1 - lead) passes undrained; f3 decodes at v1+1ms.
  CHECK(!reg.offer(frame(2 * kFrame), phase + kVb + 1'000, &disp));  // @v3
  CHECK(disp.n == 0);
  CHECK(reg.replaced_count() == 0);
  CHECK(reg.vsync_skips() == 0);
  // All three drain in pts order on consecutive vblanks (f1 late).
  CHECK(reg.release_due(phase + kVb + 1'001, &out));
  CHECK(out.opaque == frame(0).opaque);
  CHECK(reg.release_due(phase + 2 * kVb - kLead, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
  CHECK(reg.release_due(phase + 3 * kVb - kLead, &out));
  CHECK(out.opaque == frame(2 * kFrame).opaque);
  CHECK(!reg.holding());
}

TEST(discont_flushes_three_held_frames) {
  // Same backlog as above, then a drone-restart pts jump: the discont
  // path must hand back ALL held frames, so Displaced must carry the
  // full queue depth.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  CHECK(!reg.offer(frame(0), phase + kVb - 9'000, &disp));
  CHECK(!reg.offer(frame(kFrame), phase + kVb - 1'000, &disp));
  CHECK(!reg.offer(frame(2 * kFrame), phase + kVb + 1'000, &disp));
  const uint32_t pts_jump = 2 * kFrame + 3'000'000u;  // > kResyncUs
  CHECK(reg.offer(frame(pts_jump), phase + kVb + 2'000, &disp));
  CHECK(disp.n == 3);
  CHECK(disp.f[0].opaque == frame(0).opaque);
  CHECK(disp.f[1].opaque == frame(kFrame).opaque);
  CHECK(disp.f[2].opaque == frame(2 * kFrame).opaque);
  CHECK(!reg.holding());
}

TEST(full_queue_evicts_overdue_head_on_fifth_target) {
  // The depth cap still exists -- at FOUR. Extend the stall backlog to
  // f1..f4 on v1..v4, then a fifth distinct target evicts the oldest
  // (most-overdue) head, latency-first.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), phase + kVb - 9'000, &disp));            // @v1
  CHECK(!reg.offer(frame(kFrame), phase + kVb - 1'000, &disp));       // @v2
  CHECK(!reg.offer(frame(2 * kFrame), phase + kVb + 1'000, &disp));   // @v3
  CHECK(!reg.offer(frame(3 * kFrame), phase + 2 * kVb + 1'000, &disp));  // @v4
  CHECK(disp.n == 0);
  CHECK(!reg.offer(frame(4 * kFrame), phase + 3 * kVb + 1'000, &disp));  // @v5
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(0).opaque);  // overdue head evicted
  CHECK(reg.replaced_count() == 1);
  // Survivors still release in order.
  CHECK(reg.release_due(phase + 3 * kVb + 1'001, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
  CHECK(reg.release_due(phase + 3 * kVb + 1'002, &out));
  CHECK(out.opaque == frame(2 * kFrame).opaque);
  CHECK(reg.release_due(phase + 4 * kVb - kLead, &out));
  CHECK(out.opaque == frame(3 * kFrame).opaque);
  CHECK(reg.release_due(phase + 5 * kVb - kLead, &out));
  CHECK(out.opaque == frame(4 * kFrame).opaque);
  CHECK(!reg.holding());
}

TEST(heal_slip_shifts_pending_releases_one_slot) {
  // Chain-break heal: when the main loop sees the presenter's in-flight
  // mailbox engaging continuously (a one-vsync-late present chain), it
  // calls heal_slip() -- every pending servo release slips one slot so a
  // vblank goes unfilled and the flip pipeline drains.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  // Burst pair -> held at v and v+p.
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));
  CHECK(!reg.offer(frame(kFrame), t1 + 1'000, &disp));
  reg.heal_slip();
  CHECK(reg.heals() == 1);
  // Both slipped: first now releases for v+2p's predecessor... i.e. at
  // (v+p)-lead, second at (v+2p)-lead. Nothing due at the original slot.
  CHECK(!reg.release_due(phase + kVb - kLead, &out));
  CHECK(reg.release_due(phase + 2 * kVb - kLead, &out));
  CHECK(out.opaque == frame(0).opaque);
  CHECK(reg.release_due(phase + 3 * kVb - kLead, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
}

TEST(heal_slip_noop_in_fallback_or_empty) {
  FrameRegulator reg(12, true, 3);
  reg.heal_slip();               // empty queue: no-op
  CHECK(reg.heals() == 0);
  FrameRegulator::Displaced disp;
  // Fallback-held frame (estimator cold): target 0, heal must not touch.
  CHECK(!reg.offer(frame(0), kT0, &disp));
  reg.heal_slip();
  CHECK(reg.heals() == 0);
  DmaFrame out;
  CHECK(reg.release_due(kT0 + kD, &out));  // fallback release unchanged
}

TEST(next_release_us_reports_earliest_pending) {
  FrameRegulator reg(12);
  CHECK(reg.next_release_us() == 0);          // empty
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));    // fallback hold at kT0 + D
  CHECK(reg.next_release_us() == kT0 + kD);
  CHECK(reg.release_due(kT0 + kD, &out));
  CHECK(reg.next_release_us() == 0);          // drained
}

TEST(servo_triple_burst_claims_the_second_slot) {
  // Three decodes 1 ms apart (a deep fec-batch burst): first takes its
  // natural slot, second the next slot, and the THIRD -- both slots
  // occupied -- claims the later slot from the middle frame
  // (freshest-wins, vsync_skips). Survivors release on consecutive
  // vblanks.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));
  CHECK(!reg.offer(frame(kFrame), t1 + 1'000, &disp));
  CHECK(disp.n == 0);
  CHECK(!reg.offer(frame(2 * kFrame), t1 + 2'000, &disp));
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(kFrame).opaque);  // middle frame dropped
  CHECK(reg.vsync_skips() == 1);
  CHECK(reg.replaced_count() == 1);
  CHECK(reg.release_due(phase + kVb - kLead, &out));
  CHECK(out.opaque == frame(0).opaque);
  CHECK(reg.release_due(phase + 2 * kVb - kLead, &out));
  CHECK(out.opaque == frame(2 * kFrame).opaque);
  CHECK(!reg.holding());
}

TEST(heal_slip_preserves_release_order_with_a_mixed_queue) {
  // Review finding 2026-08-31: heal_slip() moves only servo entries, so
  // a mixed queue [servo, fallback] could invert the sorted-by-release
  // invariant that release_due()/next_release_us() head-inspect --
  // releasing the fallback frame late and out of pts order. Construct:
  // fallback frame held first (cold estimator), then warm the grid so a
  // servo frame lands with an EARLIER release, then heal.
  FrameRegulator reg(12, true, 3);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(!reg.offer(frame(0), kT0, &disp));           // fallback @ kT0+12ms
  // Warm with flips whose next grid slot is kT0+8000.
  for (int i = 8; i >= 1; --i) reg.on_flip(kT0 + 8'000 - i * kVb, true);
  CHECK(!reg.offer(frame(kFrame), kT0 + 2'000, &disp));  // servo @ kT0+5000
  CHECK(disp.n == 0);
  CHECK(reg.next_release_us() == kT0 + 5'000);           // servo heads
  reg.heal_slip();
  CHECK(reg.heals() == 1);
  // Servo slipped past the fallback frame: head must be the fallback
  // release (kT0+12ms), not a stale claim of the slipped servo slot.
  CHECK(reg.next_release_us() == kT0 + kD);
  CHECK(reg.release_due(kT0 + kD, &out));
  CHECK(out.opaque == frame(0).opaque);                  // fallback first
  CHECK(reg.release_due(kT0 + 5'000 + kVb, &out));
  CHECK(out.opaque == frame(kFrame).opaque);             // slipped servo
  CHECK(!reg.holding());
}

TEST(chain_counters_track_sequential_slot_assignments) {
  // A collision (two decodes claiming one vblank) chains the newer frame
  // onto the next slot; every frame after it at source cadence then finds
  // its natural slot occupied and chains too, until a gap wider than a
  // period frees a slot. chained= counts every +1-slot assignment,
  // chain= is the current run length, chain_max= its high-water mark.
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  CHECK(reg.chained_count() == 0);
  CHECK(reg.chain_run() == 0);
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));                // natural slot
  CHECK(reg.chain_run() == 0);
  CHECK(!reg.offer(frame(kFrame), t1 + 1'000, &disp));   // collision -> +1
  CHECK(reg.chained_count() == 1);
  CHECK(reg.chain_run() == 1);
  // Next frame at exact source cadence: its natural slot is held by the
  // chained predecessor, so it chains as well.
  CHECK(!reg.offer(frame(2 * kFrame), t1 + 1'000 + kFrame, &disp));
  CHECK(reg.chained_count() == 2);
  CHECK(reg.chain_run() == 2);
  CHECK(reg.chain_max() == 2);
  CHECK(disp.n == 0);
  CHECK(reg.vsync_skips() == 0);
  // Drain everything, then a frame arriving with a free natural slot
  // ends the run: chain= resets, chained= and chain_max= keep the tally.
  while (reg.release_due(phase + 10 * kVb, &out)) {
  }
  CHECK(!reg.holding());
  CHECK(!reg.offer(frame(3 * kFrame), phase + 5 * kVb - 9'000, &disp));
  CHECK(reg.chain_run() == 0);
  CHECK(reg.chained_count() == 2);
  CHECK(reg.chain_max() == 2);
}

TEST(chains_counter_counts_chain_starts_not_frames) {
  FrameRegulator reg(12, true, 3);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));
  CHECK(!reg.offer(frame(kFrame), t1 + 1'000, &disp));            // start
  CHECK(!reg.offer(frame(2 * kFrame), t1 + 1'000 + kFrame, &disp));  // same chain
  CHECK(reg.chains_count() == 1);
  CHECK(reg.chained_count() == 2);
  while (reg.release_due(phase + 10 * kVb, &out)) {
  }
  // Fresh collision after the queue drained = a second chain.
  const uint64_t t2 = phase + 12 * kVb - 12'000;
  CHECK(!reg.offer(frame(3 * kFrame), t2, &disp));
  CHECK(!reg.offer(frame(4 * kFrame), t2 + 1'000, &disp));
  CHECK(reg.chains_count() == 2);
}

TEST(chain_budget_cuts_run_by_dropping_held_older_frame) {
  // display.chain_budget N: once a run has N chained frames, the next
  // colliding frame takes its natural slot and the held occupant is
  // displaced (freshest wins, one drop) instead of extending the chain.
  FrameRegulator reg(12, true, 3, /*chain_budget=*/2);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  DmaFrame out;
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));                              // slot 1
  CHECK(!reg.offer(frame(kFrame), t1 + 1'000, &disp));                 // chained, slot 2
  CHECK(!reg.offer(frame(2 * kFrame), t1 + 1'000 + kFrame, &disp));    // chained, slot 3
  CHECK(reg.chain_run() == 2);
  CHECK(disp.n == 0);
  // Fourth frame at cadence: natural slot 3 is held by frame 2 -- budget
  // spent, so frame 2 is displaced and frame 3 takes slot 3.
  CHECK(!reg.offer(frame(3 * kFrame), t1 + 1'000 + 2 * kFrame, &disp));
  CHECK(disp.n == 1);
  CHECK(disp.f[0].opaque == frame(2 * kFrame).opaque);
  CHECK(reg.chain_cuts() == 1);
  CHECK(reg.chain_run() == 0);
  CHECK(reg.chained_count() == 2);
  CHECK(reg.vsync_skips() == 0);
  // Release order: frame 0 at slot 1, frame 1 at slot 2, frame 3 at slot 3.
  CHECK(reg.release_due(phase + kVb - kLead, &out));
  CHECK(out.opaque == frame(0).opaque);
  CHECK(reg.release_due(phase + 2 * kVb - kLead, &out));
  CHECK(out.opaque == frame(kFrame).opaque);
  CHECK(reg.release_due(phase + 3 * kVb - kLead, &out));
  CHECK(out.opaque == frame(3 * kFrame).opaque);
  CHECK(!reg.holding());
}

TEST(chain_budget_zero_never_cuts) {
  FrameRegulator reg(12, true, 3, 0);
  warm(reg);
  const uint64_t phase = kT0 + 7 * kVb;
  FrameRegulator::Displaced disp;
  const uint64_t t1 = phase + kVb - 12'000;
  CHECK(!reg.offer(frame(0), t1, &disp));
  for (int i = 1; i <= 3; ++i)
    CHECK(!reg.offer(frame(i * kFrame), t1 + 1'000 + (i - 1) * kFrame, &disp));
  CHECK(reg.chain_run() == 3);
  CHECK(reg.chain_cuts() == 0);
  CHECK(disp.n == 0);
}
