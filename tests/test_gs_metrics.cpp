// Host tests for the player-measured half of the GS overlay's inputs.
//
// These exist because two shipped defects in this logic were unreachable
// from any host test while it lived inline in maburplay's hardware-only
// main loop: a link outage reading as a recording FAULT, and a truncation
// storm doing the same. Both are one-line cases here.
#include "gs_metrics.h"

#include "mtest.h"

using maburplay::AuJitter;
using maburplay::RecState;
using maburplay::RecTracker;

namespace {

// A healthy 60 fps arrival train, so the tests read in the units the
// overlay actually shows.
constexpr uint64_t kFrameMs = 17;

RecTracker::Inputs healthy(uint64_t samples, uint64_t feed) {
  RecTracker::Inputs in;
  in.open = true;
  in.samples = samples;
  in.feed = feed;
  return in;
}

}  // namespace

// --- RecTracker ------------------------------------------------------

TEST(rec_default_inputs_render_armed) {
  RecTracker t;
  RecTracker::Inputs in;  // nothing open, nothing written
  CHECK(t.update(in, 1000).kind == RecState::Kind::kArmed);
}

// The invariant the whole state machine exists to guarantee: the pilot must
// never see RECORDING while nothing is being written.
TEST(recording_implies_samples_written) {
  RecTracker t;
  uint64_t now = 1000;
  // Open, feed flowing, but the mux has not written a sample yet.
  for (int i = 0; i < 10; ++i, now += 1000) {
    const RecState s = t.update(healthy(0, (uint64_t)i * 60), now);
    CHECK(s.kind == RecState::Kind::kArmed);
  }
  // First sample: RECORDING, and the clock starts HERE.
  const RecState s = t.update(healthy(1, 600), now);
  CHECK(s.kind == RecState::Kind::kRecording);
  CHECK(s.elapsed_s == 0);
}

TEST(recorder_that_never_started_is_a_fault_not_armed) {
  RecTracker t;
  RecTracker::Inputs in;
  in.broken = true;
  CHECK(t.update(in, 1000).kind == RecState::Kind::kFault);
}

// A card that is already full when the session starts must fault
// immediately, not sit at ARMED waiting for a file that cannot be written.
TEST(low_space_faults_before_the_file_is_ever_open) {
  RecTracker t;
  RecTracker::Inputs in;
  in.open = false;
  in.low_space = true;
  CHECK(t.update(in, 1000).kind == RecState::Kind::kFault);
}

TEST(low_space_faults_a_running_recording_too) {
  RecTracker t;
  uint64_t now = 1000;
  CHECK(t.update(healthy(10, 10), now).kind == RecState::Kind::kRecording);
  now += 1000;
  RecTracker::Inputs in = healthy(20, 20);
  in.low_space = true;
  CHECK(t.update(in, now).kind == RecState::Kind::kFault);
}

// Deviation 7's case: the link dies, so no AUs arrive, so no samples are
// written. The recorder is blameless and must not be accused.
TEST(link_outage_is_not_a_recording_fault) {
  RecTracker t;
  uint64_t now = 1000;
  CHECK(t.update(healthy(60, 60), now).kind == RecState::Kind::kRecording);
  // 30 s with both counters frozen: nothing arriving, nothing to record.
  for (int i = 0; i < 30; ++i) {
    now += 1000;
    CHECK(t.update(healthy(60, 60), now).kind == RecState::Kind::kRecording);
  }
}

// The case deviation 7 MISSED, and the reason `feed` counts complete AUs
// rather than deliveries: a link bad enough that every AU is truncated
// delivers steadily while the raw DVR, which skips incomplete AUs whole,
// writes nothing. Counting deliveries here would fault after 3 s.
TEST(truncated_only_delivery_is_not_a_recording_fault) {
  RecTracker t;
  uint64_t now = 1000;
  CHECK(t.update(healthy(60, 60), now).kind == RecState::Kind::kRecording);
  // `feed` is COMPLETE AUs, so it stays put while deliveries climb.
  for (int i = 0; i < 30; ++i) {
    now += 1000;
    CHECK(t.update(healthy(60, 60), now).kind == RecState::Kind::kRecording);
  }
  // Sanity: had `feed` been deliveries (climbing), this is what would have
  // happened -- proving the assertion above is not vacuous.
  RecTracker u;
  now = 1000;
  CHECK(u.update(healthy(60, 60), now).kind == RecState::Kind::kRecording);
  bool faulted = false;
  for (int i = 1; i <= 30; ++i) {
    now += 1000;
    if (u.update(healthy(60, 60 + (uint64_t)i * 60), now).kind == RecState::Kind::kFault)
      faulted = true;
  }
  CHECK(faulted == true);
}

// The fault the stall clock is actually for: input keeps arriving and the
// writer stops.
TEST(writer_stalling_under_live_input_faults_after_three_seconds) {
  RecTracker t;
  uint64_t now = 1000;
  CHECK(t.update(healthy(60, 60), now).kind == RecState::Kind::kRecording);
  // Feed advances, samples do not. Under the 3 s threshold: still fine.
  now += 1000;
  CHECK(t.update(healthy(60, 120), now).kind == RecState::Kind::kRecording);
  now += 1000;
  CHECK(t.update(healthy(60, 180), now).kind == RecState::Kind::kRecording);
  now += 1000;  // now exactly 3000 ms since the last sample advance
  CHECK(t.update(healthy(60, 240), now).kind == RecState::Kind::kFault);
}

TEST(a_stalled_writer_that_resumes_clears_the_fault) {
  RecTracker t;
  uint64_t now = 1000;
  t.update(healthy(60, 60), now);
  now += 4000;
  CHECK(t.update(healthy(60, 300), now).kind == RecState::Kind::kFault);
  now += 1000;
  CHECK(t.update(healthy(120, 360), now).kind == RecState::Kind::kRecording);
}

// The clock's origin is the first sample written, identically in both
// modes -- not "the recorder was armed", which made burned mode
// over-report by however long the link took to come up.
TEST(elapsed_counts_from_the_first_sample_not_from_arming) {
  RecTracker t;
  uint64_t now = 10000;
  // 5 s armed with nothing written.
  for (int i = 0; i < 5; ++i, now += 1000) t.update(healthy(0, (uint64_t)i), now);
  CHECK(t.update(healthy(1, 100), now).kind == RecState::Kind::kRecording);
  const uint64_t t0 = now;
  now = t0 + 7400;
  const RecState s = t.update(healthy(500, 600), now);
  CHECK(s.kind == RecState::Kind::kRecording);
  CHECK(s.elapsed_s == 7);
}

// --- AuJitter --------------------------------------------------------

TEST(jitter_is_zero_before_anything_arrives) {
  AuJitter j;
  CHECK(j.ms() == 0.0);
  j.on_tick(100000);  // must not divide by, or read, a zero timestamp
  CHECK(j.ms() == 0.0);
}

TEST(a_perfectly_even_train_has_no_jitter) {
  AuJitter j;
  uint64_t now = 1000;
  for (int i = 0; i < 200; ++i) {
    j.on_au(now);
    now += kFrameMs;
  }
  CHECK(j.ms() == 0.0);
}

TEST(an_uneven_train_shows_jitter_bounded_by_the_wobble) {
  AuJitter j;
  uint64_t now = 1000;
  // Alternating 12/22 ms: every interval differs from the last by 10 ms, so
  // the EMA converges on 10 and can never exceed it.
  for (int i = 0; i < 400; ++i) {
    j.on_au(now);
    now += (i % 2) ? 22 : 12;
  }
  CHECK(j.ms() > 9.0);
  CHECK(j.ms() <= 10.0);
}

// The minor the reviewer flagged: a 200-900 ms freeze is the common
// bad-link event, and folding it into the EMA left a large number decaying
// over the next ~16 AUs, reading as jitter long after the freeze ended.
TEST(a_sub_second_freeze_is_excluded_from_the_ema) {
  AuJitter j;
  uint64_t now = 1000;
  for (int i = 0; i < 100; ++i) {
    j.on_au(now);
    now += kFrameMs;
  }
  CHECK(j.ms() == 0.0);
  now += 400;  // a 400 ms freeze -- under the old 1000 ms threshold
  j.on_au(now);
  CHECK(j.ms() == 0.0);  // dropped, not averaged in
  // And the first interval after the gap is not compared to the last one
  // before it either.
  now += kFrameMs;
  j.on_au(now);
  CHECK(j.ms() == 0.0);
}

// Without on_tick() an outage leaves the last EMA frozen on screen: a
// plausible-looking "JIT 8.3 ms" beside an FPS that has fallen to zero.
TEST(on_tick_clears_the_ema_once_aus_stop) {
  AuJitter j;
  uint64_t now = 1000;
  for (int i = 0; i < 400; ++i) {
    j.on_au(now);
    now += (i % 2) ? 22 : 12;
  }
  CHECK(j.ms() > 9.0);
  j.on_tick(now + 50);  // still inside the stall window: hold the value
  CHECK(j.ms() > 9.0);
  j.on_tick(now + AuJitter::kStallMs);
  CHECK(j.ms() == 0.0);
}

// The record button starts a NEW file on each press, and the OSD clock
// must count that file, not the session. Without reset() the second
// recording opens showing the first one's elapsed time.
TEST(rec_tracker_reset_restarts_the_clock_for_a_second_recording) {
  RecTracker t;
  RecTracker::Inputs in;
  in.open = true;
  in.samples = 1;
  in.feed = 1;
  CHECK(t.update(in, 10000).kind == RecState::Kind::kRecording);
  in.samples = 300;
  in.feed = 300;
  RecState s = t.update(in, 15000);
  CHECK(s.kind == RecState::Kind::kRecording);
  CHECK(s.elapsed_s == 5);

  // Stop: nothing open, nothing written -- and the DvrMux counters are
  // per file, so they return to 0 (see DvrMux::open's reset).
  t.reset();
  in.open = false;
  in.samples = 0;
  CHECK(t.update(in, 20000).kind == RecState::Kind::kArmed);

  // Second recording: start with the same sample count as the first file ended.
  // This creates a discriminator: the stall detector compares in.samples to the
  // OLD samples_, so if samples_ wasn't cleared to 0, the second file's opening
  // sample will match the first file's final sample, making the condition
  // `in.samples != samples_` false. When we then stall (feed advances but
  // samples don't), the condition `in.feed == feed_` is also false, so
  // stall_since_ms_ is NOT refreshed. If stall_since_ms_ still holds 15000
  // from the first file, the check `now_ms - stall_since_ms_ >= kStallMs`
  // would incorrectly fault at t=32000 (17000 ms > 3000). With reset(), the
  // check uses stall_since_ms_=30000, so (32000-30000)=2000 < 3000, stays OK.
  in.open = true;
  in.samples = 300;  // Same as first file's final count
  in.feed = 400;
  CHECK(t.update(in, 30000).kind == RecState::Kind::kRecording);

  // Stall: feed advances but samples stay locked at 300
  in.samples = 300;  // No change
  in.feed = 520;
  s = t.update(in, 32000);
  CHECK(s.kind == RecState::Kind::kRecording);  // Fails if stall_since_ms_ not reset
  CHECK(s.elapsed_s == 2);  // Fails if start_ms_ not reset (would be 22 if latched at 10000)
}

MTEST_MAIN
