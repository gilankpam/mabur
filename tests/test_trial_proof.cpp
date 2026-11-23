// TrialProof: when a trial maburd has linked, and when it has proven itself
// (linked with encoder frames advancing for 5 s straight).
#include "../drone/src/trial_proof.h"
#include "mtest.h"

using mabur::TrialProof;

TEST(linked_fires_once) {
  TrialProof p;
  CHECK(p.tick(0, false, 0) == TrialProof::kNone);
  CHECK(p.tick(1000, true, 0) == TrialProof::kLinked);
  CHECK(p.tick(2000, true, 0) == TrialProof::kNone);
  CHECK(p.tick(3000, false, 0) == TrialProof::kNone);
  CHECK(p.tick(4000, true, 0) == TrialProof::kNone);
}

TEST(proven_after_five_seconds_of_frames) {
  TrialProof p;
  uint64_t frames = 0;
  CHECK(p.tick(0, true, frames) == TrialProof::kLinked);   // the run starts here
  for (uint64_t t = 1000; t < 5000; t += 1000) {
    frames += 30;
    CHECK(p.tick(t, true, frames) == TrialProof::kNone);
  }
  frames += 30;
  CHECK(p.tick(5000, true, frames) == TrialProof::kProven);
  for (uint64_t t = 6000; t <= 20000; t += 1000) {
    frames += 30;
    CHECK(p.tick(t, true, frames) == TrialProof::kNone);
  }
}

TEST(a_stalled_tick_restarts_the_run) {
  TrialProof p;
  uint64_t frames = 0;
  CHECK(p.tick(0, true, frames) == TrialProof::kLinked);
  for (uint64_t t = 1000; t <= 4000; t += 1000) {
    frames += 30;
    CHECK(p.tick(t, true, frames) == TrialProof::kNone);
  }
  CHECK(p.tick(5000, true, frames) == TrialProof::kNone);   // no new frames: restart at 5000
  for (uint64_t t = 6000; t < 10000; t += 1000) {
    frames += 30;
    CHECK(p.tick(t, true, frames) == TrialProof::kNone);
  }
  frames += 30;
  CHECK(p.tick(10000, true, frames) == TrialProof::kProven);
}

TEST(unlinked_never_proves) {
  TrialProof p;
  uint64_t frames = 0;
  for (uint64_t t = 0; t <= 30000; t += 1000) {
    frames += 30;
    CHECK(p.tick(t, false, frames) == TrialProof::kNone);
  }
}

MTEST_MAIN
