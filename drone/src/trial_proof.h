#pragma once
// A trial maburd's two milestones, from once-a-second samples: kLinked on
// the first tick with a published session, kProven once the link has held
// and the encoder has produced frames on every tick for kProvenMs straight.
// A tick that is unlinked, or whose frame count did not move, starts the
// run over. Each event fires once per process.
#include <cstdint>

namespace mabur {

class TrialProof {
 public:
  enum Event { kNone, kLinked, kProven };
  static constexpr uint64_t kProvenMs = 5000;

  Event tick(uint64_t now_ms, bool linked, uint64_t frames_total) {
    const bool advanced = have_frames_ && frames_total > last_frames_;
    have_frames_ = true;
    last_frames_ = frames_total;
    if (!linked) {
      in_run_ = false;
      return kNone;
    }
    Event ev = kNone;
    if (!linked_) {
      linked_ = true;
      ev = kLinked;
    }
    if (!in_run_ || !advanced) {
      in_run_ = true;
      run_start_ms_ = now_ms;
      return ev;
    }
    if (!proven_ && now_ms - run_start_ms_ >= kProvenMs) {
      proven_ = true;
      return kProven;
    }
    return ev;
  }

 private:
  bool linked_ = false, proven_ = false, in_run_ = false, have_frames_ = false;
  uint64_t last_frames_ = 0, run_start_ms_ = 0;
};

}  // namespace mabur
