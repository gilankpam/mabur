#ifndef MABUR_PLAYER_FPS_CAP_H_
#define MABUR_PLAYER_FPS_CAP_H_

#include <cstdint>

namespace maburplay {

// The burned DVR's frame-rate cap: which decoded frames the encoder is fed.
//
// Schedule-based, not last-admit-based. The old rule ("admit if at least
// interval - slack since the last admitted frame") sounds right but loses a
// quarter of a 60 fps stream at cap 60: decoded frames do not arrive evenly,
// they arrive in pairs (one on time, the next ~10 ms later instead of 16.7),
// and the early half of every pair fails the test. Widening the slack to
// cover that would un-cap the 47..59 band against a 59.94 source, which is
// the trap the old proportional slack existed to avoid (a cap of 50 must not
// run the encoder at 60). Bench 2026-09-17: cap 60 admitted 41-45 fps of a
// 60 fps stream, three runs.
//
// Here a frame is admitted when it is due -- at or after the next slot minus
// half an interval -- and the next slot is then one interval after
// max(previous slot, now - slack). Because every admit moves the schedule a
// full interval forward and the schedule never sits more than half an
// interval in the past, N admits need at least (N - 2) intervals of wall
// time: the long-run rate is the cap whatever the slack (a source faster
// than the cap is thinned to it, with a start-up burst of at most two extra
// frames), while a source at or below the cap is passed whole as long as no
// two of its frames arrive closer than half an interval apart. A gap in the
// stream does not bank credit: the slot cannot fall further behind `now`
// than half an interval, so a burst after a hole is thinned like any other.
class FpsCap {
 public:
  // fps <= 0 is treated as 1. Also the "recording started" reset.
  void reset(int fps) {
    interval_us_ = 1000000 / (fps > 0 ? fps : 1);
    slack_us_ = interval_us_ / 2;
    have_slot_ = false;
    next_slot_us_ = 0;
  }

  // now_us: any monotonic clock, same one every call.
  bool admit(int64_t now_us) {
    if (have_slot_ && now_us < next_slot_us_ - slack_us_) return false;
    int64_t base = now_us - slack_us_;
    if (have_slot_ && next_slot_us_ > base) base = next_slot_us_;
    next_slot_us_ = base + interval_us_;
    have_slot_ = true;
    return true;
  }

  int64_t interval_us() const { return interval_us_; }

 private:
  int64_t interval_us_ = 33333;
  int64_t slack_us_ = 16666;
  bool have_slot_ = false;
  int64_t next_slot_us_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_FPS_CAP_H_
