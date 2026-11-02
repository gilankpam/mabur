#ifndef MABUR_PLAYER_DISPLAY_FOLLOW_H_
#define MABUR_PLAYER_DISPLAY_FOLLOW_H_

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "gs_snapshot.h"  // GsSnapshot, vtx_value
#include "mabur/config_params.h"  // cfgparam::parse_mode_text
#include "mode_pick.h"
#include "rate_match.h"  // RateMatch::LowPower

namespace maburplay {

// The camera rate the follow rule judges by: the fps of the drone's
// reported VENC_MODE when the sideport holds one, else the measured rate.
// The reported mode is preferred over the measured cadence because the
// measured rate lags a camera change by a whole averaging window and is
// biased low under loss (a dropped frame reads as a longer period).
inline double follow_camera_hz(const GsSnapshot* snap, double measured_hz) {
  if (!snap || !snap->vtx) return measured_hz;
  const std::optional<std::string> mode_text = vtx_value(*snap, "VENC_MODE");
  if (!mode_text) return measured_hz;
  int fps = 0, w = 0, h = 0;
  bool binned = false;
  if (!mabur::cfgparam::parse_mode_text(*mode_text, &fps, &w, &h, &binned) || fps <= 0)
    return measured_hz;
  return static_cast<double>(fps);
}

// What the display follow rule sees once a second: the configured want,
// the camera's measured rate, the drone's low-power state, and the mode
// list the running presenter was built from with the index it runs.
struct FollowIn {
  ScreenWant want;
  double camera_hz = 0.0;
  bool low_power = false;
  std::vector<ModeCand> modes;
  int running = -1;  // index into modes, -1 = none
};

// rebuild: switch the presenter to modes[mode]. mismatch: the display has
// no mode at the camera's rate, and shows it at shown_hz instead.
struct FollowOut {
  bool rebuild = false;
  int mode = -1;
  bool mismatch = false;
  double shown_hz = 0.0;
};

// Whether the display should move to follow the camera's rate
// (pick_display_mode), judged on one window. Never for a fully pinned
// screen_mode, for a drone in low power (it drops its fps on purpose), for
// a camera rate not measured yet, or with no connector modes at all -- a
// failed EDID read keeps the running mode rather than rebuilding the
// presenter over and over.
//
// The running mode is kept over the pick when it fits the want, is at
// least as large, and is as near the camera's rate as the pick within
// kModeMatchHz: within kModeMatchHz of the camera when the display has
// the camera's rate (the phase lock steers the camera onto it), else within
// kModeMatchHz of the nearest rate the display has. A camera midway between
// two display rates (90 fps between 60 and 120 Hz) jitters across the
// midpoint window to window; without this every crossing would rebuild.
inline FollowOut follow_display(const FollowIn& in) {
  FollowOut out;
  if (in.want.kind == ScreenWant::Kind::kFull || in.low_power || in.camera_hz <= 0.0 ||
      in.modes.empty())
    return out;
  // A kSize want whose exact size has no mode near the camera's rate, with
  // some other mode there, is judged as "auto" for this window: the
  // display cannot honour that size at the new rate, and the alternative
  // to its largest offered mode is to keep running the old rate.
  ScreenWant want = in.want;
  if (want.kind == ScreenWant::Kind::kSize) {
    bool size_at_rate = false;
    bool any_at_rate = false;
    for (const ModeCand& m : in.modes) {
      if (m.interlaced || std::fabs(mode_hz(m) - in.camera_hz) > kModeMatchHz) continue;
      any_at_rate = true;
      if (m.w == want.w && m.h == want.h) size_at_rate = true;
    }
    if (any_at_rate && !size_at_rate) want.kind = ScreenWant::Kind::kAuto;
  }
  bool matched = false;
  int mode = pick_display_mode(in.modes, want, in.camera_hz, &matched);
  if (mode < 0) return out;
  const int n = static_cast<int>(in.modes.size());
  if (in.running >= 0 && in.running < n && in.running != mode) {
    const ModeCand& r = in.modes[static_cast<size_t>(in.running)];
    const ModeCand& p = in.modes[static_cast<size_t>(mode)];
    double nearest_d = -1.0;
    for (const ModeCand& m : in.modes) {
      if (!fits_want(m, want)) continue;
      const double d = std::fabs(mode_hz(m) - in.camera_hz);
      if (nearest_d < 0.0 || d < nearest_d) nearest_d = d;
    }
    const double limit = matched ? kModeMatchHz : nearest_d + kModeMatchHz;
    if (fits_want(r, want) && static_cast<long>(r.w) * r.h >= static_cast<long>(p.w) * p.h &&
        std::fabs(mode_hz(r) - in.camera_hz) <= limit)
      mode = in.running;
  }
  out.mode = mode;
  out.rebuild = mode != in.running;
  out.mismatch = !matched;
  out.shown_hz = mode_hz(in.modes[static_cast<size_t>(mode)]);
  return out;
}

// The display follow over time, fed once a second. It follows only while
// screen_mode follows (not "WxH@R"), the drone reports it is out of low
// power (kNo: a quiet sideport, kUnknown, holds off as RateMatch does) and
// the camera's rate is measured; it follows whether or not RateMatch
// reports a mismatch, so "auto" also moves up to a larger mode at a rate
// the panel already runs.
//
// A rebuild is issued only once kAgreeWindows consecutive windows pick the
// same mode: one window's rate can be wrong (a fade losing over half the
// frames doubles the median step; a camera change lands mid-window), and
// each rebuild blanks the screen. A rebuild whose presenter then runs some
// other mode (init() fell back: the sink's list changed under it) is not
// issued again for that mode until the pick settles or moves elsewhere.
class DisplayFollower {
 public:
  static constexpr int kAgreeWindows = 3;

  // One window: the configured want, the camera's measured rate (0 = not
  // yet), the drone's low-power report, and the running presenter's modes
  // with the index it runs (empty and -1 with no presenter). `mismatch` and
  // `shown_hz` describe the running mode once the pick has settled on it.
  FollowOut on_window(const ScreenWant& want, double camera_hz, RateMatch::LowPower lp,
                      const std::vector<ModeCand>& modes, int running) {
    lp_ = lp;
    if (lp == RateMatch::LowPower::kNo && camera_hz > 0.0) last_hz_ = camera_hz;
    FollowOut out;
    if (want.kind == ScreenWant::Kind::kFull || lp != RateMatch::LowPower::kNo ||
        camera_hz <= 0.0 || modes.empty()) {
      restart();
      issued_.clear();
      return out;
    }
    if (!issued_.empty()) {
      const bool landed = running >= 0 && running < static_cast<int>(modes.size()) &&
                          mode_string(modes[static_cast<size_t>(running)]) == issued_;
      if (!landed) stuck_ = issued_;
      issued_.clear();
    }
    FollowIn in;
    in.want = want;
    in.camera_hz = camera_hz;
    in.modes = modes;
    in.running = running;
    const FollowOut pick = follow_display(in);
    if (!pick.rebuild) {
      restart();
      stuck_.clear();
      return pick;
    }
    const std::string target = mode_string(modes[static_cast<size_t>(pick.mode)]);
    if (target == stuck_) {
      restart();
      return out;
    }
    if (target == pending_) {
      ++agree_;
    } else {
      pending_ = target;
      agree_ = 1;
    }
    if (agree_ < kAgreeWindows) return out;
    restart();
    stuck_.clear();
    issued_ = target;
    out.rebuild = true;
    out.mode = pick.mode;
    return out;
  }

  // The camera rate to resolve a presenter build against: the current one
  // while the last window saw the drone out of low power, else the last
  // rate seen out of low power (0 before any) -- a replug or a live set
  // during low power must not move the display to the low-power rate.
  double resolve_hz(double camera_hz) const {
    return lp_ == RateMatch::LowPower::kNo ? camera_hz : last_hz_;
  }

 private:
  void restart() {
    pending_.clear();
    agree_ = 0;
  }

  RateMatch::LowPower lp_ = RateMatch::LowPower::kUnknown;
  double last_hz_ = 0.0;
  std::string pending_;  // the mode the agreeing windows pick
  int agree_ = 0;
  std::string issued_;  // the last rebuild issued, judged on the next window
  std::string stuck_;   // an issued rebuild that did not land
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DISPLAY_FOLLOW_H_
