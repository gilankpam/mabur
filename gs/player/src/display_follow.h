#ifndef MABUR_PLAYER_DISPLAY_FOLLOW_H_
#define MABUR_PLAYER_DISPLAY_FOLLOW_H_

#include <cmath>
#include <vector>

#include "mode_pick.h"

namespace maburplay {

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
// (pick_display_mode). Never for a fully pinned screen_mode, for a drone in
// low power (it drops its fps on purpose), for a camera rate not measured
// yet, or with no connector modes at all -- a failed EDID read keeps the
// running mode rather than rebuilding the presenter over and over. A
// running mode of the picked size already within kModeMatchHz of the
// camera's rate is kept: the phase lock steers the camera onto it, and a
// rebuild between two modes in its reach would only blank the screen.
inline FollowOut follow_display(const FollowIn& in) {
  FollowOut out;
  if (in.want.kind == ScreenWant::Kind::kFull || in.low_power || in.camera_hz <= 0.0 ||
      in.modes.empty())
    return out;
  bool matched = false;
  int mode = pick_display_mode(in.modes, in.want, in.camera_hz, &matched);
  if (mode < 0) return out;
  const int n = static_cast<int>(in.modes.size());
  if (in.running >= 0 && in.running < n && in.running != mode) {
    const ModeCand& r = in.modes[static_cast<size_t>(in.running)];
    const ModeCand& p = in.modes[static_cast<size_t>(mode)];
    if (!r.interlaced && r.w == p.w && r.h == p.h &&
        std::fabs(mode_hz(r) - in.camera_hz) <= kModeMatchHz)
      mode = in.running;
  }
  out.mode = mode;
  out.rebuild = mode != in.running;
  out.mismatch = !matched;
  out.shown_hz = mode_hz(in.modes[static_cast<size_t>(mode)]);
  return out;
}

}  // namespace maburplay

#endif  // MABUR_PLAYER_DISPLAY_FOLLOW_H_
