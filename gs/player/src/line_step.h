#pragma once
#include <algorithm>
#include <cmath>

// NUDGE's step: one line at a time, one way only -- frames made longer
// where the screen is the faster than the camera, shorter where it is the
// slower (`shorter`). Either way the drift between them brings the vblank
// back the other way, so it never has to move against it.

namespace maburplay {

struct LineStep {
  int lines;    // this frame's lines over the mode's: -1, 0 or +1
  bool moving;  // carried to the next frame
  bool arrived; // reached the target this frame
};

// One frame of NUDGE. e: the target minus where the vblank is, us, wrapped
// into [-P/2, P/2) (> 0: the vblank early); P the refresh period, line one
// line's time; moving as the last step left it; drift the lines a frame
// the drift moves the vblank back the other way. Stepping against the
// drift gains (line - drift) a frame, the drift alone carries it round at
// drift a frame, so a target x ahead (the way lines move it) is reached
// sooner stepping while x < P * (1 - drift), and sooner waiting past that.
// The worst case either way is P / line frames: one per line of the mode.
// The default, a quarter line, keeps a fixed three quarters of a refresh.
inline LineStep line_step(double e, double P, double line, bool shorter, bool moving,
                          double drift = 0.25) {
  const int s = shorter ? -1 : 1;
  const double d = std::clamp(drift, 0.0, 0.95);
  double x = std::fmod(s * e, P);  // how far to go, the way lines move it
  if (x < 0) x += P;
  if (x >= P * (1 - d)) x -= P;  // nearer by the drift: wait for it
  LineStep r{0, moving, false};
  if (!r.moving && x > 3 * line) r.moving = true;
  if (r.moving && x < 0.5 * line) {
    r.moving = false;
    r.arrived = true;
  }
  r.lines = r.moving && x > 0 ? s : 0;
  return r;
}
}  // namespace maburplay
