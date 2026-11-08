#pragma once

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
// line's time; moving as the last step left it. A vblank a little on the
// drift's side waits for the drift; one far the other way goes round, up to
// a refresh, the one way lines can move it.
inline LineStep line_step(double e, double P, double line, bool shorter, bool moving) {
  const int s = shorter ? -1 : 1;
  double x = s * e;  // how far to go, the way lines move it
  if (x < -P / 4) x += P;
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
