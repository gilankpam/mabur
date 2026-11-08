#include <cmath>

#include "mtest.h"
#include "line_step.h"

using maburplay::LineStep;
using maburplay::line_step;

namespace {

double wrap(double x, double P) {  // into [-P/2, P/2)
  x = std::fmod(x, P);
  if (x >= P / 2) x -= P;
  if (x < -P / 2) x += P;
  return x;
}
double mod(double x, double P) {
  x = std::fmod(x, P);
  return x < 0 ? x + P : x;
}

struct Run {
  int min_lines = 0, max_lines = 0;  // the frame lengths used, over the mode's
  double worst_us = 0;               // the largest error once in step, us
  int arrived = 0;                   // frames it reached the target on
};

// A 1080p60 mode (1125 lines) against a camera frame of Pc, for n frames,
// starting a third of a refresh off. Each frame the pictures land Pm - Pc
// earlier in the refresh (later, with the screen the faster); the vblank
// moves by the lines the step asked for.
Run run(double Pc, bool shorter, int n) {
  const double Pm = 16666.667, line = Pm / 1125;
  double target = Pm / 3, G = 0;
  bool moving = false, in_step = false;
  Run r;
  for (int f = 0; f < n; f++) {
    target = mod(target - (Pm - Pc), Pm);
    const LineStep s = line_step(wrap(target - G, Pm), Pm, line, shorter, moving);
    moving = s.moving;
    if (s.arrived) {
      r.arrived++;
      in_step = true;
    }
    if (s.lines < r.min_lines) r.min_lines = s.lines;
    if (s.lines > r.max_lines) r.max_lines = s.lines;
    G = mod(G + s.lines * line, Pm);
    if (in_step) r.worst_us = std::fmax(r.worst_us, std::fabs(wrap(target - G, Pm)));
  }
  return r;
}

}  // namespace

// The screen the faster (a 60.000 Hz mode, a 59.988 fps camera): longer
// frames only, held within a few lines.
TEST(faster_screen_longer_only_holds) {
  const Run r = run(16670.0, false, 20000);
  CHECK(r.arrived >= 1);
  CHECK(r.min_lines == 0 && r.max_lines == 1);
  CHECK(r.worst_us < 4 * 16666.667 / 1125);
}

// The screen the slower: longer frames only never hold it -- the vblank
// goes round a whole refresh again and again.
TEST(slower_screen_longer_only_never_holds) {
  const Run r = run(16658.1, false, 20000);
  CHECK(r.worst_us > 16666.667 / 4);
}

// ... and shorter frames only hold it, within a few lines.
TEST(slower_screen_shorter_holds) {
  const Run r = run(16658.1, true, 20000);
  CHECK(r.arrived >= 1);
  CHECK(r.min_lines == -1 && r.max_lines == 0);
  CHECK(r.worst_us < 4 * 16666.667 / 1125);
}

// A vblank a little on the drift's side waits for the drift; one a little
// the other way goes round, the one way lines can move it.
TEST(step_direction) {
  const double P = 16666.667, line = P / 1125;
  // e is the target less the vblank: > 0, the vblank early. Longer only: a
  // little late waits, early moves it later, far too late goes round.
  CHECK(line_step(-10 * line, P, line, false, false).lines == 0);
  CHECK(line_step(10 * line, P, line, false, false).lines == 1);
  CHECK(line_step(-P / 3, P, line, false, false).lines == 1);
  // Shorter only, the mirror: a little early waits, late moves it earlier,
  // far too early goes round.
  CHECK(line_step(10 * line, P, line, true, false).lines == 0);
  CHECK(line_step(-10 * line, P, line, true, false).lines == -1);
  CHECK(line_step(P / 3, P, line, true, false).lines == -1);
  // Within half a line while moving: there.
  const LineStep s = line_step(-0.2 * line, P, line, true, true);
  CHECK(s.lines == 0 && !s.moving && s.arrived);
}

MTEST_MAIN
