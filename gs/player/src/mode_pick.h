#ifndef MABUR_PLAYER_MODE_PICK_H_
#define MABUR_PLAYER_MODE_PICK_H_

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "vblank_estimator.h"  // mode_period_us

namespace maburplay {

// screen_mode, one of three forms:
//  - "auto": follow the camera's rate at the largest size the display
//    offers at it (kAuto);
//  - "WIDTHxHEIGHT": follow the camera's rate at this size (kSize);
//  - "WIDTHxHEIGHT@RATE", RATE an integer or a decimal (59.94): fully
//    pinned, the camera's rate aside (kFull).
struct ScreenWant {
  enum class Kind { kAuto, kSize, kFull } kind = Kind::kFull;
  int w = 0, h = 0;
  double hz = 0.0;
  bool ok = false;
};

inline ScreenWant parse_screen_want(const std::string& s) {
  ScreenWant r;
  if (s == "auto") {
    r.kind = ScreenWant::Kind::kAuto;
    r.ok = true;
    return r;
  }
  int n = 0;
  if (std::sscanf(s.c_str(), "%dx%d@%lf%n", &r.w, &r.h, &r.hz, &n) == 3 &&
      n == static_cast<int>(s.size()) && r.w > 0 && r.h > 0 && r.hz > 0.0) {
    r.ok = true;
    return r;
  }
  r = ScreenWant{};
  n = 0;
  if (std::sscanf(s.c_str(), "%dx%d%n", &r.w, &r.h, &n) == 2 &&
      n == static_cast<int>(s.size()) && r.w > 0 && r.h > 0) {
    r.kind = ScreenWant::Kind::kSize;
    r.ok = true;
    return r;
  }
  return ScreenWant{};
}

// One connector mode, reduced to what the pick needs (no DRM types, so the
// rule is host-testable).
struct ModeCand {
  int w = 0;
  int h = 0;
  int clock_khz = 0;
  int htotal = 0;
  int vtotal = 0;
  bool interlaced = false;
  bool preferred = false;
};

// The mode's exact refresh rate: DRM's integer vrefresh says "60" for both
// 60.000 and 59.94 Hz.
inline double mode_hz(const ModeCand& m) {
  return 1e6 / mode_period_us(m.clock_khz, m.htotal, m.vtotal);
}

constexpr double kModeMatchHz = 0.5;

// The preferred non-interlaced mode, else the first; -1 for an empty list.
inline int preferred_mode(const std::vector<ModeCand>& modes) {
  for (size_t i = 0; i < modes.size(); ++i)
    if (!modes[i].interlaced && modes[i].preferred) return static_cast<int>(i);
  return modes.empty() ? -1 : 0;
}

// A fully pinned want (kFull): among non-interlaced modes of the wanted size whose exact rate is within
// kModeMatchHz of the wanted rate, the closest (first on a tie): a sink that
// lists 60.000 and 59.94 under "@60" gets 60.000, "@59.94" gets 59.94, and a
// 59.94-only sink still matches "@60" -- the phase lock steers the camera to
// whichever rate the panel runs. No match: the preferred non-interlaced mode,
// else the first. -1 for an empty list.
inline int pick_mode(const std::vector<ModeCand>& modes, const ScreenWant& want,
                     bool* matched) {
  *matched = false;
  int best = -1;
  double best_d = 0.0;
  if (want.ok && want.kind == ScreenWant::Kind::kFull) {
    for (size_t i = 0; i < modes.size(); ++i) {
      const ModeCand& m = modes[i];
      if (m.interlaced || m.w != want.w || m.h != want.h) continue;
      const double d = std::fabs(mode_hz(m) - want.hz);
      if (d > kModeMatchHz) continue;
      if (best < 0 || d < best_d) {
        best = static_cast<int>(i);
        best_d = d;
      }
    }
  }
  if (best >= 0) {
    *matched = true;
    return best;
  }
  return preferred_mode(modes);
}

// Whether `m` can carry a following want: progressive, and of the wanted
// size for kSize.
inline bool fits_want(const ModeCand& m, const ScreenWant& want) {
  return !m.interlaced &&
         (want.kind != ScreenWant::Kind::kSize || (m.w == want.w && m.h == want.h));
}

// The display mode for `want` with the camera running at `camera_hz` (0 =
// not measured yet). kFull is pick_mode's exact-rate rule. kAuto and kSize
// follow the camera: the candidates are the fitting modes (fits_want)
// within kModeMatchHz of the camera's rate, and *matched says whether there
// are any. kAuto takes the largest w*h among them (a tie: the closest
// rate), kSize the closest rate. With none, the display cannot show that
// rate: the same rule runs over the modes at the display's rate nearest
// the camera's -- the fitting mode closest in rate (an exact tie, a camera
// midway between two rates: the higher one) and every fitting mode within
// kModeMatchHz of it, so a 99.98 and a 100.000 Hz timing are one rate and
// a 60 and a 120 Hz mode never share a group. A kSize the display has at
// no rate at all takes the preferred mode. An unknown camera rate: kAuto
// takes the preferred mode; kSize that size at the rate closest to the
// preferred mode's (*matched), else the preferred mode. -1 for an empty
// list.
inline int pick_display_mode(const std::vector<ModeCand>& modes, const ScreenWant& want,
                             double camera_hz, bool* matched) {
  *matched = false;
  if (want.kind == ScreenWant::Kind::kFull) return pick_mode(modes, want, matched);
  if (modes.empty()) return -1;
  const int pref = preferred_mode(modes);
  // The rate everything is judged against: the camera's, or with that not
  // known yet the preferred mode's.
  const double target = camera_hz > 0.0 ? camera_hz : mode_hz(modes[static_cast<size_t>(pref)]);
  if (camera_hz <= 0.0 && want.kind == ScreenWant::Kind::kAuto) {
    *matched = true;
    return pref;
  }
  // The fitting mode nearest the target in rate; an exact tie goes to the
  // higher rate.
  int nearest = -1;
  double nearest_d = 0.0;
  for (size_t i = 0; i < modes.size(); ++i) {
    if (!fits_want(modes[i], want)) continue;
    const double hz = mode_hz(modes[i]);
    const double d = std::fabs(hz - target);
    if (nearest < 0 || d < nearest_d ||
        (d == nearest_d && hz > mode_hz(modes[static_cast<size_t>(nearest)]))) {
      nearest = static_cast<int>(i);
      nearest_d = d;
    }
  }
  if (nearest < 0) return pref;  // no mode of that size (or none progressive)
  *matched = camera_hz <= 0.0 || nearest_d <= kModeMatchHz;
  // The group: within kModeMatchHz of the target when the display has its
  // rate, else within kModeMatchHz of the nearest rate it has.
  const double centre =
      nearest_d <= kModeMatchHz ? target : mode_hz(modes[static_cast<size_t>(nearest)]);
  int best = -1;
  long best_area = 0;
  double best_d = 0.0;
  for (size_t i = 0; i < modes.size(); ++i) {
    const ModeCand& m = modes[i];
    if (!fits_want(m, want)) continue;
    const double hz = mode_hz(m);
    if (std::fabs(hz - centre) > kModeMatchHz) continue;
    const double d = std::fabs(hz - target);
    const long area = static_cast<long>(m.w) * m.h;
    if (best < 0 || area > best_area || (area == best_area && d < best_d)) {
      best = static_cast<int>(i);
      best_area = area;
      best_d = d;
    }
  }
  return best;
}

// Whether the display can honour `want` with the camera at `camera_hz`:
// kAuto always; kFull by pick_mode's exact-rate rule; kSize when that size
// runs at the camera's rate, or -- the rate not known yet -- at any rate.
inline bool want_offered(const std::vector<ModeCand>& modes, const ScreenWant& want,
                         double camera_hz) {
  if (!want.ok) return false;
  if (want.kind == ScreenWant::Kind::kAuto) return true;
  bool matched = false;
  pick_display_mode(modes, want, camera_hz, &matched);
  return matched;
}

// "WxH@R" with R to the millihertz: parse_screen_want reads it back as a
// kFull want that pick_mode matches to this very mode (within kModeMatchHz,
// the closest rate).
inline std::string mode_string(const ModeCand& m) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%dx%d@%.3f", m.w, m.h, mode_hz(m));
  return buf;
}

}  // namespace maburplay

#endif  // MABUR_PLAYER_MODE_PICK_H_
