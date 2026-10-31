#pragma once
// Where a verify pass parks each rate, shared by the drone (which actually
// transmits the verify cells, drone/src/cal_link.cpp) and the GS (which
// predicts the same cells to know its listen window, gs/src/cal_plan.cpp).
// Both ends must agree on exactly the same parked index or the GS's verify
// window disagrees with what the drone sends.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "mabur/cal_proto.h"
#include "mabur/rc_proto.h"

namespace mabur {

// The wall margin in the chip's 0.25 dB index steps: what every parked
// index (wall - m) subtracts from its wall.
inline int margin_steps(double margin_db) {
  return static_cast<int>(std::lround(margin_db * 4.0));
}

// One parked index per rate: clamp(wall - margin_steps(margin_db), kRelMin,
// kRelMax) for every wall an apply actually wrote. cal::kWallLeave entries
// (left untouched by the apply) stay cal::kWallLeave -- there is nothing to
// verify for a rate that never got a new wall.
inline std::array<int, 8> park_rel_for(const std::array<int8_t, 8>& walls,
                                        double margin_db) {
  std::array<int, 8> park;
  park.fill(cal::kWallLeave);
  const int m = margin_steps(margin_db);
  for (size_t r = 0; r < 8; ++r) {
    if (walls[r] == cal::kWallLeave) continue;
    park[r] = std::clamp(static_cast<int>(walls[r]) - m, rc::kRelMin, rc::kRelMax);
  }
  return park;
}

}  // namespace mabur
