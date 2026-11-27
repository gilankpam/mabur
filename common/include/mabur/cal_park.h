#pragma once
// The wall margin in TXAGC index steps, shared by the drone's power plan
// and any GS that reasons about the same walls.
#include <cmath>

namespace mabur {

// The wall margin in the chip's 0.25 dB index steps: what every flown
// index subtracts from its wall.
inline int margin_steps(double margin_db) {
  return static_cast<int>(std::lround(margin_db * 4.0));
}

}  // namespace mabur
