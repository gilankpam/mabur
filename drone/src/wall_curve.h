#pragma once
#include "mabur/cal_proto.h"

#include <array>
#include <optional>

namespace mabur {

constexpr int kWallIdxLo = 28, kWallIdxStep = 4, kWallPoints = 25;
constexpr int wall_point_index(int k) { return kWallIdxLo + k * kWallIdxStep; }

struct WallCurve {
  int anchor = 0;                                          // the chip's MCS7 index, diffs zeroed
  std::array<std::optional<float>, kWallPoints> evm_db{};  // rounded to 0.1 dB
  bool weak_park = false;  // the park check read a weak rise: thresholds are lowered
};

std::optional<float> curve_floor_db(const WallCurve& c);   // the best (lowest) point
// Ok, TooFewPoints (a point without a value) or NotSaturated.
cal::WallResult check_curve(const WallCurve& c);
// Per-MCS wall relative to the anchor, clamped to [-64, 63]; 63 = the curve never rises to the threshold.
std::array<int, 8> walls_from_curve(const WallCurve& c, bool ldpc);

}  // namespace mabur
