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
};

std::optional<float> curve_floor_db(const WallCurve& c);   // the best (lowest) point
// Ok, TooFewPoints, FloorTooHigh or NotSaturated, tested in that order.
cal::WallResult check_curve(const WallCurve& c);
// Per-MCS wall relative to the anchor, clamped to [-64, 63]; 63 = no crossing / no threshold.
std::array<int, 8> walls_from_curve(const WallCurve& c, bool ldpc);

}  // namespace mabur
