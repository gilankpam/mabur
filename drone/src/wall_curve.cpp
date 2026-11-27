#include "wall_curve.h"

#include <algorithm>
#include <cmath>

namespace mabur {

namespace {
// EVM dB thresholds (LDPC with STBC); MCS0-2 have none.
constexpr float kThreshold[8] = {0, 0, 0, -9.0f, -11.0f, -15.0f, -15.5f, -17.0f};
constexpr float kNoLdpcPenaltyDb = 4.0f;
}  // namespace

std::optional<float> curve_floor_db(const WallCurve& c) {
  std::optional<float> best;
  for (const auto& v : c.evm_db)
    if (v && (!best || *v < *best)) best = v;
  return best;
}

cal::WallResult check_curve(const WallCurve& c) {
  int n = 0;
  std::optional<float> last;
  for (const auto& v : c.evm_db)
    if (v) { ++n; last = v; }
  if (n < 20) return cal::WallResult::TooFewPoints;
  if (*curve_floor_db(c) > -26.0f) return cal::WallResult::FloorTooHigh;
  if (*last < -11.0f) return cal::WallResult::NotSaturated;
  return cal::WallResult::Ok;
}

std::array<int, 8> walls_from_curve(const WallCurve& c, bool ldpc) {
  std::array<int, 8> walls;
  walls.fill(63);
  // Start at the floor point (first occurrence of the minimum).
  int start = -1;
  for (int k = 0; k < kWallPoints; ++k)
    if (c.evm_db[k] && (start < 0 || *c.evm_db[k] < *c.evm_db[start])) start = k;
  if (start < 0) return walls;

  for (int mcs = 3; mcs < 8; ++mcs) {
    const float t = kThreshold[mcs] - (ldpc ? 0.0f : kNoLdpcPenaltyDb);
    int a = -1;  // previous point with a value
    for (int k = start; k < kWallPoints; ++k) {
      if (!c.evm_db[k]) continue;
      if (a >= 0) {
        const float ea = *c.evm_db[a], eb = *c.evm_db[k];
        if (ea <= t && t < eb) {
          const float ia = static_cast<float>(wall_point_index(a));
          const float ib = static_cast<float>(wall_point_index(k));
          const float crossing = ia + (ib - ia) * (t - ea) / (eb - ea);
          walls[mcs] = std::clamp(static_cast<int>(std::floor(crossing - c.anchor)), -64, 63);
          break;
        }
      }
      a = k;
    }
  }
  return walls;
}

}  // namespace mabur
