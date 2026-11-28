#include "wall_curve.h"

#include <algorithm>
#include <cmath>

namespace mabur {

namespace {
// EVM dB thresholds (LDPC with STBC); MCS0-2 have none.
// MCS4's sits 1.5 dB under the EVM its delivery fails at on most channels: its wall lies
// where the PA is compressed and the curve is nearly flat (about 0.15 dB per index), so
// the channel-to-channel spread of that EVM (about 2 dB) is a dozen index steps there,
// while the lost index steps cost almost no radiated power.
constexpr float kThreshold[8] = {0, 0, 0, -9.0f, -12.5f, -15.0f, -15.5f, -17.0f};
constexpr float kNoLdpcPenaltyDb = 4.0f;
// A curve parked on a weak rise reads that much worse than the same chip's usual curve.
constexpr float kWeakParkPenaltyDb = 2.5f;
}  // namespace

std::optional<float> curve_floor_db(const WallCurve& c) {
  std::optional<float> best;
  for (const auto& v : c.evm_db)
    if (v && (!best || *v < *best)) best = v;
  return best;
}

cal::WallResult check_curve(const WallCurve& c) {
  for (const auto& v : c.evm_db)
    if (!v) return cal::WallResult::TooFewPoints;
  if (*c.evm_db.back() < -11.0f) return cal::WallResult::NotSaturated;
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
  // The highest index holding the minimum: the cleanest power the rate can have.
  int clean = start;
  for (int k = start; k < kWallPoints; ++k)
    if (c.evm_db[k] && *c.evm_db[k] == *c.evm_db[start]) clean = k;

  for (int mcs = 3; mcs < 8; ++mcs) {
    const float t = kThreshold[mcs] - (ldpc ? 0.0f : kNoLdpcPenaltyDb) -
                    (c.weak_park ? kWeakParkPenaltyDb : 0.0f);
    if (t < *c.evm_db[start]) {  // under the whole curve: no crossing, take the cleanest point
      walls[mcs] = std::clamp(wall_point_index(clean) - c.anchor, -64, 63);
      continue;
    }
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
