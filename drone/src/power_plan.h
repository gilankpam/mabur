#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace mabur {

// Wall-equalized power plan: derives per-rate qdB diffs from measured
// clean-air TXAGC ceilings ("walls") so that, at offset 0, every rate's
// effective TXAGC index (base_ref_idx + diff[r]) equals walls[r] - margin —
// every rate parked at wall-minus-margin, level-continuous with today's
// baseline. diff[r] = walls[r] - m - base_ref_idx, where m = round(margin_db
// * 4) is the margin in qdB steps. max_offset_qdb is always 0 in this
// formulation: offset 0 already sits every rate at its equalized ceiling, so
// the controller only ever backs off (offset <= 0), scaling every rate down
// uniformly by the commanded amount.
struct PowerPlan {
  int8_t cck;
  int8_t legacy;
  int8_t mcs[8];
  int max_offset_qdb;
};

namespace detail {
inline int8_t clamp_i8(int v) {
  return static_cast<int8_t>(std::clamp(v, -128, 127));
}
}  // namespace detail

// walls_idx: per-MCS max clean TXAGC index (measured); legacy_wall_idx same
// for the OFDM control rate; base_ref_idx: this unit's efuse reference
// index (the anchor rate's index at offset 0); margin_db: uniform safety
// margin applied to every rate's wall.
inline PowerPlan make_power_plan(const std::array<int, 8>& walls_idx,
                                  int legacy_wall_idx, int base_ref_idx,
                                  double margin_db) {
  const int m = static_cast<int>(std::lround(margin_db * 4.0));
  PowerPlan p{};
  for (int r = 0; r < 8; ++r) {
    p.mcs[r] = detail::clamp_i8(walls_idx[static_cast<size_t>(r)] - m - base_ref_idx);
  }
  p.legacy = detail::clamp_i8(legacy_wall_idx - m - base_ref_idx);
  p.cck = p.legacy;  // v1: cck rides the legacy wall
  p.max_offset_qdb = 0;
  return p;
}

}  // namespace mabur
