#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace mabur {

// Wall-equalized power plan: derives per-rate qdB diffs from measured
// clean-air TXAGC ceilings ("walls") so that every rate's effective TXAGC
// index (base_ref_idx + diff[r]) equals walls[r] - margin — every rate parked
// at wall-minus-margin, level-continuous with the power_mode "none" baseline.
// diff[r] = walls[r] - m - base_ref_idx, where m = round(margin_db * 4) is
// the margin converted from dB to the chip's 0.25 dB (qdB) index steps.
//
// This plan is the WHOLE of mabur's power policy. It is programmed once at
// bring-up and the global offset is zeroed once beside it; there is no
// runtime power control left to interact with (no GS-commanded offset, no
// thermal derate — deleted 2026-08-12, spec
// 2026-08-12-constant-txpower-design.md). So the park below is the operating
// power for the life of the process: raising a wall or lowering the margin
// radiates more, permanently, with nothing downstream to pull it back.
struct PowerPlan {
  int8_t cck;
  int8_t legacy;
  int8_t mcs[8];
};

namespace detail {
// The 8822E's per-rate diff field is 7-bit two's complement (devourer's
// pack_rate_diff_word masks each byte & 0x7f before packing), so its valid
// range is [-64, 63], not the full int8 [-128, 127]. A diff outside this
// range would silently wrap on air (e.g. +70 -> -58) with no error. This is
// a defensive belt-and-braces clamp only — config.cpp's radio-section
// validation is the loud layer that refuses to load a config whose diffs
// would land out of range in the first place; this function still always
// returns a value (never throws).
inline int8_t clamp_i8(int v) {
  return static_cast<int8_t>(std::clamp(v, -64, 63));
}
}  // namespace detail

// walls_idx: per-MCS max clean TXAGC index (measured); legacy_wall_idx same
// for the OFDM control rate; base_ref_idx: this unit's efuse reference
// index (the anchor rate's index the diffs are relative to); margin_db:
// uniform safety margin in dB applied to every rate's wall.
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
  return p;
}

}  // namespace mabur
