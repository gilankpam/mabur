#pragma once
#include <algorithm>
#include <array>
#include <cstdint>

#include "mabur/cal_park.h"
#include "mabur/rc_proto.h"

namespace mabur {

// Relative-wall power plan: every wall is a signed index RELATIVE to the
// chip's own per-channel TXAGC anchor (the efuse reference devourer programs
// into 0x18e8 on each channel set). diff[r] = rel[r] - m parks rate r at
// wall-minus-margin on whatever channel the chip is on, because the chip
// adds the right anchor itself.
// m = round(margin_db * 4) converts dB to the chip's 0.25 dB index steps.
//
// anchor_idx: the chip's own MCS7 reference on the channel the walls were
// measured on (WallCurve::anchor), used only to keep reference + diff <=
// 127 (the vendor driver guarantees that in software; hardware behaviour
// beyond it is undefined). <= 0 means unknown: no cap. The anchor the
// diffs ride on is re-derived by devourer on every channel change, driven
// by the ReApplyTxPower() call maburd makes right after each FastRetune
// (drone/src/maburd/real_actuator.cpp: RealActuator::retune_now_).
//
// One plan per measured (channel, width) (power_tables.h) is the WHOLE of
// mabur's power policy: bring-up programs the start channel and width's
// plan with the global offset zeroed beside it, and the live table
// (LivePowerTable) swaps plans when a rung crosses to the other width and
// on every retune. A RADIO_WALL_MARGN or RADIO_LDPC set rebuilds every plan
// and writes the live one. A wall measurement only records the new plans:
// the run's end programs the link's. Nothing else moves power.
struct PowerPlan {
  int8_t cck;
  int8_t legacy;
  int8_t mcs[8];
};

// margin_steps (the dB->index-step conversion) is in mabur/cal_park.h.

inline PowerPlan make_power_plan(const std::array<int, 8>& walls_rel,
                                  int legacy_wall_rel, double margin_db,
                                  int anchor_idx) {
  const int m = margin_steps(margin_db);
  const int hi =
      anchor_idx > 0 ? std::min(rc::kRelMax, 127 - anchor_idx) : rc::kRelMax;
  auto clamp = [&](int v) {
    return static_cast<int8_t>(std::clamp(v, rc::kRelMin, hi));
  };
  PowerPlan p{};
  for (int r = 0; r < 8; ++r)
    p.mcs[r] = clamp(walls_rel[static_cast<size_t>(r)] - m);
  p.legacy = clamp(legacy_wall_rel - m);
  p.cck = p.legacy;  // cck rides the legacy wall
  return p;
}

}  // namespace mabur
