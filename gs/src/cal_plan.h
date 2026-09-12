#pragma once
// Builds the three calibration sweep plans. Pure: no state, no I/O.
// Spec: docs/superpowers/specs/2026-09-10-tx-power-calibration-design.md.
#include <array>
#include <cstdint>

#include "cal_analysis.h"
#include "mabur/rc_proto.h"

namespace maburgs {

constexpr uint8_t kCoarseStep = 4;
constexpr int kFineHalfWidth = 8;
constexpr uint16_t kCoarseFrames = 20;
constexpr uint16_t kFineFrames = 100;
constexpr uint16_t kVerifyFrames = 100;
constexpr uint16_t kSettleMs = 100;   // MEASURE THIS ON HARDWARE -- see docs
constexpr uint16_t kGapUs = 2000;

mabur::rc::CalCmd make_coarse_plan(uint32_t vtx_id, uint32_t nonce);

// Only rows whose coarse pass found a real dip get refined. A kCalNoDip row's
// wall came from the RSSI knee on a flat ceiling, where +/-2 indices costs no
// measurable power; a kCalUndetermined row has nothing to refine.
mabur::rc::CalCmd make_fine_plan(uint32_t vtx_id, uint32_t nonce,
                                 const std::array<RateWall, 8>& coarse);

// One cell per rate at its parked index (wall - margin). A negative park
// index means that rate was undetermined and is skipped.
mabur::rc::CalCmd make_verify_plan(uint32_t vtx_id, uint32_t nonce,
                                   const std::array<int, 8>& park_idx);

// How long the drone will transmit for this plan. The GS uses it to know when
// the listen window opens, so it must count cells exactly as CalSweep walks
// them.
uint32_t plan_duration_ms(const mabur::rc::CalCmd& c);

}  // namespace maburgs
