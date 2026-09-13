#pragma once
// Builds the three calibration sweep plans. Pure: no state, no I/O.
// Spec: docs/superpowers/specs/2026-09-10-tx-power-calibration-design.md.
#include <array>
#include <cstdint>

#include "cal_analysis.h"
#include "mabur/rc_proto.h"

namespace maburgs {

// The coarse grid is [kCoarseLo, kCoarseHi] step kCoarseStep: -41, -37,
// ..., 59, 63. kCoarseLo is -41 and not the rounder -40 SO THAT THE LAST
// CELL LANDS EXACTLY ON kRailRel (63). A row that never dips parks at the
// rail (gs/src/cal_analysis.cpp), and the whole argument for the rail over
// the old RSSI knee is that it sits inside territory this very run measured
// at >=90% delivery -- which is only true while the rail IS a swept cell.
// With -40 the top cell was 60 and the rail sat three indices past the top
// of the sweep, unmeasured. Move kCoarseLo and kCoarseHi together or that
// property silently breaks again.
constexpr int kCoarseLo = -41;
constexpr int kCoarseHi = 63;
constexpr uint8_t kCoarseStep = 4;
static_assert((kCoarseHi - kCoarseLo) % kCoarseStep == 0,
              "the coarse grid must land on kCoarseHi exactly");
static_assert(kCoarseHi == mabur::rc::kRailRel,
              "a no-dip row parks at kRailRel, so the sweep must measure it");
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

// One cell per rate at its parked index (wall - margin). kNoWall or a park
// index out of [-64,63] means that rate was undetermined and is skipped.
mabur::rc::CalCmd make_verify_plan(uint32_t vtx_id, uint32_t nonce,
                                   const std::array<int, 8>& park_idx);

// How long the drone will transmit for this plan. The GS uses it to know when
// the listen window opens, so it must count cells exactly as CalSweep walks
// them.
uint32_t plan_duration_ms(const mabur::rc::CalCmd& c);

}  // namespace maburgs
