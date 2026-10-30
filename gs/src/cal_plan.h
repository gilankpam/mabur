#pragma once
// Builds the GS's calibration sweep commands (one-pass power, capacity) and
// predicts how long the drone will transmit for a given SweepCmd or verify
// pass. Pure: no state, no I/O.
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "mabur/cal_proto.h"
#include "mabur/rc_proto.h"

namespace maburgs {

// The power/verify sweep is one pass, full resolution, over the whole
// relative index range. kSweepLo is -41 and not the rounder -40 SO THAT THE
// LAST CELL LANDS EXACTLY ON kSweepHi == kRailRel: a no-dip row parks at
// the rail, and the whole argument for the rail over the old RSSI knee is
// that it sits inside territory this very sweep measured -- which is only
// true while the rail IS a swept cell. Move kSweepLo and kSweepHi together
// or that property silently breaks again.
constexpr int kSweepLo = -41, kSweepHi = 63;
static_assert(kSweepHi == mabur::rc::kRailRel,
              "a no-dip row parks at the rail, so the sweep must measure it");

constexpr uint16_t kPowerFrames = 20, kPowerSettleMs = 5, kGapUs = 1000;
constexpr uint16_t kVerifyFrames = 20, kCapDwellMs = 2000, kCapSettleMs = 300;

// One full-resolution pass over every rate at `bw`: no_agg, LDPC/STBC per
// the radio's own flags (the drone stamps those; this plan only names the
// cells).
mabur::cal::SweepCmd make_power_sweep(uint32_t nonce, uint8_t seq, uint8_t bw);

// One capacity dwell per rate named by mcs_mask (all eight by default), at
// the production A-MPDU mode. The drone's capacity engine reads
// SweepCmd::settle_ms as its per-point settle, so this sets settle_ms to
// kCapSettleMs rather than the power/verify kPowerSettleMs.
mabur::cal::SweepCmd make_capacity_sweep(uint32_t nonce, uint8_t seq, uint8_t bw);

// The (rate, parked index) pairs the drone's verify pass will actually fly,
// in rate order. `park_rel` is normally cal::park_rel_for's output (shared
// with the drone, mabur/cal_park.h) -- an entry left at cal::kWallLeave
// means that rate's wall was never written and so has nothing to verify.
std::vector<std::pair<uint8_t, int>> verify_cells(const std::array<int, 8>& park_rel);

// How long the drone transmits for a power or capacity SweepCmd. The GS
// uses this to know when its listen window opens, so it must count cells
// exactly as the drone's sweep engine walks them.
// Power: 8 rates * (idx_hi - idx_lo + 1) cells, each settle_ms + frames_per_cell
//   * gap_us / 1000.
// Capacity: one dwell per rate in mcs_mask, each settle_ms + dwell_ms.
uint32_t sweep_duration_ms(const mabur::cal::SweepCmd& c);

// How long a verify pass over `cells` parked rates takes: cells *
// (kPowerSettleMs + kVerifyFrames * kGapUs / 1000).
uint32_t verify_duration_ms(size_t cells);

}  // namespace maburgs
