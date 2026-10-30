#include "cal_plan.h"

#include <algorithm>
#include <bitset>

namespace maburgs {

mabur::cal::SweepCmd make_power_sweep(uint32_t nonce, uint8_t seq, uint8_t bw) {
  mabur::cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = mabur::cal::Kind::Power;
  c.bw = bw;
  c.frames_per_cell = kPowerFrames;
  c.settle_ms = kPowerSettleMs;
  c.gap_us = kGapUs;
  c.idx_lo = static_cast<int8_t>(kSweepLo);
  c.idx_hi = static_cast<int8_t>(kSweepHi);
  return c;
}

mabur::cal::SweepCmd make_capacity_sweep(uint32_t nonce, uint8_t seq, uint8_t bw) {
  mabur::cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = mabur::cal::Kind::Capacity;
  c.bw = bw;
  c.settle_ms = kCapSettleMs;
  c.dwell_ms = kCapDwellMs;
  return c;
}

std::vector<std::pair<uint8_t, int>> verify_cells(const std::array<int, 8>& park_rel) {
  std::vector<std::pair<uint8_t, int>> out;
  for (uint8_t r = 0; r < 8; ++r) {
    if (park_rel[r] == mabur::cal::kWallLeave) continue;
    out.emplace_back(r, park_rel[r]);
  }
  return out;
}

uint32_t sweep_duration_ms(const mabur::cal::SweepCmd& c) {
  constexpr uint32_t kRates = 8;
  if (c.kind == mabur::cal::Kind::Capacity) {
    const uint32_t rows = std::bitset<8>(c.mcs_mask).count();
    return rows * (static_cast<uint32_t>(c.settle_ms) + c.dwell_ms);
  }
  // Power (and a Verify-kind SweepCmd swept the same uniform way): one
  // pass across all eight rates at every index from idx_lo to idx_hi.
  const int cells_per_row = c.idx_hi - c.idx_lo + 1;
  const uint32_t cells = kRates * static_cast<uint32_t>(std::max(0, cells_per_row));
  const uint32_t per_cell_ms =
      c.settle_ms + static_cast<uint32_t>(c.frames_per_cell) * c.gap_us / 1000u;
  return cells * per_cell_ms;
}

uint32_t verify_duration_ms(size_t cells) {
  const uint32_t per_cell_ms =
      kPowerSettleMs + static_cast<uint32_t>(kVerifyFrames) * kGapUs / 1000u;
  return static_cast<uint32_t>(cells) * per_cell_ms;
}

// --- Legacy shim -------------------------------------------------------
// Values and behaviour unchanged from the pre-revamp coarse/fine/verify
// plan; kept only so gs/src/cal_session.cpp (rewritten in a later task)
// keeps compiling. Not exported from cal_plan.h as named constants since
// several of them (kVerifyFrames, kGapUs) clash in value with the new
// one-pass plan's constants of the same name.
namespace {
constexpr int kLegacyCoarseLo = -41;
constexpr int kLegacyCoarseHi = 63;
constexpr uint8_t kLegacyCoarseStep = 4;
constexpr int kLegacyFineHalfWidth = 8;
constexpr uint16_t kLegacyCoarseFrames = 20;
constexpr uint16_t kLegacyFineFrames = 100;
constexpr uint16_t kLegacyVerifyFrames = 100;
constexpr uint16_t kLegacySettleMs = 100;
constexpr uint16_t kLegacyGapUs = 2000;
}  // namespace

mabur::rc::CalCmd make_coarse_plan(uint32_t nonce) {
  mabur::rc::CalCmd c;
  c.nonce = nonce;
  c.phase = 1;  // coarse
  c.frames_per_cell = kLegacyCoarseFrames;
  c.settle_ms = kLegacySettleMs;
  c.gap_us = kLegacyGapUs;
  for (uint8_t r = 0; r < 8; ++r)
    c.windows.push_back({r, static_cast<int8_t>(kLegacyCoarseLo),
                         static_cast<int8_t>(kLegacyCoarseHi), kLegacyCoarseStep});
  return c;
}

mabur::rc::CalCmd make_fine_plan(uint32_t nonce,
                                 const std::array<RateWall, 8>& coarse) {
  mabur::rc::CalCmd c;
  c.nonce = nonce;
  c.phase = 2;  // fine
  c.frames_per_cell = kLegacyFineFrames;
  c.settle_ms = kLegacySettleMs;
  c.gap_us = kLegacyGapUs;
  for (uint8_t r = 0; r < 8; ++r) {
    const auto& w = coarse[r];
    if (w.flags & (kCalNoDip | kCalUndetermined)) continue;
    if (w.wall == kNoWall) continue;
    const int lo = std::max(mabur::rc::kRelMin, w.wall - kLegacyFineHalfWidth);
    const int hi = std::min(mabur::rc::kRelMax, w.wall + kLegacyFineHalfWidth);
    c.windows.push_back({r, static_cast<int8_t>(lo),
                         static_cast<int8_t>(hi), 1});
  }
  return c;
}

mabur::rc::CalCmd make_verify_plan(uint32_t nonce,
                                   const std::array<int, 8>& park_idx) {
  mabur::rc::CalCmd c;
  c.nonce = nonce;
  c.phase = 3;  // verify
  c.frames_per_cell = kLegacyVerifyFrames;
  c.settle_ms = kLegacySettleMs;
  c.gap_us = kLegacyGapUs;
  for (uint8_t r = 0; r < 8; ++r) {
    const int idx = park_idx[r];
    if (idx == kNoWall || idx < mabur::rc::kRelMin || idx > mabur::rc::kRelMax)
      continue;
    c.windows.push_back({r, static_cast<int8_t>(idx),
                         static_cast<int8_t>(idx), 1});
  }
  return c;
}

uint32_t plan_duration_ms(const mabur::rc::CalCmd& c) {
  uint32_t cells = 0;
  for (const auto& w : c.windows) {
    if (w.idx_step == 0) continue;
    for (int i = w.idx_lo; i <= w.idx_hi; i += w.idx_step) ++cells;
  }
  const uint32_t per_cell_ms =
      c.settle_ms +
      static_cast<uint32_t>(c.frames_per_cell) * c.gap_us / 1000u;
  return cells * per_cell_ms;
}

}  // namespace maburgs
