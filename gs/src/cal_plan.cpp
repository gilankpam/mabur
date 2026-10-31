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

}  // namespace maburgs
