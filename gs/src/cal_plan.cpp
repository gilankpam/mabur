#include "cal_plan.h"

#include <algorithm>

namespace maburgs {

mabur::rc::CalCmd make_coarse_plan(uint32_t vtx_id, uint32_t nonce) {
  mabur::rc::CalCmd c;
  c.vtx_id = vtx_id;
  c.nonce = nonce;
  c.phase = mabur::cal::kPhaseCoarse;
  c.frames_per_cell = kCoarseFrames;
  c.settle_ms = kSettleMs;
  c.gap_us = kGapUs;
  for (uint8_t r = 0; r < 8; ++r)
    c.windows.push_back({r, 0, 124, kCoarseStep});
  return c;
}

mabur::rc::CalCmd make_fine_plan(uint32_t vtx_id, uint32_t nonce,
                                 const std::array<RateWall, 8>& coarse) {
  mabur::rc::CalCmd c;
  c.vtx_id = vtx_id;
  c.nonce = nonce;
  c.phase = mabur::cal::kPhaseFine;
  c.frames_per_cell = kFineFrames;
  c.settle_ms = kSettleMs;
  c.gap_us = kGapUs;
  for (uint8_t r = 0; r < 8; ++r) {
    const auto& w = coarse[r];
    if (w.flags & (kCalNoDip | kCalUndetermined)) continue;
    if (w.wall < 0) continue;
    const int lo = std::max(0, w.wall - kFineHalfWidth);
    const int hi = std::min(127, w.wall + kFineHalfWidth);
    c.windows.push_back({r, static_cast<uint8_t>(lo),
                         static_cast<uint8_t>(hi), 1});
  }
  return c;
}

mabur::rc::CalCmd make_verify_plan(uint32_t vtx_id, uint32_t nonce,
                                   const std::array<int, 8>& park_idx) {
  mabur::rc::CalCmd c;
  c.vtx_id = vtx_id;
  c.nonce = nonce;
  c.phase = mabur::cal::kPhaseVerify;
  c.frames_per_cell = kVerifyFrames;
  c.settle_ms = kSettleMs;
  c.gap_us = kGapUs;
  for (uint8_t r = 0; r < 8; ++r) {
    const int idx = park_idx[r];
    if (idx < 0 || idx > 127) continue;
    c.windows.push_back({r, static_cast<uint8_t>(idx),
                         static_cast<uint8_t>(idx), 1});
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
