#include "cal_link.h"

#include <cstdio>

#include "mabur/cal_wire.h"

namespace mabur {

CalWrite cal_write_for(const cal::ApplyCmd& a) {
  CalWrite w;
  w.bw = a.bw;
  for (size_t r = 0; r < 8; ++r) w.eff[r] = a.eff_x1000[r] == 0 ? 0.0 : a.eff_x1000[r] / 1000.0;
  return w;
}

std::optional<cal::Applied> CalApplyLedger::cached(uint32_t nonce, uint8_t seq) const {
  if (!have_nonce_ || nonce != nonce_) return std::nullopt;
  const auto it = by_seq_.find(seq);
  if (it == by_seq_.end()) return std::nullopt;
  return it->second;
}

bool CalApplyLedger::want_backup(uint32_t nonce) const {
  return !(have_nonce_ && nonce == nonce_ && backed_up_);
}

void CalApplyLedger::record(const cal::Applied& a) {
  if (!have_nonce_ || a.nonce != nonce_) {
    have_nonce_ = true;
    nonce_ = a.nonce;
    backed_up_ = false;
    by_seq_.clear();
  }
  if (a.ok) backed_up_ = true;
  by_seq_[a.seq] = a;
}

}  // namespace mabur
