#pragma once
#include <cstdint>
#include <optional>

#include "NoiseFloorMath.h"   // devourer::nf::kNhmAbsThDbm
#include "scout_radio.h"

namespace maburgs {

// Busy-airtime share of one NHM window (spec 2026-09-25-nhm-airtime §6):
// the buckets whose LOWER edge is >= busy_dbm over all buckets. Bucket 0
// sits below th[0]; bucket i (1..11) starts at th[i-1].
inline bool busy_dbm_is_edge(int dbm) {
  for (int8_t e : devourer::nf::kNhmAbsThDbm) if (e == dbm) return true;
  return false;
}
inline std::optional<double> nhm_busy_pct(const NhmBusyRead& r, int busy_dbm) {
  if (!r.valid) return std::nullopt;
  int first = -1;
  for (int i = 0; i < 11; ++i)
    if (devourer::nf::kNhmAbsThDbm[i] == busy_dbm) { first = i + 1; break; }
  if (first < 0) return std::nullopt;
  uint32_t all = 0, above = 0;
  for (int i = 0; i < 12; ++i) {
    all += r.buckets[i];
    if (i >= first) above += r.buckets[i];
  }
  if (all == 0) return std::nullopt;   // ready but empty: no reading, not "0 % busy"
  return 100.0 * above / all;
}
inline uint16_t nhm_period_4us(int ms) {
  const long v = static_cast<long>(ms) * 250;
  return static_cast<uint16_t>(v < 1 ? 1 : v > 65535 ? 65535 : v);
}

}  // namespace maburgs
