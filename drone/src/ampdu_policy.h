#pragma once
// Per-rung A-MPDU policy. The 2026-09-17 saturation sweep
// (docs/bandwidth-sweep-findings-2026-09-17.md) measured the prod agg6
// aggregation DELIVERING LESS than QoS-Data singles at mcs0-3 (0.87/0.78/
// 0.71/0.76 of nominal vs 0.93/0.88/0.84/0.80) and more at mcs5+ (0.76 vs
// 0.68/0.63): a ~100 us fixed per-PPDU cost the aggregation mode pays even
// when aggregates barely form, amortised only once the MPDUs are short
// enough to pack six per PPDU. The rung-pinned A/B (sessions 0121-0128)
// found singles never worse on fec/air/e2e at rungs 0-3. So the mode
// follows the op's MCS: below ampdu.min_mcs the chip flies singles, at and
// above it the configured aggregate. devourer's SetAmpduMode is a live
// switch (per-frame descriptor half + one 0x455 timer write), so the
// actuator flips it on the agent thread as the ladder moves.
#include <cstdint>

#include "AmpduMode.h"
#include "config.h"

namespace mabur {

inline devourer::AmpduMode ampdu_mode_for(const AmpduCfg& c, uint8_t mcs) {
  devourer::AmpduMode m;  // enabled=false: singles, the chip's post-InitWrite state
  if (c.max_num <= 0 || mcs < c.min_mcs) return m;
  m.enabled = true;
  m.tid = 0;
  m.max_num = static_cast<uint8_t>(c.max_num);
  m.density = 7;
  m.no_ack = true;
  m.max_time = static_cast<uint8_t>(c.max_time);
  return m;
}

// Field-wise equality on the halves the actuator programs; the actuator
// writes the chip only across a change.
inline bool ampdu_mode_same(const devourer::AmpduMode& a, const devourer::AmpduMode& b) {
  if (a.enabled != b.enabled) return false;
  if (!a.enabled) return true;
  return a.tid == b.tid && a.max_num == b.max_num && a.density == b.density &&
         a.no_ack == b.no_ack && a.max_time == b.max_time;
}

}  // namespace mabur
