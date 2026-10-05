#pragma once
// A relay sweep entry (mabur-relay v4 SCAN_RESULT) as a ranker visit and a
// scan.log D record (spec 2026-10-05-cpe-relay-hop §4). Pure.
#include <algorithm>
#include <cstdint>

#include "channel_scout.h"
#include "hop_ranker.h"
#include "sweep_types.h"

namespace maburgs {

// fa = OFDM/HT PHY errors, foreign = non-mabur frames, cca = own = 0 (ath9k
// has no CCA-event count; fa + 4*foreign covers WiFi-busy channels). busy is
// the RAW busy % of the observe -- no rx subtraction: none of our frames are
// on a candidate, exactly like the USB ranker's NHM reading.
inline HopVisit sweep_visit(const SweepEntry& e, double t_ms) {
  HopVisit v;
  v.ch = e.ch;
  v.t_ms = t_ms;
  v.fa = e.ofdm_err;
  v.foreign = e.foreign;
  v.src = VisitSrc::Relay;
  if (e.active_ms > 0) {
    v.busy_valid = true;
    v.busy_pct = std::min(100.0, 100.0 * e.busy_ms / e.active_ms);
  }
  return v;
}

inline ScoutDwell sweep_dwell(const SweepEntry& e, uint64_t round) {
  ScoutDwell d;
  d.survey.def.primary = e.ch;
  d.survey.def.width = CHANNEL_WIDTH_20;
  d.survey.round = round;
  d.survey.observe_ms = e.active_ms;
  d.survey.valid_fa = true;
  d.survey.fa_ofdm = e.ofdm_err;
  d.survey.frames = e.foreign;
  d.survey.dvr_frames = 0;
  d.in_session = true;
  if (e.active_ms > 0) {
    d.busy_valid = true;
    d.busy_pct = std::min(100.0, 100.0 * e.busy_ms / e.active_ms);
    d.rx_valid = true;
    d.rx_pct = std::min(100.0, 100.0 * e.rx_ms / e.active_ms);
  }
  return d;
}

}  // namespace maburgs
