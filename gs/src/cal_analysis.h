#pragma once
// Per-rate wall finding for the TX-power calibration kit. Pure: no I/O, no
// state, no clock -- one row of measured cells in, one wall out.
// Spec: docs/superpowers/specs/2026-09-10-tx-power-calibration-design.md.
#include <array>
#include <cstdint>
#include <vector>

namespace maburgs {

// Sentinel for "this card measured no RSSI in this cell".
constexpr int kRssiNone = -999;

// One (rate, idx) measurement. `expected` comes from the plan the GS itself
// sent, never from anything the drone reports -- so a drone that dies
// mid-phase reads as loss rather than as a shrinking denominator.
struct CalCell {
  uint8_t idx = 0;
  uint16_t expected = 0;
  std::array<uint16_t, 2> received{};   // per card, CRC-clean and parseable
  uint16_t corrupt = 0;                 // CRC-bad, diagnostics only
  std::array<int, 2> rssi_dbm{};        // per-card median
  std::array<bool, 2> have_rssi{};
};

enum CalFlag : uint32_t {
  kCalNoDip = 1u << 0,          // never fell below threshold; wall = knee
  kCalUndetermined = 1u << 1,   // never reached threshold; no wall derivable
  kCalNarrow = 1u << 2,         // usable window too thin to trust
  kCalSaturated = 1u << 3,      // RX front-end compression suspected
  kCalCardDisagree = 1u << 4,   // per-card walls differ materially
  kCalDrift = 1u << 5,          // coarse and fine disagree (set by CalSession)
};

struct CalThresholds {
  double deliver_pct = 90.0;    // the >=90% in "first contiguous >=90% run"
  double sat_rssi_dbm = -45.0;  // above this, suspect RX saturation
  int narrow_span = 4;          // wall - floor <= this => kCalNarrow
  int card_disagree = 2;        // per-card wall gap beyond this => flag
  double knee_tol_db = 1.0;     // within this of peak RSSI => on the ceiling
};

struct RateWall {
  int wall = -1;       // -1 = undetermined; caller leaves config untouched
  int floor_idx = -1;  // lowest index that reached threshold
  int best_card = -1;
  uint32_t flags = 0;
};

// `cells` must be sorted ascending by idx. Returns the wall as the END OF THE
// FIRST CONTIGUOUS run at or above `deliver_pct`, scanned upward from the
// sensitivity-floor edge -- never "the last index that happened to be good",
// which the measured comb makes meaningless.
RateWall analyze_rate(const std::vector<CalCell>& cells,
                      const CalThresholds& th);

}  // namespace maburgs
