#pragma once
// Per-rate wall finding for the TX-power calibration kit. Pure: no I/O, no
// state, no clock -- one row of measured cells in, one wall out.
#include <array>
#include <cstdint>
#include <vector>

#include "mabur/rc_proto.h"

namespace maburgs {

// Sentinel for "this card measured no RSSI in this cell".
constexpr int kRssiNone = -999;

// Indices are RELATIVE to the drone chip's per-channel anchor, in the 7-bit
// signed diff-field range. -1 is a legal index.
constexpr int kNoWall = -128;   // undetermined: caller leaves config untouched
static_assert(kNoWall == mabur::rc::kWallUndetermined, "one sentinel");
constexpr int kRailRel = mabur::rc::kRailRel;  // a no-dip row parks at the field's top

// One (rate, idx) measurement. `expected` comes from the plan the GS itself
// sent, never from anything the drone reports -- so a drone that dies
// mid-phase reads as loss rather than as a shrinking denominator.
struct CalCell {
  int idx = 0;
  uint16_t expected = 0;
  std::array<uint16_t, 2> received{};   // per card, CRC-clean and parseable
  uint16_t corrupt = 0;                 // CRC-bad, diagnostics only
  std::array<int, 2> rssi_dbm{};        // per-card median
  std::array<bool, 2> have_rssi{};
};

enum CalFlag : uint32_t {
  kCalNoDip = 1u << 0,          // never fell below threshold; wall = the rail
  kCalUndetermined = 1u << 1,   // never reached threshold; no wall derivable
  kCalNarrow = 1u << 2,         // usable window too thin to trust
  kCalSaturated = 1u << 3,      // RX front-end compression suspected
  kCalCardDisagree = 1u << 4,   // per-card walls differ materially
  // 1u << 5 (32, drift) retired -- never reused.
  kCalTxBound = 1u << 6,        // sender rarely blocked: measures CPU/USB, not air
  kCalLossy = 1u << 7,          // too few frames arrived to trust the count
  kCalLost = 1u << 8,           // under half arrived: measured the loss, not written
};

struct CalThresholds {
  double deliver_pct = 90.0;    // the >=90% in "first contiguous >=90% run"
  double sat_rssi_dbm = -45.0;  // above this, suspect RX saturation
  int narrow_span = 4;          // wall - floor <= this => kCalNarrow
  int card_disagree = 2;        // per-card wall gap beyond this => flag
  int tx_bound_blocked_pct = 50;  // blocked_pct below this => kCalTxBound
  double lossy_ratio = 0.9;       // best-card frames below this * offered => kCalLossy
  double lost_ratio = 0.5;        // ...and below this * offered => kCalLost too
};

struct RateWall {
  int wall = kNoWall;       // kNoWall = undetermined; caller leaves config untouched
  int floor_idx = kNoWall;  // lowest index that reached threshold, kNoWall if none
  int best_card = -1;
  uint32_t flags = 0;
};

// `cells` must be sorted ascending by idx. Returns the wall as the END OF THE
// FIRST CONTIGUOUS run at or above `deliver_pct`, scanned upward from the
// sensitivity-floor edge -- never "the last index that happened to be good",
// which the measured comb makes meaningless.
RateWall analyze_rate(const std::vector<CalCell>& cells,
                      const CalThresholds& th);

// One capacity dwell's tally at a single (mcs, bw) operating point, flown at
// the production A-MPDU mode rather than the no_agg power/verify frames.
// `offered` is the frame count the GS plan itself committed to sending, the
// same expected-side-channel discipline as CalCell::expected. `blocked_pct`
// is the share of the dwell the sender spent blocked waiting for a TX
// opportunity -- the air being the bottleneck, not the drone's CPU/USB.
struct CapCell {
  uint32_t offered = 0;
  uint8_t blocked_pct = 0;
  std::array<uint32_t, 2> frames{};
  std::array<uint64_t, 2> air_bits{};
  std::array<int, 2> rssi_dbm{};
  std::array<bool, 2> have_rssi{};
};

struct CapResult {
  double eff = 0.0;    // delivered air bits / nominal PHY bits for the dwell
  int best_card = -1;
  uint32_t flags = 0;
};

// Delivered air efficiency for one capacity dwell at (mcs, bw): the best
// card's air bits over the dwell's nominal PHY capacity at that rate,
// rounded to 0.01 and clamped to [0.05, 1.0] -- never zero, which on the
// wire means "leave the config's entry". Flags a dwell that measured the
// drone's CPU/USB rather than the air (kCalTxBound: the sender was rarely
// blocked, so more air was available than the drone could feed it), one
// whose frame count arrived too thin to trust (kCalLossy), and one where
// under half of what was offered arrived (kCalLost, with kCalLossy): that
// dwell measured the loss, not the air, and like a tx_bound point is not
// written.
CapResult analyze_capacity(const CapCell& c, uint8_t mcs, uint8_t bw,
                           uint32_t dwell_ms, const CalThresholds& th);

}  // namespace maburgs
