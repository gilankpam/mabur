#include "cal_analysis.h"

#include <algorithm>
#include <cstdlib>
#include <utility>

namespace maburgs {
namespace {

double delivery_pct(const CalCell& c, int card) {
  if (c.expected == 0) return 0.0;
  return 100.0 * static_cast<double>(c.received[static_cast<size_t>(card)]) /
         static_cast<double>(c.expected);
}

// End of the first contiguous run at or above threshold, scanning up from the
// first cell that reaches it. Returns {floor_idx, wall}; both -1 when the row
// never reaches threshold at all.
std::pair<int, int> first_run(const std::vector<CalCell>& cells, int card,
                              double pct) {
  size_t i = 0;
  while (i < cells.size() && delivery_pct(cells[i], card) < pct) ++i;
  if (i == cells.size()) return {-1, -1};
  const int floor_idx = static_cast<int>(cells[i].idx);
  int wall = floor_idx;
  while (i < cells.size() && delivery_pct(cells[i], card) >= pct) {
    wall = static_cast<int>(cells[i].idx);
    ++i;
  }
  // i == cells.size() here means the run never ended -- the caller detects
  // that as no-dip by comparing wall against the last measured index.
  return {floor_idx, wall};
}

}  // namespace

RateWall analyze_rate(const std::vector<CalCell>& cells,
                      const CalThresholds& th) {
  RateWall out;
  if (cells.empty()) {
    out.flags |= kCalUndetermined;
    return out;
  }

  // Best single card, never the union: diversity is protection against
  // fading, not against PA compression -- both cards receive the same
  // degraded waveform. Scoring the union would inflate delivery and push the
  // wall up, which is the overdriven direction.
  int best = 0;
  long best_total = -1;
  for (int card = 0; card < 2; ++card) {
    long total = 0;
    for (const auto& c : cells) total += c.received[static_cast<size_t>(card)];
    if (total > best_total) {
      best_total = total;
      best = card;
    }
  }
  out.best_card = best;

  // Computed here, before any wall-finding can return early: saturation is a
  // property of the RSSI this row was measured at, not of whether a wall came
  // out of it. A row that ends undetermined most needs this flag -- it is
  // often the reason there is nothing to find.
  int peak = kRssiNone;
  for (const auto& c : cells)
    if (c.have_rssi[static_cast<size_t>(best)])
      peak = std::max(peak, c.rssi_dbm[static_cast<size_t>(best)]);
  if (peak != kRssiNone && static_cast<double>(peak) > th.sat_rssi_dbm)
    out.flags |= kCalSaturated;

  const auto [floor_idx, wall] = first_run(cells, best, th.deliver_pct);
  out.floor_idx = floor_idx;

  if (floor_idx < 0) {
    // Never reached threshold anywhere: there is no first contiguous run to
    // end, so no wall exists. Do NOT invent one.
    out.flags |= kCalUndetermined;
    return out;
  }

  const int last_idx = static_cast<int>(cells.back().idx);
  if (wall >= last_idx) {
    // Never dipped: this rate has no compression wall inside the swept
    // range, so there is nothing for delivery to find and nothing to back
    // off from. Park it at the rail instead -- the highest wall the chip's
    // 7-bit per-rate diff field can express for this unit (th.max_wall).
    //
    // The rail is below the top of the sweep, so the parked index sits
    // inside territory this very run measured at >=90% delivery. That is a
    // stronger guarantee than the RSSI "saturation knee" this replaced,
    // which was an inference from a curve -- and an unreproducible one: on
    // one unit it read 72, 56, 84 and 56 across four runs, and once put
    // mcs0 and mcs1, the same PA and the same modulation class, 3 dB apart.
    out.flags |= kCalNoDip;
    if (th.max_wall < 0) {
      // No anchor, no rail, no wall. Never invent one: a base_ref_idx of 0
      // would park the rate ~10 dB low on a unit whose real anchor is 39,
      // silently, on the rates the link falls back to when it is struggling.
      out.flags |= kCalUndetermined;
      out.wall = -1;
      out.floor_idx = -1;
      return out;
    }
    out.wall = th.max_wall;
  } else {
    out.wall = wall;
  }

  if (out.wall - floor_idx <= th.narrow_span) out.flags |= kCalNarrow;


  // Card disagreement means an antenna or card problem, not a PA: compression
  // is a property of the transmitter and both cards should see it together.
  bool other_has_data = false;
  const int other = best == 0 ? 1 : 0;
  for (const auto& c : cells)
    if (c.received[static_cast<size_t>(other)] > 0) other_has_data = true;
  if (other_has_data) {
    const auto [ofloor, owall] = first_run(cells, other, th.deliver_pct);
    if (ofloor >= 0 && std::abs(owall - wall) > th.card_disagree)
      out.flags |= kCalCardDisagree;
  }

  return out;
}

}  // namespace maburgs
