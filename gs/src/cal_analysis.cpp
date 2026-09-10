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

// The PA saturation knee: the lowest index whose median RSSI is already
// within knee_tol_db of the row's peak. Above it the transfer curve is flat,
// so more index buys no more radiated power and only burns diff range.
// Returns -1 when the row carries no RSSI at all.
int saturation_knee(const std::vector<CalCell>& cells, int card,
                    double tol_db) {
  int peak = kRssiNone;
  for (const auto& c : cells)
    if (c.have_rssi[static_cast<size_t>(card)])
      peak = std::max(peak, c.rssi_dbm[static_cast<size_t>(card)]);
  if (peak == kRssiNone) return -1;
  for (const auto& c : cells) {
    if (!c.have_rssi[static_cast<size_t>(card)]) continue;
    if (static_cast<double>(c.rssi_dbm[static_cast<size_t>(card)]) >=
        static_cast<double>(peak) - tol_db)
      return static_cast<int>(c.idx);
  }
  return -1;
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
    // Never dipped. Delivery cannot see the PA ceiling (it stays at 100%
    // straight through), so the wall is the RSSI knee instead.
    out.flags |= kCalNoDip;
    const int knee = saturation_knee(cells, best, th.knee_tol_db);
    out.wall = knee >= 0 ? knee : wall;
  } else {
    out.wall = wall;
  }

  if (out.wall - floor_idx <= th.narrow_span) out.flags |= kCalNarrow;

  int peak = kRssiNone;
  for (const auto& c : cells)
    if (c.have_rssi[static_cast<size_t>(best)])
      peak = std::max(peak, c.rssi_dbm[static_cast<size_t>(best)]);
  if (peak != kRssiNone && static_cast<double>(peak) > th.sat_rssi_dbm)
    out.flags |= kCalSaturated;

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
