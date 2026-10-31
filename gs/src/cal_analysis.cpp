#include "cal_analysis.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

#include "mabur/profile.h"

namespace maburgs {
namespace {

double delivery_pct(const CalCell& c, int card) {
  if (c.expected == 0) return 0.0;
  return 100.0 * static_cast<double>(c.received[static_cast<size_t>(card)]) /
         static_cast<double>(c.expected);
}

// End of the first contiguous run at or above threshold, scanning up from the
// first cell that reaches it. Returns {floor_idx, wall}; both kNoWall when
// the row never reaches threshold at all.
std::pair<int, int> first_run(const std::vector<CalCell>& cells, int card,
                              double pct) {
  size_t i = 0;
  while (i < cells.size() && delivery_pct(cells[i], card) < pct) ++i;
  if (i == cells.size()) return {kNoWall, kNoWall};
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

  if (floor_idx == kNoWall) {
    // Never reached threshold anywhere: there is no first contiguous run to
    // end, so no wall exists. Do NOT invent one.
    out.flags |= kCalUndetermined;
    return out;
  }

  const int last_idx = static_cast<int>(cells.back().idx);
  if (wall >= last_idx) {
    // Never dipped: this rate has no compression wall inside the swept
    // range, so there is nothing for delivery to find and nothing to back
    // off from. Park it at the rail instead -- kRailRel, the top of the
    // relative range every unit's diff field can express, regardless of
    // where this unit's own anchor happens to sit.
    //
    // The rail IS the top cell of the sweep -- kSweepLo is -41 precisely
    // so the grid lands on kRailRel exactly (gs/src/cal_plan.h)
    // -- so the parked index is an index this very run measured at >=90%
    // delivery, not one past the end of the sweep. That is a
    // stronger guarantee than the RSSI "saturation knee" this replaced,
    // which was an inference from a curve -- and an unreproducible one: on
    // one unit it read 72, 56, 84 and 56 across four runs, and once put
    // mcs0 and mcs1, the same PA and the same modulation class, 3 dB apart.
    out.flags |= kCalNoDip;
    out.wall = kRailRel;
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
    if (ofloor != kNoWall && std::abs(owall - wall) > th.card_disagree)
      out.flags |= kCalCardDisagree;
  }

  return out;
}

CapResult analyze_capacity(const CapCell& c, uint8_t mcs, uint8_t bw,
                           uint32_t dwell_ms, const CalThresholds& th) {
  CapResult out;

  // Best single card by delivered air bits -- same "best, never the union"
  // reasoning as the wall pass: the two cards do not independently
  // bottleneck the same TX stream.
  int best = 0;
  if (c.air_bits[1] > c.air_bits[0]) best = 1;
  out.best_card = best;
  const size_t bi = static_cast<size_t>(best);

  // Saturation rule reused verbatim from the wall pass: a single peak RSSI
  // reading above the threshold is enough to suspect RX compression.
  if (c.have_rssi[bi] && static_cast<double>(c.rssi_dbm[bi]) > th.sat_rssi_dbm)
    out.flags |= kCalSaturated;

  // Rarely blocked means the air had slots free that the drone never filled
  // -- this dwell measured the drone's CPU/USB, not the air's capacity.
  if (static_cast<int>(c.blocked_pct) < th.tx_bound_blocked_pct)
    out.flags |= kCalTxBound;

  // Too few frames arrived to trust the count, independent of whether the
  // bits that did arrive look efficient.
  if (static_cast<double>(c.frames[bi]) <
      th.lossy_ratio * static_cast<double>(c.offered))
    out.flags |= kCalLossy;
  // Under half arrived: whatever the bits say, they measured the loss.
  if (static_cast<double>(c.frames[bi]) <
      th.lost_ratio * static_cast<double>(c.offered))
    out.flags |= kCalLost | kCalLossy;

  const double nominal_bps =
      mabur::rc::phy_rate_mbps({mabur::rc::PhyMode::HT, mcs, bw}) * 1e6;
  // Timed by the GS's own receive clock: the drone's TX queue keeps airing
  // after its blast returns, so the air was busy longer than the commanded
  // dwell. The bits after the first frame over first-to-last is the rate the
  // saturated air delivered; the first frame's bits were on the air before
  // its stamp, so they fall outside the span.
  const uint64_t span_ms =
      c.last_ms[bi] > c.first_ms[bi] ? c.last_ms[bi] - c.first_ms[bi] : 0;
  out.span_ms = static_cast<uint32_t>(std::min<uint64_t>(span_ms, UINT32_MAX));
  double eff = 0.0;
  if (nominal_bps > 0.0) {
    if (c.frames[bi] >= th.span_min_frames && span_ms >= th.span_min_ms) {
      const double bits = static_cast<double>(c.air_bits[bi] - c.first_bits[bi]);
      eff = bits / (static_cast<double>(span_ms) / 1000.0 * nominal_bps);
    } else {
      // Too few frames, or too short a span, to time: the commanded dwell
      // is the only clock left, and it reads high by the queue's drain.
      out.flags |= kCalShortSpan;
      if (dwell_ms > 0) {
        const double dwell_s = static_cast<double>(dwell_ms) / 1000.0;
        eff = static_cast<double>(c.air_bits[bi]) / (dwell_s * nominal_bps);
      }
    }
  }
  eff = std::round(eff * 100.0) / 100.0;
  out.eff = std::clamp(eff, 0.05, 1.0);

  return out;
}

}  // namespace maburgs
