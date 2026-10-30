#pragma once
// The pick of a calibration run's measure round: which member of the set
// the run should calibrate on. Pure, header-only.
//
// Op and the candidates are read by different instruments: each candidate
// half by the scout card's 20 MHz observe, op by the link cards' verdict
// window at the link width (the scout never dwells on op's own pair while
// linked: it carries the drone's video). So only terms that mean the same
// thing on both sides are compared:
//  - NHM busy %: foreign airtime share (op's reading has our own video's
//    airtime taken out), and
//  - decoded foreign frames per dwell_ms observe.
// The CCA event term is not used: on op, A-MPDU video makes `cca - own`
// (PPDUs minus MPDUs) clamp to zero, while a candidate keeps its CCA, so the
// term would score op clean no matter what.
//
// score = foreign + busy % (one unit per frame, one per percentage point;
// the busy term only when both sides being compared have a reading). A
// candidate pair scores as its worse half. Leaving op takes beating it by
// `margin` in every tier: a blocked candidate never replaces an unblocked
// op, and a candidate that is unblocked while op is blocked still has to be
// `margin` better. Op unread, or no candidate read on every half: stay.
// Both rules lean toward staying where the link already works.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "channel_ranker.h"
#include "mabur/ht40.h"

namespace maburgs {

// One reading of one channel: a 20 MHz half from the round's scout dwell,
// or op from the verdict window.
struct CalReading {
  uint8_t ch = 0;
  uint32_t foreign = 0;      // decoded foreign frames, per dwell_ms observe
  bool busy_valid = false;
  double busy_pct = 0;       // NHM foreign busy airtime, %
};

struct CalPickOut {
  uint8_t best = 0;
  // The K record's rows: every half read this round plus op's (both halves
  // at width 40), worst_busy = the score above, one visit each.
  std::vector<RankEntry> rows;
};

namespace cal_pick_detail {
struct Side {
  bool ok = false;
  uint32_t foreign = 0;
  bool busy_valid = false;
  double busy_pct = 0;
};
inline double score(const Side& s, bool use_busy) {
  return static_cast<double>(s.foreign) + (use_busy ? s.busy_pct : 0.0);
}
inline bool blocked(const Side& s, double blocked_pct) {
  return s.busy_valid && s.busy_pct >= blocked_pct;
}
inline std::vector<uint8_t> halves_of(uint8_t member, int width_mhz) {
  if (width_mhz != 40) return {member};
  const uint8_t other = mabur::ht40_pair_other(member);
  if (other == 0) return {member};
  return {member, other};
}
// The worse half's terms; not ok unless every half has a reading.
inline Side member_side(const std::vector<CalReading>& rd, uint8_t member, int width_mhz) {
  Side s;
  s.ok = true;
  s.busy_valid = true;
  for (uint8_t h : halves_of(member, width_mhz)) {
    bool seen = false, busy_seen = false;
    for (const CalReading& r : rd) {
      if (r.ch != h) continue;
      seen = true;
      s.foreign = std::max(s.foreign, r.foreign);
      if (r.busy_valid) {
        busy_seen = true;
        s.busy_pct = std::max(s.busy_pct, r.busy_pct);
      }
    }
    if (!seen) s.ok = false;
    if (!busy_seen) s.busy_valid = false;
  }
  if (!s.busy_valid) s.busy_pct = 0;
  return s;
}
inline RankEntry row(const CalReading& r) {
  RankEntry e;
  e.ch = r.ch;
  e.worst_busy = static_cast<uint32_t>(std::lround(
      score(Side{true, r.foreign, r.busy_valid, r.busy_pct}, r.busy_valid)));
  e.visits = 1;
  e.busy_valid = r.busy_valid;
  e.worst_busy_pct = r.busy_pct;
  return e;
}
}  // namespace cal_pick_detail

inline CalPickOut cal_round_pick(const std::vector<CalReading>& halves,
                                 const std::optional<CalReading>& op_read, uint8_t op,
                                 const std::vector<uint8_t>& set, int width_mhz, uint32_t margin,
                                 double blocked_pct) {
  using namespace cal_pick_detail;
  CalPickOut out;
  out.best = op;
  for (const CalReading& r : halves) out.rows.push_back(row(r));
  if (op_read) {
    for (uint8_t h : halves_of(op, width_mhz)) {
      CalReading r = *op_read;
      r.ch = h;
      out.rows.push_back(row(r));
    }
  }
  if (!op_read) return out;
  Side o{true, op_read->foreign, op_read->busy_valid, op_read->busy_valid ? op_read->busy_pct : 0.0};

  // Best candidate: unblocked first, then score, then config order.
  std::optional<uint8_t> best;
  Side bs;
  for (uint8_t c : set) {
    if (c == op) continue;
    const Side s = member_side(halves, c, width_mhz);
    if (!s.ok) continue;
    if (!best) {
      best = c;
      bs = s;
      continue;
    }
    const bool sb = blocked(s, blocked_pct), bb = blocked(bs, blocked_pct);
    if (sb != bb) {
      if (!sb) { best = c; bs = s; }
      continue;
    }
    if (score(s, s.busy_valid) < score(bs, bs.busy_valid)) {
      best = c;
      bs = s;
    }
  }
  if (!best) return out;
  if (blocked(bs, blocked_pct) && !blocked(o, blocked_pct)) return out;
  const bool use_busy = bs.busy_valid && o.busy_valid;
  if (score(bs, use_busy) + static_cast<double>(margin) <= score(o, use_busy)) out.best = *best;
  return out;
}

}  // namespace maburgs
