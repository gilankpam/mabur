#pragma once
// The pick of a calibration run's measure round: which member of the set
// the run should calibrate on. Pure, header-only.
//
// Op and the candidates are read by different instruments: each candidate
// by the scout card's own observe of its channel at the link width (at 40
// MHz one dwell on the pair's primary reads the whole pair), op by the
// link cards' verdict window (the scout never dwells on op's own pair
// while linked: it carries the drone's video). So only terms that mean the
// same thing on both sides are compared:
//  - NHM busy %: foreign airtime share (op's reading has our own video's
//    airtime taken out), and
//  - decoded foreign frames per dwell_ms observe.
// The CCA event term is not used: on op, A-MPDU video makes `cca - own`
// (PPDUs minus MPDUs) clamp to zero, while a candidate keeps its CCA, so the
// term would score op clean no matter what.
//
// Candidate selection among the set (which one is "best") still scores
// foreign + busy % (one unit per frame, one per percentage point; the busy
// term only when both sides being compared have a reading) -- foreign
// stays in the row's score for ordering only. A candidate with no busy
// reading is not ranked while any candidate has one.
//
// Leaving op is the same margin rule as ChannelRanker::proposal and
// pair_proposal: the best candidate replaces op only when it is in a
// better tier (a blocked op loses to any unblocked candidate at once; a
// blocked candidate never replaces an unblocked op), has a busy reading
// where op has none, or both read and beats op's busy % by `margin_pct`
// points. An exact busy tie, or neither side reading busy, stays -- as
// does op unread, or no candidate read at all.
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "channel_ranker.h"

namespace maburgs {

// One reading of one channel: the member's own observe from the round's
// scout dwell, or op from the verdict window.
struct CalReading {
  uint8_t ch = 0;
  uint32_t foreign = 0;      // decoded foreign frames, per dwell_ms observe
  bool busy_valid = false;
  double busy_pct = 0;       // NHM foreign busy airtime, %
};

struct CalPickOut {
  uint8_t best = 0;
  // The K record's rows: every member read this round plus op's,
  // worst_busy = the score above, one visit each.
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
// The member's own reading; not ok unless it was read this round.
inline Side member_side(const std::vector<CalReading>& rd, uint8_t member) {
  for (const CalReading& r : rd) {
    if (r.ch != member) continue;
    return Side{true, r.foreign, r.busy_valid, r.busy_valid ? r.busy_pct : 0.0};
  }
  return Side{};
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

inline CalPickOut cal_round_pick(const std::vector<CalReading>& members,
                                 const std::optional<CalReading>& op_read, uint8_t op,
                                 const std::vector<uint8_t>& set, double margin_pct,
                                 double blocked_pct) {
  using namespace cal_pick_detail;
  CalPickOut out;
  out.best = op;
  // A candidate with no busy reading would score on foreign alone, a term
  // short of what the others pay: it is not eligible while any candidate
  // has a busy reading. Gathered alongside the rows: both are one pass
  // over the set's members.
  bool any_busy = false;
  for (uint8_t c : set) {
    if (c == op) continue;
    const Side s = member_side(members, c);
    if (!s.ok) continue;
    out.rows.push_back(row(CalReading{c, s.foreign, s.busy_valid, s.busy_pct}));
    if (s.busy_valid) any_busy = true;
  }
  if (op_read) out.rows.push_back(row(*op_read));
  if (!op_read) return out;
  Side o{true, op_read->foreign, op_read->busy_valid, op_read->busy_valid ? op_read->busy_pct : 0.0};

  // Best candidate: unblocked first, then score, then config order.
  std::optional<uint8_t> best;
  Side bs;
  for (uint8_t c : set) {
    if (c == op) continue;
    const Side s = member_side(members, c);
    if (!s.ok) continue;
    if (any_busy && !s.busy_valid) continue;
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
  // Leaving op: a better tier (op blocked, the candidate not) or a busy
  // reading where op has none moves at once; both read, the busy-%
  // margin decides; an exact tie, or neither read, stays -- the same rule
  // as ChannelRanker::proposal and pair_proposal.
  const bool bb = blocked(bs, blocked_pct), ob = blocked(o, blocked_pct);
  if (bb != ob) {
    if (!bb) out.best = *best;   // a blocked candidate never replaces an unblocked op
    return out;
  }
  if (bs.busy_valid != o.busy_valid) {
    if (bs.busy_valid) out.best = *best;
    return out;
  }
  if (bs.busy_valid && o.busy_pct - bs.busy_pct >= margin_pct) out.best = *best;
  return out;
}

}  // namespace maburgs
