#pragma once
#include <cstdint>
#include <vector>

#include "channel_ranker.h"

namespace maburgs {

// The 20 MHz channels a 40 MHz boot scan dwells on: both halves of every
// channel in `set`'s pair, deduplicated, config order (each pair low half
// then high half). A channel with no pair is skipped (config rejects it at
// radio.width 40 anyway).
std::vector<uint8_t> scan_half_set(const std::vector<uint8_t>& set);

// Pair pick over per-half RankEntry rows (ChannelRanker::all()). A pair is
// ranked once BOTH halves have >= min_rounds visits; its score is the worse
// half's worst_busy. A pair is blocked if EITHER half's worst-visit NHM busy
// % reaches blocked_pct; the blocked tier takes precedence over score AND
// margin -- an unblocked pair always beats a blocked one regardless of
// score, and a blocked current channel loses to any unblocked candidate
// outright (spec 2026-09-25-nhm-airtime §7). There is no fixed home: `set`
// includes `current`, and current's pair keeps `margin` the way
// ChannelRanker::proposal does for a single channel, applied within a tier;
// ties stay on current; among candidates the lowest score wins, config
// order breaking ties. Unlike ChannelRanker::ranked(), an exact worst_busy
// tie ignores the noise-floor tie-break (valid floor first, then lower
// floor_dbm): config order alone decides it. Returns the winning pair's
// PRIMARY (current, or the candidate as listed in `set`), current when no
// pair is ranked, and the best ranked candidate when current's pair is not.
//
// When current's pair and the best candidate pair are BOTH blocked,
// `better()` orders them by busy (the worse half's worst_busy_pct) -- but
// the margin check below it compares `score` (the worse half's worst_busy
// event count), not busy, so a candidate that out-orders current on busy
// alone usually still loses to margin on score and current keeps the pair.
uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t current,
                      const std::vector<uint8_t>& set, int min_rounds,
                      uint32_t margin, double blocked_pct = 1e9);

// True when any pair in `set` is ranked under the same both-halves rule
// pair_proposal uses. The boot-pick freeze's "did the scan measure
// anything" test at radio.width 40: one half at min_rounds (e.g. the
// one-card home half, which also books home-window visits) is not a pick.
bool any_pair_ranked(const std::vector<RankEntry>& all, const std::vector<uint8_t>& set,
                     int min_rounds);

// True when EVERY pair in `set` is ranked (both halves >= min_rounds).
// False on an empty set.
bool all_pairs_ranked(const std::vector<RankEntry>& all, const std::vector<uint8_t>& set,
                      int min_rounds);

// All ranked pairs in `set`, best first: tier (unblocked before blocked),
// then score (blocked tiebroken by busy), then config order -- the same
// order pair_proposal picks its winner from. Unranked primaries are
// skipped, not just sorted last.
std::vector<uint8_t> pair_ranking(const std::vector<RankEntry>& all,
                                  const std::vector<uint8_t>& set, int min_rounds,
                                  double blocked_pct = 1e9);

}  // namespace maburgs
