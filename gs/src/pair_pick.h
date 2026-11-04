#pragma once
#include <cstdint>
#include <vector>

#include "channel_ranker.h"

namespace maburgs {

// Pair pick for a 40 MHz boot scan over one RankEntry row per pair, keyed
// by the pair's PRIMARY (ChannelRanker::all()): the scout card is tuned 40,
// so one dwell on the primary reads the whole pair -- NHM busy % over both
// halves, frame terms from the primary. A pair is ranked once its
// primary's row has >= min_rounds visits.
//
// Order (pair_ranking, and the winner pair_proposal picks among the
// candidates): tier first -- unblocked before blocked, blocked = a busy
// reading at or above blocked_pct; within a tier rows with a busy reading
// before rows without; then the lower worst_busy_pct; then the lower
// worst_busy (frame score); then config order. Unlike
// ChannelRanker::ranked() there is no noise-floor tie-break.
//
// There is no fixed home: `set` includes `current`, and current's pair
// keeps the pick unless the best candidate
//  - is in a better tier (a blocked current loses to any unblocked
//    candidate outright), or has a busy reading where current has none;
//  - or, both read, beats current's busy by margin_pct points.
// An exact busy tie, or neither side reading busy, stays. Returns the
// winning primary (current, or the candidate as listed in `set`), current
// when nothing is ranked, and the best ranked candidate when current's
// pair is not ranked.
uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t current,
                      const std::vector<uint8_t>& set, int min_rounds,
                      double margin_pct, double blocked_pct = 1e9);

// True when any pair in `set` is ranked (its primary's row at min_rounds).
// The boot-pick freeze's "did the scan measure anything" test at
// radio.width 40.
bool any_pair_ranked(const std::vector<RankEntry>& all, const std::vector<uint8_t>& set,
                     int min_rounds);

// True when EVERY pair in `set` is ranked. False on an empty set.
bool all_pairs_ranked(const std::vector<RankEntry>& all, const std::vector<uint8_t>& set,
                      int min_rounds);

// All ranked pairs in `set`, best first, in the order above -- the same
// order pair_proposal picks its winner from. Unranked primaries are
// skipped, not just sorted last.
std::vector<uint8_t> pair_ranking(const std::vector<RankEntry>& all,
                                  const std::vector<uint8_t>& set, int min_rounds,
                                  double blocked_pct = 1e9);

}  // namespace maburgs
