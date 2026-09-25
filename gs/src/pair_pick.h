#pragma once
#include <cstdint>
#include <vector>

#include "channel_ranker.h"

namespace maburgs {

// The 20 MHz channels a 40 MHz boot scan dwells on: both halves of home's
// pair and of every candidate's pair, deduplicated, config order (home's
// pair first, each pair low half then high half). A channel with no pair
// is skipped (config rejects it at radio.width 40 anyway).
std::vector<uint8_t> scan_half_set(uint8_t home, const std::vector<uint8_t>& candidates);

// Pair pick over per-half RankEntry rows (ChannelRanker::all()). A pair is
// ranked once BOTH halves have >= min_rounds visits; its score is the worse
// half's worst_busy. A pair is blocked if EITHER half's worst-visit NHM busy
// % reaches blocked_pct; the blocked tier takes precedence over score AND
// home_margin -- an unblocked pair always beats a blocked one regardless of
// score, and a blocked home loses to any unblocked candidate outright (spec
// 2026-09-25-nhm-airtime §7). Home's pair keeps home_margin the way
// ChannelRanker::proposal does for a single channel, applied within a tier;
// ties go to home; among candidates the lowest score wins, config order
// breaking ties. Unlike
// ChannelRanker::ranked(), an exact worst_busy tie ignores the noise-floor
// tie-break (valid floor first, then lower floor_dbm): config order alone
// decides it. Returns the
// winning pair's PRIMARY (home, or the candidate as listed), home when no
// pair is ranked, and the best ranked candidate when home's pair is not.
uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t home,
                      const std::vector<uint8_t>& candidates, int min_rounds,
                      uint32_t home_margin, double blocked_pct = 1e9);

// True when home's pair or any candidate's pair is ranked under the same
// both-halves rule pair_proposal uses. The boot-pick freeze's "did the scan
// measure anything" test at radio.width 40: one half at min_rounds (e.g. the
// one-card home half, which also books home-window visits) is not a pick.
bool any_pair_ranked(const std::vector<RankEntry>& all, uint8_t home,
                     const std::vector<uint8_t>& candidates, int min_rounds);

}  // namespace maburgs
