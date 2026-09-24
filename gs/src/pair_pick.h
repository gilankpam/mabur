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
// half's worst_busy. Home's pair keeps home_margin exactly as
// ChannelRanker::proposal does for a single channel; ties go to home; among
// candidates the lowest score wins, config order breaking ties. Returns the
// winning pair's PRIMARY (home, or the candidate as listed), home when no
// pair is ranked, and the best ranked candidate when home's pair is not.
uint8_t pair_proposal(const std::vector<RankEntry>& all, uint8_t home,
                      const std::vector<uint8_t>& candidates, int min_rounds,
                      uint32_t home_margin);

// True when home's pair or any candidate's pair is ranked under the same
// both-halves rule pair_proposal uses. The boot-pick freeze's "did the scan
// measure anything" test at radio.width 40: one half at min_rounds (e.g. the
// one-card home half, which also books home-window visits) is not a pick.
bool any_pair_ranked(const std::vector<RankEntry>& all, uint8_t home,
                     const std::vector<uint8_t>& candidates, int min_rounds);

}  // namespace maburgs
