#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "config.h"

namespace maburgs {

// One brief visit to a candidate channel: raw energy (fa, cca), our own
// airtime on it (own), and the count of frames we could actually decode
// from a foreign transmitter (foreign).
struct HopVisit {
  uint8_t ch = 0;
  double t_ms = 0;
  uint32_t fa = 0, cca = 0, own = 0, foreign = 0;
};

struct HopRankEntry {
  uint8_t ch = 0;
  uint32_t score = 0;
  int visits = 0;
  bool ranked = false;
};

// Ranks candidate channels by how busy recent brief visits found them.
// Pure: no I/O, no clock of its own -- the caller passes now_ms (spec
// 2026-09-14-inflight-channel-hop §3).
class HopRanker {
 public:
  HopRanker(HopCfg cfg, std::vector<uint8_t> candidates, uint8_t home, uint8_t boot_pick);

  void add(const HopVisit& v);

  // The boot scan's real pick, which is not known until the drone answers
  // the first DISC -- long after this object is constructed. Until it is
  // set, the ranked tiebreak falls through to home, then config order.
  void set_boot_pick(uint8_t ch) { boot_pick_ = ch; }

  // The channels best() may return; empty (the default) = every candidate.
  // At radio.width 40 the ranker also holds the pairs' secondary halves as
  // scored evidence (the boot scan dwells on them), but a hop must land on
  // a primary: FastRetune keeps the card's offset, so a secondary target
  // would tune an off-grid pair.
  void set_targets(std::vector<uint8_t> targets) { targets_ = std::move(targets); }

  // fa + max(cca - own, 0) + 4*foreign
  static uint32_t score(const HopVisit& v);

  // Best (lowest score) first, unranked last, deterministic tiebreak:
  // boot-time pick, then home, then config order.
  std::vector<HopRankEntry> ranking(double now_ms) const;

  // First ranked target (set_targets) that is neither exclude nor in skip.
  std::optional<uint8_t> best(double now_ms, uint8_t exclude, const std::vector<uint8_t>& skip) const;

 private:
  HopCfg cfg_;
  std::vector<uint8_t> candidates_;   // config order, home appended if not already present
  uint8_t home_ = 0;
  uint8_t boot_pick_ = 0;
  std::vector<uint8_t> targets_;   // empty = every candidate
  std::vector<std::deque<HopVisit>> visits_;   // one deque per entry in candidates_
};

}  // namespace maburgs
