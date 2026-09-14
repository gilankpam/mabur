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

  // fa + max(cca - own, 0) + 4*foreign
  static uint32_t score(const HopVisit& v);

  // Best (lowest score) first, unranked last, deterministic tiebreak:
  // boot-time pick, then home, then config order.
  std::vector<HopRankEntry> ranking(double now_ms) const;

  // First ranked candidate that is neither exclude nor in skip.
  std::optional<uint8_t> best(double now_ms, uint8_t exclude, const std::vector<uint8_t>& skip) const;

 private:
  HopCfg cfg_;
  std::vector<uint8_t> candidates_;   // config order, home appended if not already present
  uint8_t home_ = 0;
  uint8_t boot_pick_ = 0;
  std::vector<std::deque<HopVisit>> visits_;   // one deque per entry in candidates_
};

}  // namespace maburgs
