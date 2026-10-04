#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "config.h"

namespace maburgs {

// Which card gathered a visit: a USB dwell (5 ms, CCA-event counts) or a
// relay sweep (20 ms survey, no CCA count).
enum class VisitSrc : uint8_t { Usb, Relay };

// One brief visit to a candidate channel: raw energy (fa, cca), our own
// airtime on it (own), and the count of frames we could actually decode
// from a foreign transmitter (foreign).
struct HopVisit {
  uint8_t ch = 0;
  double t_ms = 0;
  uint32_t fa = 0, cca = 0, own = 0, foreign = 0;
  // NHM busy-airtime evidence (spec 2026-09-25-nhm-airtime §6), from the
  // in-flight dwell that gathered this visit. Absent (busy_valid false) for
  // visits from radios/fakes without NHM support.
  bool busy_valid = false;
  double busy_pct = 0;
  // Which kind of card gathered the visit: USB dwells (5 ms, CCA-event
  // counts) and relay sweeps (20 ms survey, no CCA count) are not on one
  // scale, so ranking() uses only the newest visit's kind (spec
  // 2026-10-05 §6).
  VisitSrc src = VisitSrc::Usb;
};

struct HopRankEntry {
  uint8_t ch = 0;
  uint32_t score = 0;
  int visits = 0;
  bool ranked = false;
  // Mean busy_pct over this entry's fresh, busy_valid visits (0 when none
  // had a reading), and whether that mean clears radio.scan.blocked_pct
  // -- a channel with any busy evidence at or above the threshold ranks
  // behind every non-blocked ranked channel, tiebroken by lower busy_pct.
  bool blocked = false;
  double busy_pct = 0;
};

// Ranks candidate channels by how busy recent brief visits found them, with
// NHM-blocked channels (mean busy_pct >= radio.scan.blocked_pct) pushed
// into their own tier below every non-blocked ranked channel, tiebroken by
// lower busy_pct (spec 2026-09-25-nhm-airtime §6).
// Pure: no I/O, no clock of its own -- the caller passes now_ms (spec
// 2026-09-14-inflight-channel-hop §3). No home: `channels` is the full set
// (spec 2026-10-03-auto-channel-set §5); ties go to the boot-time pick,
// then config order.
class HopRanker {
 public:
  HopRanker(HopCfg cfg, BusyCfg busy, std::vector<uint8_t> channels, uint8_t boot_pick);

  void add(const HopVisit& v);

  // The boot scan's real pick, which is not known until the drone answers
  // the first DISC -- long after this object is constructed. Until it is
  // set, the ranked tiebreak falls through to config order.
  void set_boot_pick(uint8_t ch) { boot_pick_ = ch; }

  // fa + max(cca - own, 0) + 4*foreign
  static uint32_t score(const HopVisit& v);

  // Best (lowest score) first, unranked last, deterministic tiebreak:
  // boot-time pick, then config order.
  std::vector<HopRankEntry> ranking(double now_ms) const;

  // First ranked candidate that is neither exclude nor in skip.
  // require_unblocked: also skip entries whose `blocked` is set -- the hop
  // never orders a channel the dwells read as blocked (Task 11 (a)).
  std::optional<uint8_t> best(double now_ms, uint8_t exclude, const std::vector<uint8_t>& skip,
                              bool require_unblocked = false) const;

 private:
  HopCfg cfg_;
  BusyCfg busy_;
  std::vector<uint8_t> candidates_;   // config order, exactly as given
  uint8_t boot_pick_ = 0;
  std::vector<std::deque<HopVisit>> visits_;   // one deque per entry in candidates_
};

}  // namespace maburgs
