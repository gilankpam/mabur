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
  // NHM busy-airtime evidence, from the
  // in-flight dwell that gathered this visit. Absent (busy_valid false) for
  // visits from radios/fakes without NHM support.
  bool busy_valid = false;
  double busy_pct = 0;
  // Which kind of card gathered the visit: USB dwells (5 ms, CCA-event
  // counts) and relay sweeps (20 ms survey, no CCA count) are not on one
  // scale, so ranking() uses only the newest visit's kind.
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
  // True when at least one fresh visit carried an NHM busy reading. At
  // 40 MHz this orders entries with a reading ahead of entries without one,
  // within a tier, before the score/boot-pick/config-order chain.
  bool busy_valid = false;
};

// Ranks candidate channels by how busy recent brief visits found them, with
// NHM-blocked channels (mean busy_pct >= radio.scan.blocked_pct) pushed
// into their own tier below every non-blocked ranked channel, tiebroken by
// lower busy_pct. At 40 MHz a 20 MHz dwell's frame terms (fa/cca/foreign)
// see the primary only, so within a tier entries carrying an NHM busy
// reading rank ahead of entries without one, then by lower busy_pct, before
// the score/boot-pick/config-order chain -- NHM busy % reads the whole pair
// (`docs/findings/2026-11-03-ht40-dwell-secondary.md`), the frame terms do
// not. At 20 MHz this busy-first ordering is skipped; score alone orders.
// Pure: no I/O, no clock of its own -- the caller passes now_ms. No home:
// `channels` is the full set; ties go to the boot-time pick, then config
// order.
class HopRanker {
 public:
  HopRanker(HopCfg cfg, BusyCfg busy, std::vector<uint8_t> channels, uint8_t boot_pick,
            uint8_t width_mhz = 20);

  void add(const HopVisit& v);

  // The boot scan's real pick, which is not known until the drone answers
  // the first DISC -- long after this object is constructed. Until it is
  // set, the ranked tiebreak falls through to config order.
  void set_boot_pick(uint8_t ch) { boot_pick_ = ch; }

  // The receiver width the ranking orders for (see the class comment); a
  // live width flip moves it. Visits already held are kept and age out
  // as usual (hop.rank_max_age_ms).
  void set_width_mhz(uint8_t w) { width_mhz_ = w; }
  uint8_t width_mhz() const { return width_mhz_; }

  // fa + max(cca - own, 0) + 4*foreign
  static uint32_t score(const HopVisit& v);

  // Best (lowest score) first, unranked last, deterministic tiebreak:
  // boot-time pick, then config order.
  std::vector<HopRankEntry> ranking(double now_ms) const;

  // First ranked candidate that is neither exclude nor in skip.
  // require_unblocked: also skip entries whose `blocked` is set -- the hop
  // never orders a channel the dwells read as blocked.
  std::optional<uint8_t> best(double now_ms, uint8_t exclude, const std::vector<uint8_t>& skip,
                              bool require_unblocked = false) const;

 private:
  HopCfg cfg_;
  BusyCfg busy_;
  std::vector<uint8_t> candidates_;   // config order, exactly as given
  uint8_t boot_pick_ = 0;
  uint8_t width_mhz_ = 20;
  std::vector<std::deque<HopVisit>> visits_;   // one deque per entry in candidates_
};

}  // namespace maburgs
