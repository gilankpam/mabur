#pragma once
// ProbeTrack — score the probe stream (SBI stream 5) against the enh AU
// cadence, per spec §3.2.
//
// Why expected comes from the AU count, not the probe seq: the probe
// stream shares the enh AU's send opportunity, so a probe body can be
// lost exactly like any other enh frame. If "expected" were derived from
// seq gaps in what arrived, a 100%-lost probe stream would never book any
// expectation at all -- seq is carried IN the lost bodies -- and would
// read as silence (no data, ladder ignores it) instead of loss (data,
// score it zero). Booking one expectation of `bpb` blocks per enh AU,
// independent of whether any probe body for that AU is ever seen, is what
// makes total loss show up as `arrived_blocks == 0` against a nonzero
// `expected_blocks` rather than as an empty window.
//
// Why the union (OR of survivor bitmaps across cards) is what gets
// scored: production video comes off the two-card FEC dedup, i.e. a block
// counts as delivered if EITHER radio card heard it. The probe stream
// exists to predict that path's loss rate, so it must be scored the same
// way -- per-card counters exist alongside the union for diagnostics
// (which card is dragging), not as the number the ladder consumes.
//
// Counters are strictly non-decreasing (never unbooked, never reset) so
// a downstream consumer that diffs successive samples (e.g. the ladder's
// S1LossWindow) can treat any backward step as "the counter wrapped/reset"
// without ProbeTrack itself ever producing a false one. A body that
// finalizes at a profile other than the commanded one is therefore never
// allowed to subtract from `expected_blocks` after the fact. Instead,
// AU expectations are keyed by the enh AU's frame id (the same id the
// probe body carries as `ProbeRx::hdr.enh_fid`): the FIRST sight of an
// off-profile body cancels its own AU's still-pending expectation (marks
// it unscored) rather than decrementing a total that may already have
// been sampled. A body arrives well within the AU's own finalize_ms
// window in the normal case (bodies land <30 ms after their AU begins;
// the AU finalizes at +100 ms), so the AU is essentially always still
// pending when its body's profile is checked. A body that somehow lands
// late enough to miss its own AU's window books `bpb` of phantom loss
// instead -- bounded to the profile-switch window and considered an
// acceptable, documented cost of keeping the counters monotonic.
//
// Both an AU expectation and a received body sit in a small pending
// window for `finalize_ms` before they count, so that a body's later
// arrival on the second card (or a duplicate) can still fold into the
// same entry instead of double-booking. commanded() gates whether a body
// is "on profile": the ladder only ever cares about loss at the profile
// it currently has commanded, and the per-body log (Task 9) already
// carries every body's own profile/mcs, so ProbeTrack keeps counters for
// the commanded profile only plus a single `off_profile` count instead of
// a full per-profile map.
//
// Single-threaded (core loop). Time is the core loop's mono ms; tick()
// must be called once per iteration to advance finalization.
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include "mabur/probe_wire.h"

namespace maburgs {

struct ProbeTrackCfg {
  int bpb = 4;            // blocks per probe body (ENH layer geometry)
  int finalize_ms = 100;  // kProbeFinalizeMs
  int max_cards = 2;
};

// Cumulative, strictly non-decreasing counters. Never reset, never
// unbooked -- callers diff between samples (the ladder's loss windows,
// ProbeLog rows) and may treat any backward step as a counter reset.
struct ProbeCounts {
  uint64_t expected_blocks = 0;  // bpb x finalized, scored enh AUs (while commanded)
  uint64_t arrived_blocks = 0;   // survivors at the commanded profile
  uint64_t bodies_rx = 0;        // finalized bodies at the commanded profile
};

// One row per finalized received body, drained by ProbeLog (Task 9) --
// emitted for EVERY body, on- or off-profile (an RCF-lag body still logs
// its own `profile`); only the ProbeCounts booking above is gated to the
// commanded profile.
struct ProbeFinalized {
  double t_ms = 0;
  uint32_t seq = 0;
  uint8_t profile = 0;
  uint16_t enh_fid = 0;
  int blocks_ok = 0;
  uint32_t card_mask = 0;    // bit c set = card c saw at least one survivor
  // This body's per-card label; NaN where the card heard nothing. Default
  // members so a struct built without looping over every card still reads
  // as "unheard" rather than zero (a real EVM/SNR value).
  double snr_db[8] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
  double evm_db[8] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
};

class ProbeTrack {
 public:
  explicit ProbeTrack(ProbeTrackCfg cfg);

  // Sets the profile the ladder currently has commanded. 0xFF = none
  // (probe stream not running / not booked yet): on_enh_au is a no-op
  // while this holds.
  void set_commanded(uint8_t profile, double now_ms);
  uint8_t commanded() const;

  // The core loop saw one enh AU begin, identified by its frame id (the
  // same id probe bodies for it carry as ProbeRx::hdr.enh_fid). Books an
  // expectation of `bpb` blocks, finalized `finalize_ms` later unless an
  // off-profile body for this fid cancels it first. No-op while
  // commanded() == 0xFF.
  void on_enh_au(uint16_t enh_fid, double now_ms);

  // A parsed probe body arrived on `card`. snr_db/evm_db are this card's
  // RF labels for the body (NaN if unavailable). Bodies are keyed by
  // seq: the first sight opens a ring entry that later sights (the other
  // card, a duplicate) OR their bitmaps into, finalized `finalize_ms`
  // after first sight. The first sight of a body at a non-commanded
  // profile also cancels its AU's expectation (see header comment).
  void on_body(int card, const mabur::probe::ProbeRx& rx, double snr_db,
               double evm_db, double now_ms);

  // Finalizes AU expectations and body entries whose window has elapsed.
  // Call once per core-loop iteration.
  void tick(double now_ms);

  const ProbeCounts& union_counts() const;
  const ProbeCounts& card_counts(int card) const;
  uint64_t off_profile() const;  // bodies finalized at a non-commanded profile

  // Drains the rows finalized since the last call -- one row per finalized
  // body, on- or off-profile; wholly-lost bodies are the only seq gaps.
  std::vector<ProbeFinalized> take_finalized();

 private:
  struct Pending {
    uint32_t seq;
    uint8_t profile;
    uint16_t enh_fid;
    double first_ms;
    uint32_t bitmap = 0, card_mask = 0;
    std::array<uint32_t, 8> card_bits{};
    std::array<double, 8> snr{}, evm{};
  };
  struct PendingAu {
    uint16_t enh_fid;
    double t_ms;
    bool scored = true;  // false = an off-profile body already cancelled this AU
  };

  void finalize_body(Pending& p, double now_ms);
  void finalize_au(const PendingAu& au);

  ProbeTrackCfg cfg_;
  uint8_t commanded_ = 0xFF;
  std::deque<Pending> ring_;
  std::deque<PendingAu> au_pending_;
  ProbeCounts union_;
  std::array<ProbeCounts, 8> cards_{};
  uint64_t off_profile_ = 0;
  std::vector<ProbeFinalized> finalized_;
  static constexpr size_t kRingMax = 64;
};

}  // namespace maburgs
