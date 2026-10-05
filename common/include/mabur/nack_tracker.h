#pragma once
// NackTracker (fec-nack, spec 2026-10-05): decides which base-layer (sid 0)
// FEC source symbols the GS asks the drone to re-send.
//
// Two triggers admit a wire seq:
//   gap  -- the decoder's erasure set reports it missing; t0 = first poll
//           that saw it.
//   tail -- the newest frame's header says it has `count` fragments but
//           only up to `max_idx` arrived; the seqs after seq_at_max that
//           the decoder still reports unknown are admitted with
//           t0 = last_progress_ms.
// A seq is first requested once now >= t0 + settle (adaptive: max natural
// lateness over settle_window_ms + 2, clamped), then repeated every
// repeat_ms up to max_tries (0 = observe only: lateness stats, no sends).
//
// Deadline: an entry still unknown gap_timeout_ms after t0 is dead (never
// requested again) but stays tracked until the decoder's state is
// terminal, so the erasure set cannot re-admit it. A requested entry books
// one resolution (filled / late_fill / wasted) when its state turns
// terminal, and dropped_deadline once if it is still unknown at the
// deadline or falls below the floor -- the buckets are NOT disjoint: an
// entry that booked dropped_deadline and resolves later books both, so the
// outcomes can sum above syms_requested. A requested entry the stop rule
// killed still books dropped_deadline at the deadline/floor.
// Stop rule: while util >= down_util, a poll with due entries counts one
// `suppressed` and marks those entries dead -- no catch-up burst later.
//
// Core-thread-only, like the decoder it reads.
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "mabur/rc_proto.h"
#include "mabur/sw_decoder.h"

namespace mabur {

struct NackCfg {
  bool enable = false;
  int lookback = 256;        // symbols behind newest the erasure view scans (< fec.seq_horizon)
  int repeat_ms = 16;
  int max_tries = 2;         // 0 = observe only
  int settle_min_ms = 4, settle_max_ms = 24, settle_seed_ms = 12;
  int settle_window_ms = 10000, settle_min_samples = 50;
};

struct NackTailView { uint16_t count; uint16_t max_idx; uint32_t seq_at_max; uint64_t last_progress_ms; };

struct NackInputs {           // all sid 0
  std::function<std::vector<uint32_t>()> missing;                // decoder erasure set (ascending)
  std::function<SwDecoder::SourceState(uint32_t)> state;
  std::function<std::optional<NackTailView>()> tail;
  std::function<double()> util;                                 // ladder util input, sid 0
  double down_util = 0.35;
  uint64_t gap_timeout_ms = 50;
};

struct NackStats {            // cumulative
  uint64_t requests = 0, repeats = 0, syms_requested = 0, tail_requests = 0;
  uint64_t filled = 0, late_fill = 0, wasted = 0, dropped_deadline = 0, suppressed = 0;
};

struct NackWindow {           // since the last take_window()
  // Only the stats sideport export drains the window, so fill_ms is capped:
  // past kMaxFillSamples it stops appending (keeps the window's first
  // samples); `filled` still counts every fill.
  static constexpr size_t kMaxFillSamples = 4096;
  std::vector<uint32_t> fill_ms;  // first request -> retx-filled
  uint32_t late_ms_max = 0;        // natural lateness seen (never-requested seqs)
  uint64_t filled = 0;
};

class NackTracker {
 public:
  explicit NackTracker(NackCfg cfg);
  std::optional<rc::Nack> poll(uint64_t now_ms, const NackInputs& in);
  void clear();                      // frame_wire edge: entries only, counter kept
  void restart_counter() { counter_ = 0; }  // new vtx nonce: next NACK is counter 1
  const NackStats& stats() const;
  NackWindow take_window();
  int settle_ms() const;
  size_t outstanding() const;        // live entries: not dead, tries < max_tries

 private:
  struct Entry {
    uint64_t first_missing_ms = 0, first_sent_ms = 0, last_sent_ms = 0;
    int tries = 0;
    bool from_tail = false;
    bool dead = false;               // deadline or stop rule: never requested again
    bool deadline_counted = false;   // dropped_deadline already booked for this entry
  };
  void resolve(uint64_t now_ms, const NackInputs& in);
  void admit(uint64_t now_ms, const NackInputs& in);
  void note_late(uint64_t now_ms, uint32_t late_ms);
  NackCfg cfg_;
  std::map<uint32_t, Entry> entries_;         // wire seq -> entry (sid 0)
  std::deque<std::pair<uint64_t, uint32_t>> late_;  // (t_ms, late_ms) sliding window
  int settle_ms_;
  uint32_t counter_ = 0;
  NackStats stats_;
  NackWindow win_;
};

}  // namespace mabur
