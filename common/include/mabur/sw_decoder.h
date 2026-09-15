#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "mabur/sw_encoder.h"  // SwConfig
#include "mabur/arrival_tracker.h"

namespace mabur {

// Per-envelope transition-boundary hint for loss attribution (spec
// 2026-08-14-fec-generation-attribution-design.md). kPre = the air frame
// carrying this envelope was heard at a pre-transition PHY rate (provably
// old-op, however late it arrived); kPost = heard at the expected
// post-transition rate; kNone = no boundary armed or rate unknown.
enum class SwBoundary : uint8_t { kNone = 0, kPre, kPost };

// One loss episode (fec.log gauge, 2026-09-15): a run of source seqs the
// channel never delivered directly, grouped with any other such run within
// one repair window of it (they compete for the same repairs). Final at
// horizon eviction, so `missing` counts only seqs whose direct copy never
// showed inside the horizon: repair-recovered-then-heard symbols are NOT
// missing (the repair merely won an arrival race). `repairs` is every
// distinct repair received whose window intersects [first_seq, first_seq +
// span) -- the parity the decoder had to work with, whether it needed it
// or not. The reader (tools/flightreport.py) turns missing/repairs and the
// applied overhead into "the overhead this episode would have needed".
struct LossEpisode {
  uint64_t first_seq = 0;  // virtual seq of the first missing source
  uint32_t span = 0;       // last missing - first missing + 1
  uint32_t missing = 0;    // recovered + abandoned
  uint32_t recovered = 0;  // repaired, direct copy never heard
  uint32_t abandoned = 0;  // fell off the horizon unknown
  uint32_t stale = 0;      // of missing, below the transition watermark
  uint32_t repairs = 0;    // distinct covering repairs received
  uint32_t window = 0;     // repair window as flown at close
};

// Streaming decoder for SwEncoder envelopes. Sources deliver IMMEDIATELY
// and register as known; repairs reduce against known symbols, then join an
// incremental Gaussian elimination over the missing seqs — a row down to one
// coefficient is a recovered symbol (delivered late, then substituted back,
// which can cascade). No in-order contract: symbols are self-contained
// (packets never span symbols) and FrameStream tolerates any fragment arrival
// order, so there is no reorder-and-wait stage before delivery.
//
// Wire u32 seqs are unwrapped to internal u64 ("virtual") seqs so std::map
// ordering survives wrap. A source seq jumping more than kResetSpan from the
// newest known seq means the encoder restarted (drone reboot) — the decoder
// resets state and re-anchors rather than dropping everything as stale.
//
// State floor: base_ = newest - horizon. Seqs falling below base_ while
// unknown count syms_abandoned (the layer's loss number); known payloads
// below base_ are evicted; rows referencing anything below base_ are
// unsolvable and dropped. expire_rows_older_than() is the wall-clock
// backstop for low-rate layers where seq barely advances; rows it drops are
// NOT counted abandoned (the horizon owns loss accounting).
class SwDecoder {
 public:
  explicit SwDecoder(const SwConfig& cfg, uint32_t seq_horizon = 0);

  // Feeds one received envelope; returns app packets unpacked from every
  // symbol that became known (source first, cascades after). Malformed or
  // config-mismatched envelopes are counted and dropped, never applied.
  // clean: the carrying body's FCS verdict (ArrivalTracker::on_source);
  // decode is identical either way, only arr_salvage_only depends on it.
  std::vector<std::vector<uint8_t>> add_symbol(const uint8_t* env, size_t len,
                                               uint64_t now_ms,
                                               SwBoundary b = SwBoundary::kNone,
                                               bool clean = true);

  // Drops repair rows first seen more than deadline_ms ago. Call ~1 Hz.
  // Precondition: now_ms monotonic non-decreasing.
  int expire_rows_older_than(uint64_t deadline_ms, uint64_t now_ms);

  uint64_t syms_delivered() const { return syms_delivered_; }
  uint64_t syms_recovered() const { return syms_recovered_; }
  // Recovered symbols whose direct source copy later arrived anyway: the
  // repair merely won an arrival race, the channel did deliver the symbol.
  // Loss metrics must treat these as arrived — recovered alone reads a
  // reorder-heavy healthy link as lossy (2x-parity rung 0 measured 19-26%
  // phantom pre-FEC loss on a clean bench, 2026-07-27).
  uint64_t syms_recovered_arrived() const { return syms_recovered_arrived_; }
  uint64_t syms_abandoned() const { return syms_abandoned_; }
  // Symbol-space transition watermark (loss attribution). mark_transition()
  // snapshots the newest seq; while the boundary is open every abandonment
  // books stale, kPre envelopes advance the watermark, and the first kPost
  // SOURCE closes it at (its seq - 1). Owner (UepDecoder) drives hints and
  // force-closes on expiry. Vseqs are monotonic u64: no wrap, no decay —
  // seqs at or below the watermark book stale forever, which is exactly
  // right (they cannot recur).
  void mark_transition();
  void close_boundary() { wm_open_ = false; }
  bool boundary_open() const { return wm_open_; }
  uint64_t syms_abandoned_stale() const { return syms_abandoned_stale_; }
  // Drains the loss episodes closed since the last call (see LossEpisode).
  // An episode closes once eviction has passed its last seq by a full
  // repair window with nothing else joining it. An owner that never drains
  // (MspSink, bench tools) is bounded: past kMaxQueuedEpisodes the oldest
  // closed episode is dropped.
  static constexpr size_t kMaxQueuedEpisodes = 1024;
  std::vector<LossEpisode> take_episodes();
  // Arrival-time pre-FEC accounting (ArrivalTracker, spec 2026-09-05):
  // expected = seq advance past the settle line, arrived = heard; stale
  // twins follow the same watermark boundary abandonment uses.
  uint64_t arr_expected() const { return arr_.expected(); }
  uint64_t arr_arrived() const { return arr_.arrived(); }
  uint64_t arr_expected_stale() const { return arr_.expected_stale(); }
  uint64_t arr_arrived_stale() const { return arr_.arrived_stale(); }
  uint64_t arr_late() const { return arr_.late(); }
  uint64_t arr_salvage_only() const { return arr_.salvage_only(); }
  uint64_t symbols_in() const { return symbols_in_; }
  // Highest virtual seq seen or implied — its ADVANCE rate is the stream's
  // send rate (loss-robust; any arriving symbol moves it). 0 before the
  // first source anchors the seq space. Feeds GapTimeoutPolicy.
  uint64_t newest_seq() const { return have_seq_ ? newest_v_ : 0; }
  // Largest repair window_len observed — the TX sliding window as actually
  // flown, self-calibrating (the GS config does not know the drone's
  // fec.window). 0 until the first repair. Feeds GapTimeoutPolicy.
  int repair_window() const { return repair_window_hwm_; }
  uint64_t symbols_dropped_bad_cfg() const { return symbols_dropped_bad_cfg_; }
  uint64_t symbols_dropped_stale() const { return symbols_dropped_stale_; }
  uint64_t packets_out() const { return packets_out_; }
  uint64_t resets() const { return resets_; }
  size_t rows_in_flight() const { return rows_.size(); }

 private:
  struct Row {
    std::map<uint64_t, uint8_t> coeffs;  // virtual seq -> coefficient
    std::vector<uint8_t> payload;
    uint64_t first_seen_ms = 0;
  };

  uint64_t unwrap(uint32_t s) const;
  // Genuinely-unrecoverable floor (newest_v_ - horizon_); admit/drop checks
  // use this rather than base_, which lags at newest_v_ during the opening
  // horizon after a join. See sw_decoder.cpp for the join-loss bug this fixes.
  uint64_t live_floor() const;
  void reset_state(uint64_t v);
  void advance(uint64_t newest_candidate);
  // Reduce r against existing pivot rows, normalize, insert. Newly solved
  // (seq, payload) pairs are appended to solved.
  void insert_row(Row r, std::vector<std::pair<uint64_t, std::vector<uint8_t>>>& solved);
  // Deliver symbol v, register known, substitute into rows, cascade.
  void ingest(uint64_t v, std::vector<uint8_t> sym, bool source,
              std::vector<std::vector<uint8_t>>& out);
  void unpack_symbol(const uint8_t* sym, std::vector<std::vector<uint8_t>>& out);

  SwConfig cfg_;
  uint64_t horizon_;
  bool have_seq_ = false;
  uint64_t newest_v_ = 0;  // highest virtual seq seen or implied
  uint64_t base_ = 0;      // state floor (inclusive)
  std::map<uint64_t, std::vector<uint8_t>> known_;  // vseq -> payload
  std::map<uint64_t, Row> rows_;                    // pivot vseq -> row
  std::set<uint64_t> recovered_await_src_;  // recovered, direct copy not yet seen

  uint64_t syms_delivered_ = 0, syms_recovered_ = 0, syms_abandoned_ = 0;
  uint64_t syms_recovered_arrived_ = 0;
  uint64_t symbols_in_ = 0, symbols_dropped_bad_cfg_ = 0;
  uint64_t symbols_dropped_stale_ = 0, packets_out_ = 0, resets_ = 0;
  int repair_window_hwm_ = 0;

  bool wm_open_ = false, wm_valid_ = false;
  uint64_t wm_ = 0;
  uint64_t syms_abandoned_stale_ = 0;

  // --- loss episodes (LossEpisode) ---
  struct RepairSpan {
    uint64_t ws = 0, we = 0;  // covered vseqs [ws, we)
    uint32_t key = 0;         // repair_key: (ws, key) identifies the repair
  };
  std::deque<RepairSpan> repairs_seen_;  // admitted repairs, oldest first
  LossEpisode ep_;                       // the open episode (ep_open_)
  uint64_t ep_last_ = 0;                 // its last missing seq
  bool ep_open_ = false;
  std::vector<LossEpisode> episodes_;    // closed, awaiting take_episodes()
  uint64_t episode_window() const;
  void note_repair(uint64_t ws, uint64_t we, uint32_t key);
  // Per evicted seq: books it into the open episode (or opens one).
  void note_missing(uint64_t v, bool recovered, bool stale);
  // Closes the open episode if eviction has passed it, prunes repairs_seen_.
  void settle_episodes(uint64_t evict_end);

  // Stale boundary in the form ArrivalTracker::advance() takes: everything
  // while a boundary is open, seqs <= wm_ once closed, nothing when inactive.
  uint64_t arr_stale_end() const {
    return wm_open_ ? ~0ull : (wm_valid_ ? wm_ + 1 : 0);
  }
  ArrivalTracker arr_;
};

}  // namespace mabur
