#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "mabur/sw_encoder.h"  // SwConfig

namespace mabur {

// Streaming decoder for SwEncoder envelopes. Sources deliver IMMEDIATELY
// and register as known; repairs reduce against known symbols, then join an
// incremental Gaussian elimination over the missing seqs — a row down to one
// coefficient is a recovered symbol (delivered late, then substituted back,
// which can cascade). No in-order contract: symbols are self-contained
// (packets never span symbols) and FragReassembler tolerates any arrival
// order, so there is no reorder stage — that's where the interleaver's
// latency went.
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
  std::vector<std::vector<uint8_t>> add_symbol(const uint8_t* env, size_t len,
                                               uint64_t now_ms);

  // Drops repair rows first seen more than deadline_ms ago. Call ~1 Hz.
  // Precondition: now_ms monotonic non-decreasing.
  int expire_rows_older_than(uint64_t deadline_ms, uint64_t now_ms);

  uint64_t syms_delivered() const { return syms_delivered_; }
  uint64_t syms_recovered() const { return syms_recovered_; }
  uint64_t syms_abandoned() const { return syms_abandoned_; }
  uint64_t symbols_in() const { return symbols_in_; }
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

  uint64_t syms_delivered_ = 0, syms_recovered_ = 0, syms_abandoned_ = 0;
  uint64_t symbols_in_ = 0, symbols_dropped_bad_cfg_ = 0;
  uint64_t symbols_dropped_stale_ = 0, packets_out_ = 0, resets_ = 0;
};

}  // namespace mabur
