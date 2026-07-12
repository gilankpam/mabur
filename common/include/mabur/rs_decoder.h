#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "mabur/rs_encoder.h"

namespace mabur {

// Streaming decoder for RsEncoder's wire envelopes. Byte-exact port of
// devourer/tools/precoder/stream_fec_rs.py's RsDecoder with an explicit
// clock: callers pass now_ms (monotonic) so behavior is deterministic in
// tests. Any k distinct ESIs solve a block (systematic fast path when they
// are exactly 0..k-1); duplicate symbols are ignored, which is what makes
// multi-card merge dedup-free. cfg.overhead is unused on the decode side.
class RsDecoder {
 public:
  explicit RsDecoder(const RsConfig& cfg);

  // Feeds one received envelope. Returns the block's recovered packets the
  // moment its k-th distinct symbol arrives (empty otherwise). Malformed or
  // config-mismatched envelopes are counted and dropped, never applied.
  std::vector<std::vector<uint8_t>> add_symbol(const uint8_t* env, size_t len,
                                               uint64_t now_ms);

  // Drops blocks first seen more than max_age_ms ago; undecoded ones count
  // as unrecoverable. Returns how many were unrecoverable.
  // Precondition: now_ms must be monotonic non-decreasing across calls.
  int expire_blocks_older_than(uint64_t max_age_ms, uint64_t now_ms);

  uint64_t blocks_decoded() const { return blocks_decoded_; }
  uint64_t blocks_unrecoverable() const { return blocks_unrecoverable_; }
  uint64_t symbols_in() const { return symbols_in_; }
  uint64_t symbols_dropped_bad_cfg() const { return symbols_dropped_bad_cfg_; }
  uint64_t symbols_dropped_stale_block() const { return symbols_dropped_stale_block_; }
  uint64_t packets_out() const { return packets_out_; }
  size_t in_flight_blocks() const { return blocks_.size(); }

 private:
  struct Block {
    int k = 0, n = 0, kreal = 0;
    uint64_t first_seen_ms = 0;
    std::map<int, std::vector<uint8_t>> symbols;  // esi -> payload, sorted
    bool decoded = false;
  };

  std::vector<std::vector<uint8_t>> solve(Block& st);
  std::vector<std::vector<uint8_t>> unpack(
      const std::vector<std::vector<uint8_t>>& source, int kreal) const;

  RsConfig cfg_;
  std::map<uint16_t, Block> blocks_;
  uint64_t blocks_decoded_ = 0, blocks_unrecoverable_ = 0;
  uint64_t symbols_in_ = 0, symbols_dropped_bad_cfg_ = 0;
  uint64_t symbols_dropped_stale_block_ = 0, packets_out_ = 0;
};

}  // namespace mabur
