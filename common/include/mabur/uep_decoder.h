#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/frag_reassembler.h"
#include "mabur/rs_decoder.h"
#include "mabur/uep_encoder.h"

namespace mabur {

struct DecodedRtp {
  uint8_t stream_id = 0;
  std::vector<uint8_t> pkt;
};

// Receiver mirror of UepEncoder — port of svc_uep_fec.py's SvcUepDecoder
// (fragment=True): route a body by its SBI stream_id to that layer's
// sbi_unpack -> RsDecoder -> FragReassembler chain. Duplicate bodies/symbols
// (the same air frame heard by several cards) are idempotent end-to-end, so
// multi-card merge needs no dedup step. Callers pass now_ms (monotonic).
class UepDecoder {
 public:
  UepDecoder(const std::array<UepLayerCfg, 4>& layers,
             uint64_t block_max_age_ms = 2000);

  std::vector<DecodedRtp> add_body(const uint8_t* body, size_t len,
                                   uint64_t now_ms);

  // Expires RS blocks older than block_max_age_ms on every layer. Call ~1 Hz.
  void poll(uint64_t now_ms);

  struct LayerStats {
    uint64_t bodies = 0, subblocks_failed = 0, blocks_decoded = 0,
             blocks_unrecoverable = 0, packets_out = 0, frag_evicted = 0;
  };
  LayerStats stats(int sid) const;
  uint64_t bodies_misrouted() const { return bodies_misrouted_; }

  // Post-FEC delivery over a resettable window, from FRAG-seq continuity of
  // completed packets: percent [0,100] delivered of expected; 100 when the
  // window saw no traffic. Reordered/duplicate completions (backward gap)
  // count as delivered without inflating expected.
  int window_delivery_pct(int sid) const;
  void reset_window();

  // Raw window counters {delivered, expected} for stream sid — callers that
  // combine streams (e.g. residual loss over the never-shed base layers)
  // need the counts, not the rounded percent.
  std::pair<uint64_t, uint64_t> window_counts(int sid) const;

 private:
  struct Layer {
    explicit Layer(const UepLayerCfg& cfg)
        : env_size(11 + cfg.fec.symbol_size), rs(cfg.fec) {}
    int env_size;
    RsDecoder rs;
    FragReassembler reasm;
    uint64_t bodies = 0, subblocks_failed = 0;
    // delivery window
    bool has_last_seq = false;
    uint16_t last_seq = 0;
    uint64_t win_delivered = 0, win_expected = 0;
  };
  std::array<Layer, 4> layers_;
  uint64_t block_max_age_ms_;
  uint64_t bodies_misrouted_ = 0;
};

}  // namespace mabur
