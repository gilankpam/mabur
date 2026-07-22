#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/frag_reassembler.h"
#include "mabur/sw_decoder.h"
#include "mabur/uep_encoder.h"

namespace mabur {

struct DecodedRtp {
  uint8_t stream_id = 0;
  std::vector<uint8_t> pkt;
};

// Receiver mirror of UepEncoder: route a body by its SBI stream_id to that
// layer's sbi_unpack -> SwDecoder -> FragReassembler chain. Multi-card merge
// needs no dedup step: duplicate bodies/symbols (the same air frame heard by
// several cards) are idempotent end-to-end via seq identity and SwDecoder's
// GE-redundancy dedup. Callers pass now_ms (monotonic).
class UepDecoder {
 public:
  UepDecoder(const std::array<UepLayerCfg, 4>& layers,
             uint64_t decode_deadline_ms = 200, uint32_t seq_horizon = 0);

  std::vector<DecodedRtp> add_body(const uint8_t* body, size_t len,
                                   uint64_t now_ms);

  // Expires stale sliding-window rows/FRAG entries on every layer. Call ~1 Hz.
  void poll(uint64_t now_ms);

  // Switches between narrow (RTP, completed-unit reassembly) and wide
  // (frame-shm, raw-fragment passthrough) FRAG decoding. Rebuilds every
  // layer's FragReassembler at the new width and resets delivery-window
  // continuity, since a format flip invalidates any half-assembled units.
  void set_wide_frag(bool wide);

  struct LayerStats {
    uint64_t bodies = 0, subblocks_failed = 0, syms_delivered = 0,
             syms_recovered = 0, syms_abandoned = 0, packets_out = 0,
             frag_evicted = 0;
    // diagnostic depth (SwDecoder internals)
    uint64_t symbols_in = 0, symbols_stale = 0, symbols_bad_cfg = 0;
    size_t rows_in_flight = 0;
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
    // FRAG entries older than the decode deadline can never complete (their
    // missing fragments' rows have expired) — evict by that same age.
    Layer(const UepLayerCfg& cfg, uint64_t decode_deadline_ms, uint32_t seq_horizon)
        : env_size(static_cast<int>(sw::kSwHeaderLen) + cfg.fec.symbol_size),
          sw(cfg.fec, seq_horizon),
          reasm(512, decode_deadline_ms) {}
    int env_size;
    SwDecoder sw;
    FragReassembler reasm;
    uint64_t bodies = 0, subblocks_failed = 0;
    // delivery window
    bool has_last_seq = false;
    uint16_t last_seq = 0;
    uint64_t win_delivered = 0, win_expected = 0;
  };
  // Shared delivery-window accounting, called on each completed unit
  // (narrow path) or each last-fragment arrival (wide path) so the FRAG-seq
  // continuity logic stays single-sourced.
  void note_delivery(Layer& l, uint16_t seq);

  std::array<Layer, 4> layers_;
  uint64_t decode_deadline_ms_;
  uint64_t bodies_misrouted_ = 0;
  bool wide_frag_ = false;
};

}  // namespace mabur
