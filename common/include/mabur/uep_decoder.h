#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/frag.h"
#include "mabur/sw_decoder.h"
#include "mabur/uep_encoder.h"

namespace mabur {

// One FRAG fragment (6-byte header + chunk) recovered from a layer's
// sliding-window decoder, still to be assembled into a frame by the GS's
// FrameStream.
struct DecodedFrag {
  uint8_t stream_id = 0;
  std::vector<uint8_t> frag;
  uint64_t body_mono_us = 0;  // RX stamp of the body whose arrival emitted
                              // this fragment (repair-recovered fragments
                              // carry the COMPLETING body's stamp — spec'd
                              // approximation). 0 = unknown (tests).
  uint16_t q_ms = 0;          // SBI q_ms of that body (0 = unknown)
  uint16_t enc_us = 0;        // SBI enc_us of that body (0 = unknown)
};

// Receiver mirror of UepEncoder: route a body by its SBI stream_id to that
// layer's sbi_unpack -> SwDecoder chain and emit the fragments that come out.
// Multi-card merge needs no dedup step: duplicate bodies/symbols (the same air
// frame heard by several cards) are idempotent end-to-end via seq identity and
// SwDecoder's GE-redundancy dedup. Callers pass now_ms (monotonic).
//
// Fragments are emitted raw rather than reassembled here: FrameStream does
// per-frame assembly with streaming prefix emission, which a
// complete-units-only reassembler can't do (a frame whose tail is lost must
// still play its head).
class UepDecoder {
 public:
  explicit UepDecoder(const std::array<UepLayerCfg, 2>& layers,
                      uint32_t seq_horizon = 0);

  static constexpr uint8_t kMcsUnknown = 255;

  // Transition-attribution boundary (spec 2026-08-14). Call when the GS
  // commands stream sid onto a new op: new_mcs is the PHY rate the stream
  // is expected at from now on (the op MCS for s0-s2, the probe
  // candidate's for s3 while a probe runs). Arms per-layer watermarks in
  // both seq spaces; add_body()'s rx_mcs then classifies arrivals as
  // pre/post-boundary until the boundary closes (first frame heard at
  // new_mcs) or expires (kBoundaryExpiryMs). The forced first mark (prior
  // cur_mcs unknown) arms with a known cur_mcs but cannot open a boundary
  // (there is no "prev" to have been wrong), so up to ~1 s of session-start
  // loss below the first-completed unit books stale and is excluded from
  // demote inputs — deliberate, since session-start warm-up loss is exactly
  // the debris class this mechanism exists to exclude; early-session
  // abandoned_stale > 0 on the bench is expected, not a bug.
  void mark_transition(int sid, uint8_t new_mcs, uint64_t now_ms);

  // body_crc_ok: the 802.11 FCS verdict for this body. Corrupt bodies are
  // still decoded (per-sub-block CRC salvage is the whole point), but the
  // SBI header's q_ms/enc_us bytes sit OUTSIDE those CRCs, so from a
  // corrupt body they are untrustworthy and degrade to 0 = unknown — the
  // latency accounting's snap-down anchor consumes them and one corrupt
  // duration must never drag its floor.
  std::vector<DecodedFrag> add_body(const uint8_t* body, size_t len,
                                    uint64_t now_ms,
                                    uint8_t rx_mcs = kMcsUnknown,
                                    uint64_t body_mono_us = 0,
                                    bool body_crc_ok = true);

  // Open->close latency (ms) of layer sid's last CLOSED boundary; -1 if a
  // boundary never closed on this layer. Observability only.
  double last_boundary_close_ms(int sid) const;

  // Layer sid's newest virtual seq (SwDecoder::newest_seq passthrough) —
  // the send-rate signal GapTimeoutPolicy differentiates. 0 on bad sid.
  uint64_t newest_seq(int sid) const;
  // Layer sid's observed TX window (SwDecoder::repair_window). 0 on bad sid
  // or before the first repair.
  int repair_window(int sid) const;

  // Drops per-layer decode state — call on a session change, where the peer's
  // seqs restart from an unrelated value.
  void reset_continuity();

  struct LayerStats {
    uint64_t bodies = 0, subblocks_failed = 0, syms_delivered = 0,
             syms_recovered = 0, syms_recovered_arrived = 0,
             syms_abandoned = 0, packets_out = 0;
    // diagnostic depth (SwDecoder internals)
    uint64_t symbols_in = 0, symbols_stale = 0, symbols_bad_cfg = 0;
    size_t rows_in_flight = 0;
    uint64_t syms_abandoned_stale = 0;
  };
  LayerStats stats(int sid) const;
  uint64_t bodies_misrouted() const { return bodies_misrouted_; }

  // NOTE: the packet-level delivery window (window_counts/window_counts_cur/
  // window_delivery_pct/reset_window) was DELETED 2026-09-02. It inferred
  // loss from FRAG-seq gaps, which cannot distinguish a lost unit from one
  // that has not completed yet, so a late sliding-window repair read as
  // 25-33% loss and demoted the ladder ~200 times an hour on a clean bench.
  // Post-FEC loss now comes from syms_abandoned/syms_abandoned_stale below
  // (order-independent), via gs/src/ladder_residual.cpp. That measure
  // strictly dominates: the packet one shared this decoder's join blind spot
  // AND under-reported real loss (a unit missing a MIDDLE fragment but
  // keeping its tail counted as delivered).

 private:
  struct Layer {
    Layer(const UepLayerCfg& cfg, uint32_t seq_horizon)
        : env_size(static_cast<int>(sw::kSwHeaderLen) + cfg.fec.symbol_size),
          sw(cfg.fec, seq_horizon) {}
    int env_size;
    SwDecoder sw;
    uint64_t bodies = 0, subblocks_failed = 0;
    // --- transition-attribution boundary (spec 2026-08-14) ---
    // Kept after the 2026-09-02 delivery-window deletion: these drive the
    // kPre/kPost hint handed to SwDecoder::add_symbol (which is what splits
    // syms_abandoned into stale/current) and the shipped close_ms metric.
    uint8_t cur_mcs = kMcsUnknown;  // expected PHY rate; kMcsUnknown = never set
    bool bnd_armed = false;         // watermark comparisons live
    bool bnd_open = false;          // boundary not yet closed by a kPost body
    uint64_t bnd_arm_ms = 0;
    double bnd_close_ms = -1.0;     // last open->close latency
  };

  std::array<Layer, 2> layers_;
  uint64_t bodies_misrouted_ = 0;
};

}  // namespace mabur
