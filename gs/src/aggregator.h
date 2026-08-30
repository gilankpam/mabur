#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include "mabur/node.h"
#include "mabur/uep_decoder.h"
#ifdef MABUR_LOSS_SIM
#include "loss_sim.h"
#endif

namespace maburgs {

// RF traffic class: stream_id 0..3 map to S0..S3 (video sub-streams), Msp is
// the OSD sideband, Ctrl is RC-magic frames that aren't GS-self-originated
// (e.g. DISC_ACK). Indexes CardTrack::cls[].
enum class RfClass { S0 = 0, S1, S2, S3, Msp, Ctrl };
constexpr int kNumRfClasses = 6;

// Per-class signal track, same shape/semantics as the pooled EMAs below but
// scoped to one RfClass — lets the stats sideport (Task 3/4) report signal
// quality per traffic class instead of only the card-wide pool.
struct ClassTrack {
  uint64_t frames = 0, bytes = 0;
  double rssi_ema = 0, rssi_a_ema = 0, rssi_b_ema = 0;
  double snr_ema = 0, snr_a_ema = 0, snr_b_ema = 0;
  // Per-chain RX EVM EMAs (raw half-dB; dB at the exporter). Folded only
  // from nonzero samples — 0 means "no phy status", not 0 dB — so each has
  // its own validity flag instead of sharing has_ema. evm_ema tracks the
  // per-frame best (most negative) sampled chain, mirroring snr_ema's
  // max-of-chains.
  double evm_ema = 0, evm_a_ema = 0, evm_b_ema = 0;
  bool evm_has = false, evm_a_has = false, evm_b_has = false;
  bool has_ema = false;
};

// Rolling per-card link state, maintained from CRC-clean frames only (a
// corrupt frame's seq and phystatus are untrustworthy). rssi_a/b_ema and
// snr_a/b_ema track the two RX chains; rssi_ema/snr_ema track the per-frame
// best chain. (An older claim that chain A reads off-scale 128-131 on the
// 8822E was refuted on hardware — docs/chain-a-rssi-validation-handoff.md.)
struct CardTrack {
  uint64_t frames = 0, crc_fail = 0, rc_frames = 0, video_bodies = 0;
  uint64_t seq_expected = 0, seq_received = 0;
  uint64_t rx_bytes = 0;  // body bytes, CRC-fail included: what the air carried
  double rssi_ema = 0.0, rssi_a_ema = 0.0, rssi_b_ema = 0.0, snr_ema = 0.0;
  // Per-RX-chain SNR EMAs (path A / path B). A dead path B = no MRC
  // diversity, worth real dB in NLOS multipath (2026-07-12 perf-gap triage).
  double snr_a_ema = 0.0, snr_b_ema = 0.0;
  // Per-chain RX EVM EMAs (raw half-dB; dB at the exporter). Folded only
  // from nonzero samples — 0 means "no phy status", not 0 dB — so each has
  // its own validity flag instead of sharing has_ema. evm_ema tracks the
  // per-frame best (most negative) sampled chain, mirroring snr_ema's
  // max-of-chains.
  double evm_ema = 0, evm_a_ema = 0, evm_b_ema = 0;
  bool evm_has = false, evm_a_has = false, evm_b_has = false;
  bool has_ema = false, has_seq = false;
  uint16_t last_seq = 0;
  uint64_t last_frame_us = 0;
  // Per-RF-class signal tracks (video streams, MSP, ctrl) — pooled EMAs
  // above REMAIN untouched (TxSelector + stderr consume them).
  std::array<ClassTrack, kNumRfClasses> cls{};
  // base+enh pooled RF track (spec 2026-08-15, re-scoped for the airtime-
  // balance-uep split-rate ladder). The RF label source and the predictive
  // fade trigger read THIS, not cls[S0]/cls[S1] alone. base and enh no
  // longer share a PHY rate (base mirrors mcs-1, enh runs the profile mcs),
  // but RSSI/SNR/EVM are channel properties, not rate-dependent ones, and TX
  // power is constant across MCS (spec 2026-08-12-constant-txpower) — so the
  // two streams stay statistically homogeneous and pooling both still beats
  // a single stream's sample count; msp/ctrl are excluded because their mix
  // ratio drifts with rung and shed state. Folded at frame time, NOT
  // blended from cls[S0]/cls[S1] -- those have different sample rates, so
  // no weighted average of them is the EMA of the union.
  ClassTrack rf_pool{};
  // GS-originated RC frames (RCF/DISC, sent to the drone): the GS's own
  // monitor-mode capture hears its own transmission, so these show up on
  // on_rx_body too. Counted here, excluded from every other total
  // (frames/rx_bytes/seq/EMA).
  uint64_t self_frames = 0;
};

// Core-thread router: RC-magic frames go to the control sink (Plan 2's
// agent), everything else through the UepDecoder to the fragment sink (whose
// consumer is FrameStream, the frame assembler). Multi-card
// dedup happens inside the decoder (seq identity / GE-redundancy dedup), so
// bodies from all cards are fed straight in. Single-threaded by contract
// (core thread only).
class Aggregator {
 public:
  using FragSink = std::function<void(const mabur::DecodedFrag&)>;
  using RcSink = std::function<void(uint8_t card_id,
                                    const std::vector<uint8_t>& frame,
                                    uint64_t mono_us)>;
  using MspSink = std::function<void(const uint8_t* body, size_t len,
                                     uint64_t mono_us)>;

  Aggregator(const std::array<mabur::UepLayerCfg, 2>& layers,
             uint32_t seq_horizon, int n_cards);

  void set_frag_sink(FragSink s) { frag_sink_ = std::move(s); }
  void set_rc_sink(RcSink s) { rc_sink_ = std::move(s); }
  void set_msp_sink(MspSink s) { msp_sink_ = std::move(s); }

  void on_rx_body(const mabur::node::RxBody& m);

  const CardTrack& card(int id) const { return cards_[static_cast<size_t>(id)]; }
  int n_cards() const { return static_cast<int>(cards_.size()); }
  mabur::UepDecoder& decoder() { return dec_; }
  uint16_t last_video_seq() const { return last_video_seq_; }
  uint64_t last_video_us() const { return last_video_us_; }
  uint64_t bad_card_msgs() const { return bad_card_msgs_; }

#ifdef MABUR_LOSS_SIM
  // BENCH RIG (MABUR_LOSS_SIM): injected per-stream loss. Inert unless
  // configure() has been called with a nonzero rate.
  LossSim& loss_sim() { return loss_sim_; }
#endif

 private:
  mabur::UepDecoder dec_;
#ifdef MABUR_LOSS_SIM
  LossSim loss_sim_;
#endif
  std::vector<CardTrack> cards_;
  FragSink frag_sink_;
  RcSink rc_sink_;
  MspSink msp_sink_;
  uint16_t last_video_seq_ = 0;
  uint64_t last_video_us_ = 0;
  uint64_t bad_card_msgs_ = 0;
};

}  // namespace maburgs
