#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include "mabur/node.h"
#include "mabur/uep_decoder.h"

namespace maburgs {

// Rolling per-card link state, maintained from CRC-clean frames only (a
// corrupt frame's seq and phystatus are untrustworthy). rssi_a/b_ema track
// the two RX chains; snr_ema tracks the best chain. (An older claim that
// chain A reads off-scale 128-131 on the 8822E was refuted on hardware —
// docs/chain-a-rssi-validation-handoff.md.)
struct CardTrack {
  uint64_t frames = 0, crc_fail = 0, rc_frames = 0, video_bodies = 0;
  uint64_t seq_expected = 0, seq_received = 0;
  double rssi_a_ema = 0.0, rssi_b_ema = 0.0, snr_ema = 0.0;
  // Per-RX-chain SNR EMAs (path A / path B). A dead path B = no MRC
  // diversity, worth real dB in NLOS multipath (2026-07-12 perf-gap triage).
  double snr_a_ema = 0.0, snr_b_ema = 0.0;
  bool has_ema = false, has_seq = false;
  uint16_t last_seq = 0;
  uint64_t last_frame_us = 0;
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

  Aggregator(const std::array<mabur::UepLayerCfg, 4>& layers,
             uint64_t decode_deadline_ms, uint32_t seq_horizon, int n_cards);

  void set_frag_sink(FragSink s) { frag_sink_ = std::move(s); }
  void set_rc_sink(RcSink s) { rc_sink_ = std::move(s); }
  void set_msp_sink(MspSink s) { msp_sink_ = std::move(s); }

  void on_rx_body(const mabur::node::RxBody& m);
  void poll(uint64_t now_ms) { dec_.poll(now_ms); }

  const CardTrack& card(int id) const { return cards_[static_cast<size_t>(id)]; }
  int n_cards() const { return static_cast<int>(cards_.size()); }
  mabur::UepDecoder& decoder() { return dec_; }
  uint16_t last_video_seq() const { return last_video_seq_; }
  uint64_t last_video_us() const { return last_video_us_; }
  uint64_t bad_card_msgs() const { return bad_card_msgs_; }

 private:
  mabur::UepDecoder dec_;
  std::vector<CardTrack> cards_;
  FragSink frag_sink_;
  RcSink rc_sink_;
  MspSink msp_sink_;
  uint16_t last_video_seq_ = 0;
  uint64_t last_video_us_ = 0;
  uint64_t bad_card_msgs_ = 0;
};

}  // namespace maburgs
