#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mabur/uep_encoder.h"

namespace maburgs {

/// USB device identifier for a radio card (VID, PID, instance index).
struct CardCfg {
  uint16_t usb_vid = 0x0bda;
  uint16_t usb_pid = 0;
  int index = 0;
};

/// Radio hardware: channel, bandwidth, cards, and transmit card selection.
struct RadioCfg {
  uint8_t channel = 149;
  uint8_t width = 20;
  std::vector<CardCfg> cards;  // default: one auto-scan card
  int tx_card = -1;            // -1 = auto-select (Plan 2)
};

/// FEC configuration: sliding-window decoder parameters.
struct FecCfg {
  // Per-layer symbol size (stream 0..3); must match the drone's fec config
  // layer-for-layer or that layer's SBI framing misparses on the receive
  // side: subblocks_failed (sbf) climbs for that layer, or — if the
  // mismatched stride exceeds the body region — bodies increments while
  // symbols_in (si) stays frozen at 0. Not silent, but not bad_cfg either;
  // symbols never reach the sliding-window decoder to be flagged there.
  // JSON: scalar fans out, or 4-array.
  std::array<int, 4> symbol_size = {64, 64, 64, 64};
  int decode_deadline_ms = 200;
  int seq_horizon = 512;
};

/// Link-layer configuration: VTX ID, feedback rate, keepalive, silence timeout.
struct LinkCfg {
  uint32_t vtx_id = 1;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  int video_silence_ms = 3000;
  // Source bitrate the controller's energy model plans for (Python
  // controller.py src_bitrate default 4 Mbps). Rungs whose effective PHY
  // rate cannot carry src*(1+overhead) are infeasible, so this sets the
  // floor rung the ladder converges to: at 4 the controller parks around
  // mcs2 (~6-8 Mbps video); declare the bitrate you actually want and it
  // selects the cheapest rung that carries it (bench 2026-07-12: 17 ->
  // mcs5 on a 33 dB NLOS link).
  double src_bitrate_mbps = 4.0;
  // Energy-model SNR margin (Python controller.py margin_db default 2.0).
  // Doubles as the calibration shim for chips whose reported per-frame SNR
  // is optimistic vs the real channel (8822E reports 35-40 dB on survivor
  // frames while delivering 10% at agc0 — bench 2026-07-12): raising it
  // forces the resolver to command real TX gain despite the inflated
  // path-loss estimate. The durable fix (delivery-closed power loop) is
  // v1.1 work.
  double margin_db = 2.0;
  // Static-link mode: when static_mcs >= 0 the adaptive controller is
  // bypassed entirely and every RCF commands exactly this op (HT, 20 MHz).
  // Rendezvous/keep-alive/failsafe machinery is unaffected. For bench
  // debugging with a fixed operating point (2026-07-12).
  int static_mcs = -1;
  double static_overhead = 0.25;
  // qdB power offset from the calibrated baseline (RCF wire semantics,
  // rc_proto bias-64); used only when static_mcs >= 0.
  int static_offset_qdb = 0;
  // Controller qdB offset rail + PA-draw base index (see ControllerConfig).
  int min_offset_qdb = -40;
  int max_offset_qdb = 0;
  int base_ref_idx = 53;
};

/// Video output destination.
struct VideoOutCfg {
  std::string host = "127.0.0.1";
  int port = 5600;
  // FrameStream tuning for the session-negotiated frame-wire tail (Task 10):
  // gap_timeout_ms before an unfilled chunk gap is truncated, and lookahead
  // frames ahead of head-of-line before it is force-advanced.
  int frame_gap_timeout_ms = 50;
  int frame_lookahead = 8;
};

/// MSP DisplayPort OSD side-channel output. symbol_size/window must match the
/// drone's msp config.
struct MspCfg {
  bool enable = false;
  std::string out_host = "127.0.0.1";
  int out_port = 14560;
  int symbol_size = 1312;
  int window = 16;
  std::string render = "udp";      // "udp" | "shm"
  std::string shm_name = "msp";    // shm mode: region name (== osd.json widget name)
  int shm_x_offset = 0;            // shm mode: scale-to-fill inset (px)
  int shm_y_offset = 0;
};

/// Ground station configuration: radio, FEC, link, video output, and MSP OSD output.
struct Config {
  RadioCfg radio;
  FecCfg fec;
  LinkCfg link;
  VideoOutCfg video_out;
  MspCfg msp;

  /// Builds decoder configuration with per-stream RS and UEP overhead.
  std::array<mabur::UepLayerCfg, 4> uep_layers() const;
};

/// Loads configuration from a JSON file (MABUR_GS_BUNDLE_DIR/maburgs.default.json).
/// Fail-fast: missing keys use struct defaults; unknown keys, out-of-range values,
/// or missing file throw std::runtime_error("config: <field>: <why>").
Config load_config(const std::string& path);

}  // namespace maburgs
