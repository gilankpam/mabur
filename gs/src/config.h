#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "ladder_controller.h"
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

/// Link-layer configuration: VTX ID, feedback rate, keepalive.
struct LinkCfg {
  uint32_t vtx_id = 1;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  // Static-link mode: when static_mcs >= 0 the adaptive controller is
  // bypassed entirely and every RCF commands exactly this op (HT, 20 MHz).
  // Rendezvous/keep-alive/failsafe machinery is unaffected. For bench
  // debugging with a fixed operating point (2026-07-12).
  int static_mcs = -1;
  double static_overhead = 0.25;
  // qdB power offset from the calibrated baseline (RCF wire semantics,
  // rc_proto bias-64); used only when static_mcs >= 0.
  int static_offset_qdb = 0;

  // Measured-loss ladder controller config (spec
  // docs/superpowers/specs/2026-07-27-ladder-controller-design.md): rungs
  // (post-`max_mcs` filter) plus the util/timing thresholds LadderController
  // decides on. Default ladder is the spec's static feasibility floor —
  // rung 0 is the failsafe every controller starts and falls back to.
  LadderCfg ladder_cfg{
      {{0, 1.0}, {2, 0.5}, {4, 0.25}, {5, 0.25}, {6, 0.15}, {7, 0.1}}};
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

/// Stats sideport: periodic UDP JSON datagram with link/FEC/video stats
/// (docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md).
struct StatsCfg {
  bool enable = false;
  std::string host = "127.0.0.1";
  int port = 8300;
  int interval_ms = 500;  // clamped to [100, 10000] at load
};

/// Ground station configuration: radio, FEC, link, video output, and MSP OSD output.
struct Config {
  RadioCfg radio;
  FecCfg fec;
  LinkCfg link;
  VideoOutCfg video_out;
  MspCfg msp;
  StatsCfg stats;

  /// Builds decoder configuration with per-stream RS and UEP overhead.
  std::array<mabur::UepLayerCfg, 4> uep_layers() const;
};

/// Loads configuration from a JSON file (MABUR_GS_BUNDLE_DIR/maburgs.default.json).
/// Fail-fast: missing keys use struct defaults; unknown keys, out-of-range values,
/// or missing file throw std::runtime_error("config: <field>: <why>").
Config load_config(const std::string& path);

}  // namespace maburgs
