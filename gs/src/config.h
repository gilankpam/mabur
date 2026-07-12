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

/// FEC configuration: Reed-Solomon parameters and block aging.
struct FecCfg {
  int k = 8;
  int symbol_size = 64;
  int block_max_age_ms = 2000;
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
};

/// Video output destination.
struct VideoOutCfg {
  std::string host = "127.0.0.1";
  int port = 5600;
};

/// Ground station configuration: radio, FEC, link, and video output.
struct Config {
  RadioCfg radio;
  FecCfg fec;
  LinkCfg link;
  VideoOutCfg video_out;

  /// Builds decoder configuration with per-stream RS and UEP overhead.
  std::array<mabur::UepLayerCfg, 4> uep_layers() const;
};

/// Loads configuration from a JSON file (MABUR_GS_BUNDLE_DIR/maburgs.default.json).
/// Fail-fast: missing keys use struct defaults; unknown keys, out-of-range values,
/// or missing file throw std::runtime_error("config: <field>: <why>").
Config load_config(const std::string& path);

}  // namespace maburgs
