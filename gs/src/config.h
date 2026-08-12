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
  // bypassed entirely and every RCF commands exactly this MCS/FEC overhead
  // (HT, 20 MHz). Rendezvous/keep-alive/failsafe machinery is unaffected.
  // For bench debugging with a fixed operating point (2026-07-12).
  int static_mcs = -1;
  double static_overhead = 0.25;

  // Measured-loss ladder controller config (spec
  // docs/superpowers/specs/2026-07-27-ladder-controller-design.md): rungs
  // (post-`max_mcs` filter) plus the util/timing thresholds LadderController
  // decides on. Default ladder is the spec's static feasibility floor —
  // rung 0 is the failsafe every controller starts and falls back to.
  // mcs6 rides ov 0.25 (not the spec's 0.15): with bpb=4 a dead body is a
  // 4-symbol cluster, and 0.15 leaves the s3 window unable to absorb two
  // dead bodies — see docs/mcs6-bench-anomaly.md ov0.25 experiment.
  LadderCfg ladder_cfg{
      {{0, 1.0}, {2, 0.5}, {4, 0.25}, {5, 0.25}, {6, 0.25}, {7, 0.1}}};

  // Dedicated adaptive-link log (spec 2026-08-05 section 5). Dir is
  // overridable for host tests; the device default is the DVR SD card.
  bool ctl_log = false;
  std::string ctl_log_dir = "/media/dvr";
};

/// Video reassembly tuning (PR C: the RTP output destination is gone --
/// video leaves maburgs via the shm AU ring; see AuRingOutCfg):
/// gap_timeout_ms before an unfilled chunk gap is truncated, and lookahead
/// frames ahead of head-of-line before it is force-advanced.
struct VideoCfg {
  int frame_gap_timeout_ms = 50;
  int frame_lookahead = 8;
};

/// MSP DisplayPort OSD side-channel output: whole reassembled snapshots go
/// out as UDP datagrams (maburplay renders them -- see
/// docs/superpowers/specs/2026-08-03-maburplay-msp-osd-design.md).
/// symbol_size/window must match the drone's msp config.
struct MspCfg {
  bool enable = false;
  std::string out_host = "127.0.0.1";
  int out_port = 14560;
  int symbol_size = 1312;
  int window = 16;
};

/// One stats-sideport destination.
struct StatsOut {
  std::string host = "127.0.0.1";
  int port = 8300;
};

/// Stats sideport: periodic UDP JSON datagram with link/FEC/video stats
/// (docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md).
///
/// `out` is the destination list and is ALWAYS non-empty after load: with
/// no `out` key it holds the single legacy host/port pair. It exists
/// because UDP unicast delivers a datagram to exactly one socket
/// (SO_REUSEPORT load-balances, it does not duplicate), so consumers
/// cannot simply share a port -- which is why statsrec has to re-emit, and
/// why maburplay's OSD would otherwise depend on statsrec staying alive.
struct StatsCfg {
  bool enable = false;
  int interval_ms = 500;  // clamped to [100, 10000] at load
  std::vector<StatsOut> out{StatsOut{}};
};

/// Shared-memory AU ring for the native player / ausniff gate
/// (docs/superpowers/specs/2026-08-02-gs-player-au-ring-design.md).
struct AuRingOutCfg {
  bool enable = false;
  std::string path = "/dev/shm/mabur-au";
  std::string socket = "/run/mabur-au.sock";
  int slot_kb = 512;    // payload capacity per slot
  int slot_count = 16;  // 16 x 512 KiB = 8 MiB default ring
};

/// Ground station configuration: radio, FEC, link, video reassembly, AU ring, MSP OSD.
struct Config {
  RadioCfg radio;
  FecCfg fec;
  LinkCfg link;
  VideoCfg video;
  MspCfg msp;
  StatsCfg stats;
  AuRingOutCfg au_ring;

  /// Builds decoder configuration with per-stream RS and UEP overhead.
  std::array<mabur::UepLayerCfg, 4> uep_layers() const;
};

/// Loads configuration from a JSON file (MABUR_GS_BUNDLE_DIR/maburgs.default.json).
/// Fail-fast: missing keys use struct defaults; unknown keys, out-of-range values,
/// or missing file throw std::runtime_error("config: <field>: <why>").
Config load_config(const std::string& path);

}  // namespace maburgs
