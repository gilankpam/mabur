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
  // Per-layer symbol size (stream 0..1); must match the drone's fec config
  // layer-for-layer or that layer's SBI framing misparses on the receive
  // side: subblocks_failed (sbf) climbs for that layer, or — if the
  // mismatched stride exceeds the body region — bodies increments while
  // symbols_in (si) stays frozen at 0. Not silent, but not bad_cfg either;
  // symbols never reach the sliding-window decoder to be flagged there.
  // JSON: scalar fans out, or 2-array.
  std::array<int, 2> symbol_size = {64, 64};
  int seq_horizon = 512;
};

/// Link-layer configuration: VTX ID, feedback rate, keepalive.
struct LinkCfg {
  uint32_t vtx_id = 1;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  // RCF slotting (gs-uplink-self-blanking findings 2026-09-02): while video
  // flows, hold each control frame until the next AU completes so the send
  // lands in the drone's inter-AU idle instead of killing the next PPDU on
  // both RX cards. Max hold before sending anyway; 0 disables.
  int rcf_slot_hold_ms = 30;
  // Static-link mode: when static_mcs >= 0 the adaptive controller is
  // bypassed entirely and every RCF commands exactly this MCS/FEC overhead
  // (HT, 20 MHz). Rendezvous/keep-alive/failsafe machinery is unaffected.
  // For bench debugging with a fixed operating point (2026-07-12).
  int static_mcs = -1;
  // Actual-air overhead (airtime-balance-uep): literal, not a scaled cmd
  // value. Default 0.5 is the old cmd-value default (0.25) x2 -- see the
  // rule note in gs/src/config.cpp's load_config. Same-rate-fixed-pairs
  // (Task 3): split into a base/enh pair, both defaulting to the old
  // scalar's value -- Task 4 gives them independent semantics.
  double static_overhead_base = 0.5;
  double static_overhead_enh = 0.5;

  // Measured-loss ladder controller config (spec
  // docs/superpowers/specs/2026-07-27-ladder-controller-design.md): rungs
  // (post-`max_mcs` filter) plus the util/timing thresholds LadderController
  // decides on. Default ladder is the spec's static feasibility floor —
  // rung 0 is the failsafe every controller starts and falls back to.
  // mcs6 rides ov 0.5, i.e. cmd-value 0.25 (not the spec's cmd-value
  // 0.15): with bpb=4 a dead body is a 4-symbol cluster, and 0.15 leaves
  // the s3 window unable to absorb two dead bodies — see
  // docs/mcs6-bench-anomaly.md ov0.25 experiment. Values are actual-air
  // overhead (airtime-balance-uep, global rule: every cmd-value default
  // this migration touches x2) — {0,1.0}->{0,2.0}, {2,0.5}->{2,1.0},
  // {4,0.25}->{4,0.5}, {5,0.25}->{5,0.5}, {6,0.25}->{6,0.5}, {7,0.1}->{7,0.2}.
  // Same-rate-fixed-pairs (Task 3): overhead is now a base/enh pair; the
  // struct default duplicates each rung's value into both fields.
  LadderCfg ladder_cfg{{{0, 2.0, 2.0},
                        {2, 1.0, 1.0},
                        {4, 0.5, 0.5},
                        {5, 0.5, 0.5},
                        {6, 0.5, 0.5},
                        {7, 0.2, 0.2}}};

  // Dedicated adaptive-link log (spec 2026-08-05 section 5). Dir is
  // overridable for host tests; the device default is the DVR SD card.
  bool ctl_log = false;
  std::string ctl_log_dir = "/media/dvr";

  // S-line cadence, ms (2026-08-15). Was hardcoded at 1000 and shared with
  // main.cpp's stderr stats line; the two are now separate timers, because
  // the ctl log is the adaptive-link instrument and wants to run FASTER than
  // a human-readable log should. The floor exists for fade-trigger tuning:
  // the predictive trigger decides every feedback_ms (50) against a 300 ms
  // fast tau and a trigger_ms (300) sustain, so a 1 Hz record cannot tell a
  // sustained fade from a sub-second transient — the exact question the
  // 2026-08-15 sweep of 41 flights could not answer. 100 ms or finer
  // resolves it. Cost is linear: ~80 B per S line, so 50 ms ~= 1.6 MB per
  // 6-minute flight against a 58 GB DVR card.
  int ctl_log_period_ms = 1000;

  // R-line emission cadence for the per-rung EWMA store (spec 2026-08-13).
  // The store's own tuning (half_life_samples) lives in
  // ladder_cfg.rung_stats; this is main.cpp's log cadence only.
  int rung_log_period_s = 10;
};

/// Video reassembly tuning (PR C: the RTP output destination is gone --
/// video leaves maburgs via the shm AU ring; see AuRingOutCfg):
/// gap_timeout_ms before an unfilled chunk gap is truncated, and lookahead
/// frames ahead of head-of-line before it is force-advanced.
struct VideoCfg {
  int frame_gap_timeout_ms = 50;
  // Rate-aware ceiling for the gap timeout (gap_timeout_policy.h): at low
  // rungs the TX window's repair lifetime outlives the fixed floor, so the
  // timeout stretches toward the window's time-span, clamped here. 0
  // disables the stretch (fixed floor). Must stay below FrameStream's
  // 500 ms stall-reset backstop; the config range enforces that.
  int frame_gap_timeout_max_ms = 150;
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

  /// Builds decoder configuration with per-stream RS and UEP overhead
  /// (2 streams since the airtime-balance-uep fold-in).
  std::array<mabur::UepLayerCfg, 2> uep_layers() const;
};

/// Loads configuration from a JSON file (MABUR_GS_BUNDLE_DIR/maburgs.default.json).
/// Fail-fast: missing keys use struct defaults; unknown keys, out-of-range values,
/// or missing file throw std::runtime_error("config: <field>: <why>").
Config load_config(const std::string& path);

}  // namespace maburgs
