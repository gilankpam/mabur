#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "mabur/profile.h"
#include "mabur/uep_encoder.h"

namespace mabur {

struct RadioCfg {
  uint16_t usb_vid = 0x0bda;
  uint16_t usb_pid = 0;  // 0 = scan
  uint8_t channel = 149;
  uint8_t width = 20;
  // How bring-up programs TX power:
  //   "offset" — program the wall-equalized per-rate diff table
  //              (SetTxPowerRateDiffs) once, then zero the global offset
  //              once. Power is constant for the life of the process.
  //   "none"   — never touch power (efuse table as-is, streamtx-proven).
  // There is no runtime power control: no per-op power, no thermal derate.
  // Spec 2026-08-12-constant-txpower-design.md.
  std::string power_mode = "none";
  // Wall-equalization inputs (Task 9): measured per-rate clean-air TXAGC
  // ceilings and the plan derived from them. rate_walls_idx is REQUIRED
  // when power_mode == "offset" (the plan can't be built without it);
  // otherwise it may be left absent/default. See power_plan.h for the
  // diff[r] = walls[r] - m - base_ref_idx formulation.
  std::array<int, 8> rate_walls_idx = {0, 0, 0, 0, 0, 0, 0, 0};
  int legacy_wall_idx = 91;
  double wall_margin_db = 1.0;
  int base_ref_idx = 53;
  // Parallel USB sender threads (URBs in flight). The 8822E flow-controls
  // sync bulk-OUT URBs (~0.4 ms acceptance handshake + FIFO drain), so a
  // single blocking sender caps air throughput at ~26 Mbps regardless of
  // MCS; ~4 saturate (linkbench bisect 2026-07-14, devourer
  // docs/aggregation.md). 1 = strict on-air frame order (>1 can swap
  // ≤3-frame URB batches, which the seq-addressed FEC datapath and
  // the GS max-seq delivery accounting both tolerate).
  int tx_threads = 4;
};

struct FecCfg {
  // Per-layer symbol size (stream 0..3). JSON accepts a scalar (fans out)
  // or a 4-array. Big symbols suit the bulk video layers (1..3): fewer
  // symbols lost per body, window spans more airtime, cheaper GF per byte
  // (burst_sim table, spec 2026-07-15). Layer 0 (critical NALs) stays
  // small so VPS/SPS/PPS seal without padding/latency.
  std::array<int, 4> symbol_size = {64, 64, 64, 64};
  // Sliding-window burst budget: a layer at overhead ov survives a hole of
  // up to L <= window*ov/(1+ov) consecutive lost symbols. 128 lets the
  // ov-0.25 layer survive one full bpb-16 body loss.
  int window = 128;
  std::array<int, 4> blocks_per_body = {4, 8, 16, 16};
  double base_overhead = 0.25;
  int flush_ms = 15;
};

struct WaybeamCfg {
  std::string host = "127.0.0.1";
  int port = 80;
  std::string idr_path = "/request/idr";  // waybeam IDR route (bench-confirmed: GET -> {"ok":true,"data":{"idr":true}})
  int bitrate_min_kbps = 1000, bitrate_max_kbps = 20000;
  double airtime_budget = 0.65;
  int roi_threshold_kbps = 3000;
  int roi_qp_low = 8, roi_qp_normal = 0;
};

struct LinkCfg {
  uint32_t vtx_id = 1;
  int failsafe_ms = 1000;
  int rendezvous_ms = 30000;
  int tick_ms = 100;
  // Agent-loop wake period for draining queued RCFs (spec 2026-08-14
  // fade-demote §3b). Decoupled from tick_ms: op actuation latency is
  // U(0, rc_drain_ms) while ALL per-tick housekeeping (USB health polls,
  // state timers, congestion guard, watchdog, stats/telem) stays on
  // tick_ms. >= tick_ms reproduces the legacy single-cadence loop.
  int rc_drain_ms = 5;
};

struct MspCfg {
  bool enable = false;
  std::string serial = "/dev/ttyS2";
  int baud = 115200;
  double update_rate_hz = 1.0;  // ceiling on forwarded snapshots
  int symbol_size = 1312;       // one snapshot per symbol
  int window = 16;
  double overhead = 1.0;
};

struct Config {
  RadioCfg radio;
  FecCfg fec;
  WaybeamCfg waybeam;
  LinkCfg link;
  MspCfg msp;
  // Name of waybeam's frame-shm ring (its outgoing.server =
  // frame-shm://<name>), the one and only video ingest.
  std::string frame_ring_name = "mabur_f";
  std::array<UepLayerCfg, 4> uep_layers() const;
};

// Loads and validates a mabur.json config file. Missing keys fall back to
// the struct defaults above; unknown keys and out-of-range values throw
// std::runtime_error("config: <field>: <why>"). Missing file throws too.
Config load_config(const std::string& path);

}  // namespace mabur
