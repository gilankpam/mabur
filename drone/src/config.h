#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mabur/profile.h"
#include "mabur/uep_encoder.h"

namespace mabur {

struct RadioCfg {
  uint16_t usb_vid = 0x0bda;
  uint16_t usb_pid = 0;  // 0 = scan
  uint8_t channel = 149;
  uint8_t width = 20;
  std::vector<uint8_t> bw_set = {20};
  int max_txagc = 63;
  int thermal_max_delta = 25;
  // How apply_op drives TX power (bench 2026-07-13, docs/handover-video-
  // delivery.md §5.1: the flat override costs ~25dB-equivalent of delivery
  // at high MCS vs the efuse per-rate table):
  //   "override" — flat SetTxPowerIndexOverride(pwr_idx), Python parity.
  //   "offset"   — SetTxPowerOffsetQdb(power_offset_qdb): shape-preserving
  //                trim on the calibrated table; commanded pwr_idx ignored.
  //   "none"     — never touch power (efuse table as-is, streamtx-proven).
  // offset/none bypass the thermal derate (it acts via pwr_idx).
  std::string power_mode = "override";
  int power_offset_qdb = 0;
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
  int symbol_size = 64;
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
};

struct Config {
  RadioCfg radio;
  FecCfg fec;
  WaybeamCfg waybeam;
  LinkCfg link;
  std::string ring_name = "mabur";
  rc::FlagPolicy flags;
  std::array<int8_t, 4> power_offset_db = {0, 0, 0, 0};

  std::array<UepLayerCfg, 4> uep_layers() const;
};

// Loads and validates a mabur.json config file. Missing keys fall back to
// the struct defaults above; unknown keys and out-of-range values throw
// std::runtime_error("config: <field>: <why>"). Missing file throws too.
Config load_config(const std::string& path);

}  // namespace mabur
