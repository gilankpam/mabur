#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "mabur/profile.h"
#include "mabur/uep_encoder.h"
#include "venc_cfg.h"  // VencCfg, VENC_RING_NAME — plain C99, host-safe

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
  // Per-layer symbol size (stream 0..1). JSON accepts a scalar (fans out)
  // or a 2-array. Big symbols suit the bulk enhance layer (1): fewer
  // symbols lost per body, window spans more airtime, cheaper GF per byte
  // (burst_sim table, spec 2026-07-15). Layer 0 (critical NALs) stays
  // small so VPS/SPS/PPS seal without padding/latency.
  std::array<int, 2> symbol_size = {64, 64};
  // Sliding-window burst budget: a layer at overhead ov survives a hole of
  // up to L <= window*ov/(1+ov) consecutive lost symbols. 128 lets the
  // ov-0.50 layer (since the 2026-08-29 UEP flatten, was ov-0.25) survive
  // one full bpb-16 body loss with room to spare (L=128*0.50/1.50≈42.7 vs
  // 16 lost symbols; was ≈25.6 at the pre-flatten ov-0.25).
  int window = 128;
  std::array<int, 2> blocks_per_body = {4, 8};
  // Literal air overhead (Task 3, airtime-balance-uep): the fraction of
  // repair bytes over source bytes actually put on air, not a scaled
  // command value — no uep_layer_overhead ladder translation anymore.
  // Default 0.5 is the old effective value at the flattened reference
  // ladder (was 0.25 pre-literal, doubled by the ×2 rule this migration
  // applies everywhere a cmd-overhead default crosses into actual-overhead
  // space).
  double base_overhead = 0.5;
  int flush_ms = 15;
  // Feed grouping: release sealed bodies to the TX writer in groups of N
  // (TxQueue wakeup batching + grouped pool submit) so URBs fill their
  // 3-descriptor cap and A-MPDU aggregates can form, instead of the
  // per-body trickle that ships every body alone (~360 µs metronome,
  // dq-spike findings §17). 0/1 = streaming push (per-body wakeups, the
  // 2026-08-31 shape). Bench lever — default off until the batching+agg
  // compound is accepted end-to-end (watch arrival-jitter EMA: coarser
  // quanta, the 656-rollback lesson).
  int feed_batch = 0;
};

// RcAgent's bitrate/ROI policy knobs (ex-WaybeamCfg; the HTTP fields
// host/port/idr_path went with the waybeam section they lived in — venc is
// in-process now, no control-plane HTTP left to address).
struct EncoderCfg {
  int bitrate_min_kbps = 2000;
  int bitrate_max_kbps = 10000;
  double airtime_budget = 0.60;
  int roi_threshold_kbps = 3000;
  // NEGATIVE, and that is the point: the ROI delta-QP is applied to the
  // centre region, and a LOWER QP means MORE bits there. -24 is the
  // hardware-validated value the shipped bundle carries (configs/, and
  // test_config's bundle assertions); the +8 that used to sit here was a
  // sign-flipped placeholder that would have spent fewer bits on the
  // centre of frame exactly when the link is poorest.
  int roi_qp_low = -24;
  int roi_qp_normal = 0;
};

// Boot-time encoder pipeline config, handed to venc_core_start() as a
// VencCfg (spec 2026-08-28 venc-foldin §3). `core` is the pure-mechanism
// struct venc_cfg.h defines (B3); `debug_port` is mabur-side (B7's thin
// debug endpoint), not part of the vendored VencCfg surface.
struct VencSectionCfg {
  VencCfg core{};
  int debug_port = 8301;
  // Not aggregate-initialised on purpose: VencCfg is a plain C struct, so
  // `VencCfg core{}` alone would zero every field and an absent venc key
  // would hand the encoder fps 0 / 0x0 / gop 0.0 rather than a fallback.
  // venc_cfg_defaults() (drone/venc/venc_cfg.c) is the one table of truth
  // for those values; it leaves sensor_bin empty, which parse_venc treats
  // as a boot failure.
  VencSectionCfg() { venc_cfg_defaults(&core); }
};

struct LinkCfg {
  uint32_t vtx_id = 1;
  int failsafe_ms = 1000;
  int rendezvous_ms = 30000;
  // Housekeeping cadence for the agent loop's TickGate. Bounded [1,1000]
  // at load: behind the gate a non-positive value stops every per-tick job
  // silently (see parse_link in config.cpp).
  int tick_ms = 100;
  // Agent-loop wake period for draining queued RCFs (spec 2026-08-14
  // fade-demote §3b). Decoupled from tick_ms: op actuation latency is
  // U(0, rc_drain_ms) while ALL per-tick housekeeping (USB health polls,
  // state timers, congestion guard, watchdog, stats/telem) stays on
  // tick_ms. Bounded [1,1000] AND required to be <= tick_ms, since a drain
  // slower than the tick would silently retime the housekeeping to the
  // drain period; == tick_ms reproduces the legacy single-cadence loop.
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

// A-MPDU TX aggregation (spec 2026-09-01-ampdu-design.md). OFF by default
// since the 2026-09-01 bench verdict (dq-spike-findings §12): the depth
// sweep measured NO fec improvement at any depth — maburd's per-body tax is
// host/USB-side, not medium access — while aggregate subframes carry
// missing-or-untrustworthy PHY reports that degrade the GS's RF telemetry
// even with the phy_valid filtering in place. Enable (max_num > 0) only for
// experiments, e.g. after a USB feed rework. The QoS-Data wire header does
// NOT depend on this block — max_num 0 turns off only the aggregation
// (frames become QoS-Data singles, measured identical to the old pace).
struct AmpduCfg {
  int max_num = 0;    // MAX_AGG_NUM cap, 0 = aggregation off, max 31 (5-bit)
  int max_time = 32;  // raw 0x455 fill-timer value (0x20 ~= 0.8 ms);
                      // 1..8 is a hardware cliff (disables aggregation) and
                      // is rejected at load; 0 keeps the chip bring-up
                      // default (0x70 ~= 3 ms — too slow, but valid for A/B)
};

// Drone air clock (spec 2026-09-06 air-clock): per-frame virtual
// air-serialization model + enh admission. shed_ms 0 = observe only (the
// model runs and exports, nothing is dropped); > 0 drops an enh AU whose
// arrival finds the modelled backlog at or past shed_ms. efficiency is the
// fraction of nominal PHY rate treated as capacity; body_us a fixed
// per-body cost. Both are calibrated on the bench (Stage A of the spec),
// not derived.
struct AirClockCfg {
  int shed_ms = 0;
  double efficiency = 0.7;
  int body_us = 0;
};

struct Config {
  RadioCfg radio;
  FecCfg fec;
  EncoderCfg encoder;
  VencSectionCfg venc;
  LinkCfg link;
  MspCfg msp;
  AmpduCfg ampdu;
  AirClockCfg air_clock;
  std::array<UepLayerCfg, 2> uep_layers() const;
};

// Loads and validates a mabur.json config file. Missing keys fall back to
// the struct defaults above; unknown keys and out-of-range values throw
// std::runtime_error("config: <field>: <why>"). Missing file throws too.
Config load_config(const std::string& path);

}  // namespace mabur
