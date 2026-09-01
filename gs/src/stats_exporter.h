#ifndef MABURGS_STATS_EXPORTER_H_
#define MABURGS_STATS_EXPORTER_H_

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "lat_window.h"
#include "mabur/rc_proto.h"
#include "op_point.h"

namespace maburgs {

struct StatsClassIn {  // copied from ClassTrack (gs/src/aggregator.h)
  uint64_t frames = 0, bytes = 0;
  bool has_ema = false;
  double rssi_ema = 0, rssi_a_ema = 0, rssi_b_ema = 0;
  double snr_ema = 0, snr_a_ema = 0, snr_b_ema = 0;
  double evm_ema = 0, evm_a_ema = 0, evm_b_ema = 0;
  bool evm_has = false, evm_a_has = false, evm_b_has = false;
};

// Class index order matches RfClass in gs/src/aggregator.h: s0,s1,s2,s3,msp,ctrl.
constexpr int kNumStatsClasses = 6;

struct StatsCardIn {
  bool up = false;
  uint64_t frames = 0, crc_fail = 0;
  uint64_t seq_expected = 0, seq_received = 0;
  uint64_t rx_bytes = 0;
  uint64_t last_frame_us = 0;  // GS monotonic; 0 = never heard
  uint64_t self_frames = 0;    // GS-originated RC types heard cross-card
  uint64_t foreign = 0;        // CRC-clean, non-canonical-SA frames dropped upstream
  uint64_t tx_frames = 0;      // control frames this card sent OK
  uint64_t tx_fail = 0;        // send_control failures (cumulative)
  std::array<StatsClassIn, kNumStatsClasses> classes{};
};

struct StatsStreamIn {  // copied from mabur::UepDecoder::LayerStats
  uint64_t bodies = 0, subblocks_failed = 0, syms_recovered = 0,
           syms_recovered_arrived = 0, syms_abandoned = 0, symbols_in = 0,
           symbols_stale = 0, symbols_bad_cfg = 0, rows_in_flight = 0;
  uint64_t syms_abandoned_stale = 0;
};

// One rung of the per-rung EWMA store (spec 2026-08-13), copied plain from
// RungStore::stat() by main — same no-controller-reference pattern as
// StatsCtlIn. evm/evm_sd NaN -> JSON null; ages -1 = never sampled.
struct StatsRungIn {
  int mcs = 0;
  double ov_base = 0.0, ov_enh = 0.0;
  double u = 0.0, resid = 0.0, u3 = 0.0, resid3 = 0.0;
  double evm_db = 0.0, evm_sd_db = 0.0;
  uint64_t n = 0, probe_n = 0;
  double age_s = -1.0, probe_age_s = -1.0;
  double dwell_s = 0.0;
  uint32_t visits = 0, exits_bad = 0;
  double probe_u = 0.0;
};

// Copied from LadderController's accessors (gs/src/ladder_controller.h) —
// plain values only, no controller reference: main fills this from
// vrx.ctl() each poll, matching the exporter's existing pattern for `op`.
struct StatsCtlIn {
  int rung_idx = 0;
  int rung_mcs = 0;
  double rung_ov_base = 0.0;
  double rung_ov_enh = 0.0;
  double util = 0.0;
  double pre_fec_loss = 0.0;
  double budget = 0.0;
  int probation_ms_left = 0;
  std::vector<std::pair<int, int>> penalized;  // {rung, ms_left}
  uint64_t demotes_residual = 0, demotes_util = 0, promotes = 0,
           probation_fails = 0, starved_drops = 0, timeout_drops = 0;
  double last_event_t_ms = 0;
  int last_event_from = 0, last_event_to = 0;
  std::string last_event_reason = "none";  // to_string(CtlReason), lowercase
  double last_event_u = 0.0;
  // Effective ladder ({mcs, overhead_base, overhead_enh} per rung, index =
  // rung index) and the util thresholds, copied from LadderCfg — static
  // per-run but re-sent every datagram so consumers stay stateless.
  std::vector<std::tuple<int, double, double>> ladder;  // {mcs, ov_base, ov_enh}
  double down_util = 0.0, up_util = 0.0;

  // --- s3 probe-before-promote / s3 steady-state (Tasks 4/5) ---
  double util3 = 0.0;  // LadderController::util3(); may carry a 1e9
                        // division-guard sentinel -> clamped at the exporter
  uint64_t probes_started = 0, probes_ok = 0, probe_fails = 0,
           probe_aborts = 0;
  uint64_t demotes_s3_residual = 0, demotes_s3_util = 0;
  double last_event_snr_db = 0.0;  // NaN -> JSON null
  double last_event_evm_db = 0.0;  // NaN -> JSON null (label-only, like snr)
  // Last completed probe; last_probe_t_ms == 0 means "never probed" -> null.
  double last_probe_t_ms = 0;
  int last_probe_rung = 0;
  std::string last_probe_outcome = "none";
  double last_probe_snr_db = 0.0;
  double last_probe_evm_db = 0.0;  // NaN -> JSON null
  double last_probe_u_pred = 0.0;  // may carry the 1e9 sentinel -> clamped
  int last_probe_dur_ms = 0;

  // Per-rung EWMA store snapshot, index = rung index (spec 2026-08-13).
  std::vector<StatsRungIn> rungs;

  // --- fade-aware demotes (spec 2026-08-14) ---
  // demotes_fade counts fade EVENTS that produced a step, not rungs lost to
  // fade: the predictive trigger latches to exactly one demote per fade
  // event and re-arms only on an observed recovery (both deltas measurably
  // back under threshold).
  uint64_t demotes_fade = 0;
  bool fade_active = false;          // regime state (raw, cascade-independent)
  double fade_drssi = 0.0;           // baseline-minus-fast deltas; NaN -> null
  double fade_dsnr = 0.0;
};

struct StatsInput {
  uint32_t vtx_id = 0;
  bool in_session = false;  // VrxState::SESSION
  int tx_card = 0;
  OpPoint op;
  int gap_timeout_ms[2] = {0, 0};  // FrameStream's live per-sid gap timeout
  std::optional<double> residual_loss;  // nullopt -> JSON null
  // Transition attribution (spec 2026-08-14, unconditional since
  // 2026-08-15). residual_cur = the attributed (current-rung-only) sibling
  // of residual_loss; close_ms = s1's last boundary open->close latency,
  // nullopt = never closed. The `suppressed` counter was deleted 2026-09-02
  // with the packet-level delivery window it was defined against (it counted
  // windows where the total-vs-attributed views disagreed).
  std::optional<double> residual_cur;
  std::optional<double> attrib_close_ms;
  std::array<int, 2> layer_delivery_pct{};
  std::vector<StatsCardIn> cards;
  std::array<StatsStreamIn, 2> streams;
  // LadderController snapshot; nullopt in static-pin mode -> JSON "ctl": null
  // (the ladder is never ticked there — see VrxController::ctl() comment).
  std::optional<StatsCtlIn> ctl;
  uint64_t frames_clean = 0, frames_truncated = 0, frames_dropped = 0;
  uint64_t stall_resets = 0;
  // AU ring publish health (PR C: replaced the rtp/udp blocks -- video
  // leaves maburgs via the shm ring now; schema note in stats_exporter.cpp).
  uint64_t ring_published = 0, ring_dropped_oversize = 0, ring_bytes = 0;
  uint64_t q_drop = 0;

  // Latest drone telemetry, if any this session: wire struct + GS arrival clock.
  std::optional<mabur::rc::Telem> telem;
  uint64_t telem_rx_ms = 0;   // GS monotonic arrival stamp

  // Head-segment latency aggregates (Task 10, spec 2026-08-30-latency-
  // accounting): nullopt while the pts anchor isn't usable() yet, or while
  // it's usable but due() said this poll() won't actually emit -- the core
  // loop only pays LatWindow::flush()'s destructive read+clear on a poll
  // that is about to emit (see main.cpp), so this is deliberately absent
  // far more often than video_lat's would-be n==0 case suggests.
  std::optional<LatWindow::Out> video_lat;
};

class StatsExporter {
 public:
  using SendFn = std::function<bool(const std::string&)>;
  StatsExporter(uint32_t session_id, int interval_ms, SendFn send);
  void on_frame(uint64_t now_ms);                      // per emitted video frame
  bool poll(uint64_t now_ms, const StatsInput& in);    // true when a datagram went out
  // Mirrors poll()'s own interval gate without the side effects: lets a
  // caller decide whether to pay for a destructive read (LatWindow::flush())
  // BEFORE calling poll(), since poll() only consumes StatsInput when it is
  // about to actually emit.
  bool due(uint64_t now_ms) const {
    return !emitted_ || now_ms - last_emit_ms_ >= static_cast<uint64_t>(interval_ms_);
  }
  uint64_t send_failed() const;

 private:
  struct CardPrev {
    uint64_t frames = 0, rx_bytes = 0, seq_expected = 0, seq_received = 0;
    uint64_t self_frames = 0, foreign = 0, tx_frames = 0;
  };
  struct StreamPrev {
    uint64_t syms_recovered = 0, syms_recovered_arrived = 0,
             syms_abandoned = 0, symbols_in = 0;
  };
  uint32_t session_;
  int interval_ms_;
  SendFn send_;
  uint64_t seq_ = 0;
  bool emitted_ = false;             // first-poll gate + null-rates flag
  uint64_t last_emit_ms_ = 0;
  std::vector<CardPrev> prev_cards_;
  // Per-(card, class) previous frame counts (for pps) and sticky-seen mask,
  // resized alongside prev_cards_ whenever the card count changes.
  std::vector<std::array<uint64_t, kNumStatsClasses>> prev_class_frames_;
  std::vector<std::array<uint64_t, kNumStatsClasses>> prev_class_bytes_;
  std::vector<std::array<bool, kNumStatsClasses>> class_seen_;
  std::array<StreamPrev, 2> prev_streams_{};
  uint64_t prev_ring_bytes_ = 0;
  std::array<bool, 2> stream_seen_{};   // sticky link.streams[] rows
  // frame-emit tracking (fps + RFC3550 jitter)
  uint64_t frames_in_window_ = 0;
  uint64_t last_frame_ms_ = 0;
  int64_t last_interval_ms_ = -1;    // -1 = fewer than 2 frames since reset
  bool have_jitter_ = false;
  double jitter_ms_ = 0.0;
  uint64_t send_failed_ = 0;

  // Drone telemetry: rates come from the delta between consecutive DISTINCT
  // snapshots (tlm_seq changed), over their real GS arrival interval — not
  // the exporter's own poll window.
  bool prev_telem_valid_ = false;
  mabur::rc::Telem prev_telem_{};
  uint64_t prev_telem_rx_ms_ = 0;
  bool have_telem_rates_ = false;
  double telem_enc_fps_ = 0, telem_enc_mbps_ = 0, telem_rcf_rx_pps_ = 0,
         telem_txq_drop_pps_ = 0, telem_radio_sent_pps_ = 0;
};

}  // namespace maburgs

#endif  // MABURGS_STATS_EXPORTER_H_
