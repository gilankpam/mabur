#ifndef MABURGS_STATS_EXPORTER_H_
#define MABURGS_STATS_EXPORTER_H_

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "mabur/rc_proto.h"
#include "op_point.h"

namespace maburgs {

struct StatsClassIn {  // copied from ClassTrack (gs/src/aggregator.h)
  uint64_t frames = 0, bytes = 0;
  bool has_ema = false;
  double rssi_ema = 0, rssi_a_ema = 0, rssi_b_ema = 0;
  double snr_ema = 0, snr_a_ema = 0, snr_b_ema = 0;
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
};

// Copied from LadderController's accessors (gs/src/ladder_controller.h) —
// plain values only, no controller reference: main fills this from
// vrx.ctl() each poll, matching the exporter's existing pattern for `op`.
struct StatsCtlIn {
  int rung_idx = 0;
  int rung_mcs = 0;
  double rung_ov = 0.0;
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
  // Effective ladder ({mcs, overhead} per rung, index = rung index) and the
  // util thresholds, copied from LadderCfg — static per-run but re-sent every
  // datagram so consumers stay stateless.
  std::vector<std::pair<int, double>> ladder;  // {mcs, ov}
  double down_util = 0.0, up_util = 0.0;
};

struct StatsInput {
  uint32_t vtx_id = 0;
  bool in_session = false;  // VrxState::SESSION
  int tx_card = 0;
  OpPoint op;
  int deadline_ms = 0;
  std::optional<double> residual_loss;  // nullopt -> JSON null
  std::array<int, 4> layer_delivery_pct{};
  std::vector<StatsCardIn> cards;
  std::array<StatsStreamIn, 4> streams;
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
};

class StatsExporter {
 public:
  using SendFn = std::function<bool(const std::string&)>;
  StatsExporter(uint32_t session_id, int interval_ms, SendFn send);
  void on_frame(uint64_t now_ms);                      // per emitted video frame
  bool poll(uint64_t now_ms, const StatsInput& in);    // true when a datagram went out
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
  std::array<StreamPrev, 4> prev_streams_{};
  uint64_t prev_ring_bytes_ = 0;
  std::array<bool, 4> stream_seen_{};   // sticky link.streams[] rows
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
