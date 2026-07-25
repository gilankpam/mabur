#ifndef MABURGS_STATS_EXPORTER_H_
#define MABURGS_STATS_EXPORTER_H_

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "op_table.h"

namespace maburgs {

struct StatsCardIn {
  bool up = false;
  uint64_t frames = 0, crc_fail = 0;
  uint64_t seq_expected = 0, seq_received = 0;
  uint64_t rx_bytes = 0;
  bool has_ema = false;
  double rssi_ema = 0, rssi_a_ema = 0, rssi_b_ema = 0;
  double snr_ema = 0, snr_a_ema = 0, snr_b_ema = 0;
  uint64_t last_frame_us = 0;  // GS monotonic; 0 = never heard
};

struct StatsStreamIn {  // copied from mabur::UepDecoder::LayerStats
  uint64_t bodies = 0, subblocks_failed = 0, syms_recovered = 0,
           syms_abandoned = 0, symbols_in = 0, symbols_stale = 0,
           symbols_bad_cfg = 0, rows_in_flight = 0;
};

struct StatsInput {
  bool in_session = false;  // VrxState::SESSION
  int tx_card = 0;
  OpPoint op;
  int deadline_ms = 0;
  std::optional<double> residual_loss;  // nullopt -> JSON null
  std::array<int, 4> layer_delivery_pct{};
  std::vector<StatsCardIn> cards;
  std::array<StatsStreamIn, 4> streams;
  uint64_t frames_clean = 0, frames_truncated = 0, frames_dropped = 0;
  uint64_t stall_resets = 0;
  uint64_t rtp_ok = 0, rtp_gap = 0, rtp_gap_seqs = 0, rtp_back = 0;
  uint64_t udp_sent = 0, udp_failed = 0, udp_bytes = 0;
  uint64_t q_drop = 0;
};

class StatsExporter {
 public:
  using SendFn = std::function<bool(const std::string&)>;
  StatsExporter(uint32_t session_id, int interval_ms, SendFn send);
  void on_frame(uint64_t now_ms);                      // per emitted video frame
  bool poll(uint64_t now_ms, const StatsInput& in);    // true when a datagram went out
  uint64_t send_failed() const;

 private:
  struct CardPrev { uint64_t frames = 0, rx_bytes = 0, seq_expected = 0, seq_received = 0; };
  struct StreamPrev { uint64_t syms_recovered = 0, syms_abandoned = 0, symbols_in = 0; };
  uint32_t session_;
  int interval_ms_;
  SendFn send_;
  uint64_t seq_ = 0;
  bool emitted_ = false;             // first-poll gate + null-rates flag
  uint64_t last_emit_ms_ = 0;
  std::vector<CardPrev> prev_cards_;
  std::array<StreamPrev, 4> prev_streams_{};
  uint64_t prev_udp_bytes_ = 0;
  std::array<bool, 4> fec_seen_{};   // sticky rows
  // frame-emit tracking (fps + RFC3550 jitter)
  uint64_t frames_in_window_ = 0;
  uint64_t last_frame_ms_ = 0;
  int64_t last_interval_ms_ = -1;    // -1 = fewer than 2 frames since reset
  bool have_jitter_ = false;
  double jitter_ms_ = 0.0;
  uint64_t send_failed_ = 0;
};

}  // namespace maburgs

#endif  // MABURGS_STATS_EXPORTER_H_
