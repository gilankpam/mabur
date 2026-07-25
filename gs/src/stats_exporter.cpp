#include "stats_exporter.h"

#include <cstdio>

#include "json.hpp"

namespace maburgs {

using nlohmann::json;

namespace {
// Rate over the window; null-safe caller passes elapsed_s > 0. Negative
// deltas (impossible counter regressions) clamp to zero rather than emitting
// a negative rate.
double rate(uint64_t cur, uint64_t prev, double elapsed_s) {
  return cur > prev ? static_cast<double>(cur - prev) / elapsed_s : 0.0;
}
}  // namespace

StatsExporter::StatsExporter(uint32_t session_id, int interval_ms, SendFn send)
    : session_(session_id), interval_ms_(interval_ms), send_(std::move(send)) {}

void StatsExporter::on_frame(uint64_t now_ms) {
  ++frames_in_window_;
  if (last_frame_ms_ != 0) {
    const int64_t iv = static_cast<int64_t>(now_ms - last_frame_ms_);
    if (iv > 1000) {  // a stall is a stall, not jitter
      last_interval_ms_ = -1;
      jitter_ms_ = 0.0;
      have_jitter_ = true;  // 0.0 is a statement now, not "no data"
    } else {
      if (last_interval_ms_ >= 0) {
        const double d = static_cast<double>(
            iv > last_interval_ms_ ? iv - last_interval_ms_ : last_interval_ms_ - iv);
        jitter_ms_ += (d - jitter_ms_) / 16.0;
        have_jitter_ = true;
      }
      last_interval_ms_ = iv;
    }
  }
  last_frame_ms_ = now_ms;
}

bool StatsExporter::poll(uint64_t now_ms, const StatsInput& in) {
  if (emitted_ && now_ms - last_emit_ms_ < static_cast<uint64_t>(interval_ms_))
    return false;
  const bool have_window = emitted_ && now_ms > last_emit_ms_;
  const double elapsed_s =
      have_window ? static_cast<double>(now_ms - last_emit_ms_) / 1000.0 : 0.0;
  if (prev_cards_.size() != in.cards.size()) prev_cards_.assign(in.cards.size(), {});

  json j;
  j["v"] = 1;
  j["session"] = session_;
  j["seq"] = seq_;
  j["t_ms"] = now_ms;

  json& link = j["link"];
  link["state"] = in.in_session ? "session" : "beaconing";
  link["tx_card"] = in.tx_card;
  link["op"] = {{"mcs", in.op.mcs},           {"bw", in.op.bw},
                {"sgi", in.op.sgi},           {"vht", in.op.vht},
                {"overhead", in.op.overhead}, {"offset_qdb", in.op.pwr_offset_qdb},
                {"snr_req", in.op.snr_req}};
  link["deadline_ms"] = in.deadline_ms;
  if (in.residual_loss) link["residual_loss"] = *in.residual_loss;
  else link["residual_loss"] = nullptr;
  link["layer_delivery_pct"] = in.layer_delivery_pct;

  j["cards"] = json::array();
  for (size_t i = 0; i < in.cards.size(); ++i) {
    const StatsCardIn& c = in.cards[i];
    const CardPrev& p = prev_cards_[i];
    json cj;
    cj["id"] = i;
    cj["up"] = c.up;
    cj["frames"] = c.frames;
    cj["crc_fail"] = c.crc_fail;
    if (have_window) {
      const uint64_t d_exp = c.seq_expected > p.seq_expected
                                 ? c.seq_expected - p.seq_expected : 0;
      const uint64_t d_rcv = c.seq_received > p.seq_received
                                 ? c.seq_received - p.seq_received : 0;
      if (d_exp > 0) {
        const double got = d_rcv > d_exp ? 1.0
                             : static_cast<double>(d_rcv) / static_cast<double>(d_exp);
        cj["loss_pct"] = 100.0 * (1.0 - got);
      } else {
        cj["loss_pct"] = nullptr;
      }
      cj["rx_mbps"] = rate(c.rx_bytes, p.rx_bytes, elapsed_s) * 8.0 / 1e6;
      cj["pps"] = rate(c.frames, p.frames, elapsed_s);
    } else {
      cj["loss_pct"] = nullptr;
      cj["rx_mbps"] = nullptr;
      cj["pps"] = nullptr;
    }
    if (c.has_ema) {
      cj["rssi"] = c.rssi_ema - 110.0;
      cj["rssi_a"] = c.rssi_a_ema - 110.0;
      cj["rssi_b"] = c.rssi_b_ema - 110.0;
      cj["snr"] = c.snr_ema;
      cj["snr_a"] = c.snr_a_ema;
      cj["snr_b"] = c.snr_b_ema;
    } else {
      cj["rssi"] = nullptr; cj["rssi_a"] = nullptr; cj["rssi_b"] = nullptr;
      cj["snr"] = nullptr;  cj["snr_a"] = nullptr;  cj["snr_b"] = nullptr;
    }
    if (c.last_frame_us != 0) {
      const uint64_t f_ms = c.last_frame_us / 1000;
      cj["last_frame_age_ms"] = now_ms > f_ms ? now_ms - f_ms : 0;
    } else {
      cj["last_frame_age_ms"] = nullptr;
    }
    j["cards"].push_back(std::move(cj));
  }

  j["fec"] = json::array();
  for (int s = 0; s < 4; ++s) {
    const StatsStreamIn& st = in.streams[static_cast<size_t>(s)];
    if (st.bodies > 0) fec_seen_[static_cast<size_t>(s)] = true;
    if (!fec_seen_[static_cast<size_t>(s)]) continue;
    const StreamPrev& p = prev_streams_[static_cast<size_t>(s)];
    json fj;
    fj["stream"] = s;
    if (have_window) {
      fj["recovered_s"] = rate(st.syms_recovered, p.syms_recovered, elapsed_s);
      fj["abandoned_s"] = rate(st.syms_abandoned, p.syms_abandoned, elapsed_s);
      fj["syms_in_s"] = rate(st.symbols_in, p.symbols_in, elapsed_s);
    } else {
      fj["recovered_s"] = nullptr;
      fj["abandoned_s"] = nullptr;
      fj["syms_in_s"] = nullptr;
    }
    fj["recovered"] = st.syms_recovered;
    fj["abandoned"] = st.syms_abandoned;
    fj["stale"] = st.symbols_stale;
    fj["bad_cfg"] = st.symbols_bad_cfg;
    fj["sub_fail"] = st.subblocks_failed;
    fj["in_flight"] = st.rows_in_flight;
    j["fec"].push_back(std::move(fj));
  }

  json& v = j["video"];
  if (have_window) {
    v["fps"] = static_cast<double>(frames_in_window_) / elapsed_s;
    v["mbps"] = rate(in.udp_bytes, prev_udp_bytes_, elapsed_s) * 8.0 / 1e6;
  } else {
    v["fps"] = nullptr;
    v["mbps"] = nullptr;
  }
  if (have_jitter_) v["jitter_ms"] = jitter_ms_;
  else v["jitter_ms"] = nullptr;
  v["clean"] = in.frames_clean;
  v["truncated"] = in.frames_truncated;
  v["dropped"] = in.frames_dropped;
  v["stall_resets"] = in.stall_resets;
  v["rtp"] = {{"ok", in.rtp_ok},           {"gap", in.rtp_gap},
              {"gap_seqs", in.rtp_gap_seqs}, {"back", in.rtp_back}};
  v["udp"] = {{"sent", in.udp_sent}, {"failed", in.udp_failed},
              {"bytes", in.udp_bytes}};
  v["q_drop"] = in.q_drop;

  // Roll the window forward whether or not the send succeeds — the sample
  // was taken; a lost datagram is a lost sample, not a longer next window.
  for (size_t i = 0; i < in.cards.size(); ++i)
    prev_cards_[i] = {in.cards[i].frames, in.cards[i].rx_bytes,
                      in.cards[i].seq_expected, in.cards[i].seq_received};
  for (size_t s = 0; s < 4; ++s)
    prev_streams_[s] = {in.streams[s].syms_recovered, in.streams[s].syms_abandoned,
                        in.streams[s].symbols_in};
  prev_udp_bytes_ = in.udp_bytes;
  frames_in_window_ = 0;
  last_emit_ms_ = now_ms;
  emitted_ = true;
  ++seq_;

  if (!send_(j.dump())) {
    if (send_failed_ == 0)
      std::fprintf(stderr, "maburgs: stats sideport send failed (muting)\n");
    ++send_failed_;
    return false;
  }
  return true;
}

uint64_t StatsExporter::send_failed() const { return send_failed_; }

}  // namespace maburgs
