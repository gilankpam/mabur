#include "stats_exporter.h"

#include <algorithm>
#include <cstdio>

#include "json.hpp"
#include "mabur/profile.h"
#include "mabur/uep_encoder.h"

namespace maburgs {

using nlohmann::json;

namespace {
// Rate over the window; null-safe caller passes elapsed_s > 0. Negative
// deltas (impossible counter regressions) clamp to zero rather than emitting
// a negative rate.
double rate(uint64_t cur, uint64_t prev, double elapsed_s) {
  return cur > prev ? static_cast<double>(cur - prev) / elapsed_s : 0.0;
}

// Index order matches RfClass in gs/src/aggregator.h: s0,s1,s2,s3,msp,ctrl.
constexpr const char* kClassKeys[kNumStatsClasses] = {"s0", "s1", "s2", "s3", "msp", "ctrl"};

constexpr const char* kTelemStateNames[4] = {"boot", "rendezvous", "linked", "failsafe"};
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
  if (prev_cards_.size() != in.cards.size()) {
    prev_cards_.assign(in.cards.size(), {});
    prev_class_frames_.assign(in.cards.size(), {});
    prev_class_bytes_.assign(in.cards.size(), {});
    class_seen_.assign(in.cards.size(), {});
  }

  json j;
  j["v"] = 1;
  j["session"] = session_;
  j["seq"] = seq_;
  j["t_ms"] = now_ms;

  j["cards"] = json::array();
  // Per-card window rates collected for the stream-level TX estimates below:
  // received Mbps per stream class and the card's delivery fraction.
  std::vector<std::array<double, 4>> stream_mbps(in.cards.size(),
                                                 std::array<double, 4>{});
  std::vector<double> delivery(in.cards.size(), 1.0);
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
        delivery[i] = got;
      } else {
        cj["loss_pct"] = nullptr;
      }
      cj["rx_mbps"] = rate(c.rx_bytes, p.rx_bytes, elapsed_s) * 8.0 / 1e6;
      cj["pps"] = rate(c.frames, p.frames, elapsed_s);
      cj["foreign_pps"] = rate(c.foreign, p.foreign, elapsed_s);
      cj["self_pps"] = rate(c.self_frames, p.self_frames, elapsed_s);
      cj["tx_pps"] = rate(c.tx_frames, p.tx_frames, elapsed_s);
      // Drone injection estimate: the drone's hw seq counter numbers every
      // frame it injects, so the expected-seq advance IS its TX rate as
      // observed (lost frames included).
      cj["inj_pps"] = rate(c.seq_expected, p.seq_expected, elapsed_s);
    } else {
      cj["loss_pct"] = nullptr;
      cj["rx_mbps"] = nullptr;
      cj["pps"] = nullptr;
      cj["foreign_pps"] = nullptr;
      cj["self_pps"] = nullptr;
      cj["tx_pps"] = nullptr;
      cj["inj_pps"] = nullptr;
    }
    cj["tx_fail"] = c.tx_fail;
    if (c.last_frame_us != 0) {
      const uint64_t f_ms = c.last_frame_us / 1000;
      cj["last_frame_age_ms"] = now_ms > f_ms ? now_ms - f_ms : 0;
    } else {
      cj["last_frame_age_ms"] = nullptr;
    }

    json classes = json::object();
    for (int k = 0; k < kNumStatsClasses; ++k) {
      const size_t ku = static_cast<size_t>(k);
      const StatsClassIn& cls = c.classes[ku];
      if (cls.frames > 0) class_seen_[i][ku] = true;
      if (!class_seen_[i][ku]) continue;
      json kj;
      if (have_window) {
        kj["pps"] = rate(cls.frames, prev_class_frames_[i][ku], elapsed_s);
        const double mbps_c =
            rate(cls.bytes, prev_class_bytes_[i][ku], elapsed_s) * 8.0 / 1e6;
        kj["mbps"] = mbps_c;
        if (k < 4) stream_mbps[i][ku] = mbps_c;
      } else {
        kj["pps"] = nullptr;
        kj["mbps"] = nullptr;
      }
      if (cls.has_ema) {
        kj["rssi"] = cls.rssi_ema - 110.0;
        kj["rssi_a"] = cls.rssi_a_ema - 110.0;
        kj["rssi_b"] = cls.rssi_b_ema - 110.0;
        kj["snr"] = cls.snr_ema;
        kj["snr_a"] = cls.snr_a_ema;
        kj["snr_b"] = cls.snr_b_ema;
      } else {
        kj["rssi"] = nullptr; kj["rssi_a"] = nullptr; kj["rssi_b"] = nullptr;
        kj["snr"] = nullptr;  kj["snr_a"] = nullptr;  kj["snr_b"] = nullptr;
      }
      classes[kClassKeys[k]] = std::move(kj);
    }
    cj["classes"] = std::move(classes);

    j["cards"].push_back(std::move(cj));
  }

  json& link = j["link"];
  link["vtx_id"] = in.vtx_id;
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

  // The drone's per-rung TX spec is deterministic from the commanded op
  // (ladder_from; flag policy assumed default) — display-grade, like the
  // injection estimates below (received rate scaled by the best card's
  // delivery fraction; lost frames' bytes are unknowable at the GS).
  const auto ladder = mabur::rc::ladder_from(
      in.op.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
      static_cast<uint8_t>(in.op.mcs), static_cast<uint8_t>(in.op.bw), {});
  double air_pct_sum = 0.0;
  link["streams"] = json::array();
  for (int s = 0; s < 4; ++s) {
    const StatsStreamIn& st = in.streams[static_cast<size_t>(s)];
    if (st.bodies > 0) stream_seen_[static_cast<size_t>(s)] = true;
    if (!stream_seen_[static_cast<size_t>(s)]) continue;
    const StreamPrev& p = prev_streams_[static_cast<size_t>(s)];
    const mabur::rc::LayerTxSpec& rung = ladder[static_cast<size_t>(s)];
    const double phy = mabur::rc::phy_rate_mbps(rung);
    json fj;
    fj["stream"] = s;
    fj["ov"] = mabur::uep_layer_overhead(s, in.op.overhead);
    fj["rung_mcs"] = rung.mcs;
    fj["rung_ldpc"] = rung.ldpc;
    fj["rung_stbc"] = rung.stbc;
    fj["phy_mbps"] = phy;
    if (have_window) {
      double inj_mbps = 0.0;
      for (size_t i = 0; i < in.cards.size(); ++i) {
        const double est =
            stream_mbps[i][static_cast<size_t>(s)] / std::max(0.01, delivery[i]);
        if (est > inj_mbps) inj_mbps = est;
      }
      fj["inj_kbps"] = inj_mbps * 1000.0;
      if (phy > 0.0) air_pct_sum += 100.0 * inj_mbps / phy;
    } else {
      fj["inj_kbps"] = nullptr;
    }
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
    link["streams"].push_back(std::move(fj));
  }
  // Airtime estimate: injected bits vs each rung's PHY rate, summed over the
  // active streams (msp/ctrl are noise at this scale). Duty of the channel
  // the drone is burning — compare against the ~75% throttle ceiling.
  if (have_window) link["air_pct"] = air_pct_sum;
  else link["air_pct"] = nullptr;

  json& v = link["video"];
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

  if (in.telem) {
    const mabur::rc::Telem& t = *in.telem;
    // A new distinct snapshot (tlm_seq changed since the last one we kept)
    // gets a fresh rate computed from the GS-clock interval between the two
    // snapshots' arrivals; a repeat of the same tlm_seq keeps whatever rate
    // window was last computed (age still advances every poll).
    const bool is_new_snapshot =
        !prev_telem_valid_ || t.tlm_seq != prev_telem_.tlm_seq;
    // A maburd restart resets tlm_seq/generation/every cumulative counter
    // back to ~0. Naively wrap-safe-subtracting the u16/u32 counters against
    // the pre-restart baseline then yields a ~4e9-scale (or huge tlm_seq
    // delta) garbage rate for exactly one window. Detect the restart instead
    // of computing a rate across it: an (unsigned) tlm_seq delta outside
    // [1, 32767] is either a huge forward jump (impossible at ~1 Hz) or a
    // seq that went backwards (delta wraps to something huge); generation
    // regressing (cur < prev) is the same signal from the op-state side.
    // is_new_snapshot already guarantees tlm_seq changed, so the delta is
    // never 0 here.
    bool is_restart = false;
    if (is_new_snapshot && prev_telem_valid_) {
      const uint16_t seq_delta =
          static_cast<uint16_t>(t.tlm_seq - prev_telem_.tlm_seq);
      is_restart = seq_delta > 32767 || t.generation < prev_telem_.generation;
    }
    if (is_new_snapshot && prev_telem_valid_ && !is_restart &&
        in.telem_rx_ms > prev_telem_rx_ms_) {
      const double dt_s =
          static_cast<double>(in.telem_rx_ms - prev_telem_rx_ms_) / 1000.0;
      // Wrap-safe: subtract in the counter's own (unsigned) width before
      // widening to double, so a wrapped counter yields the correct small
      // delta instead of a huge one.
      const uint32_t d_frames = t.enc_frames - prev_telem_.enc_frames;
      const uint32_t d_kbytes = t.enc_kbytes - prev_telem_.enc_kbytes;
      const uint32_t d_rcf = t.rcf_rx - prev_telem_.rcf_rx;
      const uint32_t d_txq_drops = t.txq_drops - prev_telem_.txq_drops;
      const uint32_t d_radio_sent = t.radio_sent - prev_telem_.radio_sent;
      telem_enc_fps_ = static_cast<double>(d_frames) / dt_s;
      telem_enc_mbps_ =
          static_cast<double>(d_kbytes) * 1024.0 * 8.0 / 1e6 / dt_s;
      telem_rcf_rx_pps_ = static_cast<double>(d_rcf) / dt_s;
      telem_txq_drop_pps_ = static_cast<double>(d_txq_drops) / dt_s;
      telem_radio_sent_pps_ = static_cast<double>(d_radio_sent) / dt_s;
      have_telem_rates_ = true;
    }
    if (is_restart) {
      // Reseed the baseline on the restart snapshot itself but withhold
      // rates: the next distinct snapshot after this one gets a clean
      // interval to compute from.
      have_telem_rates_ = false;
    }
    if (is_new_snapshot) {
      prev_telem_ = t;
      prev_telem_rx_ms_ = in.telem_rx_ms;
      prev_telem_valid_ = true;
    }

    mabur::rc::PhyMode mode;
    uint8_t mcs = 0, bw = 0;
    mabur::rc::decode_profile(t.applied_profile, mode, mcs, bw);

    json& d = j["drone"];
    d["tlm_age_ms"] = now_ms > in.telem_rx_ms ? now_ms - in.telem_rx_ms : 0;
    d["tlm_seq"] = t.tlm_seq;
    d["state"] = t.state < 4 ? kTelemStateNames[t.state] : "unknown";
    d["gen"] = t.generation;
    d["failsafe_shed"] = (t.flags & 0x01) != 0;
    d["radio_rx_ok"] = (t.flags & 0x02) != 0;
    d["applied"] = {{"mcs", mcs},
                    {"bw", bw},
                    {"vht", mode == mabur::rc::PhyMode::VHT},
                    {"overhead", t.applied_ov_x100 / 100.0},
                    {"offset_qdb", mabur::rc::decode_pwr_offset_qdb(t.applied_off_qdb)},
                    {"derate_qdb", t.derate_qdb}};
    json& rcf = d["rcf"];
    rcf["age_ms"] = t.rcf_age_ms;
    rcf["rx_pps"] = have_telem_rates_ ? json(telem_rcf_rx_pps_) : json(nullptr);
    json& enc = d["enc"];
    enc["fps"] = have_telem_rates_ ? json(telem_enc_fps_) : json(nullptr);
    enc["mbps"] = have_telem_rates_ ? json(telem_enc_mbps_) : json(nullptr);
    enc["cmd_kbps"] = t.cmd_kbps;
    enc["qp"] = t.qp;
    enc["ring_drops"] = t.ring_drops;
    json& txq = d["txq"];
    txq["depth"] = t.txq_depth;
    txq["cap"] = t.txq_cap;  // wire value as-is (256 saturates to 255 on the wire)
    txq["drop_pps"] = have_telem_rates_ ? json(telem_txq_drop_pps_) : json(nullptr);
    txq["drops"] = t.txq_drops;
    json& radio = d["radio"];
    radio["sent_pps"] = have_telem_rates_ ? json(telem_radio_sent_pps_) : json(nullptr);
    radio["drops"] = t.radio_drops;
    radio["usb_fail"] = t.usb_fail;
    // Raw rssi 0 on both chains is never a legitimate live reading — it is
    // the wire's all-zero default for "no RC frame ever heard" (deaf radio /
    // pre-DISC). Rendering it as -110.0 dBm would read as plausible signal,
    // so surface the honest "no data" instead.
    if (t.up_rssi[0] == 0 && t.up_rssi[1] == 0) {
      d["uplink"] = {{"rssi_a", nullptr}, {"rssi_b", nullptr},
                     {"snr_a", nullptr}, {"snr_b", nullptr}};
    } else {
      d["uplink"] = {{"rssi_a", t.up_rssi[0] - 110.0},
                     {"rssi_b", t.up_rssi[1] - 110.0},
                     {"snr_a", t.up_snr[0]},
                     {"snr_b", t.up_snr[1]}};
    }
    d["sys"] = {{"soc_temp_c", t.soc_temp_c},
                {"thermal_delta", t.thermal_delta},
                {"load", t.load_x100 / 100.0}};
  } else {
    j["drone"] = nullptr;
  }

  // Roll the window forward whether or not the send succeeds — the sample
  // was taken; a lost datagram is a lost sample, not a longer next window.
  for (size_t i = 0; i < in.cards.size(); ++i) {
    prev_cards_[i] = {in.cards[i].frames, in.cards[i].rx_bytes,
                      in.cards[i].seq_expected, in.cards[i].seq_received,
                      in.cards[i].self_frames, in.cards[i].foreign,
                      in.cards[i].tx_frames};
    for (int k = 0; k < kNumStatsClasses; ++k) {
      prev_class_frames_[i][static_cast<size_t>(k)] = in.cards[i].classes[static_cast<size_t>(k)].frames;
      prev_class_bytes_[i][static_cast<size_t>(k)] = in.cards[i].classes[static_cast<size_t>(k)].bytes;
    }
  }
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
