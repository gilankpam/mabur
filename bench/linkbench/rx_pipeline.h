#pragma once
// RX side of the bench: SBI bodies (from RadioFrontend's BodyQueue) →
// stream filter → sbi_unpack → SwDecoder → bench-packet verification, with
// cumulative counters snapshotted once a second by rx_main. Header-only and
// devourer-free so the host e2e test drives it directly.
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "bench_wire.h"
#include "mabur/sbi.h"
#include "mabur/sw_decoder.h"
#include "tx_pipeline.h"  // FecParams

namespace linkbench {

// Forward distance cur-prev in 12-bit sequence space. A "gap" of >= 2048 is
// a reorder/stale frame, not loss (matches the aggregator's treatment of
// mac_seq).
inline int seq_fwd_delta12(uint16_t prev, uint16_t cur) {
  return static_cast<int>((cur - prev) & 0x0FFF);
}

struct RxSnapshot {
  uint64_t frames = 0;        // bench-stream bodies seen (any CRC state)
  uint64_t crc_bad = 0;       // ...of which FCS-corrupt
  uint64_t air_bytes = 0;     // dot11 + body bytes (excl radiotap/PLCP/FCS)
  uint64_t mac_lost = 0;      // expected-vs-received deficit over CRC-ok bench
                              // frames (max-seq advance estimator: reorder from
                              // the threaded TX feed's URB swaps is credited,
                              // not counted as loss)
  uint64_t sub_blocks = 0;    // SBI sub-blocks scanned
  uint64_t sub_crc_fail = 0;  // ...of which CRC16-failed (erasures)
  uint64_t syms_delivered = 0;  // source symbols delivered as-is
  uint64_t syms_recovered = 0;  // symbols solved from repairs
  uint64_t syms_abandoned = 0;  // lost to the seq horizon, never recovered
  uint64_t sym_badcfg = 0;    // symbol_size mismatch vs TX (window rides
                               // per-repair in window_len on the wire, so
                               // only symbol_size is checked here)
  uint64_t pkts = 0;          // bench packets recovered post-FEC
  uint64_t pkts_expected = 0; // max_seq - first_seq + 1 (0 until first pkt)
  uint64_t pattern_bad = 0;   // decoded but fill mismatch (decode bug!)
  uint64_t good_bytes = 0;    // post-FEC app payload bytes
  double rssi_sum[2] = {0, 0};  // raw PWDB sums over CRC-ok bench frames
  double snr_sum[2] = {0, 0};   // dB sums (half-dB raw converted on ingest)
  uint64_t sig_frames = 0;      // denominator for the sums
};

inline RxSnapshot snapshot_delta(const RxSnapshot& n, const RxSnapshot& o) {
  RxSnapshot d;
  d.frames = n.frames - o.frames;
  d.crc_bad = n.crc_bad - o.crc_bad;
  d.air_bytes = n.air_bytes - o.air_bytes;
  // Cumulative mac_lost can shrink when a frame counted lost in a prior
  // interval arrives late (reorder across the boundary) — clamp instead of
  // wrapping unsigned.
  d.mac_lost = n.mac_lost >= o.mac_lost ? n.mac_lost - o.mac_lost : 0;
  d.sub_blocks = n.sub_blocks - o.sub_blocks;
  d.sub_crc_fail = n.sub_crc_fail - o.sub_crc_fail;
  d.syms_delivered = n.syms_delivered - o.syms_delivered;
  d.syms_recovered = n.syms_recovered - o.syms_recovered;
  d.syms_abandoned = n.syms_abandoned - o.syms_abandoned;
  d.sym_badcfg = n.sym_badcfg - o.sym_badcfg;
  d.pkts = n.pkts - o.pkts;
  d.pkts_expected = n.pkts_expected - o.pkts_expected;
  d.pattern_bad = n.pattern_bad - o.pattern_bad;
  d.good_bytes = n.good_bytes - o.good_bytes;
  d.rssi_sum[0] = n.rssi_sum[0] - o.rssi_sum[0];
  d.rssi_sum[1] = n.rssi_sum[1] - o.rssi_sum[1];
  d.snr_sum[0] = n.snr_sum[0] - o.snr_sum[0];
  d.snr_sum[1] = n.snr_sum[1] - o.snr_sum[1];
  d.sig_frames = n.sig_frames - o.sig_frames;
  return d;
}

class RxPipeline {
 public:
  explicit RxPipeline(const FecParams& p) : p_(p), dec_(p.sw()) {}

  void on_body(const uint8_t* body, size_t len, uint16_t mac_seq, bool crc_ok,
               const uint8_t rssi[2], const int8_t snr[2], uint64_t now_ms) {
    if (mabur::sbi_peek_stream_id(body, len) != kBenchStreamId) return;
    ++c_.frames;
    c_.air_bytes += kDot11HeaderLen + len;
    if (!crc_ok) {
      // A corrupt frame's mac_seq and phystatus are untrustworthy
      // (gs/src/aggregator.h) — keep its surviving sub-blocks, skip the
      // seq/signal accounting.
      ++c_.crc_bad;
    } else {
      c_.rssi_sum[0] += rssi[0];
      c_.rssi_sum[1] += rssi[1];
      // The phystatus rxsnr field is the vendor's s(8,1) format — HALF-dB
      // units (devourer FrameParserJaguar3.h; RxQuality.h converts with
      // snr_db = snr_raw/2). Convert at ingestion so every display reads dB.
      c_.snr_sum[0] += snr[0] / 2.0;
      c_.snr_sum[1] += snr[1] / 2.0;
      ++c_.sig_frames;
      // Max-seq advance estimator: expected frames = total forward advance
      // of the highest seq seen (+1); every CRC-ok frame counts as received
      // whether in-order or late. Loss = expected − received, derived in
      // snapshot(). Robust to the ≤3-frame URB swaps a multi-threaded TX
      // feed produces (a per-frame gap counter books those as phantom loss).
      //
      // Seq baseline reset on idle (656 hole sweep 2026-09-01): a fresh TX
      // invocation restarts its 12-bit seq, and against a stale max the
      // estimator masks that cell's losses until seq re-crosses the old max
      // (up to a whole ~3600-frame cell in 4096 space). A >2 s bench-frame
      // gap can only be a TX restart on a cell rig, so re-anchor there.
      if (have_mac_seq_ && last_seq_ms_ && now_ms - last_seq_ms_ > 2000)
        have_mac_seq_ = false;
      last_seq_ms_ = now_ms;
      if (have_mac_seq_) {
        const int d = seq_fwd_delta12(max_mac_seq_, mac_seq);
        if (d >= 1 && d < 2048) {
          mac_advance_ += static_cast<uint64_t>(d);
          max_mac_seq_ = mac_seq;
        }
        // d == 0 (dup/max) or >= 2048 (behind max): received, no advance.
      } else {
        max_mac_seq_ = mac_seq;
        have_mac_seq_ = true;
      }
    }
    auto r = mabur::sbi_unpack(body, len, p_.envelope_len());
    c_.sub_blocks += static_cast<uint64_t>(r.n_blocks);
    c_.sub_crc_fail += static_cast<uint64_t>(r.n_failed);
    for (auto& env : r.survivors)
      for (auto& pkt : dec_.add_symbol(env.data(), env.size(), now_ms))
        on_app_packet(pkt);
  }

  // Drop stuck repair rows (rows it drops are NOT counted abandoned — the
  // seq horizon owns loss accounting; see sw_decoder.h). 300 ms is generous
  // at bench body rates; now_ms must be the same monotonic clock passed to
  // on_body (the GS stale-clock lesson, commit 1e6ece3).
  void expire(uint64_t now_ms) { dec_.expire_rows_older_than(300, now_ms); }

  RxSnapshot snapshot() const {
    RxSnapshot s = c_;
    s.syms_delivered = dec_.syms_delivered();
    s.syms_recovered = dec_.syms_recovered();
    s.syms_abandoned = dec_.syms_abandoned();
    s.sym_badcfg = dec_.symbols_dropped_bad_cfg();
    if (have_mac_seq_) {
      const uint64_t expected = mac_advance_ + 1;
      s.mac_lost = expected > c_.sig_frames ? expected - c_.sig_frames : 0;
    }
    if (have_app_seq_)
      s.pkts_expected = static_cast<uint64_t>(max_app_seq_ - first_app_seq_) + 1;
    return s;
  }

 private:
  void on_app_packet(const std::vector<uint8_t>& pkt) {
    uint32_t seq = 0;
    bool pattern_ok = false;
    if (!parse_bench_packet(pkt.data(), pkt.size(), &seq, &pattern_ok)) return;
    ++c_.pkts;
    c_.good_bytes += pkt.size();
    if (!pattern_ok) ++c_.pattern_bad;
    if (!have_app_seq_) {
      first_app_seq_ = max_app_seq_ = seq;
      have_app_seq_ = true;
    } else if (seq > max_app_seq_) {
      max_app_seq_ = seq;
    }
  }

  FecParams p_;
  mabur::SwDecoder dec_;
  RxSnapshot c_;
  uint16_t max_mac_seq_ = 0;
  uint64_t mac_advance_ = 0;
  bool have_mac_seq_ = false;
  uint64_t last_seq_ms_ = 0;  // last CRC-ok bench frame (idle re-anchor)
  uint32_t first_app_seq_ = 0, max_app_seq_ = 0;
  bool have_app_seq_ = false;
};

// One per-second console line from an interval delta. RSSI is shown as
// dBm ~= raw PWDB - 110 (devourer LinkHealth.h convention), both chains
// (never averaged across chains — chain A reads off-scale on some cards).
inline std::string format_line(uint64_t t_sec, const RxSnapshot& d) {
  const double air_mbps = static_cast<double>(d.air_bytes) * 8.0 / 1e6;
  const double good_mbps = static_cast<double>(d.good_bytes) * 8.0 / 1e6;
  const uint64_t ok_frames = d.frames - d.crc_bad;
  const double frm_loss =
      (ok_frames + d.mac_lost) ? 100.0 * static_cast<double>(d.mac_lost) /
                                     static_cast<double>(ok_frames + d.mac_lost)
                               : 0.0;
  const double pkt_loss =
      d.pkts_expected ? 100.0 *
                            static_cast<double>(d.pkts_expected - d.pkts) /
                            static_cast<double>(d.pkts_expected)
                      : 0.0;
  const double n = d.sig_frames ? static_cast<double>(d.sig_frames) : 1.0;
  char buf[256];
  std::snprintf(buf, sizeof buf,
                "[%3llus] air %.2fM good %.2fM | frm %llu loss %.1f%% | "
                "rec %llu abn %llu | pkt %llu loss %.1f%% | "
                "rssi %.0f/%.0f snr %.0f/%.0f",
                static_cast<unsigned long long>(t_sec), air_mbps, good_mbps,
                static_cast<unsigned long long>(d.frames), frm_loss,
                static_cast<unsigned long long>(d.syms_recovered),
                static_cast<unsigned long long>(d.syms_abandoned),
                static_cast<unsigned long long>(d.pkts), pkt_loss,
                d.rssi_sum[0] / n - 110.0, d.rssi_sum[1] / n - 110.0,
                d.snr_sum[0] / n, d.snr_sum[1] / n);
  return buf;
}

inline std::string format_json(uint64_t t_sec, const RxSnapshot& d) {
  const double n = d.sig_frames ? static_cast<double>(d.sig_frames) : 1.0;
  char buf[512];
  std::snprintf(
      buf, sizeof buf,
      "{\"t\":%llu,\"air_bytes\":%llu,\"good_bytes\":%llu,\"frames\":%llu,"
      "\"crc_bad\":%llu,\"mac_lost\":%llu,\"sub_blocks\":%llu,"
      "\"sub_crc_fail\":%llu,\"syms_recovered\":%llu,\"syms_abandoned\":%llu,"
      "\"sym_badcfg\":%llu,\"pkts\":%llu,\"pkts_expected\":%llu,"
      "\"pattern_bad\":%llu,\"rssi\":[%.1f,%.1f],\"snr\":[%.1f,%.1f]}",
      static_cast<unsigned long long>(t_sec),
      static_cast<unsigned long long>(d.air_bytes),
      static_cast<unsigned long long>(d.good_bytes),
      static_cast<unsigned long long>(d.frames),
      static_cast<unsigned long long>(d.crc_bad),
      static_cast<unsigned long long>(d.mac_lost),
      static_cast<unsigned long long>(d.sub_blocks),
      static_cast<unsigned long long>(d.sub_crc_fail),
      static_cast<unsigned long long>(d.syms_recovered),
      static_cast<unsigned long long>(d.syms_abandoned),
      static_cast<unsigned long long>(d.sym_badcfg),
      static_cast<unsigned long long>(d.pkts),
      static_cast<unsigned long long>(d.pkts_expected),
      static_cast<unsigned long long>(d.pattern_bad),
      d.rssi_sum[0] / n - 110.0, d.rssi_sum[1] / n - 110.0,
      d.snr_sum[0] / n, d.snr_sum[1] / n);
  return buf;
}

}  // namespace linkbench
