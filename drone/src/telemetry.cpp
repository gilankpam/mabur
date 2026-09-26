#include "telemetry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace mabur {

namespace {
constexpr double kEmaAlpha = 0.1;

template <typename T, typename V>
T saturate(V v) {
  constexpr V lo = static_cast<V>(std::numeric_limits<T>::min());
  constexpr V hi = static_cast<V>(std::numeric_limits<T>::max());
  if (v < lo) return std::numeric_limits<T>::min();
  if (v > hi) return std::numeric_limits<T>::max();
  return static_cast<T>(v);
}

}  // namespace

void UplinkTrack::on_rc_frame(const uint8_t rssi[2], const int8_t snr[2]) {
  std::lock_guard<std::mutex> l(m_);
  if (!has_) {
    rssi_[0] = rssi[0];
    rssi_[1] = rssi[1];
    snr_[0] = snr[0];
    snr_[1] = snr[1];
    has_ = true;
  } else {
    for (int i = 0; i < 2; ++i) {
      rssi_[i] = kEmaAlpha * static_cast<double>(rssi[i]) + (1.0 - kEmaAlpha) * rssi_[i];
      snr_[i] = kEmaAlpha * static_cast<double>(snr[i]) + (1.0 - kEmaAlpha) * snr_[i];
    }
  }
}

UplinkTrack::Snap UplinkTrack::snap() const {
  std::lock_guard<std::mutex> l(m_);
  Snap s;
  s.has = has_;
  s.rssi[0] = rssi_[0];
  s.rssi[1] = rssi_[1];
  s.snr[0] = snr_[0];
  s.snr[1] = snr_[1];
  return s;
}

rc::Telem make_telem(uint16_t tlm_seq, const TelemInputs& in) {
  rc::Telem t;
  t.tlm_seq = tlm_seq;
  t.state = static_cast<uint8_t>(in.state);
  t.flags = static_cast<uint8_t>((in.failsafe_shed ? 0x01 : 0) | (in.radio_rx_ok ? 0x02 : 0) |
                                  (in.probe_on ? 0x04 : 0) |
                                  (in.rcf_seq_echo_valid ? 0x08 : 0) |
                                  (in.congestion_shed ? 0x10 : 0) |
                                  (in.air_shed ? 0x20 : 0) |
                                  (in.low_power ? 0x80 : 0));
  t.generation = saturate<uint32_t>(in.generation);
  t.applied_profile = rc::encode_profile(in.mode, in.mcs, in.bw);
  // Per-stream applied overhead: the commanded op pair (Task 6, RC_VERSION
  // 5), or the debug-HTTP per-layer override when armed — see main.cpp's
  // telemetry collector and TelemInputs.applied_ov_*.
  t.applied_ov_base = in.applied_ov_base;
  t.applied_ov_enh = in.applied_ov_enh;
  t.rcf_age_ms = saturate<uint16_t>(in.rcf_age_ms);
  t.rcf_seq_echo = in.rcf_seq_echo;
  t.pts_at_build = in.pts_at_build_us;
  t.rcf_rx = saturate<uint32_t>(in.rcf_rx);
  t.enc_frames = saturate<uint32_t>(in.enc_frames);
  t.enc_kbytes = saturate<uint32_t>(in.enc_bytes / 1024);
  t.cmd_kbps = saturate<uint16_t>(in.cmd_kbps);
  t.roi_qp = saturate<int8_t>(in.roi_qp);
  t.ring_drops = saturate<uint16_t>(in.ring_drops);
  t.txq_depth = saturate<uint8_t>(in.txq_depth);
  t.txq_cap = saturate<uint8_t>(in.txq_cap);
  t.txq_drops = saturate<uint32_t>(in.txq_drops);
  t.txq_wait_max_ms = saturate<uint16_t>(in.txq_wait_max_ms);
  t.radio_sent = saturate<uint32_t>(in.radio_sent);
  t.radio_drops = saturate<uint32_t>(in.radio_drops);
  t.usb_fail = saturate<uint16_t>(in.usb_fail);
  t.rx_own = saturate<uint16_t>(in.rx_own);
  t.rx_foreign = saturate<uint16_t>(in.rx_foreign);
  t.rx_crcfail = saturate<uint16_t>(in.rx_crcfail);
  if (in.uplink.has) {
    t.up_rssi[0] = saturate<uint8_t>(std::lround(in.uplink.rssi[0]));
    t.up_rssi[1] = saturate<uint8_t>(std::lround(in.uplink.rssi[1]));
    t.up_snr[0] = saturate<int8_t>(std::lround(in.uplink.snr[0]));
    t.up_snr[1] = saturate<int8_t>(std::lround(in.uplink.snr[1]));
  } else {
    t.up_rssi[0] = 0;
    t.up_rssi[1] = 0;
    t.up_snr[0] = 0;
    t.up_snr[1] = 0;
  }
  t.soc_temp_c = saturate<int8_t>(in.soc_temp_c);
  t.thermal_delta = saturate<int8_t>(in.thermal_delta);
  t.cpu_busy_x100 =
      in.cpu_pct ? saturate<uint16_t>(std::lround(std::clamp(*in.cpu_pct, 0.0, 100.0) * 100.0))
                 : 65535;
  t.idr_disagree = saturate<uint16_t>(in.idr_disagree);
  t.enhance_disagree = saturate<uint16_t>(in.enhance_disagree);
  t.vanished_base = saturate<uint16_t>(in.vanished_base);
  t.vanished_enh = saturate<uint16_t>(in.vanished_enh);
  t.self_idr_refused = saturate<uint16_t>(in.self_idr_refused);
  t.venc_full_drops = saturate<uint16_t>(in.venc_full_drops);
  // Clamped, not just saturated: the wire field is documented 0..100 and a
  // garbage percentage would read as a plausible occupancy.
  t.venc_ring_fill_pct =
      saturate<uint8_t>(std::clamp(in.venc_ring_fill_pct, 0, 100));
  t.air_backlog_max_ms = saturate<uint16_t>(in.air_backlog_max_ms);
  t.air_shed_drops = saturate<uint16_t>(in.air_shed_drops);
  t.channel = in.channel;
  t.hop_epoch = in.hop_epoch;
  t.rec_status = in.rec_status;
  return t;
}

int read_soc_temp_c(const char* path) {
  FILE* f = std::fopen(path, "r");
  if (!f) return -128;
  long millideg = 0;
  int n = std::fscanf(f, "%ld", &millideg);
  std::fclose(f);
  if (n != 1) return -128;
  return static_cast<int>(millideg / 1000);
}

int read_soc_temp_c_sigmastar(const char* path) {
  FILE* f = std::fopen(path, "r");
  if (!f) return -128;
  int deg = 0;
  int n = std::fscanf(f, "Temp=%d", &deg);
  std::fclose(f);
  if (n != 1) return -128;
  return deg;
}

std::optional<double> CpuBusySampler::sample(const char* path) {
  FILE* f = std::fopen(path, "r");
  if (!f) { have_ = false; return std::nullopt; }
  // user nice system idle iowait irq softirq steal (guest columns are
  // already folded into user/nice by the kernel).
  unsigned long long c[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const int n = std::fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                            &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7]);
  std::fclose(f);
  if (n < 4) { have_ = false; return std::nullopt; }
  uint64_t total = 0;
  for (int i = 0; i < 8; ++i) total += c[i];
  const uint64_t idle = c[3] + c[4];
  const uint64_t busy = total - idle;
  std::optional<double> out;
  if (have_ && total > total_ && busy >= busy_)
    out = 100.0 * static_cast<double>(busy - busy_) / static_cast<double>(total - total_);
  have_ = true;
  busy_ = busy;
  total_ = total;
  return out;
}

}  // namespace mabur
