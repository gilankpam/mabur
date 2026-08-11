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
                                  (in.probing ? 0x04 : 0));
  t.generation = saturate<uint32_t>(in.generation);
  t.applied_profile = rc::encode_profile(in.mode, in.mcs, in.bw);
  t.applied_ov_x100 = saturate<uint8_t>(std::lround(in.applied_ov * 100.0));
  t.applied_off_qdb = rc::encode_pwr_offset_qdb(in.applied_off_qdb);
  t.derate_qdb = saturate<uint8_t>(in.derate_qdb);
  t.rcf_age_ms = saturate<uint16_t>(in.rcf_age_ms);
  t.rcf_rx = saturate<uint32_t>(in.rcf_rx);
  t.enc_frames = saturate<uint32_t>(in.enc_frames);
  t.enc_kbytes = saturate<uint32_t>(in.enc_bytes / 1024);
  t.cmd_kbps = saturate<uint16_t>(in.cmd_kbps);
  t.qp = saturate<uint8_t>(in.qp);
  t.ring_drops = saturate<uint16_t>(in.ring_drops);
  t.txq_depth = saturate<uint8_t>(in.txq_depth);
  t.txq_cap = saturate<uint8_t>(in.txq_cap);
  t.txq_drops = saturate<uint32_t>(in.txq_drops);
  t.radio_sent = saturate<uint32_t>(in.radio_sent);
  t.radio_drops = saturate<uint32_t>(in.radio_drops);
  t.usb_fail = saturate<uint16_t>(in.usb_fail);
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
  t.load_x100 = saturate<uint16_t>(std::lround(in.load1 * 100.0));
  t.idr_disagree = saturate<uint16_t>(in.idr_disagree);
  t.enhance_disagree = saturate<uint16_t>(in.enhance_disagree);
  t.idr_grants = saturate<uint16_t>(in.idr_grants);
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

double read_load1(const char* path) {
  FILE* f = std::fopen(path, "r");
  if (!f) return 0.0;
  double load1 = 0.0;
  int n = std::fscanf(f, "%lf", &load1);
  std::fclose(f);
  if (n != 1) return 0.0;
  return load1;
}

}  // namespace mabur
