#pragma once
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "mabur/profile.h"
#include "mabur/rc_proto.h"

namespace mabur {

// Fed from maburd's rx_callback with pkt.RxAtrib of CRC-clean RC frames;
// thread-safe for one writer (RX thread) + one reader (main tick):
// doubles guarded by a std::mutex — 20 Hz writes, 1 Hz reads.
class UplinkTrack {
 public:
  void on_rc_frame(const uint8_t rssi[2], const int8_t snr[2]);  // kEmaAlpha 0.1, seed-then-EMA

  struct Snap {
    bool has = false;
    double rssi[2] = {0.0, 0.0};
    double snr[2] = {0.0, 0.0};
  };
  Snap snap() const;

 private:
  mutable std::mutex m_;
  bool has_ = false;
  double rssi_[2] = {0.0, 0.0};
  double snr_[2] = {0.0, 0.0};
};

// Pure: input struct -> rc::Telem. All clamping/saturation here.
struct TelemInputs {
  int state = 0;
  bool failsafe_shed = false;
  bool radio_rx_ok = false;
  uint64_t generation = 0;
  rc::PhyMode mode = rc::PhyMode::HT;
  uint8_t mcs = 0, bw = 20;
  double applied_ov = 0.0;
  int applied_off_qdb = 0;
  int derate_qdb = 0;
  uint64_t rcf_age_ms = 0, rcf_rx = 0;
  uint64_t enc_frames = 0, enc_bytes = 0;
  int cmd_kbps = 0, qp = 0;
  uint64_t ring_drops = 0;
  size_t txq_depth = 0, txq_cap = 0;
  uint64_t txq_drops = 0;
  uint64_t radio_sent = 0, radio_drops = 0, usb_fail = 0;
  UplinkTrack::Snap uplink;
  int soc_temp_c = -128;
  int thermal_delta = 0;
  double load1 = 0.0;
  uint64_t idr_disagree = 0, enhance_disagree = 0;
};

rc::Telem make_telem(uint16_t tlm_seq, const TelemInputs& in);

// Reads /sys/class/thermal/thermal_zone0/temp (millidegrees) and
// /proc/loadavg; -128 / 0.0 when unreadable. Trivial, separated for tests
// via path injection.
// Standard kernel thermal zone (millidegrees). -128 when unreadable.
int read_soc_temp_c(const char* path = "/sys/class/thermal/thermal_zone0/temp");
// SigmaStar fallback: cpufreq's "Temp=NN" (already degrees C) — the OpenIPC
// drone SoC ships no thermal_zone (found on hw 2026-07-26). -128 when
// unreadable.
int read_soc_temp_c_sigmastar(
    const char* path = "/sys/devices/system/cpu/cpufreq/temp_out");
double read_load1(const char* path = "/proc/loadavg");

}  // namespace mabur
