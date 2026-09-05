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
  bool probe_on = false;  // RcAgent::probe_on() — flags bit2, spec 2026-09-04 probe-stream
  bool congestion_shed = false;  // RcAgent::congestion_shed() — flags bit4
  uint64_t generation = 0;
  rc::PhyMode mode = rc::PhyMode::HT;
  uint8_t mcs = 0, bw = 20;
  double applied_ov_base = 0.0;
  double applied_ov_enh = 0.0;
  uint64_t rcf_age_ms = 0, rcf_rx = 0;
  // link-rtt: seq of the RCF rcf_age_ms ages against + the pts-domain clock
  // (MI timebase, µs) at telem build. Straight pass-through, no saturation.
  // echo_valid maps to flags bit3; false whenever RcAgent's seq window was
  // reset (DISC re-establish, failsafe rebase) and the echo would be stale.
  uint16_t rcf_seq_echo = 0;
  bool rcf_seq_echo_valid = false;
  uint64_t pts_at_build_us = 0;
  uint64_t enc_frames = 0, enc_bytes = 0;
  int cmd_kbps = 0;
  int roi_qp = 0;  // RcAgent's ROI override as commanded (actuator.last_roi_qp)
  uint64_t ring_drops = 0;
  size_t txq_depth = 0, txq_cap = 0;
  uint64_t txq_drops = 0;
  // Per-telemetry-window max TxQueue wait (Task 4's txq_wait_max_ms atomic,
  // consumed via .exchange(0) at the 1 Hz tick) — saturating.
  uint64_t txq_wait_max_ms = 0;
  uint64_t radio_sent = 0, radio_drops = 0, usb_fail = 0;
  UplinkTrack::Snap uplink;
  int soc_temp_c = -128;
  int thermal_delta = 0;
  double load1 = 0.0;
  uint64_t idr_disagree = 0, enhance_disagree = 0;
  // FramePipeline vanish counters (venc-ring vanish detection,
  // docs/venc-ring-vanish-findings-2026-08-12.md).
  uint64_t vanished_base = 0, vanished_enh = 0, self_idr_refused = 0;
  // venc_get_stats() (venc_core.h), zero on host builds where the encoder
  // isn't compiled in. See rc::Telem for what they mean.
  uint64_t venc_full_drops = 0;
  int venc_ring_fill_pct = 0;
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
