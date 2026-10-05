#pragma once
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

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
  bool congestion_shed = false;  // RcAgent::congestion_shed() — flags bit4
  bool low_power = false;  // RcAgent::low_power() — flags bit7, spec 2026-09-20
  // RcAgent::take_auth_reject() — flags bit1 (rc::kTelemAuthReject): a
  // control frame failed its tag since the last Telem (spec 2026-10-01 §8).
  bool auth_reject = false;
  uint64_t rcf_age_ms = 0, rcf_rx = 0;
  // link-rtt: seq of the RCF rcf_age_ms ages against + the pts-domain clock
  // (MI timebase, µs) at telem build. Straight pass-through, no saturation.
  // echo_valid maps to flags bit3; false outside LINKED (failsafe rebase),
  // where the echo would be stale.
  uint16_t rcf_seq_echo = 0;
  bool rcf_seq_echo_valid = false;
  uint64_t pts_at_build_us = 0;
  int cmd_kbps = 0;
  uint64_t txq_drops = 0;
  // Per-telemetry-window max TxQueue wait (Task 4's txq_wait_max_ms atomic,
  // consumed via .exchange(0) at the 1 Hz tick) — saturating.
  uint64_t txq_wait_max_ms = 0;
  uint64_t usb_fail = 0;
  // fec-nack, per period (spec 2026-10-05 §5): verified T_NACKs answered,
  // symbols re-sent, symbols the token bucket refused.
  uint64_t nack_rx = 0, retx_syms = 0, retx_refused = 0;
  // RX-side channel view for this telemetry period (cca-on 2026-09-23):
  // the RX callback's own / foreign / CRC-failed frame split.
  uint64_t rx_own = 0, rx_foreign = 0, rx_crcfail = 0;
  UplinkTrack::Snap uplink;
  int soc_temp_c = -128;
  // CpuBusySampler::sample() -- empty until two ticks have been read.
  std::optional<double> cpu_pct;
  // VtxRecorder::status_byte() (spec 2026-09-26), straight pass-through.
  uint8_t rec_status = 0;
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

// CPU busy percent between consecutive sample() calls, from the aggregate
// "cpu" line of /proc/stat: busy = total - idle - iowait. Empty on the
// first call (no baseline yet) and whenever the file cannot be read or
// parsed, in which case the baseline is dropped so the next good pair
// starts fresh. Replaces loadavg, which on the SigmaStar image counts the
// SDK's permanently parked D-state workers (~13, load-independent). One
// instance per reader thread; not thread-safe by design.
class CpuBusySampler {
 public:
  std::optional<double> sample(const char* path = "/proc/stat");

 private:
  bool have_ = false;
  uint64_t busy_ = 0, total_ = 0;
};

}  // namespace mabur
