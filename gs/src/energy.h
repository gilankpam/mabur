#pragma once

// Port of devourer energy_model.py (airtime/rate/baseline-power dimensions)
// plus mabur's own linear offset power model (gain_db/pa_w — see
// tools/pyref/offset_power.py, which DIVERGES from devourer's frozen
// txagc-index gain curve since 2026-07-17). Calibration constants from
// gen/gen_tables.h; mirrors Python operation order exactly so vector
// compares are bit-exact.

namespace maburgs {

struct TxPoint {
  bool vht = false;
  int mcs = 0;
  int bw = 20;
  bool sgi = false;
  // Signed qdB offset from the calibrated baseline (RCF wire semantics,
  // rc_proto bias-64), and the base kPaW index that offset=0 maps to.
  int pwr_offset_qdb = 0;
  int base_ref_idx = 53;
};

// On-air PHY data rate (Mbps) via mabur::rc tables
double phy_rate_mbps(const TxPoint& p);

// Extra noise power (dB) vs 20 MHz reference: 10*log10(bw/20), 0 for bw<=20
double bw_noise_db(int bw);

// Radiated-power gain (dB) at a signed qdB offset from baseline: linear,
// 0.25 dB/qdB (bench-validated slope, docs/txagc-calibration.md).
double gain_db(int offset_qdb);

// Smallest offset (qdB) giving >= need_db radiated gain. No clamping —
// the controller clamps against the deployable offset range.
int min_offset_qdb_for_gain(double need_db);

// Raw kPaW[] accessor (W), clamped 0..63 — the PA-draw curve's table edge.
double pa_w_index(int idx);

// PA DC draw (W) at a signed qdB offset from base_ref_idx, clamped 0..63
// (i.e. the PA-draw model tops out past the table edge).
double pa_w(int offset_qdb, int base_ref_idx);

// Effective goodput (bits/s) including per-frame preamble overhead
double phy_rate_eff_bps(const TxPoint& p, int payload_bytes);

// Fraction of wall time TX path is on-air to carry src_bitrate_bps at this
// MCS/FEC; >1.0 means infeasible. Airtime doesn't involve power.
double airtime_fraction(const TxPoint& p, double src_bitrate_bps,
                        double overhead, int payload_bytes);

// Average DC power (W): baseline always on + PA only over airtime
double avg_power_w(const TxPoint& p, double airtime_frac);

// J per delivered SOURCE bit; +inf when infeasible (airtime>1 or nothing
// delivered)
double energy_per_delivered_bit(const TxPoint& p, double src_bitrate_bps,
                                double overhead, int payload_bytes,
                                double p_deliver);

}  // namespace maburgs
