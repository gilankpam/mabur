#pragma once

// Port of devourer energy_model.py; calibration constants from gen/gen_tables.h;
// mirrors Python operation order exactly so vector compares are bit-exact.

namespace maburgs {

struct TxPoint {
  bool vht = false;
  int mcs = 0;
  int bw = 20;
  bool sgi = false;
  int txagc = 32;
};

// On-air PHY data rate (Mbps) via mabur::rc tables
double phy_rate_mbps(const TxPoint& p);

// Extra noise power (dB) vs 20 MHz reference: 10*log10(bw/20), 0 for bw<=20
double bw_noise_db(int bw);

// Radiated-power gain (dB) at TXAGC index, clamped 0..63
double gain_db(int txagc);

// PA DC draw (W) at TXAGC index, clamped 0..63
double pa_w(int txagc);

// Smallest TXAGC index giving >= need_db radiated gain, or -1 if none
int min_txagc_for_gain(double need_db);

// Effective goodput (bits/s) including per-frame preamble overhead
double phy_rate_eff_bps(const TxPoint& p, int payload_bytes);

// Fraction of wall time TX path is on-air to carry src_bitrate_bps at this
// MCS/FEC; >1.0 means infeasible
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
