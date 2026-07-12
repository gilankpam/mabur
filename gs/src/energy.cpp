#include "energy.h"

#include <cmath>
#include <limits>

#include "gen/gen_tables.h"
#include "mabur/profile.h"

namespace maburgs {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
int clamp_idx(int idx) { return idx < 0 ? 0 : (idx > 63 ? 63 : idx); }
}  // namespace

double phy_rate_mbps(const TxPoint& p) {
  mabur::rc::LayerTxSpec s;
  s.mode = p.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT;
  s.mcs = static_cast<uint8_t>(p.mcs);
  s.bw = static_cast<uint8_t>(p.bw);
  s.sgi = p.sgi;
  return mabur::rc::phy_rate_mbps(s);
}

double bw_noise_db(int bw) {
  if (bw <= 20) return 0.0;
  return 10.0 * std::log10(bw / 20.0);
}

double gain_db(int txagc) { return gen::kTxagcGainDb[clamp_idx(txagc)]; }
double pa_w(int txagc) { return gen::kPaW[clamp_idx(txagc)]; }

int min_txagc_for_gain(double need_db) {
  for (int idx = 0; idx < 64; ++idx)
    if (gen::kTxagcGainDb[idx] >= need_db) return idx;
  return -1;
}

double phy_rate_eff_bps(const TxPoint& p, int payload_bytes) {
  const double r_bps = phy_rate_mbps(p) * 1e6;
  const double t_air = (8.0 * payload_bytes) / r_bps;
  const double t_on = gen::kTPreUs * 1e-6 + t_air;
  return (8.0 * payload_bytes) / t_on;
}

double airtime_fraction(const TxPoint& p, double src_bitrate_bps,
                        double overhead, int payload_bytes) {
  const double on_air_bps = src_bitrate_bps * (1.0 + overhead);
  return on_air_bps / phy_rate_eff_bps(p, payload_bytes);
}

double avg_power_w(const TxPoint& p, double airtime_frac) {
  const double af = airtime_frac < 1.0 ? airtime_frac : 1.0;
  return gen::kPBaselineW + af * pa_w(p.txagc);
}

double energy_per_delivered_bit(const TxPoint& p, double src_bitrate_bps,
                                double overhead, int payload_bytes,
                                double p_deliver) {
  if (p_deliver <= 0.0) return kInf;
  const double af = airtime_fraction(p, src_bitrate_bps, overhead, payload_bytes);
  if (af > 1.0) return kInf;
  return avg_power_w(p, af) / (src_bitrate_bps * p_deliver);
}

}  // namespace maburgs
