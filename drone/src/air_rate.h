#pragma once
// Delivered air rate of a TX spec: nominal PHY rate × the per-MCS efficiency
// the bench measured at saturation (docs/bandwidth-sweep-findings-2026-09-17.md,
// config air_clock.efficiency_20 / efficiency_40, picked by the spec's bw).
// Both drone consumers of "how fast does the air drain" price off this,
// never off the nominal rate alone: RcAgent::run_bitrate_policy (so
// encoder.airtime_budget is a fraction of capacity the link actually
// delivers) and the air clock (so the enh shed gate and the SBI air_ms
// stamp see the same pipe the policy filled).
#include <array>

#include "config.h"
#include "mabur/profile.h"

namespace mabur {

inline double delivered_mbps(const rc::LayerTxSpec& s, const AirClockCfg& c) {
  const std::array<double, 8>& eff = s.bw == 40 ? c.efficiency_40 : c.efficiency_20;
  const size_t i = s.mcs < eff.size() ? s.mcs : eff.size() - 1;
  return rc::phy_rate_mbps(s) * eff[i];
}

}  // namespace mabur
