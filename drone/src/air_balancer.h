#pragma once
#include <cstddef>
#include "rc_agent.h"  // BalancerFeed

namespace mabur {

struct OvSplit { double ov_base; double ov_enh; };

// Equalizes per-frame air between the two video streams by redistributing
// the commanded FEC overhead, anchored on ACTUAL emitted bytes (spec §2;
// the nominal len*(1+ov)/rate model measured a reproducible ~2x gain
// error — docs/airtime-balance-spike-findings-2026-08-29.md).
class AirBalancer {
 public:
  explicit AirBalancer(BalancerFeed* feed);  // feed may be nullptr (tests)

  // One shipped frame: sid 0/1, frame-unit bytes in, body bytes out.
  // IDR frames are excluded by the caller (spec: 2-10x outliers).
  void on_frame(int sid, size_t len_in, size_t emitted);

  // Solve for the current op. rate_* in Mbps (from the applied ladder),
  // ov_cmd = the commanded literal overhead. Returns the split to apply
  // via UepEncoder::set_layer_overhead, already clamped. Also publishes
  // share/excess/applied-ov into the feed.
  OvSplit solve(double rate_b_mbps, double rate_e_mbps, double ov_cmd);

  bool seeded() const { return len_[0] > 0.0 && len_[1] > 0.0; }

 private:
  double len_[2] = {0.0, 0.0};    // EWMA frame-unit bytes, alpha 1/16
  double emit_[2] = {0.0, 0.0};   // EWMA emitted body bytes, alpha 1/16
  double applied_[2] = {-1.0, -1.0};  // ov last returned (anchor); <0 = none
  BalancerFeed* feed_;
};

}  // namespace mabur
