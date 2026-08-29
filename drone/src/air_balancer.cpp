#include "air_balancer.h"
#include <algorithm>
#include <cmath>

namespace mabur {

AirBalancer::AirBalancer(BalancerFeed* feed) : feed_(feed) {}

void AirBalancer::on_frame(int sid, size_t len_in, size_t emitted) {
  if (sid < 0 || sid > 1 || len_in == 0) return;
  auto ema = [](double& e, double v) { e = e == 0.0 ? v : e + (v - e) / 16.0; };
  ema(len_[sid], static_cast<double>(len_in));
  ema(emit_[sid], static_cast<double>(emitted));
}

OvSplit AirBalancer::solve(double rate_b, double rate_e, double ov_cmd) {
  const double lo = 0.5 * ov_cmd, hi = 2.0 * ov_cmd;
  OvSplit out{ov_cmd, ov_cmd};
  // Anchor: the ov each stream was last told to fly (first solve anchors at
  // ov_cmd — the encoder is flying the commanded value until we say
  // otherwise). Captured ONCE here, before applied_ is overwritten below,
  // so both the solver and the feed-publish block below report against
  // the ov that was ACTUALLY in effect while the EWMA-averaged frames
  // were measured — not this solve's freshly computed output (spec §2;
  // publishing against the new ov would make excess_base measure the
  // wrong thing: this solve's own correction instead of the drift that
  // motivated it).
  const double ab = applied_[0] >= 0.0 ? applied_[0] : ov_cmd;
  const double ae = applied_[1] >= 0.0 ? applied_[1] : ov_cmd;
  if (!seeded() || rate_b <= 0.0 || rate_e <= 0.0) { /* publish + return */ }
  else {
    // Measured multiplier m_s = emit/len; anchor at the ov each stream was
    // last told to fly (first solve anchors at ov_cmd — the encoder is
    // flying the commanded value until we say otherwise).
    const double m_b = emit_[0] / len_[0], m_e = emit_[1] / len_[1];
    // air_s(ov) = len_s*(m_s + ov - a_s)/rate_s. Solve air_b == air_e
    // with the repair-byte budget len_b*ov_b + len_e*ov_e = (len_b+len_e)*ov_cmd.
    const double Lb = len_[0], Le = len_[1];
    const double kb = Lb / rate_b, ke = Le / rate_e;   // air per (1+ov)-unit
    const double B = (Lb + Le) * ov_cmd;               // repair-byte budget
    // ov_e from budget: ov_e = (B - Lb*ov_b)/Le. Substitute into balance:
    // kb*(m_b + ov_b - ab) = ke*(m_e + (B - Lb*ov_b)/Le - ae)
    const double denom = kb + ke * Lb / Le;
    double ov_b = (ke * (m_e + B / Le - ae) - kb * (m_b - ab)) / denom;
    double ov_e = (B - Lb * ov_b) / Le;
    // 2% deadband: keep the current anchors only if they are BOTH (a)
    // already balanced and (b) still consistent with the CURRENT budget
    // B. Balance alone is not enough: after ov_cmd changes (promote/
    // demote), both streams' measured air can shift near-proportionally
    // and stay balanced within 2% while still flying the stale total
    // repair budget from before the change (finding: deadband holds
    // stale anchors across an ov_cmd change). Gate on the budget too so
    // a genuine ov_cmd change always falls through to the solver.
    const double air_b0 = kb * m_b, air_e0 = ke * m_e;
    const bool balanced = std::abs(air_b0 - air_e0) < 0.02 * std::max(air_b0, air_e0);
    const double budget_now = Lb * ab + Le * ae;
    const bool budget_ok = std::abs(budget_now - B) < 0.02 * B;
    if (balanced && budget_ok) {
      ov_b = ab; ov_e = ae;
    }
    // Rails: clamp one side, re-solve the other from the budget, clamp it.
    if (ov_b < lo || ov_b > hi) {
      ov_b = std::clamp(ov_b, lo, hi);
      ov_e = (B - Lb * ov_b) / Le;
    } else if (ov_e < lo || ov_e > hi) {
      ov_e = std::clamp(ov_e, lo, hi);
      ov_b = (B - Le * ov_e) / Lb;
    }
    out.ov_base = std::clamp(ov_b, lo, hi);
    out.ov_enh = std::clamp(ov_e, lo, hi);
    applied_[0] = out.ov_base;
    applied_[1] = out.ov_enh;
  }
  if (feed_) {
    const double tot = len_[0] + len_[1];
    feed_->share_base.store(tot > 0.0 ? static_cast<float>(len_[0] / tot) : 0.5f,
                            std::memory_order_relaxed);
    feed_->excess_base.store(len_[0] > 0.0
        ? static_cast<float>(emit_[0] / len_[0] - (1.0 + ab)) : 0.0f,
        std::memory_order_relaxed);
    feed_->excess_enh.store(len_[1] > 0.0
        ? static_cast<float>(emit_[1] / len_[1] - (1.0 + ae)) : 0.0f,
        std::memory_order_relaxed);
    feed_->ov_base.store(static_cast<float>(out.ov_base), std::memory_order_relaxed);
    feed_->ov_enh.store(static_cast<float>(out.ov_enh), std::memory_order_relaxed);
  }
  return out;
}

}  // namespace mabur
