#include "air_feed.h"

namespace mabur {

AirFeed::AirFeed(AirFeedOut* out) : out_(out) {}

void AirFeed::set_applied(double ovb, double ove) {
  applied_[0] = ovb;
  applied_[1] = ove;
}

void AirFeed::on_frame(int sid, size_t len_in, size_t emitted) {
  if (sid < 0 || sid > 1 || len_in == 0) return;
  auto ema = [](double& e, double v) { e = e == 0.0 ? v : e + (v - e) / 16.0; };
  ema(len_[sid], static_cast<double>(len_in));
  ema(emit_[sid], static_cast<double>(emitted));
  if (!out_) return;

  // Anchor: the ov each stream is currently flying. set_applied() tracks
  // the op pair (Task 6's apply_op_to_uep call sites); an armed
  // debug-HTTP override wins over it here too, mirroring main.cpp's hot
  // loop where the override wins over the op pair on the UEP layers
  // themselves — the published anchor must track what's ACTUALLY flying.
  double ab = applied_[0] >= 0.0 ? applied_[0] : 0.0;
  double ae = applied_[1] >= 0.0 ? applied_[1] : 0.0;
  const int fb_pct = out_->ovr_base_pct.load(std::memory_order_relaxed);
  const int fe_pct = out_->ovr_enh_pct.load(std::memory_order_relaxed);
  if (fb_pct >= 0 && fe_pct >= 0) {
    ab = fb_pct / 100.0;
    ae = fe_pct / 100.0;
  }

  const double tot = len_[0] + len_[1];
  out_->share_base.store(tot > 0.0 ? static_cast<float>(len_[0] / tot) : 0.5f,
                         std::memory_order_relaxed);
  out_->excess_base.store(len_[0] > 0.0
      ? static_cast<float>(emit_[0] / len_[0] - (1.0 + ab)) : 0.0f,
      std::memory_order_relaxed);
  out_->excess_enh.store(len_[1] > 0.0
      ? static_cast<float>(emit_[1] / len_[1] - (1.0 + ae)) : 0.0f,
      std::memory_order_relaxed);
  out_->ov_base.store(static_cast<float>(ab), std::memory_order_relaxed);
  out_->ov_enh.store(static_cast<float>(ae), std::memory_order_relaxed);
}

}  // namespace mabur
