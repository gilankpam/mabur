#include "rcf_slot.h"

#include <cmath>

namespace maburgs {

void RcfSlotter::on_au_first(uint64_t now_ms) {
  if (have_first_ && now_ms > last_first_ms_) {
    const double iv = static_cast<double>(now_ms - last_first_ms_);
    // Only plausible frame periods train the predictor (a gap/stall or a
    // duplicate-ms pair must not drag it).
    if (iv >= 5.0 && iv <= 100.0) period_ms_ += (iv - period_ms_) / 8.0;
  }
  last_first_ms_ = now_ms;
  have_first_ = true;
}

bool RcfSlotter::idle_ahead(uint64_t now_ms) const {
  if (!have_first_) return true;
  const double next = static_cast<double>(last_first_ms_) + period_ms_;
  return static_cast<double>(now_ms) + cfg_.lead_ms < next - cfg_.guard_ms;
}

int RcfSlotter::tail_ub_ms() const {
  const double floor = static_cast<double>(cfg_.probe_tail_ms);
  const double est = tail_est_ms_ > floor ? tail_est_ms_ : floor;
  return static_cast<int>(std::ceil(est)) + 1;
}

void RcfSlotter::on_au_complete(uint64_t now_ms, bool probe_follows) {
  last_au_ms_ = now_ms;
  have_au_ = true;
  if (probe_follows) {
    // The burst is still on air (probe to come): nothing releases here.
    // Wait for the probe, or for the deadline if it is lost.
    probe_wait_ = true;
    probe_wait_from_ms_ = now_ms;
    probe_deadline_ms_ = now_ms + static_cast<uint64_t>(tail_ub_ms());
    idle_from_ms_ = probe_deadline_ms_;
    return;
  }
  probe_wait_ = false;
  idle_from_ms_ = now_ms;
  if (!pending_.empty() && idle_ahead(now_ms)) {
    release_pending_ = true;
    release_reason_ = SlotReason::Au;
  }
}

void RcfSlotter::on_probe_tail(uint64_t now_ms) {
  if (probe_wait_ && now_ms >= probe_wait_from_ms_) {
    // Learn the completion->probe offset: jump up to a larger one at once,
    // decay slowly (2 %/observation) so the deadline tracks the current
    // rung/frame-size regime rather than a one-off.
    const double obs = static_cast<double>(now_ms - probe_wait_from_ms_);
    tail_est_ms_ *= 0.98;
    if (obs > tail_est_ms_) tail_est_ms_ = obs;
    if (tail_est_ms_ > kTailMaxMs) tail_est_ms_ = kTailMaxMs;
  }
  probe_wait_ = false;
  idle_from_ms_ = now_ms;
  if (!pending_.empty() && !release_pending_ && idle_ahead(now_ms)) {
    release_pending_ = true;
    release_reason_ = SlotReason::Probe;
  }
}

bool RcfSlotter::offer(SlotFrame& f, uint64_t now_ms, bool bypass) {
  f.offered_ms = now_ms;
  const bool enabled = cfg_.hold_max_ms > 0;
  const bool video = have_au_ && now_ms >= last_au_ms_ &&
                     now_ms - last_au_ms_ <= static_cast<uint64_t>(cfg_.video_recent_ms);
  if (!enabled || bypass || !video) {
    ++passthru_;
    f.reason = SlotReason::Passthru;
    return false;
  }
  if (now_ms >= idle_from_ms_ &&
      now_ms - idle_from_ms_ <= static_cast<uint64_t>(cfg_.grace_ms) &&
      idle_ahead(now_ms)) {
    // The idle has just begun (the burst -- probe included -- is off
    // air): this IS the slot.
    ++released_au_;
    f.reason = SlotReason::Grace;
    return false;
  }
  if (pending_.empty()) hold_start_ms_ = now_ms;
  pending_.push_back(std::move(f));
  return true;
}

std::vector<SlotFrame> RcfSlotter::take_due(uint64_t now_ms) {
  std::vector<SlotFrame> out;
  if (pending_.empty()) {
    release_pending_ = false;
    return out;
  }
  if (probe_wait_ && now_ms >= probe_deadline_ms_) {
    // The probe never came (lost on air): release at the deadline if the
    // idle ahead still allows it, else wait for the next completion.
    probe_wait_ = false;
    if (!release_pending_ && idle_ahead(now_ms)) {
      release_pending_ = true;
      release_reason_ = SlotReason::Au;
    }
  }
  const bool due = release_pending_;
  const bool expired =
      now_ms >= hold_start_ms_ &&
      now_ms - hold_start_ms_ >= static_cast<uint64_t>(cfg_.hold_max_ms);
  if (!due && !expired) return out;
  const SlotReason reason = due ? release_reason_ : SlotReason::Timeout;
  if (reason == SlotReason::Probe) released_probe_ += pending_.size();
  else if (reason == SlotReason::Au) released_au_ += pending_.size();
  else released_timeout_ += pending_.size();
  for (auto& f : pending_) {
    f.reason = reason;
    out.push_back(std::move(f));
  }
  pending_.clear();
  release_pending_ = false;
  return out;
}

}  // namespace maburgs
