#include "rcf_slot.h"

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

void RcfSlotter::on_au_complete(uint64_t now_ms) {
  last_au_ms_ = now_ms;
  have_au_ = true;
  if (!pending_.empty() && idle_ahead(now_ms)) release_ = true;
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
  if (now_ms - last_au_ms_ <= static_cast<uint64_t>(cfg_.grace_ms) &&
      idle_ahead(now_ms)) {
    // The idle has just begun: this IS the slot.
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
    release_ = false;
    return out;
  }
  const bool expired =
      now_ms >= hold_start_ms_ &&
      now_ms - hold_start_ms_ >= static_cast<uint64_t>(cfg_.hold_max_ms);
  if (!release_ && !expired) return out;
  if (release_) released_au_ += pending_.size();
  else released_timeout_ += pending_.size();
  for (auto& f : pending_) {
    f.reason = release_ ? SlotReason::Au : SlotReason::Timeout;
    out.push_back(std::move(f));
  }
  pending_.clear();
  release_ = false;
  return out;
}

}  // namespace maburgs
