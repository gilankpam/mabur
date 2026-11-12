#include "channel_observe.h"

namespace maburplay {

namespace {
constexpr char kObservePrefix[] = "observe ";
}  // namespace

void ChannelObserve::tick(bool picker_open, uint64_t now_ms) {
  const bool edge = picker_open != open_level_;
  open_level_ = picker_open;
  if (edge) {
    if (picker_open) {
      want_ = Want::kOpen;
    } else {
      want_ = Want::kClose;
      state_.clear();
    }
  } else if (picker_open && want_ == Want::kNone && now_ms - last_send_ms_ >= kRenewMs) {
    want_ = Want::kOpen;
  }

  // Poll first: a reply that frees the link this tick lets the pending
  // send below go out in the same tick instead of waiting a full
  // iteration.
  if (const std::optional<LiveReply> reply = link_.poll(now_ms)) {
    // Still polled while closed (it owns the link's own flight/timeout
    // bookkeeping either way), but never applied: a reply landing after
    // the close edge belongs to the close's own "observe 0" or to a
    // renewal sent just before it, and state() must read "" from the
    // close edge until the first reply to a new open, not a late
    // straggler's measurer name.
    if (open_level_ && reply->ok && reply->text.rfind(kObservePrefix, 0) == 0)
      state_ = reply->text.substr(sizeof(kObservePrefix) - 1);
    // An "err ..." or an unrecognized shape leaves state() as it was.
  }

  if (want_ != Want::kNone && link_.idle()) {
    const std::string line =
        want_ == Want::kOpen ? "observe " + std::to_string(kLeaseMs) : "observe 0";
    std::string why;
    if (link_.send_line(line, now_ms, &why)) {
      if (want_ == Want::kOpen) last_send_ms_ = now_ms;
      want_ = Want::kNone;
    }
  }
}

}  // namespace maburplay
