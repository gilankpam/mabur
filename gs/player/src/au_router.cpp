#include "au_router.h"

#include <utility>

namespace maburplay {

AuRouter::AuRouter(StreamFeeder& feeder, std::unique_ptr<VideoBackend>& backend)
    : feeder_(feeder), backend_(backend) {}

void AuRouter::set_draining() {
  draining_ = true;
  feeder_.stop_new();
}

void AuRouter::note_submit_(const maburgs::AuRecordMeta& m) {
  FeedNote n;
  n.kind = FeedNote::Kind::kSubmit;
  n.meta = m;
  n.t_us = hooks_.now_us();
  hooks_.note(std::move(n));
}

void AuRouter::deliver_(AuEvent& ev, bool complete, bool decodable, uint64_t t_deliver) {
  FeedNote n;
  n.kind = FeedNote::Kind::kDelivery;
  n.meta = ev.meta;
  n.t_us = t_deliver;
  n.bytes = static_cast<uint32_t>(ev.au.size());
  n.complete = complete;
  n.decodable = decodable;
  n.au = std::move(ev.au);
  hooks_.note(std::move(n));
}

void AuRouter::on_event(AuEvent&& ev) {
  if (ev.kind != AuEventKind::kClose) {
    feeder_.on_open(ev, armed_);
    return;
  }
  if (ev.aborted) {
    // The record that surfaced open ends with nothing to decode (writer
    // overflow, resync, writer restart, enhance dropped by policy): end what
    // the feeder started. Not a delivery.
    feeder_.on_close(ev, false);
    return;
  }
  const uint64_t t_deliver = hooks_.now_us();
  const bool complete = (ev.meta.flags & maburgs::kRecFlagComplete) != 0;
  // Slice salvage (spec §5.5): a salvaged AU is a legal, gap-free picture.
  const bool decodable = maburgs::au_decodable(ev.meta.flags);
  // Flush-ordering contract (Task 8 review): every held frame released, a
  // streamed picture ended, the decoder reset -- before this record touches it.
  if (ev.flush_before && !hooks_.flush()) return;
  // A record the feeder started streaming is finished (or already ended) there
  // and NEVER also submitted whole. dec = t_complete -> decoded, and the frame
  // cannot exist before the LAST append in on_close(): the submit note first.
  if (feeder_.owns(ev.meta.rec_no, ev.meta.pts_us)) {
    if (decodable) note_submit_(ev.meta);
    else truncated_.fetch_add(1, std::memory_order_relaxed);
    if (feeder_.on_close(ev, decodable) == StreamFeeder::Close::kStreamed)
      submits_.fetch_add(1, std::memory_order_relaxed);
    deliver_(ev, complete, decodable, t_deliver);
    return;
  }
  feeder_.on_close(ev, decodable);  // seen open but not streamed: the whole path below
  // Never feed a truncated AU (hangs rkvdec2: a truncated slice declares more
  // bitstream than exists). Before arming: a truncated sid0 must not arm.
  if (!decodable) {
    truncated_.fetch_add(1, std::memory_order_relaxed);
  } else if (!draining_ && (armed_ || (ev.meta.sid == 0 && complete))) {
    // Only a complete sid-0 AU (fresh VPS/SPS/PPS) arms; a salvaged one never.
    if (!armed_) {
      armed_ = true;
      if (t_sync_us_ == 0) t_sync_us_ = t_deliver;
    }
    note_submit_(ev.meta);
    if (backend_) backend_->submit_au(ev.au.data(), ev.au.size(), ev.meta.pts_us);
    submits_.fetch_add(1, std::memory_order_relaxed);
    feeder_.count_whole();
  }
  deliver_(ev, complete, decodable, t_deliver);
}

}  // namespace maburplay
