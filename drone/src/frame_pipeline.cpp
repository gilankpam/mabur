#include "frame_pipeline.h"

#include "mabur/frame_wire.h"
#include "mabur/nal.h"

static_assert(mabur::framewire::kFrameHdrLen == VENC_FRAME_META_SIZE,
              "FrameHdr must fit the producer meta region exactly for the "
              "in-place stamp to keep the frame unit contiguous");

namespace mabur {

std::vector<UepBody> FramePipeline::encode(UepEncoder& uep, uint8_t* buf,
                                          size_t payload_len,
                                          const VencFrameMeta& meta,
                                          uint64_t now_ms) {
  std::vector<UepBody> out;
  encode(uep, buf, payload_len, meta, now_ms,
         [&](UepBody&& b) { out.push_back(std::move(b)); });
  return out;
}

void FramePipeline::encode(UepEncoder& uep, uint8_t* buf, size_t payload_len,
                           const VencFrameMeta& meta, uint64_t now_ms,
                           const UepBodySink& sink) {
  const uint8_t* payload = buf + VENC_FRAME_META_SIZE;
  const bool meta_idr = (meta.flags & VENC_FRAME_FLAG_IDR) != 0;

  int sid = classify_frame(payload, payload_len);
  if (meta_idr && sid != 0) {
    sid = 0;          // trust the union of scan and producer flag (protect up)
    ++idr_disagree_;  // disagreement is a bug signal, not a silent choice
  }

  const bool meta_enhance = (meta.flags & VENC_FRAME_FLAG_ENHANCE) != 0;
  // Vanish detection runs on EVERY ring read — before the shed check below,
  // because a shed drop happens downstream of the read and must never look
  // like an upstream hole (read-side pts stays continuous across sheds).
  track_vanish(meta, meta_idr, meta_enhance, now_ms);

  const bool scan_enhance = frame_is_trail_n(payload, payload_len);
  // SVC-T enhance (sid 1, droppable): producer flag and TRAIL_N scan must
  // AGREE. Shedding a referenced frame corrupts decode, so a single signal
  // protects up to base (critical stays 0) and is surfaced, never silent.
  // Uses frame_is_trail_n rather than sid == 1: the scan side pinpoints
  // real TRAIL_N frames only and feeds the agreement check directly.
  if (scan_enhance != meta_enhance) {
    sid = 0;
    ++enhance_disagree_;
  }

  // Shed check BEFORE frame_id allocation: a shed frame must not punch an
  // id gap into the GS FrameStream (each gap stalls emit for gap_timeout /
  // lookahead). The drop is still booked in dropped(sid); the discont latch
  // stays pending so the window anchors on a frame that actually ships.
  if (uep.drop_if_shed(sid)) return;

  // Latches AFTER the shed return so a shed frame's enc_us never overwrites
  // the last shipped frame's value — see last_enc_us()'s doc comment.
  last_enc_us_ = meta.enc_us;

  if (discont_pending_) {
    discont_pending_ = false;
    discont_until_ms_ = now_ms + kDiscontStickyMs;
  }
  const bool discont = now_ms < discont_until_ms_;

  framewire::FrameHdr h;
  h.frame_id = next_frame_id_++;
  h.flags = static_cast<uint8_t>((meta_idr ? framewire::kFlagIdr : 0) |
                                 (discont ? framewire::kFlagDiscont : 0));
  h.codec = meta.codec;
  h.pts_us = meta.pts;
  framewire::pack_frame_hdr(h, buf);

  uep.add_frame(sid, buf, framewire::kFrameHdrLen + payload_len, now_ms, sink);
}

void FramePipeline::track_vanish(const VencFrameMeta& meta, bool meta_idr,
                                 bool meta_enhance, uint64_t now_ms) {
  if (have_prev_pts_) {
    // Unsigned subtraction: the 32-bit pts wrap (~71.6 min) is invisible.
    const uint32_t delta = meta.pts - prev_pts_;
    if (delta >= kResyncUs) {
      // Encoder clock discontinuity: re-anchor, re-confirm, count nothing.
      period_samples_ = 0;
      period_us_ = 0.0;
    } else if (delta > 0) {
      if (period_samples_ >= kMinPeriodSamples &&
          static_cast<double>(delta) > kVanishFactor * period_us_) {
        int k = static_cast<int>(delta / period_us_ + 0.5) - 1;
        if (k < 1) k = 1;
        if (k > kMaxVanishPerEvent) k = kMaxVanishPerEvent;
        int base = 0;
        for (int i = 1; i <= k; ++i) {
          // Strict base/enhance alternation out from the previous frame's
          // producer flag: odd offsets flip, even offsets match.
          const bool enh = (i % 2) ? !prev_enhance_ : prev_enhance_;
          if (!enh) ++base;
        }
        vanished_base_ += static_cast<uint64_t>(base);
        vanished_enh_ += static_cast<uint64_t>(k - base);
        if (base > 0) {
          if (have_last_idr_ && now_ms - last_idr_ms_ < kSelfIdrGuardMs)
            ++self_idr_refused_;  // likely IDR-burst-caused: see header
          else
            self_idr_pending_ = true;
        }
      } else {
        // Normal-looking step: feed the period EMA (alpha 1/8). Before the
        // first sample this seeds directly — if that first delta was itself a
        // hole, later real deltas still pass the < 1.5x gate and pull the
        // estimate down, which is why detection waits for kMinPeriodSamples.
        period_us_ = period_samples_ == 0
                         ? static_cast<double>(delta)
                         : period_us_ + (static_cast<double>(delta) - period_us_) / 8.0;
        if (period_samples_ < kMinPeriodSamples) ++period_samples_;
      }
    }
  }
  prev_pts_ = meta.pts;
  prev_enhance_ = meta_enhance;
  have_prev_pts_ = true;
  // AFTER detection, deliberately: an IDR arriving right behind a hole heals
  // that hole, so it clears the latch the same call — and its timestamp
  // guards the NEXT event, not this one.
  if (meta_idr) {
    self_idr_pending_ = false;
    have_last_idr_ = true;
    last_idr_ms_ = now_ms;
  }
}

}  // namespace mabur
