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
  const uint8_t* payload = buf + VENC_FRAME_META_SIZE;
  const bool meta_idr = (meta.flags & VENC_FRAME_FLAG_IDR) != 0;

  int sid = classify_frame(payload, payload_len);
  if (meta_idr && sid != 0) {
    sid = 0;          // trust the union of scan and producer flag (protect up)
    ++idr_disagree_;  // disagreement is a bug signal, not a silent choice
  }

  const bool meta_enhance = (meta.flags & VENC_FRAME_FLAG_ENHANCE) != 0;
  const bool scan_enhance = frame_is_trail_n(payload, payload_len);
  // SVC-T enhance (sid 3, droppable): producer flag and TRAIL_N scan must
  // AGREE. Shedding a referenced frame corrupts decode, so a single signal
  // protects up to base (critical stays 0) and is surfaced, never silent.
  // Uses frame_is_trail_n rather than sid == 3: classify_frame also routes
  // TRAIL_R tid >= 2 to sid 3 (devourer-tid routing), which is not enhance
  // and must not be flagged as a disagreement.
  if (scan_enhance != meta_enhance) {
    if (sid > 1) sid = 1;
    ++enhance_disagree_;
  }

  // Shed check BEFORE frame_id allocation: a shed frame must not punch an
  // id gap into the GS FrameStream (each gap stalls emit for gap_timeout /
  // lookahead). The drop is still booked in dropped(sid); the discont latch
  // stays pending so the window anchors on a frame that actually ships.
  if (uep.drop_if_shed(sid)) return {};

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

  return uep.add_frame(sid, buf, framewire::kFrameHdrLen + payload_len, now_ms);
}

}  // namespace mabur
