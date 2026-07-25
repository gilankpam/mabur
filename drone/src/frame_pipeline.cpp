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
