#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/uep_encoder.h"

// venc_frame_ring.h (vendored, C header) — see frame_source.h for why the
// include is wrapped.
extern "C" {
#include "venc_frame_ring.h"
}

namespace mabur {

// The per-frame ingest step shared by maburd's hot thread and its --dry-run
// replay, so both drive byte-identical wire units: classify the Annex-B frame,
// protect up on the producer's IDR flag, stamp the FrameHdr over the meta
// region, and hand the frame unit to the UEP encoder.
//
// Not thread-safe: one instance per ingest thread (frame_id and the
// discontinuity latch are per-stream state).
class FramePipeline {
 public:
  // buf: VENC_FRAME_META_SIZE bytes of meta followed by payload_len Annex-B
  // bytes — exactly how FrameSource::read fills it. The meta region is
  // OVERWRITTEN in place with the FrameHdr (kFrameHdrLen ==
  // VENC_FRAME_META_SIZE), so the unit (header + Annex-B) stays contiguous
  // with zero copies. Returns the bodies the encoder produced.
  std::vector<UepBody> encode(UepEncoder& uep, uint8_t* buf, size_t payload_len,
                              const VencFrameMeta& meta, uint64_t now_ms);

  // Marks the NEXT frame as a pts/frame_id re-base point: start of stream, or
  // a reattach that joined a new producer's ring mid-GOP. The GS rebases on
  // that frame and its decoder recovers at the next IDR.
  void mark_discontinuity() { discont_ = true; }

  // Frames whose Annex-B scan disagreed with the producer's IDR flag. A bug
  // signal (producer vs. scanner), surfaced in maburd's stats line — never a
  // silent choice.
  uint64_t idr_disagreements() const { return idr_disagree_; }
  uint16_t next_frame_id() const { return next_frame_id_; }

 private:
  uint16_t next_frame_id_ = 0;
  bool discont_ = true;  // first frame after start
  uint64_t idr_disagree_ = 0;
};

}  // namespace mabur
