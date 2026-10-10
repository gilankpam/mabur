#ifndef MABUR_PLAYER_PTS_UNWRAP_H_
#define MABUR_PLAYER_PTS_UNWRAP_H_

#include <cstdint>

namespace maburplay {

// The wire pts is u32 microseconds of drone MI_SYS uptime: it wraps every
// 71.6 min, with no discont and no flush. The patched MPP h265d hal takes a
// STREAM_APPEND for a picture it has not opened only when its RK_S64 pts is
// newer than the newest it opened (hal_h265d_vdpu34x.c h265d_strm_append),
// so a raw u32 makes the first streamed picture after a wrap "late": its
// appends AND its empty-LAST abort are refused and the hal leaves it open on
// its first slice (corrupt reference, an rkvdec reset, a stall).
//
// PtsUnwrap extends the u32 to a monotonic 64-bit value: each step moves by
// the signed 32-bit delta from the last pts (half-range rule: a step of more
// than 2^31 backwards is a forward wrap, anything less is a real step back,
// e.g. a re-sent picture, and stays behind). The first value is seeded at
// pts + 2^32 so a step back right after the seed never goes below zero.
// The low 32 bits always equal the input pts (ext == pts mod 2^32), so a
// decoded frame's u32 pts is just the truncation of the 64-bit one.
class PtsUnwrap {
 public:
  uint64_t operator()(uint32_t pts) {
    if (!seeded_) {
      seeded_ = true;
      ext_ = (uint64_t{1} << 32) + pts;
    } else {
      ext_ += static_cast<uint64_t>(static_cast<int64_t>(static_cast<int32_t>(pts - last_)));
    }
    last_ = pts;
    return ext_;
  }

 private:
  bool seeded_ = false;
  uint32_t last_ = 0;
  uint64_t ext_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_PTS_UNWRAP_H_
