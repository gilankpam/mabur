#ifndef MABUR_PLAYER_NULL_BACKEND_H_
#define MABUR_PLAYER_NULL_BACKEND_H_

#include "video_backend.h"

namespace maburplay {

// Backend that decodes nothing and emits no frames; counts submits/flushes
// only. Always available (make_backend("null")) — the host e2e's backend,
// and a decode-less diagnostic mode on-device.
class NullBackend : public VideoBackend {
 public:
  bool init(const BackendCfg& cfg, FrameSink sink) override;
  void submit_au(const uint8_t* au, size_t n, uint32_t pts_us) override;
  void flush() override;
  void release_frame(const DmaFrame& frame) override;

  uint64_t submits() const { return submits_; }
  uint64_t flushes() const { return flushes_; }

 private:
  FrameSink sink_;
  uint64_t submits_ = 0;
  uint64_t flushes_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_NULL_BACKEND_H_
