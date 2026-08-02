#ifndef MABUR_PLAYER_VIDEO_BACKEND_H_
#define MABUR_PLAYER_VIDEO_BACKEND_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// The extensibility seam (spec: docs/superpowers/specs — PR B maburplay
// design). This block is FROZEN verbatim: hardware backends (MppBackend,
// Task 7/8) and NullBackend (this task) both implement VideoBackend as-is;
// do not change the shape here without updating the design doc first.
namespace maburplay {

struct BackendCfg { int width = 1920, height = 1080, fps = 60; };
struct DmaFrame {                 // decoded frame, zero-copy handle
  int dmabuf_fd = -1;             // owned by backend until release_frame
  uint32_t fourcc = 0;            // DRM fourcc, e.g. DRM_FORMAT_NV12
  uint64_t modifier = 0;
  int width = 0, height = 0;
  int stride = 0, vstride = 0;    // hor/ver stride (RK: mpp_frame strides)
  uint32_t pts_us = 0;
  void* opaque = nullptr;         // backend's frame token
};
class VideoBackend {
 public:
  using FrameSink = std::function<void(const DmaFrame&)>;
  virtual ~VideoBackend() = default;
  virtual bool init(const BackendCfg&, FrameSink) = 0;
  virtual void submit_au(const uint8_t* au, size_t n, uint32_t pts_us) = 0;
  virtual void flush() = 0;                       // discont -> reset decoder
  virtual void release_frame(const DmaFrame&) = 0;
  virtual void poll() {}                          // drive async decoders
};
std::unique_ptr<VideoBackend> make_backend(const std::string& name);  // factory
}  // namespace maburplay

#endif  // MABUR_PLAYER_VIDEO_BACKEND_H_
