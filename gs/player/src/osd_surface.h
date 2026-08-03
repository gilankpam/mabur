#ifndef MABUR_PLAYER_OSD_SURFACE_H_
#define MABUR_PLAYER_OSD_SURFACE_H_

#include <cstdint>
#include <string>

#include "osd_raster.h"  // Surface

namespace maburplay {

// Two DRM dumb ARGB8888 buffers for the OSD plane: one being scanned out,
// one being drawn into. Each carries a KMS FB id and an exported dmabuf fd
// (the latter is what Phase 2's RGA composite reads). Cross build only --
// this translation unit is compiled behind MABUR_PLAYER_HW and pulls in
// libdrm; the header itself stays SDK-free, same rule as drm_presenter.h.
class OsdSurface {
 public:
  OsdSurface() = default;
  ~OsdSurface();
  OsdSurface(const OsdSurface&) = delete;
  OsdSurface& operator=(const OsdSurface&) = delete;

  // Allocates both buffers (full-screen, allocated once -- canvas/layout
  // changes never reallocate). All-or-nothing: on failure *err carries the
  // reason and ok() stays false; the destructor still cleans up whatever
  // was allocated before the failure.
  bool init(int drm_fd, int width, int height, std::string* err);
  bool ok() const { return buf_[0].ptr != nullptr && buf_[1].ptr != nullptr; }

  // Releases everything and resets to the default-constructed state.
  // Idempotent. MUST be called before the owner closes the DRM fd -- a
  // member destructor runs after its owner's destructor body, so
  // DrmPresenter::~Impl calls this at the top rather than letting ~Buf-era
  // cleanup land on an already-closed descriptor. Also used to reclaim the
  // buffers when the presenter decides at init time that the OSD cannot be
  // used after all.
  void destroy();

  Surface cpu(int idx) const;
  uint32_t fb_id(int idx) const { return buf_[idx & 1].fb_id; }
  int prime_fd(int idx) const { return buf_[idx & 1].prime_fd; }
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  struct Buf {
    uint32_t handle = 0;
    uint32_t fb_id = 0;
    uint32_t pitch = 0;  // bytes
    uint64_t size = 0;
    void* ptr = nullptr;
    int prime_fd = -1;
  };
  bool alloc_(Buf* b, std::string* err);

  int fd_ = -1;
  int width_ = 0, height_ = 0;
  Buf buf_[2];
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_SURFACE_H_
