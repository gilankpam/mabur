#ifndef MABUR_PLAYER_DRM_PRESENTER_H_
#define MABUR_PLAYER_DRM_PRESENTER_H_

#include <string>

#include "video_backend.h"

namespace maburplay {

// KMS/DRM direct-scanout presenter for decoded DmaFrames. Cross build only
// (MABUR_PLAYER_HW); driven by MppBackend once Task 8/9 wire it in -- not a
// VideoBackend itself, just the display half of the hardware path.
//
// STUB as of Task 7: init() always fails ("not implemented (Task 9)"), the
// rest are no-ops. Task 9 fills these bodies against libdrm KMS
// (drmModeGetResources / drmModeSetCrtc / dumb or dmabuf-imported
// framebuffers via the DmaFrame's dmabuf_fd).
//
// Deliberately free of <xf86drm.h>/<xf86drmMode.h> here -- same rationale
// as mpp_backend.h: this header only needs DmaFrame from video_backend.h.
// Task 9 adds the libdrm includes to drm_presenter.cpp, not here.
class DrmPresenter {
 public:
  bool init(const std::string& screen_mode);
  bool present(const DmaFrame& frame);
  void poll_events();
  ~DrmPresenter();
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DRM_PRESENTER_H_
