#ifndef MABUR_PLAYER_DRM_PRESENTER_H_
#define MABUR_PLAYER_DRM_PRESENTER_H_

#include <functional>
#include <memory>
#include <string>

#include "video_backend.h"

namespace maburplay {

// KMS/DRM direct-scanout presenter for decoded DmaFrames. Cross build only
// (MABUR_PLAYER_HW); driven by MppBackend via main.cpp -- not a
// VideoBackend itself, just the display half of the hardware path.
//
// Ownership contract: once present() is called, the presenter owns the
// DmaFrame (dmabuf fd is imported into a GEM handle immediately; the
// caller's fd is never touched again) until it hands the frame back via
// the ReleaseFn supplied to init(). The release happens either (a) when a
// newer frame's page-flip event confirms this one has left the screen, or
// (b) synchronously and immediately if present() has to drop the frame
// (busy -- a flip is already in flight -- or a hard commit error), or (c)
// on drop_all(), which flushes every held frame (on-screen + queued) back
// through ReleaseFn right away and leaves the presenter showing nothing
// until the next present().
//
// drop_all() exists for the flush-ordering contract carried from Task 8's
// review: MppBackend::flush()/mpi->reset() with DmaFrames still held by
// the presenter is an unverified interaction, so the call site (main.cpp)
// MUST invoke presenter->drop_all() -- returning every held frame to the
// backend -- BEFORE calling backend->flush(). See main.cpp's flush_before
// handling for the exact ordering.
//
// Deliberately free of <xf86drm.h>/<xf86drmMode.h> here -- same rationale
// as mpp_backend.h: this header only needs DmaFrame from video_backend.h.
// All libdrm includes and KMS state live behind the Impl pimpl in
// drm_presenter.cpp.
class DrmPresenter {
 public:
  using ReleaseFn = std::function<void(const DmaFrame&)>;

  DrmPresenter();
  ~DrmPresenter();
  DrmPresenter(const DrmPresenter&) = delete;
  DrmPresenter& operator=(const DrmPresenter&) = delete;

  // Opens /dev/dri/card0, picks a connected connector + mode (parsed from
  // screen_mode, "WIDTHxHEIGHT@FPS") + an NV12-capable plane, and does the
  // one-time atomic setup. `release` is called (possibly re-entrantly from
  // inside present()/poll_events()/drop_all()) whenever the presenter is
  // done with a frame it was handed.
  bool init(const std::string& screen_mode, ReleaseFn release);

  // Takes ownership of `frame` unconditionally (see class comment) and
  // returns whether it was actually queued for scanout (false = dropped,
  // already released back via ReleaseFn -- either backpressure or a
  // commit error; not fatal, caller just keeps going).
  bool present(const DmaFrame& frame);

  // Drains any pending DRM page-flip completion event (nonblocking) and
  // retires the previously-on-screen frame's buffer once its successor is
  // confirmed showing. Call once per main-loop iteration, alongside the
  // ring pump.
  void poll_events();

  // Releases every frame currently held (on-screen + queued, if any) back
  // through ReleaseFn right now, and leaves the plane blanked -- nothing
  // shows again until the next present(). See the flush-ordering contract
  // in the class comment.
  void drop_all();

  // Diagnostics for --fps-log / gate reporting.
  uint64_t commit_errors() const;
  uint64_t flips() const;           // completed page flips (frames actually shown)
  uint64_t busy_replaced() const;   // mailbox frames replaced before display
  bool async_flip_active() const;   // true once ASYNC has been confirmed working
  bool async_probed() const;        // true once the one-shot ASYNC probe has run

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DRM_PRESENTER_H_
