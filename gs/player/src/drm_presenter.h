#ifndef MABUR_PLAYER_DRM_PRESENTER_H_
#define MABUR_PLAYER_DRM_PRESENTER_H_

#include <functional>
#include <memory>
#include <string>

#include "osd_raster.h"  // Surface (host-buildable; pulls in no DRM headers)
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
  //
  // `want_osd` is the caller's up-front decision (config osd.enable AND a
  // font that actually loaded) about whether an OSD is wanted at all. When
  // false, init() never looks for an OSD-capable plane: it skips straight
  // to the pre-OSD topology (video + black backdrop on the primary), so
  // "turn the OSD off" is a real escape hatch back to the old plane layout
  // -- not a primary plane permanently claimed by two unused ARGB buffers.
  // The caller must decide this BEFORE calling init(); DrmPresenter cannot
  // load the font itself (main.cpp owns that) and must not guess.
  // `log_failures` false silences the per-failure diagnostics on this path.
  // The hotplug retry in main.cpp calls init() once a second while there is
  // no display; at that rate the "no connected connector found" line is
  // ~86,000 lines a day into /tmp/maburplay.log, which is tmpfs on the GS.
  // The caller reports the episode instead: one line when retrying starts,
  // one when a display is acquired.
  bool init(const std::string& screen_mode, bool want_osd, ReleaseFn release,
            bool log_failures = true);

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

  // Startup splash. splash_surface() hands back the CPU mapping of a
  // dedicated full-mode XRGB8888 dumb buffer that init() allocated on the
  // primary plane; a null Surface means no splash is possible on this
  // topology (no usable primary plane, or the allocation failed) and the
  // caller simply skips it. splash_show() then modesets IMMEDIATELY with
  // that buffer, so the sink locks a mode before any video exists -- the
  // whole point: without it nothing drives the CRTC until the first decoded
  // frame.
  //
  // There is deliberately no teardown call. The first present() after a
  // splash re-points the primary at its normal FB in the same commit that
  // shows the frame, then frees the splash buffer. A second modeset is NOT
  // used for the handoff: it risks the sink re-locking (a second of black)
  // at exactly the moment video starts.
  Surface splash_surface();
  bool splash_show();

  // --- MSP OSD overlay -------------------------------------------------
  // Allocated during init() -- only when the caller passed want_osd=true --
  // on a plane PROVEN able to scan out linear ARGB8888 (see
  // plane_takes_linear_argb() in the .cpp): normally the CRTC's primary
  // plane itself, occasionally a distinct third plane when the primary
  // cannot take it. All calls are safe no-ops otherwise (osd_available()
  // false, osd_back_surface() a null Surface, osd_publish() a no-op,
  // osd_front_prime_fd() -1) -- an OSD that was not wanted or could not be
  // set up never affects video.
  //
  // Usage: draw into osd_back_surface() (with the caller's ShadowGrid for
  // osd_back_index(); the presenter deliberately does not own those), then
  // osd_publish(). The buffer is attached to the next commit and the pair
  // swaps once that commit is issued. Any commit carrying the OSD plane is
  // vsync-paced -- an async page flip may only change FB_ID on one plane.
  bool osd_available() const;
  Surface osd_back_surface();      // CPU-writable; pixels == nullptr if none
  int osd_back_index() const;      // 0/1 -- picks the caller's ShadowGrid
  void osd_publish();              // back buffer is ready; shown on next commit
  int osd_front_prime_fd() const;  // committed buffer's dmabuf fd (Phase 2)

  // OSD-attributed atomic-commit failures. Deliberately NOT folded into
  // commit_errors(), which is video health: an OSD the driver refuses must
  // not make video look broken. The OSD switches itself off after a few
  // consecutive failures, so this counter is bounded and small.
  uint64_t osd_commit_errors() const;

  // Diagnostics for --fps-log / gate reporting.
  uint64_t commit_errors() const;
  uint64_t flips() const;           // completed page flips (frames actually shown)
  uint64_t busy_replaced() const;   // mailbox frames replaced before display
  bool async_flip_active() const;   // true once ASYNC has been confirmed working
  bool async_probed() const;        // true once the one-shot ASYNC probe has run
  // Mailbox engagements (vsync servo observability, Task 6): count of
  // present() calls that landed while a flip was outstanding, whether the
  // frame just waited in the single-slot mailbox or displaced one already
  // parked there. Superset of busy_replaced(), which counts only the
  // displacement case.
  uint64_t mailbox_engagements() const;

  // Vsync timestamp sink for LatTracker (Task 11): called at the pending ->
  // on_screen promotion inside on_flip() with the frame's pts and the
  // kernel-reported (or, absent DRM_CAP_TIMESTAMP_MONOTONIC, mono_us()
  // fallback) flip time. Watchdog force-completes (on_flip(false), no real
  // kernel event) do NOT call this -- those frames simply age out of
  // LatTracker's bounded map, same as any other frame that never flips.
  using FlipSink = std::function<void(uint32_t pts_us, uint64_t flip_mono_us, bool exact)>;
  void set_flip_sink(FlipSink sink);
  // True once init() has confirmed the kernel reports real
  // CLOCK_MONOTONIC page-flip timestamps (DRM_CAP_TIMESTAMP_MONOTONIC).
  // False means on_flip's timestamp is a mono_us()-at-receipt approximation
  // -- LatTracker's dsp segment is then only as exact as poll() latency.
  bool vsync_ts_exact() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DRM_PRESENTER_H_
