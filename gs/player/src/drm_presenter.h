#ifndef MABUR_PLAYER_DRM_PRESENTER_H_
#define MABUR_PLAYER_DRM_PRESENTER_H_

#include <functional>
#include <memory>
#include <string>

#include "colortrans.h"  // CubicLut
#include "osd_raster.h"  // Surface (host-buildable; pulls in no DRM headers)
#include "present_sink.h"
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
// drop_all() exists for the flush-ordering contract: MppBackend::flush()/
// mpi->reset() with DmaFrames still held by the presenter is an unverified
// interaction, so every held frame MUST be released BEFORE backend->flush().
// The release thread calls drop_all() when it parks, and main.cpp parks it
// (park_output) ahead of every flush, decoder re-creation and shutdown.
//
// Threads. present(), poll_events(), drop_all(), commit_osd_if_pending(),
// set_paced() and the rest of the PresentSink surface are called from one
// thread, the release thread, which owns every frame slot. prime() is
// called from the decode-output thread, concurrently with them: it touches
// only the framebuffer cache, which carries its own lock (fb_cache.h). The
// osd_* calls come from the housekeeping thread; the OSD hand-off between
// it and the release thread is atomic (see the OSD section). init(), the
// splash, the colortrans installers and set_flip_sink() run before frames
// flow (or, on a hotplug reacquire, while the release thread is parked).
// The diagnostics getters are read by housekeeping for its stats lines
// while the release thread runs: their counters are atomics and
// present_timing() takes the window under its own lock.
//
// Deliberately free of <xf86drm.h>/<xf86drmMode.h> here -- same rationale
// as mpp_backend.h: this header only needs DmaFrame from video_backend.h.
// All libdrm includes and KMS state live behind the Impl pimpl in
// drm_presenter.cpp.
class DrmPresenter : public PresentSink {
 public:
  using ReleaseFn = std::function<void(const DmaFrame&)>;

  DrmPresenter();
  ~DrmPresenter() override;
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
  bool present(const DmaFrame& frame) override;

  // Imports the frame's dma-buf and adds its framebuffer to the cache, so
  // the present() of this frame finds it there instead of paying the
  // import + AddFB2 (milliseconds on the GS) inside the release lead. Does
  // not take ownership and holds nothing of the frame but the cache entry.
  // Called from the decode-output thread right after it hands the frame
  // over; safe to race present() of the same frame (the cache keeps one
  // framebuffer). Never evicts: what is pinned on screen is release-thread
  // state, so present() keeps the cache within its bound.
  void prime(const DmaFrame& frame);

  // Drains any pending DRM page-flip completion event (nonblocking) and
  // retires the previously-on-screen frame's buffer once its successor is
  // confirmed showing. The release thread calls it on every wake, the
  // event fd's included. It commits no OSD: commit_osd_if_pending() is the
  // caller's separate decision.
  void poll_events() override;

  // The DRM device fd: readable when a page-flip event is waiting for
  // poll_events(). -1 before init().
  int event_fd() const override;

  // Releases every frame currently held (on-screen + queued, if any) back
  // through ReleaseFn right now, and leaves the plane blanked -- nothing
  // shows again until the next present(). See the flush-ordering contract
  // in the class comment.
  void drop_all() override;

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
  //
  // Across threads: the osd_* calls run on the housekeeping thread, the
  // commits on the release thread. The published flag, the back index and
  // the plane id are atomics, and a commit swaps the index BEFORE it clears
  // the flag, so a drawer that sees osd_publish_pending() false also sees
  // the new back index. Draw only while osd_publish_pending() is false: a
  // published back buffer belongs to the release thread until its commit,
  // and after a nonblocking video commit carried it, the buffer the swap
  // made the back stays on screen until that commit's flip completes --
  // osd_publish_pending() stays true until then.
  bool osd_available() const;
  Surface osd_back_surface();      // CPU-writable; pixels == nullptr if none
  int osd_back_index() const;      // 0/1 -- picks the caller's ShadowGrid
  void osd_publish();              // back buffer is ready; shown on next commit
  int osd_front_prime_fd() const;  // committed buffer's dmabuf fd (Phase 2)
  // A published back buffer has not been committed yet, or the commit that
  // carried it has not flipped (release thread or housekeeping).
  bool osd_publish_pending() const override;
  // The standalone OSD commit: the published back buffer shown on its own,
  // when no video commit is coming to carry it. Runs only when the OSD is
  // pending, no flip is in flight, the CRTC is up with no splash on it, and
  // video and the last OSD commit have both been quiet for 100 ms. Blocking
  // (at most one vsync). Release thread.
  void commit_osd_if_pending() override;

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
  // Mailbox engagements (vsync servo observability): count of
  // present() calls that landed while a flip was outstanding, whether the
  // frame just waited in the single-slot mailbox or displaced one already
  // parked there. Superset of busy_replaced(), which counts only the
  // displacement case.
  uint64_t mailbox_engagements() const override;

  // Paced mode (vsync servo active): a parked mailbox frame is a release
  // that missed its latch -- resubmitting it at flip completion can only
  // latch a period late and self-sustains a one-vsync-late chain (hw), so
  // in paced mode it is dropped instead; the regulator's
  // next on-grid release realigns immediately. The release thread
  // refreshes this on every wake from the regulator's servo state; unpaced
  // (false) keeps the original park-and-resubmit behavior.
  void set_paced(bool paced) override;
  uint64_t mailbox_dropped_paced() const;

  // What the last present() did, for frame.log's t_commit/pbits columns:
  // when the atomic commit ioctl returned (mono µs; 0 when the frame was
  // parked in the mailbox or the commit failed) and how it was shaped.
  enum CommitBits : uint8_t {
    kCommitOsd = 0x01,      // the OSD plane rode along (vsync-paced, no ASYNC)
    kCommitAsync = 0x02,    // DRM_MODE_PAGE_FLIP_ASYNC was set
    kCommitParked = 0x04,   // a flip was still in flight: frame went to the mailbox
    kCommitFailed = 0x08,   // the commit ioctl failed (frame released)
    kCommitModeset = 0x10,  // blocking ALLOW_MODESET commit
  };
  // CommitInfo is PresentSink's; its bits are these.
  CommitInfo last_commit() const override;

  // Where present()'s time goes, per stage, over the frames since the
  // last call (then cleared): framebuffer acquire (the cached dma-buf
  // import + AddFB2, fb_cache.h), request build, the atomic commit ioctl.
  // µs, p50 and max of the window.
  struct PresentTiming {
    uint32_t n = 0;
    uint32_t p50[3] = {0, 0, 0};
    uint32_t max[3] = {0, 0, 0};
  };
  PresentTiming present_timing();

  // The frame that just reached the glass: called at the pending ->
  // on_screen promotion inside on_flip() with the frame itself and the
  // kernel-reported (or, absent DRM_CAP_TIMESTAMP_MONOTONIC, mono_us()
  // fallback) flip time. Watchdog force-completes (on_flip(false), no real
  // kernel event) do NOT call this -- those frames simply age out of
  // LatTracker's bounded map, same as any other frame that never flips.
  using FlipSink = std::function<void(const DmaFrame& f, uint64_t flip_mono_us, bool exact)>;
  void set_flip_sink(FlipSink sink);
  // True once init() has confirmed the kernel reports real
  // CLOCK_MONOTONIC page-flip timestamps (DRM_CAP_TIMESTAMP_MONOTONIC).
  // False means on_flip's timestamp is a mono_us()-at-receipt approximation
  // -- LatTracker's dsp segment is then only as exact as poll() latency.
  bool vsync_ts_exact() const;

  // The committed mode's exact vsync period (mode_period_us over its
  // clock/htotal/vtotal — NOT the rounded integer vrefresh). Valid after
  // init() succeeds; main.cpp seeds the FrameRegulator's vblank estimator
  // with it so the servo can lock non-60 Hz panels.
  double mode_period_us() const;

  // Whether the connector this presenter drives lists a mode matching
  // `screen_mode` by pick_mode()'s rule (the wanted size, the exact rate
  // within kModeMatchHz). init() on an unlisted mode falls back to the
  // preferred one; a live display-mode change asks this first, so a mode
  // the display does not offer is refused without tearing the display
  // down. Reads the connector's current mode list (no re-probe).
  bool offers_mode(const std::string& screen_mode) const;

  // --- colortrans: CRTC 3D LUT (docs/colortrans.md) -------------------
  // True when init() found the CRTC's CUBIC_LUT property with
  // CUBIC_LUT_SIZE == 729 (VOP2 on RK3566: 9x9x9, 12-bit entries).
  bool color_lut_available() const;
  // Installs the table. Call ONCE, after init() and before the first
  // splash_show()/present(): the blob is attached to every modeset commit
  // from then on and lives until the presenter is destroyed. The VOP2
  // driver keeps a raw pointer to the last applied blob's data and
  // re-applies it on the next modeset even after the blob is gone, which
  // is why (a) the blob is never destroyed early and (b) main.cpp always
  // installs a table -- identity when colortrans is off. A second call is
  // refused (logged) so that rule cannot be broken by accident.
  bool set_color_lut(const CubicLut& lut);

  // --- colortrans: video-plane CTM (docs/colortrans.md) ----------------
  // True when init() found a "CTM" blob property on the video plane: the
  // Orange Pi image kernel's DE33 VI planes, a 3x4 applied after the
  // plane's YUV->RGB conversion, so the OSD planes never see it.
  bool plane_ctm_available() const;
  // Installs the matrix: 12 S31.32 sign-magnitude words (pack_ctm_3x4).
  // Call ONCE, after init() and before the first splash_show()/present();
  // a second call is refused (logged). The blob rides every modeset and
  // splash-handoff commit and lives until the presenter is destroyed;
  // without one, those commits state CTM = 0, so a matrix a previous run
  // left on the plane never survives a colortrans-off start. If
  // the kernel refuses a commit carrying it (a matrix outside the CSC's
  // register range) and the same commit without it succeeds, present()
  // drops it for this presenter's lifetime: the picture goes flat, never
  // dark.
  bool set_plane_ctm(const uint64_t ctm[12]);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DRM_PRESENTER_H_
