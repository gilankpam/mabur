#ifndef MABUR_PLAYER_BURN_RECORDER_H_
#define MABUR_PLAYER_BURN_RECORDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "osd_palette.h"    // OsdPalette
#include "osd_raster.h"     // Surface, DirtyRect
#include "video_backend.h"  // DmaFrame, VideoBackend

namespace maburplay {

// Encoder-side recording settings. width/height must equal the decoded
// picture size: MppEncoder latches geometry on its first frame and hard-
// rejects anything that disagrees (it never reconfigures), so a mismatch
// here means a recording that never produces a single sample.
struct BurnCfg {
  int width = 1920;
  int height = 1080;
  int fps_cap = 30;        // encode rate ceiling, independent of display rate
  int bitrate_kbps = 12000;
  int fragment_ms = 1000;  // DvrMux fragment cut, same units as the raw path
};

// dvr.mode "burned": re-encodes decoded frames with the MSP OSD composited in
// by the hardware encoder (vepu540) and muxes the result to fMP4. Cross build
// only (MABUR_PLAYER_HW) -- it reaches into DmaFrame::opaque as an MppFrame,
// which only MppBackend produces.
//
// ---------------------------------------------------------------------------
// FRAME OWNERSHIP: no copy anywhere. There are two consumers of every decoded
// frame -- the presenter (scanout) and this recorder (encode) -- and they are
// decoupled by MPP's own buffer refcount, NOT by copying and NOT by changing
// who owns the DmaFrame:
//
//   frame_sink(f):  burn->submit(f)       // mpp_buffer_inc_ref() on the
//                                         // frame's buffer, O(1), no copy
//                   presenter->present(f) // presenter owns the DmaFrame,
//                                         // exactly as before
//   presenter later: release_frame(f) -> mpp_frame_deinit -> drops ITS ref
//   recorder thread: encode from that buffer, then mpp_buffer_put()
//
// The decoder cannot recycle the buffer while either holder has a reference,
// so the presenter's contract (it owns the handle, releases on flip, obeys the
// flush ordering) is untouched. Cost: this class transiently holds at most two
// buffers (one in the mailbox, one being encoded) out of the decoder's
// 24-buffer external group.
//
// EVERY reference taken in submit() is released on exactly one path:
// encoded-then-put, displaced-in-the-mailbox-then-put, drop_pending(), or the
// drain in stop()/~BurnRecorder(). Losing one leaks a pool buffer; ~24 leaks
// stall decode outright. frames_dropped() is the observable that proves
// displacement is handled: it climbs under overload while the decoder keeps
// running at full rate.
//
// ---------------------------------------------------------------------------
// THREADING
//   start() / stop() / submit() / set_osd() / request_idr() / drop_pending()
//     are all called from the main loop. None of them blocks on the encoder.
//   The recorder thread owns the MppEncoder and the DvrMux outright: every
//     MppEncoder call except init()/set_palette() (which run inside start(),
//     before the thread exists) happens there, which is what MppEncoder's
//     "set_osd() and encode() from ONE thread" contract requires.
//   The handoff is a single-slot mailbox, latest-wins: if the encoder falls
//     behind, the older frame is dropped and its reference released. Never a
//     growing queue -- a queue under sustained overload turns into unbounded
//     buffer retention, i.e. decoder starvation, which is the one thing
//     burned mode must never do to the video path.
class BurnRecorder {
 public:
  BurnRecorder();
  ~BurnRecorder();
  BurnRecorder(const BurnRecorder&) = delete;
  BurnRecorder& operator=(const BurnRecorder&) = delete;

  // Stages the OSD palette. MUST be called before start(), which uploads it
  // to the encoder before the recorder thread exists (MPP wants the palette
  // installed before the first frame, and this keeps every encoder call on
  // one thread). Never calling it is a legal configuration -- osd.enable
  // false with dvr.mode burned records a plain transcode, and the encoder
  // then runs with no OSD region at all; set_osd() becomes a cheap no-op.
  void set_palette(const OsdPalette& pal);

  // Creates the encoder, requests an opening IDR and starts the thread.
  // `backend` must be the live MppBackend: burned mode is refused (logged,
  // false) for any other backend, because DmaFrame::opaque is only an
  // MppFrame under that one. The pointer is checked and NOT retained -- the
  // decode watchdog may destroy and recreate the backend mid-run.
  //
  // `path` is the fMP4 file. It is NOT created here: the mux opens on the
  // first encoded keyframe, because MppEncoder::header() (the source of the
  // hvcC sample entry) is empty until the first successful encode() latches
  // the geometry. An encoder that never encodes anything therefore leaves no
  // zero-frame file behind.
  bool start(const BurnCfg& cfg, const std::string& path, const VideoBackend* backend);

  // Main loop, once per decoded frame, BEFORE presenter->present(f). Applies
  // the fps cap first, so rejected frames cost one clock read and nothing
  // else; admitted frames take an mpp_buffer_inc_ref() and post to the
  // mailbox. O(1), never blocks, never touches pixels.
  void submit(const DmaFrame& f);

  // Main loop, whenever a fresh OSD screen has been rasterized. Quantizes
  // `s` against the staged palette on the CALLER's thread and publishes the
  // index map to the recorder thread, which installs it into the encoder's
  // double-buffered OSD slot before its next frame. No-op without a palette.
  //
  // THIS RUNS ON THE 2 ms MAIN LOOP, so it keeps a PERSISTENT index map and
  // re-quantizes only what changed -- the same bargain OsdRaster strikes
  // with its ShadowGrid, for the same reason. Pass the rects
  // OsdRaster::diff() produced for THIS map's own ShadowGrid; an empty list
  // is a no-op. `rects == nullptr` (the default) re-quantizes the whole
  // surface, which is what a blank or a first screen wants.
  //
  // Cost, measured at 1080p with the shipped font (tools/bench/
  // quantize_bench.cpp, x86 host; the RK3566 A55 runs ~7.4x slower):
  //   incremental, 5 cells   0.010 ms   <- the steady-state path, ~4 Hz
  //   full, typical screen   0.66 ms    <- first screen / layout change
  //   full, blank surface    0.42 ms    <- the stale-OSD blank
  // The pre-fix implementation was 5.5 ms per call, unconditionally.
  void set_osd(const Surface& s, const DirtyRect* rects = nullptr, size_t n_rects = 0);

  // Makes the recording's next encoded frame an IDR. Sets a flag consumed on
  // the recorder thread (MppEncoder is single-threaded), so it is safe from
  // the main loop. Use it wherever the decode path is reset -- flush_before,
  // the decode watchdog -- so the recording reseals after a discontinuity.
  void request_idr();

  // Drops the mailbox entry (releasing its buffer reference) without
  // stopping. Call it wherever the presenter's held frames are dropped --
  // before backend->flush() and before a watchdog decoder recreation -- so
  // the recorder is not holding decoder buffers across a reset either. The
  // frame the encoder is working on right now cannot be recalled, so this
  // narrows the window rather than closing it.
  //
  // Holding a buffer across a group teardown is BENIGN, so this is hygiene
  // rather than a guard: mpp_buffer_group_put()/clear() on a group with a
  // buffer still referenced does not free it -- service_put_group()
  // (mpp_buffer_impl.c) logs, moves the group to the orphan list, and the
  // last mpp_buffer_put() reaps it. Cost is two mpp_err lines and a slightly
  // late DMA free. That matters because one such teardown is NOT reachable
  // from here at all: MppBackend acks a mid-stream resolution change by
  // calling mpp_buffer_group_clear() inside its own drain loop
  // (mpp_backend.cpp), an event the main loop never sees and cannot precede
  // with a drop_pending().
  void drop_pending();

  // Joins the thread, releases any buffer still held, and closes the mux.
  // Idempotent; also run by the destructor.
  void stop();

  bool running() const;  // started, and not disabled by a fatal encode path

  // Counters. frames_in ~= frames_encoded + frames_dropped:
  //   frames_in      frames admitted past the fps cap (what the recorder saw)
  //   frames_encoded encode() calls that produced a packet
  //   frames_dropped admitted frames displaced in the mailbox, or dropped by
  //                  drop_pending()/stop() -- i.e. the OVERLOAD signal; fps-cap
  //                  rejections are deliberately NOT counted here
  //   encode_errors  frames the encoder refused
  uint64_t frames_in() const;
  uint64_t frames_encoded() const;
  uint64_t frames_dropped() const;
  uint64_t encode_errors() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_BURN_RECORDER_H_
