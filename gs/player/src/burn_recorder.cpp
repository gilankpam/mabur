#include "burn_recorder.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <rockchip/rk_mpi.h>

#include "dvr_mux.h"
#include "hevc_params.h"
#include "mpp_backend.h"
#include "mpp_encoder.h"

// The burned-DVR driver: an fps-capped encoder thread fed from the main loop
// by MPP buffer reference, writing fMP4 through the same DvrMux the raw path
// uses. See burn_recorder.h for the ownership and threading contracts; the
// notes below are the implementation-side consequences of them.
//
// Three things this file exists to get right:
//
//  1. EVERY inc_ref has exactly one matching put. The paths are enumerated in
//     the header and each one is marked "REF-" in the code below. The one
//     that is easy to miss -- and that costs a decoder buffer per occurrence,
//     stalling decode after ~24 -- is the mailbox DISPLACEMENT: overwriting a
//     pending entry is the normal steady-state behaviour under overload, and
//     the displaced entry still holds a reference.
//
//  2. MppEncoder is single-threaded by contract (set_osd() and encode() from
//     ONE thread, see mpp_encoder.h). init()/set_palette() run on the caller's
//     thread inside start(), before the recorder thread is launched; every
//     later encoder call -- request_idr(), set_osd(), encode() -- happens on
//     the recorder thread. request_idr() from the main loop is therefore a
//     flag, not a passthrough.
//
//  3. The mux CANNOT be opened at start(): MppEncoder::header() is empty until
//     the first successful encode() latches the geometry, and the first IDR
//     carries NO in-band VPS/SPS/PPS (MPP's GET_HDR_SYNC marks the header as
//     already added, and mpp_enc_add_sw_header then skips it for the whole
//     first task). So the hvcC sample entry is built from header() -- the
//     ENCODER's parameter sets, not the drone's -- and the file is created on
//     the first encoded keyframe. A recording that never encodes a frame
//     leaves no file at all, which is the honest outcome.
namespace maburplay {

namespace {

// Cap tolerance: with a 59.94 fps source and a 30 fps cap, the ideal spacing
// (33'333 us) lands a hair above two source intervals (33'367 us) only when
// arrivals are perfectly regular. Any jitter makes a strict ">= interval"
// test reject the frame that should have been taken and wait a further 16.7
// ms, collapsing 30 fps to 20. A tolerance of one third of a 60 fps frame
// absorbs that without ever admitting two source frames in a row.
constexpr int64_t kCapSlackUs = 5000;

// Consecutive refusals, with nothing ever encoded, after which the recorder
// gives up for good. MppEncoder's documented permanent-refusal signature is
// errors() climbing while frames() stays 0 (a picture size disagreeing with
// EncCfg, or strides smaller than the picture); retrying that at 30 fps
// forever would only flood the log. ~2 s at the default cap.
constexpr uint64_t kMaxInitialFailures = 60;

// The same backstop for a recording that HAD been working and then stopped --
// the realistic trigger being the VPU session wedging mid-flight, the failure
// class the decode watchdog exists for. Without it, a persistent failure in
// MppEncoder's MPP-error branches (mpp_frame_init / packet init /
// encode_put_frame / encode_get_packet, none of them rate-limited) is fed at
// fps_cap forever: ~2 KB/s into /tmp/maburplay.log, which is an unrotated `>`
// redirect on tmpfs, plus a decoder-buffer round-trip per frame for nothing.
// BurnRecorder is the right place to bound that because it owns the feed rate.
// Looser than kMaxInitialFailures on purpose (~20 s at the default cap): an
// established recording is worth some patience, and whatever was recorded
// before the stall is still closed cleanly by stop().
constexpr uint64_t kMaxRunningFailures = 600;

}  // namespace

struct BurnRecorder::Impl {
  // Dirty rects the recorder will copy one at a time; beyond this, taking
  // the whole map is cheaper than the per-rect row loop.
  static constexpr size_t kMaxDirtyRects = 128;

  BurnCfg cfg;
  std::string path;

  std::unique_ptr<MppEncoder> enc;
  DvrMux mux;
  bool mux_open = false;
  bool mux_failed = false;  // open refused once; do not retry per frame

  OsdPalette palette;
  bool have_palette = false;   // set_palette() was called
  bool palette_live = false;   // ...and the encoder accepted it

  // --- mailbox (main loop -> recorder thread) ------------------------------
  struct Mail {
    void* buf = nullptr;  // MppBuffer; non-null means "a reference is held"
    uint32_t pts_us = 0;
    int w = 0, h = 0, stride = 0, vstride = 0;
  };
  std::mutex mu;
  std::condition_variable cv;
  Mail box;
  bool stopping = false;

  // OSD handoff, under the same mutex.
  //
  //   osd_map    the AUTHORITATIVE index map. Persistent across calls and
  //              updated in place by the main loop, which is what lets a
  //              steady-state OSD update cost the cells that changed
  //              instead of 2.07 M pixels (see set_osd()).
  //   osd_dirty  the parts of osd_map the recorder thread has not copied
  //              out yet -- accumulated, because the main loop publishes at
  //              the MSP rate and the recorder consumes at the frame rate,
  //              and neither waits for the other.
  //   osd_work   the recorder's own copy, brought up to date under the lock
  //              and then handed to the encoder outside it.
  //
  // No swapping: a swapped-in buffer is a stale GENERATION, and an
  // incremental update on top of one is silently wrong.
  OsdIndexMap osd_map, osd_work;
  std::vector<DirtyRect> osd_dirty;
  bool osd_full_pending = false;   // osd_dirty is meaningless; copy it whole
  bool osd_pending_valid = false;  // osd_map moved since the last take
  QuantizeCache qcache;            // main loop only; palette-lifetime memo

  std::thread th;
  bool started = false;
  std::atomic<bool> dead{false};        // fatal: stop feeding the encoder
  std::atomic<bool> idr_pending{false};

  // fps cap state, main loop only.
  std::chrono::steady_clock::time_point last_admit;
  bool have_last_admit = false;
  int64_t cap_interval_us = 0;

  std::atomic<uint64_t> frames_in{0};
  std::atomic<uint64_t> frames_encoded{0};
  std::atomic<uint64_t> frames_dropped{0};
  std::atomic<uint64_t> encode_errors{0};
  std::atomic<uint64_t> osd_rejects{0};
  uint64_t consecutive_fail = 0;  // recorder thread only

  // One coded picture out of the encoder, on the recorder thread, from inside
  // encode(). `p` is only valid for the duration of the call, so DvrMux (which
  // copies into its pending fragment) is written here and now.
  void on_nal(const uint8_t* p, size_t n, uint64_t pts_us, bool keyframe) {
    if (!p || n == 0) return;
    if (!mux_open) {
      if (mux_failed) return;
      // Wait for a keyframe: an fMP4 whose first sample is a P slice cannot
      // be decoded from the start.
      if (!keyframe) return;
      // hvcC from the ENCODER's own VPS/SPS/PPS. header() is populated by the
      // geometry latch inside the first encode(), so by the time any packet
      // reaches this sink it is non-empty. The AU itself is fed too, for the
      // (currently impossible, but free) case of an encoder that does put its
      // parameter sets in band on the first IDR.
      HevcParams params;
      const std::vector<uint8_t>& hdr = enc->header();
      if (!hdr.empty()) params.feed(hdr.data(), hdr.size());
      if (!params.complete()) params.feed(p, n);
      if (!params.complete()) {
        mux_failed = true;
        std::fprintf(stderr,
                     "BurnRecorder: no VPS/SPS/PPS from the encoder (%zu header bytes); "
                     "recording disabled\n",
                     hdr.size());
        dead.store(true);
        return;
      }
      // Track header from the LATCHED picture size -- the decoded frame's,
      // which is the only thing the samples actually are. cfg.width/height
      // is a fallback that cannot be reached today (this sink only runs from
      // inside encode(), after the latch).
      int tw = 0, th = 0;
      enc->picture_size(&tw, &th);
      if (tw <= 0 || th <= 0) {
        tw = cfg.width;
        th = cfg.height;
      }
      if (!mux.open(path, params.hvcc(), tw, th, cfg.fragment_ms)) {
        mux_failed = true;
        std::fprintf(stderr, "BurnRecorder: cannot open %s; recording disabled\n", path.c_str());
        dead.store(true);
        return;
      }
      mux_open = true;
      std::fprintf(stderr, "BurnRecorder: recording %s (%dx%d, hvcC from %zu header bytes)\n",
                   path.c_str(), tw, th, hdr.size());
    }
    mux.write_sample(p, n, static_cast<uint32_t>(pts_us), keyframe);
  }

  void run() {
    for (;;) {
      Mail m;
      bool have_osd = false;
      {
        std::unique_lock<std::mutex> lk(mu);
        cv.wait(lk, [this] { return stopping || box.buf != nullptr; });
        // stop() drains whatever is left in the box after the join, so this
        // thread never has to decide who releases a half-handled entry.
        if (stopping) break;
        m = box;
        box.buf = nullptr;
        if (osd_pending_valid) {
          take_osd_locked();
          have_osd = true;
        }
      }

      if (dead.load()) {
        // REF-DEAD: a frame admitted just before the recorder gave up.
        mpp_buffer_put(static_cast<MppBuffer>(m.buf));
        frames_dropped.fetch_add(1);
        continue;
      }

      if (idr_pending.exchange(false)) enc->request_idr();
      // A refused map means the encoder's region and the OSD surface
      // disagree -- a recording that is a silent plain transcode. Counted so
      // --fps-log can say so; the encoder's own log for it is once-only.
      if (have_osd && !enc->set_osd(osd_work)) osd_rejects.fetch_add(1);

      const bool ok =
          enc->encode(m.buf, m.w, m.h, m.stride, m.vstride, static_cast<uint64_t>(m.pts_us));
      // REF-ENCODE: released the moment the encoder is done with it, whether
      // or not it produced a packet. encode() borrows, it never owns.
      mpp_buffer_put(static_cast<MppBuffer>(m.buf));

      if (ok) {
        frames_encoded.fetch_add(1);
        consecutive_fail = 0;
        continue;
      }
      encode_errors.fetch_add(1);
      ++consecutive_fail;
      // Two thresholds, both "stop feeding the encoder" rather than "retry
      // forever". The tight one is MppEncoder's documented permanent-refusal
      // signature (errors() climbing with frames() stuck at 0), which will
      // never fix itself. The loose one bounds everything else, including a
      // recording that worked and then stopped working.
      const bool never_encoded = frames_encoded.load() == 0;
      if ((never_encoded && consecutive_fail >= kMaxInitialFailures) ||
          consecutive_fail >= kMaxRunningFailures) {
        // MppEncoder logged the reason once itself; say why the recording is
        // over, and stop burning a decoder-buffer round-trip per frame.
        std::fprintf(stderr,
                     "BurnRecorder: encoder refused %llu consecutive frames (%llu encoded so "
                     "far); recording disabled%s\n",
                     static_cast<unsigned long long>(consecutive_fail),
                     static_cast<unsigned long long>(frames_encoded.load()),
                     never_encoded ? " (no file written)" : " (file closed at stop())");
        dead.store(true);
      }
    }
  }

  // Recorder thread, mu HELD: brings osd_work up to date with osd_map,
  // copying only the rows the main loop marked dirty. The full copy is the
  // fallback for a resize, a blank, or a rect list that outgrew its cap --
  // never the steady state.
  void take_osd_locked() {
    if (osd_full_pending || osd_work.mb_w != osd_map.mb_w || osd_work.mb_h != osd_map.mb_h ||
        osd_work.px.size() != osd_map.px.size()) {
      osd_work = osd_map;
    } else {
      const size_t stride = (size_t)osd_map.stride();
      for (const DirtyRect& r : osd_dirty) {
        for (int y = r.y; y < r.y + r.h; ++y)
          std::memcpy(osd_work.px.data() + (size_t)y * stride + r.x,
                      osd_map.px.data() + (size_t)y * stride + r.x, (size_t)r.w);
      }
    }
    osd_dirty.clear();
    osd_full_pending = false;
    osd_pending_valid = false;
  }

  // Empties the mailbox, releasing the reference it holds. Callable from
  // either thread (main loop via drop_pending()/stop(); never concurrently
  // with the recorder thread's own take, which is what the mutex is for).
  void drain_box() {
    void* buf = nullptr;
    {
      std::lock_guard<std::mutex> lk(mu);
      buf = box.buf;
      box.buf = nullptr;
    }
    if (buf) {
      // REF-DRAIN: displaced by a drop_pending()/stop() rather than by a
      // newer frame; the accounting is the same.
      mpp_buffer_put(static_cast<MppBuffer>(buf));
      frames_dropped.fetch_add(1);
    }
  }
};

BurnRecorder::BurnRecorder() : impl_(new Impl) {}
BurnRecorder::~BurnRecorder() { stop(); }

void BurnRecorder::set_palette(const OsdPalette& pal) {
  if (impl_->started) {
    std::fprintf(stderr, "BurnRecorder: set_palette() after start(), ignored\n");
    return;
  }
  impl_->palette = pal;
  impl_->have_palette = true;
}

bool BurnRecorder::start(const BurnCfg& cfg, const std::string& path,
                         const VideoBackend* backend) {
  Impl& im = *impl_;
  if (im.started) {
    std::fprintf(stderr, "BurnRecorder: already started\n");
    return false;
  }
  // Backend check ONCE, here, rather than a blind per-frame cast: only
  // MppBackend puts an MppFrame in DmaFrame::opaque, and a burned recording
  // fed by anything else would dereference garbage. The pointer is not kept
  // -- the decode watchdog can destroy and recreate the backend, and the
  // replacement is built from the same config, so it is an MppBackend too.
  if (dynamic_cast<const MppBackend*>(backend) == nullptr) {
    std::fprintf(stderr,
                 "BurnRecorder: dvr.mode \"burned\" needs the mpp backend "
                 "(decoded frames carry no MppFrame otherwise); recording disabled\n");
    return false;
  }
  if (cfg.width <= 0 || cfg.height <= 0 || cfg.fps_cap <= 0 || cfg.bitrate_kbps <= 0) {
    std::fprintf(stderr, "BurnRecorder: bad BurnCfg %dx%d cap %d fps %d kbps\n", cfg.width,
                 cfg.height, cfg.fps_cap, cfg.bitrate_kbps);
    return false;
  }

  im.cfg = cfg;
  im.path = path;
  im.cap_interval_us = 1000000 / cfg.fps_cap;
  im.have_last_admit = false;
  im.mux_open = false;
  im.mux_failed = false;
  im.dead.store(false);
  im.stopping = false;

  EncCfg ec;
  ec.fps = cfg.fps_cap;  // the encoder's rc: rate IS the capped rate
  ec.bitrate_kbps = cfg.bitrate_kbps;
  // The OSD region follows the OSD SURFACE, never the picture and never the
  // configured screen mode. The picture size is not passed at all: the
  // encoder latches it from the first decoded frame.
  ec.osd_width = cfg.osd_width;
  ec.osd_height = cfg.osd_height;

  im.enc.reset(new MppEncoder());
  Impl* pim = impl_.get();
  if (!im.enc->init(ec, [pim](const uint8_t* p, size_t n, uint64_t pts, bool key) {
        pim->on_nal(p, n, pts, key);
      })) {
    std::fprintf(stderr, "BurnRecorder: encoder init failed; recording disabled\n");
    im.enc.reset();
    return false;
  }

  // Palette upload before the thread exists, so every later encoder call is
  // on the recorder thread. A failure here is NOT fatal: an OSD-less burned
  // recording is a legal (and useful) outcome, so log and carry on.
  if (im.have_palette) {
    im.palette_live = im.enc->set_palette(im.palette);
    if (!im.palette_live)
      std::fprintf(stderr, "BurnRecorder: palette upload failed; recording without the OSD\n");
  }

  im.idr_pending.store(true);  // consumed before the first encode
  im.started = true;
  im.th = std::thread([pim] { pim->run(); });
  std::fprintf(stderr,
               "BurnRecorder: started cap %d fps %d kbps frag %d ms osd=%s (%dx%d px region) "
               "-> %s (picture size latches on the first decoded frame)\n",
               cfg.fps_cap, cfg.bitrate_kbps, cfg.fragment_ms, im.palette_live ? "on" : "off",
               cfg.osd_width, cfg.osd_height, path.c_str());
  return true;
}

void BurnRecorder::submit(const DmaFrame& f) {
  Impl& im = *impl_;
  if (!im.started || im.dead.load()) return;

  // Cap FIRST: a rejected frame must cost nothing but this clock read -- no
  // reference, no lock, no allocation. steady_clock rather than f.pts_us
  // deliberately: the cap is about how hard the ENCODER is driven, and it must
  // not depend on the decoder's pts round-trip being sane (a stuck pts would
  // otherwise wedge the cap shut and record a single frame). pts_us is still
  // what the recording is stamped with, further down.
  const auto now = std::chrono::steady_clock::now();
  if (im.have_last_admit) {
    const int64_t dt =
        std::chrono::duration_cast<std::chrono::microseconds>(now - im.last_admit).count();
    if (dt < im.cap_interval_us - kCapSlackUs) return;
  }

  MppFrame frame = static_cast<MppFrame>(f.opaque);
  if (!frame) return;
  MppBuffer buf = mpp_frame_get_buffer(frame);
  if (!buf) return;

  im.last_admit = now;
  im.have_last_admit = true;

  // REF-SUBMIT: from here the recorder owns a reference; the decoder cannot
  // recycle this buffer until one of the release paths runs.
  mpp_buffer_inc_ref(buf);

  void* displaced = nullptr;
  {
    std::lock_guard<std::mutex> lk(im.mu);
    displaced = im.box.buf;  // latest wins
    im.box.buf = buf;
    im.box.pts_us = f.pts_us;
    im.box.w = f.width;
    im.box.h = f.height;
    im.box.stride = f.stride;
    im.box.vstride = f.vstride;
  }
  im.cv.notify_one();
  im.frames_in.fetch_add(1);

  if (displaced) {
    // REF-DISPLACE: the single most important put in this file. Dropping the
    // entry without releasing leaks one decoder pool buffer per drop, and the
    // pool is 24 deep. Done outside the lock: mpp_buffer_put takes MPP's own
    // service lock and there is no reason to hold both.
    mpp_buffer_put(static_cast<MppBuffer>(displaced));
    im.frames_dropped.fetch_add(1);
  }
}

void BurnRecorder::set_osd(const Surface& s, const DirtyRect* rects, size_t n_rects) {
  Impl& im = *impl_;
  // No palette => the encoder has no OSD region at all, so quantizing would
  // be pure waste on the main loop. This is the osd.enable:false path.
  if (!im.started || !im.palette_live || im.dead.load()) return;
  if (!s.pixels || s.width <= 0 || s.height <= 0) return;
  if (rects && n_rects == 0) return;  // nothing changed; do not even publish

  // The lock spans the quantize deliberately: osd_map is the shared object
  // being updated in place, and the alternative (a private map plus a
  // publish copy) puts a 2 MB memcpy back on this loop, which is most of
  // what the incremental path just removed. The recorder thread waits at
  // most one quantize, and only the rare FULL one is longer than ~10 us.
  std::lock_guard<std::mutex> lk(im.mu);

  // Incremental first; a map that was never sized for this surface (first
  // call, or a resized OSD) refuses and falls back to the full pass.
  bool full = (rects == nullptr);
  if (!full) full = !quantize_rects(s, im.palette, rects, n_rects, &im.osd_map, &im.qcache);
  if (full) {
    quantize(s, im.palette, &im.osd_map, &im.qcache);
    im.osd_dirty.clear();
    im.osd_full_pending = true;
  } else if (!im.osd_full_pending) {
    for (size_t i = 0; i < n_rects; ++i) {
      // Clipped to the surface, because take_osd_locked() copies these
      // straight into osd_work with no bounds check of its own.
      DirtyRect r = rects[i];
      const int x1 = r.x + r.w > s.width ? s.width : r.x + r.w;
      const int y1 = r.y + r.h > s.height ? s.height : r.y + r.h;
      if (r.x < 0) r.x = 0;
      if (r.y < 0) r.y = 0;
      r.w = x1 - r.x;
      r.h = y1 - r.y;
      if (r.w <= 0 || r.h <= 0) continue;
      im.osd_dirty.push_back(r);
      if (im.osd_dirty.size() > Impl::kMaxDirtyRects) {
        // Past this the recorder's row-by-row copy stops being cheaper than
        // taking the whole map.
        im.osd_dirty.clear();
        im.osd_full_pending = true;
        break;
      }
    }
  }
  im.osd_pending_valid = true;
  // Not notified: the map rides along with the next frame the recorder thread
  // picks up. An OSD update with no video to burn it into is a no-op anyway.
}

void BurnRecorder::request_idr() { impl_->idr_pending.store(true); }

void BurnRecorder::drop_pending() {
  if (!impl_->started) return;
  impl_->drain_box();
}

void BurnRecorder::stop() {
  Impl& im = *impl_;
  if (!im.started) return;
  {
    std::lock_guard<std::mutex> lk(im.mu);
    im.stopping = true;
  }
  im.cv.notify_all();
  if (im.th.joinable()) im.th.join();
  // After the join nothing else touches the box, but drain_box() locks
  // anyway -- it is the same code path drop_pending() uses and correctness
  // here must not depend on the join having happened.
  im.drain_box();  // REF-STOP

  if (im.mux_open) {
    im.mux.close();
    im.mux_open = false;
  }
  const uint64_t enc_frames = im.enc ? im.enc->frames() : 0;
  const uint64_t enc_errs = im.enc ? im.enc->errors() : 0;
  im.enc.reset();
  im.started = false;
  std::fprintf(stderr,
               "BurnRecorder: stopped -- in=%llu encoded=%llu dropped=%llu errors=%llu "
               "(encoder frames=%llu errors=%llu) samples=%llu fragments=%llu\n",
               static_cast<unsigned long long>(im.frames_in.load()),
               static_cast<unsigned long long>(im.frames_encoded.load()),
               static_cast<unsigned long long>(im.frames_dropped.load()),
               static_cast<unsigned long long>(im.encode_errors.load()),
               static_cast<unsigned long long>(enc_frames),
               static_cast<unsigned long long>(enc_errs),
               static_cast<unsigned long long>(im.mux.samples()),
               static_cast<unsigned long long>(im.mux.fragments()));
}

bool BurnRecorder::running() const { return impl_->started && !impl_->dead.load(); }

uint64_t BurnRecorder::frames_in() const { return impl_->frames_in.load(); }
uint64_t BurnRecorder::frames_encoded() const { return impl_->frames_encoded.load(); }
uint64_t BurnRecorder::frames_dropped() const { return impl_->frames_dropped.load(); }
uint64_t BurnRecorder::encode_errors() const { return impl_->encode_errors.load(); }
uint64_t BurnRecorder::osd_rejects() const { return impl_->osd_rejects.load(); }

}  // namespace maburplay
