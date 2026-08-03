#include "burn_recorder.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
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

}  // namespace

struct BurnRecorder::Impl {
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

  // OSD handoff, under the same mutex. Three maps so nothing allocates after
  // warm-up: stage (main loop quantizes into it), pending (published),
  // work (recorder thread's own copy, handed to the encoder).
  OsdIndexMap osd_stage, osd_pending, osd_work;
  bool osd_pending_valid = false;

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
      if (!mux.open(path, params.hvcc(), cfg.width, cfg.height, cfg.fragment_ms)) {
        mux_failed = true;
        std::fprintf(stderr, "BurnRecorder: cannot open %s; recording disabled\n", path.c_str());
        dead.store(true);
        return;
      }
      mux_open = true;
      std::fprintf(stderr, "BurnRecorder: recording %s (%dx%d, hvcC from %zu header bytes)\n",
                   path.c_str(), cfg.width, cfg.height, hdr.size());
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
          osd_work.px.swap(osd_pending.px);
          osd_work.mb_w = osd_pending.mb_w;
          osd_work.mb_h = osd_pending.mb_h;
          osd_pending_valid = false;
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
      if (have_osd) enc->set_osd(osd_work);

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
      if (frames_encoded.load() == 0 && consecutive_fail >= kMaxInitialFailures) {
        // MppEncoder's permanent-refusal signature. It logged the reason once
        // itself; say why the recording is over and stop feeding it, rather
        // than burning a decoder buffer round-trip per frame forever.
        std::fprintf(stderr,
                     "BurnRecorder: encoder refused %llu consecutive frames with none encoded; "
                     "recording disabled (no file written)\n",
                     static_cast<unsigned long long>(consecutive_fail));
        dead.store(true);
      }
    }
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
  ec.width = cfg.width;
  ec.height = cfg.height;
  ec.fps = cfg.fps_cap;  // the encoder's rc: rate IS the capped rate
  ec.bitrate_kbps = cfg.bitrate_kbps;

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
  std::fprintf(stderr, "BurnRecorder: started %dx%d cap %d fps %d kbps frag %d ms osd=%s -> %s\n",
               cfg.width, cfg.height, cfg.fps_cap, cfg.bitrate_kbps, cfg.fragment_ms,
               im.palette_live ? "on" : "off", path.c_str());
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

void BurnRecorder::set_osd(const Surface& s) {
  Impl& im = *impl_;
  // No palette => the encoder has no OSD region at all, so quantizing would
  // be pure waste on the main loop. This is the osd.enable:false path.
  if (!im.started || !im.palette_live || im.dead.load()) return;
  if (!s.pixels || s.width <= 0 || s.height <= 0) return;

  quantize(s, im.palette, &im.osd_stage);
  {
    std::lock_guard<std::mutex> lk(im.mu);
    im.osd_stage.px.swap(im.osd_pending.px);
    im.osd_pending.mb_w = im.osd_stage.mb_w;
    im.osd_pending.mb_h = im.osd_stage.mb_h;
    im.osd_pending_valid = true;
  }
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

}  // namespace maburplay
