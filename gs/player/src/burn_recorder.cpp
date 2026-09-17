#include "burn_recorder.h"

#include <algorithm>
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
#include "fps_cap.h"
#include "mpp_backend.h"
#include "mpp_encoder.h"

#ifdef MABUR_PLAYER_GPU
#include "frame_colortrans.h"
#endif

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

  // fps cap, main loop only. The policy and why it is schedule-based rather
  // than last-admit-based live in fps_cap.h (host-tested, test_fps_cap).
  FpsCap cap;
  std::chrono::steady_clock::time_point cap_epoch;

  std::atomic<uint64_t> frames_in{0};
  std::atomic<uint64_t> frames_encoded{0};
  std::atomic<uint64_t> frames_dropped{0};   // displaced or dead: OVERLOAD
  std::atomic<uint64_t> frames_flushed{0};   // drop_pending()/stop(): HYGIENE
  std::atomic<uint64_t> encode_errors{0};
  std::atomic<uint64_t> osd_rejects{0};
  std::atomic<uint64_t> colortrans_fallbacks{0};
  std::atomic<uint64_t> ct_n{0}, ct_sum_us{0}, ct_max_us{0};    // stage_us()
  std::atomic<uint64_t> enc_n{0}, enc_sum_us{0}, enc_max_us{0};
  static void account(std::atomic<uint64_t>& n, std::atomic<uint64_t>& sum,
                      std::atomic<uint64_t>& mx, std::chrono::steady_clock::time_point t0) {
    const uint64_t us = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now() - t0)
                                                  .count());
    n.fetch_add(1);
    sum.fetch_add(us);
    uint64_t cur = mx.load();
    while (us > cur && !mx.compare_exchange_weak(cur, us)) {
    }
  }
  // --- colortrans stage (ct thread) and its handoff to the encoder ---------
  //
  // With colortrans on, the recorder is a two-stage pipeline:
  //
  //   main loop --box--> ct thread --ct_out--> recorder thread
  //                      GPU draw + RGA          encode() + mux
  //                      into ct_dst[k]
  //
  // Both stages are hardware waits (GPU/RGA ~13-22 ms, rkvenc ~13 ms at
  // 1080p) and serialising them on one thread capped the burned DVR at
  // 26-32 fps with a third of the admitted frames dropped (bench
  // 2026-09-17). Overlapped, the DVR runs at max(stage) instead of the sum,
  // and the GPU is busy enough for devfreq to actually clock it up.
  //
  // ct_out is a one-deep, latest-wins mailbox like `box`: a slow encoder
  // displaces the previous output (frames_dropped, same meaning as a box
  // displacement) rather than stalling the GPU stage. kCtDst = 3 covers the
  // worst case: one destination being rendered, one waiting in ct_out, one
  // inside encode(). ct_dst_busy[] (under mu) says which are taken.
  //
  // Without colortrans (or a build without the stage) there is no ct thread
  // and the recorder thread consumes `box` directly, exactly as before.
  struct CtMail {
    void* buf = nullptr;  // MppBuffer: ct_dst[dst_idx], or the SOURCE when dst_idx < 0
    int dst_idx = -1;     // < 0 = flat: buf is a held decoder reference
    uint32_t pts_us = 0;
    int w = 0, h = 0, stride = 0, vstride = 0;
  };
  bool ct_on = false;  // start(): colortrans requested AND the build has the stage
  std::condition_variable ct_cv;  // ct_out handoff; `cv` stays the box's
  CtMail ct_out;                  // mu
  std::thread ct_th;

  // Releases whatever a CtMail holds: a destination goes back to the free
  // set, a flat source reference goes back to the decoder pool.
  void release_ct(const CtMail& o) {
    if (!o.buf) return;
    if (o.dst_idx >= 0) {
      std::lock_guard<std::mutex> lk(mu);
      ct_dst_busy[o.dst_idx] = false;
    } else {
      mpp_buffer_put(static_cast<MppBuffer>(o.buf));
    }
  }

  static constexpr int kCtDst = 3;
  bool ct_dst_busy[kCtDst] = {};  // mu
#ifdef MABUR_PLAYER_GPU
  std::unique_ptr<FrameColorTrans> ct;  // ct thread only (EGL is thread-bound)
  bool ct_failed = false;               // init failed once: stay flat, logged once
  MppBufferGroup ct_grp = nullptr;      // private DRM group for the destinations
  MppBuffer ct_dst[kCtDst] = {};
  size_t ct_dst_size = 0;
  int ct_w = 0, ct_h = 0;

  // ct thread. Brings the stage and the destinations up for this geometry;
  // false = pass this frame through flat. The destinations are sized once,
  // on the first frame: MppEncoder latches its picture size on the first
  // encode() anyway, so a later, larger frame cannot be recorded either way.
  bool ct_prepare(int w, int h, int hs, int vs) {
    if (ct_failed) return false;
    if (!ct || ct_w != w || ct_h != h) {
      ct.reset(new FrameColorTrans());
      if (!ct->init((uint32_t)w, (uint32_t)h)) {
        std::fprintf(stderr, "BurnRecorder: colortrans GPU stage unavailable; recording FLAT\n");
        ct.reset();
        ct_failed = true;
        return false;
      }
      ct_w = w;
      ct_h = h;
    }
    const size_t need = (size_t)hs * (size_t)vs * 3 / 2;
    if (ct_dst_size) return need <= ct_dst_size;
    if (!ct_grp && mpp_buffer_group_get_internal(&ct_grp, MPP_BUFFER_TYPE_DRM) != MPP_OK) {
      std::fprintf(stderr, "BurnRecorder: colortrans buffer group failed; recording FLAT\n");
      ct_failed = true;
      return false;
    }
    for (int i = 0; i < kCtDst; ++i) {
      if (mpp_buffer_get(ct_grp, &ct_dst[i], need) != MPP_OK || !ct_dst[i]) {
        std::fprintf(stderr, "BurnRecorder: colortrans destination alloc failed; recording FLAT\n");
        ct_failed = true;
        return false;
      }
    }
    ct_dst_size = need;
    return true;
  }

  // stop(), after BOTH joins: nothing references the destinations any more.
  void ct_free_dst() {
    for (int i = 0; i < kCtDst; ++i) {
      if (ct_dst[i]) mpp_buffer_put(ct_dst[i]);
      ct_dst[i] = nullptr;
      ct_dst_busy[i] = false;
    }
    ct_dst_size = 0;
    if (ct_grp) mpp_buffer_group_put(ct_grp);
    ct_grp = nullptr;
  }

  // The ct thread: box -> GPU stage -> ct_out. Never blocks on the encoder.
  void run_ct() {
    for (;;) {
      Mail m;
      {
        std::unique_lock<std::mutex> lk(mu);
        cv.wait(lk, [this] { return stopping || box.buf != nullptr; });
        if (stopping) break;
        m = box;
        box.buf = nullptr;
      }
      if (dead.load()) {
        // REF-DEAD, ct side: the encoder gave up; nothing downstream wants this.
        mpp_buffer_put(static_cast<MppBuffer>(m.buf));
        frames_dropped.fetch_add(1);
        continue;
      }

      CtMail o;
      o.buf = m.buf;  // flat unless the stage succeeds below
      o.pts_us = m.pts_us;
      o.w = m.w;
      o.h = m.h;
      o.stride = m.stride;
      o.vstride = m.vstride;

      bool done = false;
      if (ct_prepare(m.w, m.h, m.stride, m.vstride)) {
        int k = -1;
        {
          std::lock_guard<std::mutex> lk(mu);
          for (int i = 0; i < kCtDst && k < 0; ++i)
            if (!ct_dst_busy[i]) k = i;
          if (k >= 0) ct_dst_busy[k] = true;
        }
        // k < 0 cannot happen with kCtDst = 3 (see the pipeline comment);
        // if it ever does, the frame records flat and is counted below.
        if (k >= 0) {
          const auto t_ct = std::chrono::steady_clock::now();
          done = ct->process(mpp_buffer_get_fd(static_cast<MppBuffer>(m.buf)), (uint32_t)m.w,
                             (uint32_t)m.h, (uint32_t)m.stride, (uint32_t)m.vstride,
                             mpp_buffer_get_fd(ct_dst[k]), (uint32_t)m.stride,
                             (uint32_t)m.vstride);
          if (done) {
            account(ct_n, ct_sum_us, ct_max_us, t_ct);
            // REF-ENCODE, early: the GPU and RGA are done with the source
            // (glFinish + synchronous imcvtcolor); the encoder reads ct_dst.
            mpp_buffer_put(static_cast<MppBuffer>(m.buf));
            o.buf = ct_dst[k];
            o.dst_idx = k;
          } else {
            std::lock_guard<std::mutex> lk(mu);
            ct_dst_busy[k] = false;
          }
        }
      }
      if (!done) colortrans_fallbacks.fetch_add(1);

      CtMail displaced;
      {
        std::lock_guard<std::mutex> lk(mu);
        displaced = ct_out;  // latest wins, same rule as the box
        ct_out = o;
      }
      ct_cv.notify_one();
      if (displaced.buf) {
        // REF-DISPLACE, ct side: the encoder is the slow stage right now.
        release_ct(displaced);
        frames_dropped.fetch_add(1);
      }
    }
    if (ct) {
      const FrameColorTrans::Breakdown& b = ct->breakdown();
      if (b.n)
        std::fprintf(stderr,
                     "BurnRecorder: colortrans stage mean per frame over %llu: import %.1f ms, "
                     "draw+finish %.1f ms, rga %.1f ms\n",
                     static_cast<unsigned long long>(b.n), b.import_us / 1000.0 / b.n,
                     b.draw_us / 1000.0 / b.n, b.rga_us / 1000.0 / b.n);
    }
    ct.reset();  // EGL teardown on the thread that owns the context
  }
#endif
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
      CtMail o;
      bool have_osd = false;
      {
        std::unique_lock<std::mutex> lk(mu);
        if (ct_on) {
          ct_cv.wait(lk, [this] { return stopping || ct_out.buf != nullptr; });
          if (stopping) break;
          o = ct_out;
          ct_out.buf = nullptr;
        } else {
          // stop() drains whatever is left in the box after the join, so this
          // thread never has to decide who releases a half-handled entry.
          cv.wait(lk, [this] { return stopping || box.buf != nullptr; });
          if (stopping) break;
          o.buf = box.buf;
          o.pts_us = box.pts_us;
          o.w = box.w;
          o.h = box.h;
          o.stride = box.stride;
          o.vstride = box.vstride;
          box.buf = nullptr;
        }
        if (osd_pending_valid) {
          take_osd_locked();
          have_osd = true;
        }
      }

      if (dead.load()) {
        // REF-DEAD: a frame admitted just before the recorder gave up.
        release_ct(o);
        frames_dropped.fetch_add(1);
        continue;
      }

      if (idr_pending.exchange(false)) enc->request_idr();
      // A refused map means the encoder's region and the OSD surface
      // disagree -- a recording that is a silent plain transcode. Counted so
      // --fps-log can say so; the encoder's own log for it is once-only.
      if (have_osd && !enc->set_osd(osd_work)) osd_rejects.fetch_add(1);

      // Without the ct thread every frame is flat; with it, the ct thread
      // already counted its own fallbacks.
      if (!ct_on && cfg.colortrans) colortrans_fallbacks.fetch_add(1);

      const auto t_enc = std::chrono::steady_clock::now();
      const bool ok =
          enc->encode(o.buf, o.w, o.h, o.stride, o.vstride, static_cast<uint64_t>(o.pts_us));
      account(enc_n, enc_sum_us, enc_max_us, t_enc);
      // REF-ENCODE: released the moment the encoder is done with it, whether
      // or not it produced a packet. encode() borrows, it never owns.
      release_ct(o);

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
      // newer frame. The reference accounting is identical, but the MEANING
      // is not, so it gets its own counter: this is flush hygiene (a link
      // discontinuity, a watchdog reset, shutdown), not encoder overload.
      // Sharing one counter let a link-loss storm read as overload -- and
      // let the hardware leak test, which reads frames_dropped(), be
      // satisfied entirely by flushes with the displacement path never
      // exercised at all.
      mpp_buffer_put(static_cast<MppBuffer>(buf));
      frames_flushed.fetch_add(1);
    }
  }
  // Same hygiene for the ct -> encoder handoff.
  void drain_ct_out() {
    CtMail o;
    {
      std::lock_guard<std::mutex> lk(mu);
      o = ct_out;
      ct_out.buf = nullptr;
    }
    if (o.buf) {
      release_ct(o);
      frames_flushed.fetch_add(1);
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
  im.cap.reset(cfg.fps_cap);
  im.cap_epoch = std::chrono::steady_clock::now();
  im.mux_open = false;
  im.mux_failed = false;
  im.dead.store(false);
  im.colortrans_fallbacks.store(0);
  im.ct_n.store(0);
  im.ct_sum_us.store(0);
  im.ct_max_us.store(0);
  im.enc_n.store(0);
  im.enc_sum_us.store(0);
  im.enc_max_us.store(0);
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

#ifndef MABUR_PLAYER_GPU
  if (cfg.colortrans)
    std::fprintf(stderr, "BurnRecorder: colortrans requested but this build has no GPU stage; "
                         "recording FLAT (colortrans_fallbacks will climb)\n");
#endif

  im.idr_pending.store(true);  // consumed before the first encode
  im.ct_out = Impl::CtMail{};
  for (int i = 0; i < Impl::kCtDst; ++i) im.ct_dst_busy[i] = false;
#ifdef MABUR_PLAYER_GPU
  im.ct_on = cfg.colortrans != nullptr;
  im.ct_failed = false;
#else
  im.ct_on = false;
#endif
  im.started = true;
  im.th = std::thread([pim] { pim->run(); });
#ifdef MABUR_PLAYER_GPU
  if (im.ct_on) im.ct_th = std::thread([pim] { pim->run_ct(); });
#endif
  std::fprintf(stderr,
               "BurnRecorder: started cap %d fps %d kbps frag %d ms osd=%s (%dx%d px region) "
               "colortrans=%s -> %s (picture size latches on the first decoded frame)\n",
               cfg.fps_cap, cfg.bitrate_kbps, cfg.fragment_ms, im.palette_live ? "on" : "off",
               cfg.osd_width, cfg.osd_height, cfg.colortrans ? "on" : "off", path.c_str());
  return true;
}

void BurnRecorder::submit(const DmaFrame& f) {
  Impl& im = *impl_;
  if (!im.started || im.dead.load()) return;

  // Cap before anything costs: a rejected frame is two field reads and a
  // clock read -- no reference, no lock, no allocation. steady_clock rather
  // than f.pts_us deliberately: the cap is about how hard the ENCODER is
  // driven, and it must not depend on the decoder's pts round-trip being sane
  // (a stuck pts would otherwise wedge the cap shut and record a single
  // frame). pts_us is still what the recording is stamped with, further down.
  MppFrame frame = static_cast<MppFrame>(f.opaque);
  if (!frame) return;
  MppBuffer buf = mpp_frame_get_buffer(frame);
  if (!buf) return;
  const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now() - im.cap_epoch)
                             .count();
  if (!im.cap.admit(now_us)) return;

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
  // most one quantize.
  //
  // How long that wait is, measured rather than assumed (the text here used
  // to say "only the rare FULL one is longer than ~10 us", which was wrong
  // by more than two orders of magnitude for the GS overlay):
  //   MSP steady state, a few cells        ~10 us          -- as claimed
  //   GS single field, 6,068 px            ~0.13 ms A55
  //   GS all fields, 179,392 px @1080p     ~2.2 ms A55     -- the quantize
  //   GS all fields, 649,429 px @2160p     ~6.1 ms A55        half alone
  //   full pass, whole surface             ~5.1 ms A55 @1080p
  // (tools/bench/gs_overlay_bench.cpp.) So an INCREMENTAL quantize can hold
  // this mutex for milliseconds, and the encoder thread blocks on it for
  // exactly that long. The all-fields cases are why osd_compose.cpp scopes
  // its burn restate to the fields an MSP collision actually hit instead of
  // invalidating the whole burn overlay -- see the comment there.
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
  impl_->drain_ct_out();
}

void BurnRecorder::stop() {
  Impl& im = *impl_;
  if (!im.started) return;
  {
    std::lock_guard<std::mutex> lk(im.mu);
    im.stopping = true;
  }
  im.cv.notify_all();
  im.ct_cv.notify_all();
  // ct thread first: it only ever hands frames DOWN to the encoder thread,
  // so once it is gone ct_out is quiescent for the encoder's own exit.
  if (im.ct_th.joinable()) im.ct_th.join();
  if (im.th.joinable()) im.th.join();
  // After the joins nothing else touches the mailboxes, but the drains lock
  // anyway -- they are the code path drop_pending() uses and correctness
  // here must not depend on the joins having happened.
  im.drain_box();     // REF-STOP
  im.drain_ct_out();  // REF-STOP, ct side
#ifdef MABUR_PLAYER_GPU
  im.ct_free_dst();
#endif

  if (im.mux_open) {
    im.mux.close();
    im.mux_open = false;
  }
  const uint64_t enc_frames = im.enc ? im.enc->frames() : 0;
  const uint64_t enc_errs = im.enc ? im.enc->errors() : 0;
  im.enc.reset();
  im.started = false;
  const StageUs su = stage_us();
  std::fprintf(stderr,
               "BurnRecorder: stopped -- in=%llu encoded=%llu dropped=%llu flushed=%llu "
               "errors=%llu osd_rejects=%llu ct_fallbacks=%llu (encoder frames=%llu errors=%llu) "
               "samples=%llu fragments=%llu ct_ms=%.1f/%.1f enc_ms=%.1f/%.1f (mean/max)\n",
               static_cast<unsigned long long>(im.frames_in.load()),
               static_cast<unsigned long long>(im.frames_encoded.load()),
               static_cast<unsigned long long>(im.frames_dropped.load()),
               static_cast<unsigned long long>(im.frames_flushed.load()),
               static_cast<unsigned long long>(im.encode_errors.load()),
               static_cast<unsigned long long>(im.osd_rejects.load()),
               static_cast<unsigned long long>(im.colortrans_fallbacks.load()),
               static_cast<unsigned long long>(enc_frames),
               static_cast<unsigned long long>(enc_errs),
               static_cast<unsigned long long>(im.mux.samples()),
               static_cast<unsigned long long>(im.mux.fragments()),
               su.ct_n ? su.ct_sum / 1000.0 / su.ct_n : 0.0, su.ct_max / 1000.0,
               su.enc_n ? su.enc_sum / 1000.0 / su.enc_n : 0.0, su.enc_max / 1000.0);
}

bool BurnRecorder::running() const { return impl_->started && !impl_->dead.load(); }

uint64_t BurnRecorder::frames_in() const { return impl_->frames_in.load(); }
uint64_t BurnRecorder::frames_encoded() const { return impl_->frames_encoded.load(); }
uint64_t BurnRecorder::frames_dropped() const { return impl_->frames_dropped.load(); }
uint64_t BurnRecorder::frames_flushed() const { return impl_->frames_flushed.load(); }
uint64_t BurnRecorder::encode_errors() const { return impl_->encode_errors.load(); }
uint64_t BurnRecorder::osd_rejects() const { return impl_->osd_rejects.load(); }
uint64_t BurnRecorder::colortrans_fallbacks() const { return impl_->colortrans_fallbacks.load(); }
BurnRecorder::StageUs BurnRecorder::stage_us() const {
  StageUs s;
  s.ct_n = impl_->ct_n.load();
  s.ct_sum = impl_->ct_sum_us.load();
  s.ct_max = impl_->ct_max_us.load();
  s.enc_n = impl_->enc_n.load();
  s.enc_sum = impl_->enc_sum_us.load();
  s.enc_max = impl_->enc_max_us.load();
  return s;
}

}  // namespace maburplay
