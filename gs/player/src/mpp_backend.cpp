#include "mpp_backend.h"

#include "pts_unwrap.h"

#include <unistd.h>  // usleep

#include <cstdio>
#include <utility>

#include <drm_fourcc.h>
#include <rockchip/rk_mpi.h>

// Task 8: real rockchip_mpp MPI calls, replacing the Task-7 stub. See
// mpp_backend.h for why the SDK includes live only here, not in the header.
//
// Reference for the call shape: toolchain/mpp-src/test/mpi_dec_test.c's
// dec_simple() (upstream's canonical decode loop) -- this backend follows
// its decode_put_packet/decode_get_frame async split, BUFFER_FULL retry
// discipline, AND its MPP_DEC_SET_EXT_BUF_GROUP sequence. The internal
// (group-less) mode verified in Task 8 sized the pool to the bare DPB:
// fine while the sink released every frame inline, but the presenter's
// three held frames (on-screen + queued + mailbox) starved the decoder
// within ~50 frames on hardware. The external group (buf_size x 24, the
// upstream default) gives DPB + display pipeline + margin.
namespace maburplay {

struct MppBackend::Impl {
  MppCtx ctx = nullptr;
  MppApi* mpi = nullptr;
  MppBufferGroup frm_grp = nullptr;  // external decode buffer pool (see file comment)
  FrameSink sink;
  uint64_t info_change_count = 0;
  std::atomic<uint64_t> error_count{0};  // both threads: put_packet (input), drain_frames (output)
  uint64_t concealed_count = 0;  // errinfo frames emitted for display
  std::atomic<uint64_t> stream_errors{0};   // MPP_DEC_SET_STREAM_APPEND refused
  const std::atomic<bool>* input_cancel = nullptr;

  // The u32 wire pts as a monotonic 64-bit value for every packet and
  // append MPP sees (pts_unwrap.h: the stream hal refuses a part whose pts
  // is not newer than the newest picture it opened, so a raw u32 goes
  // "late" at each 71.6-min wrap). Input side only: submit_au/start/append/
  // abort, all on the feed thread (or main while the feed is parked).
  // Output frames keep the low 32 bits (drain_frames), which ARE the wire
  // pts, so the lat tracker, regulator, DVR and shutdown pts match as before.
  //
  // Lives as long as this MPP context and is NOT reset by flush():
  //  - flush() = mpi->reset(), which (synchronously: mpp_dec_reset_normal
  //    waits on parser_reset) runs the hal's h265d_strm_reset(): opened_any
  //    = 0, so the hal takes ANY pts after it -- continuing the unwrap is
  //    always accepted, a re-seed would be too.
  //  - but a reset that fails leaves the hal's newest_pts in place; a
  //    re-seed at pts + 2^32 could then land below it (after one wrap the
  //    unwrapped value is past 2^33) and every streamed picture would be
  //    refused until the unwrap caught up. Continuing never goes backwards
  //    except for a genuine step back, which the hal refuses either way.
  //  - a recreate (init() again, a new hal with opened_any = 0) builds a
  //    new Impl, so the unwrap starts over with its MPP context.
  PtsUnwrap pts_unwrap;
  RK_S64 ext_pts(uint32_t pts_us) { return static_cast<RK_S64>(pts_unwrap(pts_us)); }

  ~Impl() {
    if (ctx) mpp_destroy(ctx);
    if (frm_grp) mpp_buffer_group_put(frm_grp);  // after mpp_destroy, per upstream order
  }

  // Drains every frame decode_get_frame currently has ready: forwards
  // decoded frames to sink (ownership of the MppFrame transfers to the
  // sink/DmaFrame::opaque, released by release_frame()), acks info_change
  // in place. Hard failures (discard/no-buffer/bad-fd) are counted and
  // dropped; errinfo (concealment) frames are counted AND emitted -- see
  // the comment at the emission site. Called from poll() (steady-state
  // drain), and from put_packet()'s BUFFER_FULL retry (drain to make room
  // before retrying) ONLY in the one-thread mode (decode-only): with
  // set_input_cancel() set -- the FeedLoop split, input on the feed thread
  // -- that retry never drains, since the FrameSink is the output thread's.
  void drain_frames() {
    for (;;) {
      MppFrame frame = nullptr;
      const MPP_RET ret = mpi->decode_get_frame(ctx, &frame);
      if (ret == MPP_ERR_TIMEOUT) break;  // nothing ready right now
      if (ret != MPP_OK) {
        std::fprintf(stderr, "MppBackend: decode_get_frame failed ret=%d\n", ret);
        if (frame) mpp_frame_deinit(&frame);
        break;
      }
      if (!frame) break;  // MPP_OK but nothing returned this round

      if (mpp_frame_get_info_change(frame)) {
        // Resolution announcement: (re)configure the external buffer pool
        // before acking, exactly per upstream mpi_dec_test. buf_size x 24
        // covers the HEVC DPB plus the presenter's held frames plus slack.
        const RK_U32 buf_size = mpp_frame_get_buf_size(frame);
        MPP_RET gret = MPP_OK;
        if (!frm_grp) {
          gret = mpp_buffer_group_get_internal(&frm_grp, MPP_BUFFER_TYPE_ION);
          if (gret == MPP_OK) gret = mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, frm_grp);
        } else {
          gret = mpp_buffer_group_clear(frm_grp);
        }
        if (gret == MPP_OK) gret = mpp_buffer_group_limit_config(frm_grp, buf_size, 24);
        if (gret != MPP_OK) {
          std::fprintf(stderr, "MppBackend: ext buffer group setup failed ret=%d (buf_size=%u)\n",
                       gret, buf_size);
        }
        const MPP_RET ack = mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
        if (ack != MPP_OK) {
          std::fprintf(stderr, "MppBackend: MPP_DEC_SET_INFO_CHANGE_READY failed ret=%d\n", ack);
        }
        ++info_change_count;
        mpp_frame_deinit(&frame);
        continue;
      }

      const RK_U32 err_info = mpp_frame_get_errinfo(frame);
      const RK_U32 discard = mpp_frame_get_discard(frame);
      MppBuffer buf = discard ? nullptr : mpp_frame_get_buffer(frame);
      const int fd = buf ? mpp_buffer_get_fd(buf) : -1;
      if (discard || !buf || fd < 0) {
        ++error_count;
        mpp_frame_deinit(&frame);
        continue;
      }
      // errinfo frames (concealment after reference loss) are counted AND
      // EMITTED. Rally mode has no IRAP to resync from, so after a loss
      // gap EVERY subsequent frame carries errinfo until the rolling
      // refresh repaints -- suppressing them froze the screen and tripped
      // the decode watchdog into a hopeless recreate/exit ladder (observed
      // live under an antenna-cover test). A corrupted-but-healing picture
      // is the correct behavior; it is what the RTP/PixelPilot path shows.
      if (err_info) ++concealed_count;

      DmaFrame df;
      df.dmabuf_fd = fd;
      df.fourcc = DRM_FORMAT_NV12;
      df.modifier = 0;
      df.width = static_cast<int>(mpp_frame_get_width(frame));
      df.height = static_cast<int>(mpp_frame_get_height(frame));
      df.stride = static_cast<int>(mpp_frame_get_hor_stride(frame));
      df.vstride = static_cast<int>(mpp_frame_get_ver_stride(frame));
      // The low 32 bits of the unwrapped pts (Impl::pts_unwrap) = the wire pts.
      df.pts_us = static_cast<uint32_t>(mpp_frame_get_pts(frame));
      df.opaque = frame;  // ownership transferred; release_frame() deinits

      if (sink) {
        sink(df);
      } else {
        mpp_frame_deinit(&frame);
      }
    }
  }

  // decode_put_packet with the BUFFER_FULL retry discipline: drain ready
  // frames first to make room, then a capped 1 ms sleep (never busy-spin).
  // kMaxRetries bounds this at ~0.5 s so a genuinely wedged decoder can't
  // hang the player forever; at the ~16.7 ms/frame cadence this stream
  // runs, the healthy path never gets remotely close to that ceiling.
  bool put_packet(MppPacket pkt) {
    constexpr int kMaxRetries = 500;
    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
      const MPP_RET ret = mpi->decode_put_packet(ctx, pkt);
      if (ret == MPP_OK) return true;
      if (ret != MPP_ERR_BUFFER_FULL) {
        std::fprintf(stderr, "MppBackend: decode_put_packet failed ret=%d\n", ret);
        ++error_count;
        return false;
      }
      if (input_cancel) {
        // Split threads: the output side is the main loop's, which keeps
        // releasing frames; draining here would run the FrameSink on this
        // thread. Give way at once to a park or a stop.
        if (input_cancel->load(std::memory_order_relaxed)) {
          ++error_count;
          std::fprintf(stderr, "MppBackend: decode_put_packet BUFFER_FULL, given up (feed parked)\n");
          return false;
        }
      } else {
        drain_frames();
      }
      usleep(1000);
    }
    std::fprintf(stderr, "MppBackend: decode_put_packet stayed BUFFER_FULL, dropping AU\n");
    ++error_count;
    return false;
  }
};

MppBackend::MppBackend() = default;
MppBackend::~MppBackend() = default;

bool MppBackend::init(const BackendCfg&, FrameSink sink) {
  impl_ = std::make_unique<Impl>();
  impl_->sink = std::move(sink);
  impl_->input_cancel = input_cancel_;

  MPP_RET ret = mpp_create(&impl_->ctx, &impl_->mpi);
  if (ret != MPP_OK || !impl_->ctx || !impl_->mpi) {
    std::fprintf(stderr, "MppBackend: mpp_create failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }

  if (stream_wanted_) {
    // [decoder] stream: parser fast mode off BEFORE mpp_init. The h265d hal
    // decides at its own init whether it can stream (h265d_strm_init():
    // never in fast mode -- two pictures in flight), from base:fast_parse,
    // which MPP defaults to 1 (mpp_dec_cfg.c). A pre-init MPP_DEC_SET_CFG
    // lands in the config mpp_init hands the decoder (mpp.c
    // mpp_control_dec). A failure here just makes probe() refuse.
    MppDecCfg dcfg = nullptr;
    MPP_RET cr = mpp_dec_cfg_init(&dcfg);
    if (cr == MPP_OK) cr = impl_->mpi->control(impl_->ctx, MPP_DEC_GET_CFG, dcfg);
    if (cr == MPP_OK) cr = mpp_dec_cfg_set_u32(dcfg, "base:fast_parse", 0);
    if (cr == MPP_OK) cr = impl_->mpi->control(impl_->ctx, MPP_DEC_SET_CFG, dcfg);
    if (dcfg) mpp_dec_cfg_deinit(dcfg);
    if (cr != MPP_OK)
      std::fprintf(stderr, "MppBackend: base:fast_parse = 0 failed ret=%d (stream probe will refuse)\n",
                   cr);
  }

  ret = mpp_init(impl_->ctx, MPP_CTX_DEC, MPP_VIDEO_CodingHEVC);
  if (ret != MPP_OK) {
    std::fprintf(stderr, "MppBackend: mpp_init failed ret=%d\n", ret);
    impl_.reset();  // Impl's dtor mpp_destroy()s the ctx mpp_create() made
    return false;
  }

  // Low-delay output: emit each frame as soon as it's decoded rather than
  // buffering for display reordering. Must be set before the first packet
  // per the brief; not fatal to decode correctness if rejected, only to
  // latency, so a failure here logs and continues rather than aborting
  // init.
  RK_U32 immediate_out = 1;
  ret = impl_->mpi->control(impl_->ctx, MPP_DEC_SET_IMMEDIATE_OUT, &immediate_out);
  if (ret != MPP_OK) {
    std::fprintf(stderr,
                 "MppBackend: MPP_DEC_SET_IMMEDIATE_OUT failed ret=%d (continuing)\n", ret);
  }

  // FPV-stream survival controls, mirrored from PixelPilot_rk's proven
  // decoder setup (../PixelPilot_rk/src/main.cpp mpi_dec_init):
  //  - DISABLE_ERROR: turn MPP's internal error handling off. With it ON
  //    (the default), a loss gap poisons the reference chain permanently
  //    -- the GDR sweep never repaints and every frame stays errinfo
  //    forever (observed live: 60 fps of permanently-broken frames).
  //    With error handling off, damaged frames decode as-is, refs keep
  //    advancing, and the intra sweep genuinely heals the picture.
  //  - ENABLE_FAST_PLAY: fast-start on the first IDR/BLA without waiting
  //    for full DPB state. NOTE (review-verified against h265d_flow.c):
  //    for HEVC this shortcut is gated on IS_IDR||IS_BLA, so on this
  //    link's IRAP-less streams it is INERT -- the mid-session join is
  //    carried by DISABLE_ERROR (+ IMMEDIATE_OUT), not by this control.
  //    Kept because it is harmless, matches PixelPilot's proven setup,
  //    and matters the moment the encoder runs an IDR-bearing resilience
  //    mode. Do not remove DISABLE_ERROR believing FAST_PLAY covers it.
  RK_U32 on = 0xffff;
  ret = impl_->mpi->control(impl_->ctx, MPP_DEC_SET_DISABLE_ERROR, &on);
  if (ret != MPP_OK)
    std::fprintf(stderr, "MppBackend: MPP_DEC_SET_DISABLE_ERROR failed ret=%d (continuing)\n",
                 ret);
  ret = impl_->mpi->control(impl_->ctx, MPP_DEC_SET_ENABLE_FAST_PLAY, &on);
  if (ret != MPP_OK)
    std::fprintf(stderr,
                 "MppBackend: MPP_DEC_SET_ENABLE_FAST_PLAY failed ret=%d (continuing)\n", ret);

  return true;
}

void MppBackend::submit_au(const uint8_t* au, size_t n, uint32_t pts_us) {
  if (!impl_ || !impl_->ctx || !au || n == 0) return;
  MppPacket pkt = nullptr;
  if (mpp_packet_init(&pkt, const_cast<uint8_t*>(au), n) != MPP_OK) {
    ++impl_->error_count;
    return;
  }
  mpp_packet_set_pts(pkt, impl_->ext_pts(pts_us));
  impl_->put_packet(pkt);
  mpp_packet_deinit(&pkt);
}

void MppBackend::flush() {
  if (!impl_ || !impl_->ctx) return;
  // Drops all in-flight decoder state; the ring client's flush_before
  // already aligns this with the next AU being an IRAP.
  const MPP_RET ret = impl_->mpi->reset(impl_->ctx);
  if (ret != MPP_OK) {
    std::fprintf(stderr, "MppBackend: reset failed ret=%d\n", ret);
    ++impl_->error_count;
  }
}

void MppBackend::release_frame(const DmaFrame& f) {
  if (!f.opaque) return;
  MppFrame frame = static_cast<MppFrame>(f.opaque);
  mpp_frame_deinit(&frame);
}

void MppBackend::poll() {
  if (!impl_ || !impl_->ctx) return;
  impl_->drain_frames();
}

void MppBackend::set_input_cancel(const std::atomic<bool>* cancel) {
  input_cancel_ = cancel;
  if (impl_) impl_->input_cancel = cancel;
}

uint64_t MppBackend::info_changes() const { return impl_ ? impl_->info_change_count : 0; }

uint64_t MppBackend::concealed() const { return impl_ ? impl_->concealed_count : 0; }
uint64_t MppBackend::errors() const {
  return impl_ ? impl_->error_count.load(std::memory_order_relaxed) : 0;
}

bool MppBackend::probe() {
  if (!impl_ || !impl_->ctx || !stream_wanted_) return false;
  // NULL asks whether this decoder can stream (fpvOS MPP 0001): MPP_OK if
  // the hal opened stream mode at init -- fast mode off and a kernel that
  // answers MPP_STREAM_PROBE with "supported".
  const MPP_RET ret = impl_->mpi->control(impl_->ctx, MPP_DEC_SET_STREAM_APPEND, nullptr);
  std::fprintf(stderr, "MppBackend: stream mode %s (probe ret=%d)\n",
               ret == MPP_OK ? "available" : "refused", ret);
  return ret == MPP_OK;
}

bool MppBackend::start(const uint8_t* p, size_t n, uint8_t nslices, uint32_t pts_us) {
  if (!impl_ || !impl_->ctx || !p || n == 0) return false;
  MppPacket pkt = nullptr;
  if (mpp_packet_init(&pkt, const_cast<uint8_t*>(p), n) != MPP_OK) {
    ++impl_->error_count;
    return false;
  }
  mpp_packet_set_pts(pkt, impl_->ext_pts(pts_us));
  // The decoder starts on this first part at once and is told the
  // picture's slice count up front (rkvdec2 reg017.slice_num).
  mpp_packet_set_flag(pkt, MPP_PACKET_FLAG_STREAM_START | MPP_PACKET_STREAM_SLICES(nslices));
  const bool ok = impl_->put_packet(pkt);
  mpp_packet_deinit(&pkt);
  return ok;
}

bool MppBackend::append(const uint8_t* p, size_t n, uint32_t pts_us, bool last) {
  if (!impl_ || !impl_->ctx) return false;
  // Taken on this thread while the picture decodes; MPP copies the bytes
  // (into the picture's stream buffer, or a pending part until the decode
  // thread starts that picture), so the caller's buffer may go at once.
  MppDecStreamAppend a{};
  a.data = p;
  a.size = static_cast<RK_U32>(n);
  a.flags = last ? MPP_STREAM_APPEND_LAST : 0;
  a.pts = impl_->ext_pts(pts_us);
  const MPP_RET ret = impl_->mpi->control(impl_->ctx, MPP_DEC_SET_STREAM_APPEND, &a);
  if (ret != MPP_OK) {
    if (impl_->stream_errors++ == 0)
      std::fprintf(stderr, "MppBackend: STREAM_APPEND refused ret=%d (first of possibly many)\n", ret);
    return false;
  }
  return true;
}

void MppBackend::abort(uint32_t pts_us) {
  // No data + LAST: the kernel gives the picture 64 zero bytes as its last
  // part and ends it -- the decoder errors and resets once, no hang.
  append(nullptr, 0, pts_us, true);
}

uint64_t MppBackend::stream_errors() const {
  return impl_ ? impl_->stream_errors.load(std::memory_order_relaxed) : 0;
}

}  // namespace maburplay
