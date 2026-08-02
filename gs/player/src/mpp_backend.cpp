#include "mpp_backend.h"

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
  uint64_t error_count = 0;

  ~Impl() {
    if (ctx) mpp_destroy(ctx);
    if (frm_grp) mpp_buffer_group_put(frm_grp);  // after mpp_destroy, per upstream order
  }

  // Drains every frame decode_get_frame currently has ready: forwards
  // decoded frames to sink (ownership of the MppFrame transfers to the
  // sink/DmaFrame::opaque, released by release_frame()), acks info_change
  // in place, and counts hw-reported errors without emitting them. Called
  // both from poll() (steady-state drain) and from submit_au()'s
  // BUFFER_FULL retry path (drain to make room before retrying put).
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
      if (err_info) ++error_count;

      DmaFrame df;
      df.dmabuf_fd = fd;
      df.fourcc = DRM_FORMAT_NV12;
      df.modifier = 0;
      df.width = static_cast<int>(mpp_frame_get_width(frame));
      df.height = static_cast<int>(mpp_frame_get_height(frame));
      df.stride = static_cast<int>(mpp_frame_get_hor_stride(frame));
      df.vstride = static_cast<int>(mpp_frame_get_ver_stride(frame));
      df.pts_us = static_cast<uint32_t>(mpp_frame_get_pts(frame));
      df.opaque = frame;  // ownership transferred; release_frame() deinits

      if (sink) {
        sink(df);
      } else {
        mpp_frame_deinit(&frame);
      }
    }
  }
};

MppBackend::MppBackend() = default;
MppBackend::~MppBackend() = default;

bool MppBackend::init(const BackendCfg&, FrameSink sink) {
  impl_ = std::make_unique<Impl>();
  impl_->sink = std::move(sink);

  MPP_RET ret = mpp_create(&impl_->ctx, &impl_->mpi);
  if (ret != MPP_OK || !impl_->ctx || !impl_->mpi) {
    std::fprintf(stderr, "MppBackend: mpp_create failed ret=%d\n", ret);
    impl_.reset();
    return false;
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
  //  - ENABLE_FAST_PLAY: start decoding from parameter sets without
  //    waiting for an IRAP -- required to join this link's IRAP-less
  //    streams mid-session (also what makes the watchdog's fresh-context
  //    recovery viable at all).
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
  mpp_packet_set_pts(pkt, static_cast<RK_S64>(pts_us));

  // Retry discipline (brief step 3): loop decode_put_packet until it's
  // accepted. MPP_ERR_BUFFER_FULL means the decoder's internal packet
  // queue is full -- drain ready frames first to make room, then a
  // capped 1 ms sleep (never busy-spin), then retry. kMaxRetries bounds
  // this at ~0.5 s so a genuinely wedged decoder can't hang the player
  // forever; at the ~16.7 ms/frame cadence this stream runs, the healthy
  // path never gets remotely close to that ceiling.
  constexpr int kMaxRetries = 500;
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    const MPP_RET ret = impl_->mpi->decode_put_packet(impl_->ctx, pkt);
    if (ret == MPP_OK) {
      mpp_packet_deinit(&pkt);
      return;
    }
    if (ret != MPP_ERR_BUFFER_FULL) {
      std::fprintf(stderr, "MppBackend: decode_put_packet failed ret=%d\n", ret);
      ++impl_->error_count;
      mpp_packet_deinit(&pkt);
      return;
    }
    impl_->drain_frames();
    usleep(1000);
  }

  std::fprintf(stderr, "MppBackend: decode_put_packet stayed BUFFER_FULL, dropping AU\n");
  ++impl_->error_count;
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

uint64_t MppBackend::info_changes() const { return impl_ ? impl_->info_change_count : 0; }
uint64_t MppBackend::errors() const { return impl_ ? impl_->error_count : 0; }

}  // namespace maburplay
