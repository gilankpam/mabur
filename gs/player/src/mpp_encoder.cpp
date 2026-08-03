#include "mpp_encoder.h"

#include <cstdio>
#include <cstring>
#include <utility>

#include <rockchip/rk_mpi.h>

// Real rockchip_mpp encoder MPI, mirroring the sequence bench/encosd proved
// on hardware (docs/superpowers/specs/2026-08-04-mpp-enc-osd-spike.md):
// mpp_create -> mpp_init(MPP_CTX_ENC, HEVC) -> mpp_enc_cfg_init +
// MPP_ENC_GET_CFG -> prep:/rc:/h265: keys -> MPP_ENC_SET_CFG ->
// MPP_ENC_SET_HEADER_MODE(EACH_IDR) -> internal DRM|CACHABLE buffer group ->
// MPP_ENC_GET_HDR_SYNC -> per frame encode_put_frame / encode_get_packet.
//
// The split against the spike: everything geometry-dependent (prep:*,
// MPP_ENC_SET_CFG, the packet buffer, MPP_ENC_GET_HDR_SYNC) is deferred to
// the FIRST encode(), which latches that frame's strides. EncCfg carries no
// strides and the decoder's real ones are only known once frames flow; after
// the latch, any disagreeing frame is rejected outright rather than
// reconfigured. See Impl::latch_geometry() for why a reconfigure is not an
// option here.
//
// The four measured traps this file is built around (all silent -- each
// produces wrong pixels with NO API error):
//   1. MppEncOSDPltVal's {v,u,y,alpha} bitfields are wrong on little-endian.
//      Only ::val is written here; OsdPalette::entry is already packed
//      Y | U<<8 | V<<16 | A<<24, which is what the hardware wants.
//   2. The index bitmap is RASTER with stride num_mb_x*16, not
//      macroblock-tiled. OsdIndexMap already has that layout; it is copied
//      in verbatim and never re-tiled.
//   3. MPP does not copy the OSD. The MppBuffer AND the MppEncOSDData struct
//      are dereferenced by the HAL on the encoder thread at gen_regs time,
//      after encode_put_frame() returns -- hence two of each, flipped.
//   4. The meta attachment is not sticky: the HAL re-reads KEY_OSD_DATA with
//      a NULL default on every frame carrying meta, and our frames always
//      carry meta (KEY_OUTPUT_PACKET). A frame that omits the key encodes
//      with the OSD off. It is attached on every frame.
// Plus the geometry one: full screen at 1080p is 120x68 macroblocks
// (1920x1088), not 120x67 -- a 67-row region leaves the bottom 8 scanlines
// bare. Regions may legally overhang the picture; the hardware clips.
namespace maburplay {

namespace {

constexpr int kMbSize = 16;

// Scans an HEVC Annex-B elementary stream for an IRAP or a VPS: the
// keyframe-detection fallback (see Impl::is_keyframe()). Every keyframe this
// encoder emits opens with an IDR NAL (type 19/20) whether or not parameter
// sets precede it, so the scan does not depend on the header being in band
// -- which for the very first IDR it is not (see MppEncoder::header()).
bool hevc_has_irap(const uint8_t* p, size_t n) {
  if (!p || n < 5) return false;
  for (size_t i = 0; i + 4 < n; ++i) {
    if (p[i] != 0 || p[i + 1] != 0) continue;
    size_t nal = 0;
    if (p[i + 2] == 1) {
      nal = i + 3;
    } else if (p[i + 2] == 0 && p[i + 3] == 1) {
      nal = i + 4;
    } else {
      continue;
    }
    if (nal >= n) break;
    const int type = (p[nal] >> 1) & 0x3F;
    // 16..23 = BLA_W_LP..CRA_NUT (every IRAP), 32 = VPS_NUT.
    if ((type >= 16 && type <= 23) || type == 32) return true;
  }
  return false;
}

}  // namespace

struct MppEncoder::Impl {
  MppCtx ctx = nullptr;
  MppApi* mpi = nullptr;
  MppEncCfg cfg = nullptr;
  MppBufferGroup grp = nullptr;

  // Output packet buffer, allocated when the geometry latches. One is
  // enough: encode() is a serialized submit-then-wait pair, so the packet is
  // consumed by the sink before the next frame can reuse the buffer.
  MppBuffer pkt_buf = nullptr;
  size_t pkt_buf_size = 0;

  // OSD double buffer (trap 3). osd_data[i].buf == osd_buf[i].
  MppBuffer osd_buf[2] = {nullptr, nullptr};
  MppEncOSDData osd_data[2];
  size_t osd_bytes = 0;
  int front = 0;
  bool osd_ready = false;    // a bitmap has been uploaded at least once
  bool palette_set = false;  // set_palette() succeeded => the OSD is wanted
  bool warned_no_palette = false;
  // Region geometry, fixed at init from EncCfg's picture size and never
  // recomputed: the picture size cannot change (encode() rejects any frame
  // that disagrees), so neither can the macroblock dimensions.
  int mb_w = 0, mb_h = 0;

  EncCfg ecfg;
  // Geometry programmed into the encoder, latched from the first submitted
  // frame. applied_* are meaningful only once geom_latched is true.
  bool geom_latched = false;
  // Raised by a latch VALIDATION refusal, which is permanent (see
  // latch_geometry): every later frame then fails fast without re-validating
  // and without logging again.
  bool geom_failed = false;
  int applied_w = 0, applied_h = 0, applied_stride = 0, applied_vstride = 0;
  uint64_t geom_rejects = 0;  // frames refused after a successful latch
  uint64_t latch_fails = 0;   // failed latch attempts on transient errors

  NalSink sink;
  std::vector<uint8_t> header;
  uint64_t frame_count = 0;
  uint64_t error_count = 0;

  Impl() { std::memset(osd_data, 0, sizeof(osd_data)); }

  ~Impl() {
    // Destroy order per bench/encosd: context first, then the buffers and
    // the group they came from.
    if (ctx) mpp_destroy(ctx);
    if (cfg) mpp_enc_cfg_deinit(cfg);
    for (MppBuffer& b : osd_buf) {
      if (b) mpp_buffer_put(b);
      b = nullptr;
    }
    if (pkt_buf) mpp_buffer_put(pkt_buf);
    if (grp) mpp_buffer_group_put(grp);
  }

  // First-frame geometry latch: prep:* + SET_CFG, the header mode, the
  // packet buffer, and the parameter sets -- committed to applied_*/
  // geom_latched only once every step has succeeded, so a failure leaves a
  // consistent state that the next frame retries from rather than a
  // half-applied one that is never revisited.
  //
  // Why this is a one-shot latch and NOT a reconfigure:
  //  - MPP_ENC_SET_CFG cannot report a rejected geometry. proc_prep_cfg()
  //    (mpp_enc_impl.c:566-591) silently restores the previous width/height/
  //    stride when width > hor_stride || height > ver_stride, logs, and is
  //    `static void` -- the control still returns MPP_OK. Believing it would
  //    run the OLD geometry against the NEW buffer: exactly the silent shear
  //    this guards against. The stride check below makes that branch
  //    unreachable instead of trusting the return value.
  //  - A mid-stream picture-size change is not something a running fMP4 mux
  //    can consume anyway (it needs a new track, not a reconfigure), so the
  //    honest answer to a disagreeing frame is a loud refusal.
  //
  // Two classes of failure, deliberately handled differently. A VALIDATION
  // refusal is PERMANENT -- a source whose picture size disagrees with
  // EncCfg, or whose strides are smaller than its picture, will not fix
  // itself on frame 2 -- so it raises geom_failed, logs once, and every
  // later frame fails fast. A TRANSIENT MPP failure (SET_CFG, header mode,
  // an allocation) keeps the retry, but its logging is rate-limited exactly
  // like the post-latch reject path: /tmp/maburplay.log is an unrotated `>`
  // redirect on tmpfs (bundle/S97maburplay), so an unbounded per-frame
  // fprintf at 60 fps is a real failure mode, not a cosmetic one.
  bool latch_geometry(int w, int h, int stride, int vstride) {
    if (w != ecfg.width || h != ecfg.height) {
      geom_failed = true;
      std::fprintf(stderr,
                   "MppEncoder: first frame %dx%d != configured %dx%d, refusing (permanent)\n", w,
                   h, ecfg.width, ecfg.height);
      return false;
    }
    if (stride <= 0 || vstride <= 0 || stride < w || vstride < h) {
      geom_failed = true;
      std::fprintf(stderr, "MppEncoder: bad stride %d/%d for %dx%d, refusing (permanent)\n",
                   stride, vstride, w, h);
      return false;
    }

    // Whether THIS attempt gets to speak if it fails: first one, then every
    // 300th (~5 s at 60 fps). The counter itself is never rate-limited, and
    // neither is errors() at the call site.
    const bool log = (latch_fails == 0) || (((latch_fails + 1) % 300) == 0);

    mpp_enc_cfg_set_s32(cfg, "prep:width", w);
    mpp_enc_cfg_set_s32(cfg, "prep:height", h);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", vstride);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
    MPP_RET ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret != MPP_OK) {
      ++latch_fails;
      if (log)
        std::fprintf(stderr,
                     "MppEncoder: MPP_ENC_SET_CFG (%dx%d stride %d/%d) failed ret=%d "
                     "(%llu latch attempts failed)\n",
                     w, h, stride, vstride, ret, (unsigned long long)latch_fails);
      return false;
    }

    // Parameter sets on every IDR, so a decoder joining an in-progress
    // recording resyncs. NOTE: this does NOT cover the FIRST IDR -- see
    // MppEncoder::header() for why, and for what the caller owes the mux.
    MppEncHeaderMode hm = MPP_ENC_HEADER_MODE_EACH_IDR;
    ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &hm);
    if (ret != MPP_OK) {
      ++latch_fails;
      if (log)
        std::fprintf(stderr,
                     "MppEncoder: MPP_ENC_SET_HEADER_MODE failed ret=%d "
                     "(%llu latch attempts failed)\n",
                     ret, (unsigned long long)latch_fails);
      return false;
    }

    if (!alloc_pkt_buf((size_t)stride * (size_t)vstride * 3 / 2, log)) {
      ++latch_fails;
      return false;
    }
    if (!fetch_header(log)) {
      ++latch_fails;
      return false;
    }

    applied_w = w;
    applied_h = h;
    applied_stride = stride;
    applied_vstride = vstride;
    geom_latched = true;
    std::fprintf(stderr, "MppEncoder: geometry latched %dx%d stride %d/%d\n", w, h, stride,
                 vstride);
    return true;
  }

  // `log` is the latch's rate-limit decision for this attempt; see
  // latch_geometry(). A failed get leaves pkt_buf null, so a retry re-gets
  // rather than overwriting a live handle.
  bool alloc_pkt_buf(size_t want, bool log) {
    if (pkt_buf && pkt_buf_size >= want) return true;
    const MPP_RET ret = mpp_buffer_get(grp, &pkt_buf, want);
    if (ret != MPP_OK || !pkt_buf) {
      if (log)
        std::fprintf(stderr, "MppEncoder: packet buffer %zu bytes failed ret=%d\n", want, ret);
      pkt_buf = nullptr;
      return false;
    }
    pkt_buf_size = want;
    return true;
  }

  // Allocates the two OSD bitmaps for the mb_w x mb_h region and builds both
  // MppEncOSDData structs around them. Called exactly once, from
  // set_palette(), whose failure path leaves palette_set false -- so a
  // partial failure here can never be reached by set_osd() or encode().
  // (Known minor: if the caller retries set_palette() after a partial
  // failure, the buffer already gotten is overwritten rather than put back
  // -- one leaked DRM buffer on a path that is already failing hard.)
  bool alloc_osd() {
    osd_ready = false;
    front = 0;
    std::memset(osd_data, 0, sizeof(osd_data));
    osd_bytes = (size_t)mb_w * (size_t)mb_h * 256;  // == stride(mb_w*16) * mb_h*16
    if (osd_bytes == 0) return false;

    for (int i = 0; i < 2; ++i) {
      const MPP_RET ret = mpp_buffer_get(grp, &osd_buf[i], osd_bytes);
      if (ret != MPP_OK || !osd_buf[i]) {
        std::fprintf(stderr, "MppEncoder: OSD buffer %zu bytes failed ret=%d\n", osd_bytes, ret);
        osd_buf[i] = nullptr;
        return false;
      }
      // Fresh DMA memory is not guaranteed zero, and a nonzero byte is an
      // opaque palette index, i.e. visible garbage burned into the picture.
      std::memset(mpp_buffer_get_ptr(osd_buf[i]), 0, osd_bytes);
      mpp_buffer_sync_end(osd_buf[i]);

      osd_data[i].buf = osd_buf[i];
      osd_data[i].num_region = 1;
      osd_data[i].region[0].enable = 1;
      osd_data[i].region[0].inverse = 0;
      osd_data[i].region[0].start_mb_x = 0;
      osd_data[i].region[0].start_mb_y = 0;
      osd_data[i].region[0].num_mb_x = (RK_U32)mb_w;
      osd_data[i].region[0].num_mb_y = (RK_U32)mb_h;
      osd_data[i].region[0].buf_offset = 0;
    }
    std::fprintf(stderr, "MppEncoder: OSD region %dx%d MB (%dx%d px), 2 x %zu bytes\n", mb_w,
                 mb_h, mb_w * kMbSize, mb_h * kMbSize, osd_bytes);
    return true;
  }

  // VPS/SPS/PPS out of the encoder into header. Uses pkt_buf, so it runs
  // inside the geometry latch, before any frame is in flight. `log` is that
  // latch attempt's rate-limit decision; see latch_geometry().
  bool fetch_header(bool log) {
    MppPacket hdr = nullptr;
    if (mpp_packet_init_with_buffer(&hdr, pkt_buf) != MPP_OK || !hdr) {
      if (log) std::fprintf(stderr, "MppEncoder: header packet init failed\n");
      return false;
    }
    mpp_packet_set_length(hdr, 0);
    const MPP_RET ret = mpi->control(ctx, MPP_ENC_GET_HDR_SYNC, hdr);
    if (ret != MPP_OK) {
      if (log) std::fprintf(stderr, "MppEncoder: MPP_ENC_GET_HDR_SYNC failed ret=%d\n", ret);
      mpp_packet_deinit(&hdr);
      return false;
    }
    const uint8_t* p = (const uint8_t*)mpp_packet_get_pos(hdr);
    const size_t n = mpp_packet_get_length(hdr);
    header.assign(p, p + n);
    mpp_packet_deinit(&hdr);
    if (header.empty()) {
      if (log) std::fprintf(stderr, "MppEncoder: MPP_ENC_GET_HDR_SYNC returned 0 bytes\n");
      return false;
    }
    std::fprintf(stderr, "MppEncoder: header %zu bytes\n", header.size());
    return true;
  }

  // MPP_PACKET_FLAG_INTRA is NOT in the installed public headers (it lives
  // in mpp/base/inc/mpp_packet_impl.h) and, on this userspace encoder path,
  // nothing sets it on the output packet anyway -- only the kmpp path does.
  // What mpp_enc_impl.c's set_enc_info_to_packet() DOES set, on the very
  // packet we handed in via KEY_OUTPUT_PACKET, is KEY_OUTPUT_INTRA. That is
  // the primary here; the Annex-B scan is the fallback if the key is absent.
  static bool is_keyframe(MppPacket pkt, const uint8_t* p, size_t n) {
    MppMeta meta = mpp_packet_get_meta(pkt);
    RK_S32 intra = 0;
    if (meta && mpp_meta_get_s32(meta, KEY_OUTPUT_INTRA, &intra) == MPP_OK) return intra != 0;
    return hevc_has_irap(p, n);
  }
};

MppEncoder::MppEncoder() = default;
MppEncoder::~MppEncoder() = default;

bool MppEncoder::init(const EncCfg& cfg, NalSink sink) {
  if (cfg.width <= 0 || cfg.height <= 0 || cfg.fps <= 0 || cfg.bitrate_kbps <= 0) {
    std::fprintf(stderr, "MppEncoder: bad EncCfg %dx%d@%d %dkbps\n", cfg.width, cfg.height,
                 cfg.fps, cfg.bitrate_kbps);
    return false;
  }

  impl_ = std::make_unique<Impl>();
  impl_->ecfg = cfg;
  impl_->sink = std::move(sink);
  // Full-picture region in macroblocks. ceil() on BOTH axes: 1080/16 is 67.5
  // and the 68th row is required (measured -- see the file comment).
  impl_->mb_w = (cfg.width + kMbSize - 1) / kMbSize;
  impl_->mb_h = (cfg.height + kMbSize - 1) / kMbSize;

  MPP_RET ret = mpp_create(&impl_->ctx, &impl_->mpi);
  if (ret != MPP_OK || !impl_->ctx || !impl_->mpi) {
    std::fprintf(stderr, "MppEncoder: mpp_create failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }
  ret = mpp_init(impl_->ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
  if (ret != MPP_OK) {
    std::fprintf(stderr, "MppEncoder: mpp_init failed ret=%d\n", ret);
    impl_.reset();  // Impl's dtor mpp_destroy()s the ctx
    return false;
  }

  ret = mpp_enc_cfg_init(&impl_->cfg);
  if (ret != MPP_OK || !impl_->cfg) {
    std::fprintf(stderr, "MppEncoder: mpp_enc_cfg_init failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }
  ret = impl_->mpi->control(impl_->ctx, MPP_ENC_GET_CFG, impl_->cfg);
  if (ret != MPP_OK) {
    std::fprintf(stderr, "MppEncoder: MPP_ENC_GET_CFG failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }

  // Everything geometry-independent is staged on the cfg object now; the
  // prep:* keys join it and the single MPP_ENC_SET_CFG fires when the first
  // frame latches the strides (Impl::latch_geometry).
  const int bps = cfg.bitrate_kbps * 1000;
  MppEncCfg c = impl_->cfg;
  mpp_enc_cfg_set_s32(c, "codec:type", MPP_VIDEO_CodingHEVC);
  mpp_enc_cfg_set_s32(c, "rc:mode", MPP_ENC_RC_MODE_CBR);
  mpp_enc_cfg_set_s32(c, "rc:bps_target", bps);
  mpp_enc_cfg_set_s32(c, "rc:bps_max", bps * 17 / 16);
  mpp_enc_cfg_set_s32(c, "rc:bps_min", bps * 15 / 16);
  mpp_enc_cfg_set_s32(c, "rc:fps_in_flex", 0);
  mpp_enc_cfg_set_s32(c, "rc:fps_in_num", cfg.fps);
  mpp_enc_cfg_set_s32(c, "rc:fps_in_denom", 1);
  mpp_enc_cfg_set_s32(c, "rc:fps_out_flex", 0);
  mpp_enc_cfg_set_s32(c, "rc:fps_out_num", cfg.fps);
  mpp_enc_cfg_set_s32(c, "rc:fps_out_denom", 1);
  mpp_enc_cfg_set_s32(c, "rc:gop", cfg.fps);  // one IDR per second
  mpp_enc_cfg_set_s32(c, "rc:qp_init", -1);
  mpp_enc_cfg_set_s32(c, "rc:qp_max", 51);
  mpp_enc_cfg_set_s32(c, "rc:qp_min", 10);
  mpp_enc_cfg_set_s32(c, "rc:qp_max_i", 51);
  mpp_enc_cfg_set_s32(c, "rc:qp_min_i", 10);
  mpp_enc_cfg_set_s32(c, "rc:qp_ip", 2);
  // Main profile, level 4.0 -- the MPP h265e defaults (h265e_api.c), spelled
  // out because the brief asks for them. MPP_PROFILE_HEVC_MAIN itself lives
  // in the private mpp/common/h265_syntax.h, so the literal is used.
  mpp_enc_cfg_set_s32(c, "h265:profile", 1);
  mpp_enc_cfg_set_s32(c, "h265:level", 120);  // covers 1080p60
  mpp_enc_cfg_set_s32(c, "h265:diff_cu_qp_delta_depth", 0);

  ret = mpp_buffer_group_get_internal(&impl_->grp,
                                      MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
  if (ret != MPP_OK || !impl_->grp) {
    std::fprintf(stderr, "MppEncoder: mpp_buffer_group_get_internal failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }

  std::fprintf(stderr,
               "MppEncoder: init %dx%d @%dfps CBR %dkbps gop %d (strides latch on frame 0)\n",
               cfg.width, cfg.height, cfg.fps, cfg.bitrate_kbps, cfg.fps);
  return true;
}

bool MppEncoder::set_palette(const OsdPalette& pal) {
  if (!impl_ || !impl_->ctx) return false;
  if (impl_->frame_count > 0)
    std::fprintf(stderr, "MppEncoder: set_palette() after %llu frames (expected before any)\n",
                 (unsigned long long)impl_->frame_count);

  // Trap 1: write ::val only. OsdPalette::entry is already
  // Y | U<<8 | V<<16 | A<<24; the union's {v,u,y,alpha} bitfield names are
  // wrong on little-endian and writing through them renders white pink.
  MppEncOSDPlt plt;
  std::memset(&plt, 0, sizeof(plt));
  for (int i = 0; i < 256; ++i) plt.data[i].val = pal.entry[i];

  MppEncOSDPltCfg plt_cfg;
  std::memset(&plt_cfg, 0, sizeof(plt_cfg));
  plt_cfg.change = MPP_ENC_OSD_PLT_CFG_CHANGE_ALL;
  plt_cfg.type = MPP_ENC_OSD_PLT_TYPE_USERDEF;
  plt_cfg.plt = &plt;  // memcpy'd into the encoder's own cfg by the control
  const MPP_RET ret = impl_->mpi->control(impl_->ctx, MPP_ENC_SET_OSD_PLT_CFG, &plt_cfg);
  if (ret != MPP_OK) {
    std::fprintf(stderr, "MppEncoder: MPP_ENC_SET_OSD_PLT_CFG failed ret=%d\n", ret);
    return false;
  }

  if (!impl_->palette_set) {
    // palette_set is raised only after the buffers exist: with it clear,
    // set_osd() refuses and encode() attaches no region, so a partially
    // allocated OSD is unreachable. (An uploaded palette with no region is
    // inert.)
    if (!impl_->alloc_osd()) return false;
    impl_->palette_set = true;
  }
  std::fprintf(stderr, "MppEncoder: palette installed (userdef, %d entries)\n", pal.n);
  return true;
}

void MppEncoder::set_osd(const OsdIndexMap& map) {
  if (!impl_ || !impl_->ctx) return;
  if (!impl_->palette_set) {
    if (!impl_->warned_no_palette) {
      impl_->warned_no_palette = true;
      std::fprintf(stderr, "MppEncoder: set_osd() without set_palette(); OSD stays off\n");
    }
    return;
  }
  // Guard rail: the HAL only WARNS about an undersized buffer and then
  // programs the register anyway, so a wrong-sized map would make the
  // encoder DMA past the end of ours. Check it here instead.
  if (map.mb_w != impl_->mb_w || map.mb_h != impl_->mb_h) {
    std::fprintf(stderr, "MppEncoder: OSD map %dx%d MB != region %dx%d MB, dropped\n", map.mb_w,
                 map.mb_h, impl_->mb_w, impl_->mb_h);
    return;
  }
  if (map.px.size() < impl_->osd_bytes) {
    std::fprintf(stderr, "MppEncoder: OSD map %zu bytes < region %zu bytes, dropped\n",
                 map.px.size(), impl_->osd_bytes);
    return;
  }

  // Trap 3: write the BACK buffer, never the one a frame may still be using,
  // then flip. Trap 2: map.px is already raster with stride mb_w*16, which
  // is what the hardware reads -- copy it verbatim, never re-tile.
  const int back = 1 - impl_->front;
  std::memcpy(mpp_buffer_get_ptr(impl_->osd_buf[back]), map.px.data(), impl_->osd_bytes);
  mpp_buffer_sync_end(impl_->osd_buf[back]);
  impl_->front = back;
  impl_->osd_ready = true;
}

bool MppEncoder::encode(void* nv12, int width, int height, int stride, int vstride,
                        uint64_t pts_us) {
  if (!impl_ || !impl_->ctx || !nv12) return false;
  Impl& im = *impl_;

  if (im.geom_failed) {
    // A latch validation refusal is permanent. Fail fast and silently -- the
    // one log line was emitted when it was diagnosed -- but keep counting,
    // so errors() stays a truthful monotone signal of frames not encoded.
    ++im.error_count;
    return false;
  }
  if (!im.geom_latched) {
    if (!im.latch_geometry(width, height, stride, vstride)) {
      ++im.error_count;
      return false;
    }
  } else if (width != im.applied_w || height != im.applied_h || stride != im.applied_stride ||
             vstride != im.applied_vstride) {
    // Hard reject, never a reconfigure (see Impl::latch_geometry). The log
    // is rate-limited: at 60 fps a persistent mismatch would flood it.
    ++im.geom_rejects;
    if (im.geom_rejects == 1 || im.geom_rejects % 300 == 0)
      std::fprintf(stderr,
                   "MppEncoder: frame %dx%d stride %d/%d != latched %dx%d stride %d/%d, "
                   "rejected (%llu so far)\n",
                   width, height, stride, vstride, im.applied_w, im.applied_h, im.applied_stride,
                   im.applied_vstride, (unsigned long long)im.geom_rejects);
    ++im.error_count;
    return false;
  }

  MppFrame frame = nullptr;
  if (mpp_frame_init(&frame) != MPP_OK || !frame) {
    std::fprintf(stderr, "MppEncoder: mpp_frame_init failed\n");
    ++im.error_count;
    return false;
  }
  mpp_frame_set_width(frame, width);
  mpp_frame_set_height(frame, height);
  mpp_frame_set_hor_stride(frame, stride);
  mpp_frame_set_ver_stride(frame, vstride);
  mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
  mpp_frame_set_eos(frame, 0);
  mpp_frame_set_pts(frame, (RK_S64)pts_us);
  // Borrowed, not owned: the caller holds its own reference across this call
  // (see the header). MppFrame deinit below drops only the frame wrapper.
  mpp_frame_set_buffer(frame, (MppBuffer)nv12);

  MppMeta meta = mpp_frame_get_meta(frame);
  MppPacket packet = nullptr;
  if (mpp_packet_init_with_buffer(&packet, im.pkt_buf) != MPP_OK || !packet) {
    std::fprintf(stderr, "MppEncoder: output packet init failed\n");
    mpp_frame_deinit(&frame);
    ++im.error_count;
    return false;
  }
  mpp_packet_set_length(packet, 0);
  mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);

  // Trap 4: every frame, or this one encodes with no OSD.
  if (im.palette_set && im.osd_ready)
    mpp_meta_set_ptr(meta, KEY_OSD_DATA, (void*)&im.osd_data[im.front]);

  const MPP_RET pr = im.mpi->encode_put_frame(im.ctx, frame);
  mpp_frame_deinit(&frame);
  if (pr != MPP_OK) {
    std::fprintf(stderr, "MppEncoder: encode_put_frame failed ret=%d\n", pr);
    mpp_packet_deinit(&packet);
    ++im.error_count;
    return false;
  }

  // One packet per frame, so no drain loop: the encoder writes into the
  // buffer we supplied via KEY_OUTPUT_PACKET and hands that same MppPacket
  // back, and the spike measured an exact 1:1 (1200 frames -> 1200 packets,
  // parameter sets folded into the IDR packets by EACH_IDR).
  MppPacket outp = nullptr;
  const MPP_RET gr = im.mpi->encode_get_packet(im.ctx, &outp);
  if (gr != MPP_OK || !outp) {
    std::fprintf(stderr, "MppEncoder: encode_get_packet failed ret=%d\n", gr);
    mpp_packet_deinit(&packet);
    ++im.error_count;
    return false;
  }

  const uint8_t* data = (const uint8_t*)mpp_packet_get_pos(outp);
  const size_t len = mpp_packet_get_length(outp);
  if (im.sink && data && len) im.sink(data, len, pts_us, Impl::is_keyframe(outp, data, len));
  ++im.frame_count;
  mpp_packet_deinit(&outp);  // same object as `packet`
  return true;
}

void MppEncoder::request_idr() {
  if (!impl_ || !impl_->ctx) return;
  const MPP_RET ret = impl_->mpi->control(impl_->ctx, MPP_ENC_SET_IDR_FRAME, nullptr);
  if (ret != MPP_OK)
    std::fprintf(stderr, "MppEncoder: MPP_ENC_SET_IDR_FRAME failed ret=%d\n", ret);
}

const std::vector<uint8_t>& MppEncoder::header() const {
  static const std::vector<uint8_t> kEmpty;
  return impl_ ? impl_->header : kEmpty;
}

uint64_t MppEncoder::frames() const { return impl_ ? impl_->frame_count : 0; }
uint64_t MppEncoder::errors() const { return impl_ ? impl_->error_count : 0; }

}  // namespace maburplay
