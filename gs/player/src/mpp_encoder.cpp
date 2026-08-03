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
// keyframe-detection fallback (see is_keyframe()). Cheap -- it stops at the
// first hit, which for a keyframe is the very first NAL (header mode
// EACH_IDR prepends VPS/SPS/PPS to every IDR).
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

  // Output packet buffer. One is enough: encode() is a serialized
  // submit-then-wait pair, so the packet is consumed by the sink before the
  // next frame can reuse the buffer.
  MppBuffer pkt_buf = nullptr;
  size_t pkt_buf_size = 0;

  // OSD double buffer (trap 3). osd_data[i].buf == osd_buf[i].
  MppBuffer osd_buf[2] = {nullptr, nullptr};
  MppEncOSDData osd_data[2];
  size_t osd_bytes = 0;
  int front = 0;
  bool osd_ready = false;   // a bitmap has been uploaded at least once
  bool palette_set = false; // set_palette() succeeded => the OSD is wanted
  bool warned_no_palette = false;
  int mb_w = 0, mb_h = 0;

  EncCfg ecfg;
  // Geometry currently programmed into the encoder. encode() reconfigures if
  // a submitted frame disagrees (the decoder's real stride is only known
  // once frames flow).
  int applied_w = 0, applied_h = 0, applied_stride = 0, applied_vstride = 0;

  NalSink sink;
  std::vector<uint8_t> header;
  uint64_t frame_count = 0;
  uint64_t error_count = 0;

  Impl() {
    std::memset(osd_data, 0, sizeof(osd_data));
  }

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

  // Programs prep:* for this geometry and pushes it. Also (re)sizes the
  // output packet buffer and, when the OSD is on, the region + its bitmaps.
  bool apply_geometry(int w, int h, int stride, int vstride) {
    mpp_enc_cfg_set_s32(cfg, "prep:width", w);
    mpp_enc_cfg_set_s32(cfg, "prep:height", h);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", vstride);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
    const MPP_RET ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret != MPP_OK) {
      std::fprintf(stderr, "MppEncoder: MPP_ENC_SET_CFG (geometry %dx%d stride %d/%d) failed ret=%d\n",
                   w, h, stride, vstride, ret);
      return false;
    }
    applied_w = w;
    applied_h = h;
    applied_stride = stride;
    applied_vstride = vstride;

    if (!ensure_pkt_buf((size_t)stride * (size_t)vstride * 3 / 2)) return false;

    // Full-picture region, in macroblocks. ceil() on BOTH axes: 1080/16 is
    // 67.5 and the 68th row is required (measured -- see the file comment).
    const int new_mb_w = (w + kMbSize - 1) / kMbSize;
    const int new_mb_h = (h + kMbSize - 1) / kMbSize;
    if (palette_set && (new_mb_w != mb_w || new_mb_h != mb_h)) {
      mb_w = new_mb_w;
      mb_h = new_mb_h;
      if (!alloc_osd()) return false;
    } else {
      mb_w = new_mb_w;
      mb_h = new_mb_h;
    }
    return true;
  }

  bool ensure_pkt_buf(size_t want) {
    if (pkt_buf && pkt_buf_size >= want) return true;
    if (pkt_buf) {
      mpp_buffer_put(pkt_buf);
      pkt_buf = nullptr;
      pkt_buf_size = 0;
    }
    const MPP_RET ret = mpp_buffer_get(grp, &pkt_buf, want);
    if (ret != MPP_OK || !pkt_buf) {
      std::fprintf(stderr, "MppEncoder: packet buffer %zu bytes failed ret=%d\n", want, ret);
      pkt_buf = nullptr;
      return false;
    }
    pkt_buf_size = want;
    return true;
  }

  // Allocates/reallocates the two OSD bitmaps for the current mb_w x mb_h
  // region and rebuilds both MppEncOSDData structs around them.
  bool alloc_osd() {
    for (MppBuffer& b : osd_buf) {
      if (b) mpp_buffer_put(b);
      b = nullptr;
    }
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

  // VPS/SPS/PPS out of the encoder into header. Uses pkt_buf, so it must run
  // when no frame is in flight (init, and after a reconfigure).
  bool fetch_header() {
    MppPacket hdr = nullptr;
    if (mpp_packet_init_with_buffer(&hdr, pkt_buf) != MPP_OK || !hdr) {
      std::fprintf(stderr, "MppEncoder: header packet init failed\n");
      return false;
    }
    mpp_packet_set_length(hdr, 0);
    const MPP_RET ret = mpi->control(ctx, MPP_ENC_GET_HDR_SYNC, hdr);
    if (ret != MPP_OK) {
      std::fprintf(stderr, "MppEncoder: MPP_ENC_GET_HDR_SYNC failed ret=%d\n", ret);
      mpp_packet_deinit(&hdr);
      return false;
    }
    const uint8_t* p = (const uint8_t*)mpp_packet_get_pos(hdr);
    const size_t n = mpp_packet_get_length(hdr);
    header.assign(p, p + n);
    mpp_packet_deinit(&hdr);
    std::fprintf(stderr, "MppEncoder: header %zu bytes\n", header.size());
    return !header.empty();
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

  // prep:* + SET_CFG. Strides start at the 16-aligned defaults; the first
  // encode() re-applies them if the real frames disagree.
  const int stride = (cfg.width + 15) & ~15;
  const int vstride = (cfg.height + 15) & ~15;

  // The buffer group has to exist before apply_geometry(), which sizes the
  // output packet buffer out of it. DRM|CACHABLE per the spike.
  ret = mpp_buffer_group_get_internal(&impl_->grp,
                                      MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
  if (ret != MPP_OK || !impl_->grp) {
    std::fprintf(stderr, "MppEncoder: mpp_buffer_group_get_internal failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }

  if (!impl_->apply_geometry(cfg.width, cfg.height, stride, vstride)) {
    impl_.reset();
    return false;
  }

  // Header on every IDR, so a recording that starts (or a decoder that
  // joins) mid-stream has parameter sets in band.
  MppEncHeaderMode hm = MPP_ENC_HEADER_MODE_EACH_IDR;
  ret = impl_->mpi->control(impl_->ctx, MPP_ENC_SET_HEADER_MODE, &hm);
  if (ret != MPP_OK) {
    std::fprintf(stderr, "MppEncoder: MPP_ENC_SET_HEADER_MODE failed ret=%d\n", ret);
    impl_.reset();
    return false;
  }

  if (!impl_->fetch_header()) {
    impl_.reset();
    return false;
  }

  std::fprintf(stderr, "MppEncoder: init %dx%d stride %d/%d @%dfps CBR %dkbps gop %d\n",
               cfg.width, cfg.height, stride, vstride, cfg.fps, cfg.bitrate_kbps, cfg.fps);
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
    impl_->palette_set = true;
    if (!impl_->alloc_osd()) {
      impl_->palette_set = false;
      return false;
    }
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

  if (width != im.applied_w || height != im.applied_h || stride != im.applied_stride ||
      vstride != im.applied_vstride) {
    std::fprintf(stderr,
                 "MppEncoder: geometry %dx%d stride %d/%d -> %dx%d stride %d/%d, reconfiguring\n",
                 im.applied_w, im.applied_h, im.applied_stride, im.applied_vstride, width, height,
                 stride, vstride);
    if (!im.apply_geometry(width, height, stride, vstride) || !im.fetch_header()) {
      ++im.error_count;
      return false;
    }
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

  // One packet per frame: the encoder writes into the buffer we supplied via
  // KEY_OUTPUT_PACKET and hands that same MppPacket back, and the spike
  // measured an exact 1:1 (1200 frames -> 1200 packets, header folded into
  // the IDR packets by EACH_IDR). A second blocking encode_get_packet() here
  // would have nothing to wait on, so this deliberately does not loop.
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
