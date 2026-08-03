#ifndef MABUR_PLAYER_MPP_ENCODER_H_
#define MABUR_PLAYER_MPP_ENCODER_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "osd_palette.h"  // OsdPalette, OsdIndexMap (host-buildable, no SDK)

namespace maburplay {

// Encoder settings.
//
// The PICTURE SIZE is deliberately not here. It, and the strides, come from
// the frames actually submitted -- the decoder's own NV12 geometry -- and
// are applied on the first encode(); see MppEncoder::encode(). Config has no
// way to know it: the decoded size comes from the bitstream, and the one
// thing it is NOT is the display mode.
struct EncCfg {
  int fps = 60;
  int bitrate_kbps = 8000;
  // OSD region size in PIXELS, which must be the size of the SURFACE the
  // index maps are quantized from -- the region is ceil(w/16) x ceil(h/16)
  // macroblocks and set_osd() rejects any map that disagrees. It is
  // unrelated to the picture size: a region may legally overhang the
  // picture (the hardware clips) and may be smaller than it. 0 disables the
  // OSD -- set_palette() then refuses, since there is nothing to size the
  // buffers from.
  int osd_width = 0;
  int osd_height = 0;
};

// Called once per coded picture, from inside encode(), on the caller's
// thread. `p` points into the encoder's own output buffer and is only valid
// for the duration of the call -- copy or write it before returning.
using NalSink =
    std::function<void(const uint8_t* p, size_t n, uint64_t pts_us, bool keyframe)>;

// Rockchip MPP H.265 encoder with the hardware (vepu540) OSD attached: the
// burned-DVR half of Phase 2. Cross build only (MABUR_PLAYER_HW).
//
// Proven call sequence and every trap below come from the spike at
// docs/superpowers/specs/2026-08-04-mpp-enc-osd-spike.md (bench GS, RK3566,
// 1200 frames / 0 errors / 73.3 fps concurrent with the live decoder); its
// working reference is bench/encosd/encosd_main.cpp.
//
// Usage:
//   init(cfg, sink)                        -- fails loudly, never half-inits
//   set_palette(pal)                       -- ONCE, before frames; also the
//                                             OSD's on switch (never called
//                                             => no OSD region at all)
//   set_osd(map) / encode(...) per frame   -- both from the SAME thread
//
// Threading: set_osd() and encode() must be called from one thread (Task 4's
// recorder thread). The double buffering exists because the ENCODER thread
// reads the bitmap after encode_put_frame() returns, not because these two
// may be called concurrently.
//
// Deliberately free of any <rockchip/...> include here -- same rationale as
// mpp_backend.h / drm_presenter.h. All MPP state lives behind the Impl pimpl
// in mpp_encoder.cpp. `nv12` in encode() is spelled void* for exactly that
// reason: rk_type.h has `typedef void* MppBuffer`, so an MppBuffer passes
// through unchanged with no cast at the call site and no SDK dependency here.
class MppEncoder {
 public:
  MppEncoder();
  ~MppEncoder();
  MppEncoder(const MppEncoder&) = delete;
  MppEncoder& operator=(const MppEncoder&) = delete;

  // Creates the encoder and stages cfg. Any MPP error is fatal: logs, tears
  // the context back down, returns false.
  //
  // Geometry is NOT applied here. EncCfg carries neither the picture size
  // nor the strides and only the frames themselves know them, so prep:*, the
  // packet buffer and the parameter sets are deferred to the first encode(),
  // which latches that frame's geometry permanently. header() is therefore
  // empty until the first successful encode().
  bool init(const EncCfg& cfg, NalSink sink);

  // Uploads the palette (MPP_ENC_OSD_PLT_TYPE_USERDEF) and allocates the two
  // index-bitmap buffers sized for EncCfg's osd_width/osd_height. This is
  // what turns the OSD on -- if it is never called the encoder runs with NO
  // OSD region at all (a legal burned-DVR configuration: recording with the
  // OSD off). Refuses if the region size is 0. Call once, before the first
  // encode().
  bool set_palette(const OsdPalette& pal);

  // Copies `map` into the back MppBuffer and flips it to the front, so the
  // next encode() attaches it. Returns false, having installed nothing, if
  // the map's macroblock dimensions do not match the region
  // (ceil(osd_width/16) x ceil(osd_height/16)) -- the HAL's own size check
  // is a WARNING and an undersized buffer makes the encoder DMA past its
  // end, so this one is real. The log for that is once-only plus every
  // 300th: a persistent mismatch is a legal-looking configuration (a clean
  // transcode with no OSD), so the caller is expected to surface the count,
  // not to read the log. Also false (with a one-shot warning) if
  // set_palette() was never called.
  bool set_osd(const OsdIndexMap& map);

  // Wraps the caller's NV12 buffer (an MppBuffer; see the class comment) in
  // an MppFrame, attaches the current OSD, submits it, and delivers the
  // resulting packet to the sink before returning.
  //
  // Does NOT take ownership: the caller must hold its reference across the
  // call and release it afterwards. Task 4 hands the decoder's own buffer
  // straight in, so there is no copy anywhere in the path.
  //
  // The FIRST call latches the geometry: this frame's picture size IS the
  // encoder's, whatever it is (it only has to be sane and carry strides >=
  // it). Every later frame must present exactly the same
  // width/height/stride/vstride: a mismatch is REJECTED -- logged
  // (rate-limited), counted in errors(), false returned -- and never
  // reconfigured. A mid-stream resolution change needs a new track in the
  // mux, not a reconfigured encoder, so the caller must handle it as such.
  //
  // If the FIRST frame fails validation (a non-positive dimension, or a
  // stride smaller than the picture) the refusal is permanent: it is logged
  // once and every later call fails fast. errors() keeps incrementing either
  // way, so a recorder that never encodes anything shows errors() climbing
  // with frames() stuck at 0.
  bool encode(void* nv12, int width, int height, int stride, int vstride, uint64_t pts_us);

  // The latched picture size, or 0x0 before the first successful encode().
  // The mux's track header wants this, not anything from config.
  void picture_size(int* w, int* h) const;

  // Makes the next submitted frame an IDR (MPP_ENC_SET_IDR_FRAME).
  void request_idr();

  // VPS+SPS+PPS (MPP_ENC_GET_HDR_SYNC), fetched when the first encode()
  // latches the geometry. Empty before that, and if init() failed.
  //
  // THE CALLER MUST CARRY THIS OUT OF BAND. The stream's first IDR does NOT
  // contain parameter sets, despite MPP_ENC_HEADER_MODE_EACH_IDR:
  // MPP_ENC_GET_HDR_SYNC sets hdr_status.added_by_ctrl (mpp_enc_impl.c:1367)
  // and mpp_enc_add_sw_header skips while HDR_ADDED_MASK is set (:2023-2041),
  // and those flags are only cleared by reset_enc_task at the END of the
  // first task -- so frame 0's IDR is header-less and only IDRs from GOP 2
  // onward carry them. Consequence for Task 4: the fMP4 sample entry
  // (hvcC) must be built from header(); a bare Annex-B dump must have it
  // prepended, or nothing can decode the first second.
  const std::vector<uint8_t>& header() const;

  // Diagnostics.
  uint64_t frames() const;  // packets delivered to the sink
  uint64_t errors() const;  // frames dropped on an MPP error

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MPP_ENCODER_H_
