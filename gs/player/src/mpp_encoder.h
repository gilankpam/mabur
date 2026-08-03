#ifndef MABUR_PLAYER_MPP_ENCODER_H_
#define MABUR_PLAYER_MPP_ENCODER_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "osd_palette.h"  // OsdPalette, OsdIndexMap (host-buildable, no SDK)

namespace maburplay {

// Encoder settings. Strides are NOT part of this: they come from the frames
// actually submitted (the decoder's own NV12 geometry) and are applied on
// the first encode(), see MppEncoder::encode().
struct EncCfg {
  int width = 0;
  int height = 0;
  int fps = 60;
  int bitrate_kbps = 8000;
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
  // Geometry is NOT applied here. EncCfg carries no strides and only the
  // frames themselves know the decoder's real ones, so prep:*, the packet
  // buffer and the parameter sets are deferred to the first encode(), which
  // latches that frame's strides permanently. header() is therefore empty
  // until the first successful encode().
  bool init(const EncCfg& cfg, NalSink sink);

  // Uploads the palette (MPP_ENC_OSD_PLT_TYPE_USERDEF) and allocates the two
  // index-bitmap buffers sized for a full-picture region. This is what turns
  // the OSD on -- if it is never called the encoder runs with NO OSD region
  // at all (a legal burned-DVR configuration: recording with the OSD off).
  // Call once, before the first encode().
  bool set_palette(const OsdPalette& pal);

  // Copies `map` into the back MppBuffer and flips it to the front, so the
  // next encode() attaches it. The map's macroblock dimensions must match
  // the region (ceil(w/16) x ceil(h/16)); a mismatch is rejected and logged
  // rather than submitted -- the HAL's own size check is a WARNING, and an
  // undersized buffer makes the encoder DMA past its end.
  // No-op (with a one-shot warning) if set_palette() was never called.
  void set_osd(const OsdIndexMap& map);

  // Wraps the caller's NV12 buffer (an MppBuffer; see the class comment) in
  // an MppFrame, attaches the current OSD, submits it, and delivers the
  // resulting packet to the sink before returning.
  //
  // Does NOT take ownership: the caller must hold its reference across the
  // call and release it afterwards. Task 4 hands the decoder's own buffer
  // straight in, so there is no copy anywhere in the path.
  //
  // The FIRST call latches the geometry (it must match EncCfg's picture size
  // and carry strides >= it). Every later frame must present exactly the
  // same width/height/stride/vstride: a mismatch is REJECTED -- logged
  // (rate-limited), counted in errors(), false returned -- and never
  // reconfigured. A mid-stream resolution change needs a new track in the
  // mux, not a reconfigured encoder, so the caller must handle it as such.
  bool encode(void* nv12, int width, int height, int stride, int vstride, uint64_t pts_us);

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
