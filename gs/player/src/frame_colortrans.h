#ifndef MABUR_PLAYER_FRAME_COLORTRANS_H_
#define MABUR_PLAYER_FRAME_COLORTRANS_H_

#include <cstdint>
#include <string>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <gbm.h>

namespace maburplay {

// GPU colortrans for the burned DVR (docs/colortrans.md), ported from
// PixelPilot_rk's FrameColorCorrect with the shader replaced by
// colortrans3.glsl (the same chain colortrans.cpp evaluates on the CPU).
//
// Per frame: import the decoder's NV12 dmabuf as an EGLImage
// (samplerExternalOES; the driver does YUV->RGB), draw a full-screen quad
// through the shader into an ARGB8888 GBM render target, glFinish(), then
// RGA converts BGRA -> NV12 into the destination dmabuf. glFinish rather
// than glFlush: panfrost does not reliably signal the dmabuf fence before
// glFlush returns and RGA would read a half-written target (PixelPilot).
//
// THREAD-BOUND: init(), process() and deinit() on ONE thread (the EGL
// context is current on it). BurnRecorder runs all three on its recorder
// thread. Same-size only: dst is w x h with its own strides.
class FrameColorTrans {
 public:
  FrameColorTrans() = default;
  ~FrameColorTrans();
  FrameColorTrans(const FrameColorTrans&) = delete;
  FrameColorTrans& operator=(const FrameColorTrans&) = delete;

  // Opens its own fd on /dev/dri/card0 (Mesa's kmsro routes rendering to
  // panfrost; PixelPilot's proven route), creates GBM + EGL + the program +
  // two render targets. false = logged, stage unusable.
  bool init(uint32_t width, uint32_t height);
  void deinit();
  bool ready() const { return ready_; }

  // src: NV12, two planes in one dmabuf, UV at hs*vs. dst: NV12 same size.
  // false = logged (rate-limited), caller encodes the source flat.
  bool process(int src_fd, uint32_t w, uint32_t h, uint32_t hs, uint32_t vs, int dst_fd,
               uint32_t dst_hs, uint32_t dst_vs);

  // Cumulative per-step time of successful process() calls, microseconds:
  // import (EGLImage + texture), draw (draw + glFinish), rga (BGRA -> NV12).
  struct Breakdown {
    uint64_t n = 0, import_us = 0, draw_us = 0, rga_us = 0;
  };
  const Breakdown& breakdown() const { return bd_; }

 private:
  bool build_program();
  bool create_targets();
  void destroy_targets();

  // GPU devfreq governor pinned to "performance" for the stage's lifetime
  // (see gpu_boost_begin in the .cpp); the previous governor is restored at
  // deinit(). Empty = nothing to restore.
  std::string gpu_gov_path_;
  std::string gpu_gov_saved_;
  void gpu_boost_begin();
  void gpu_boost_end();

  int drm_fd_ = -1;
  uint32_t width_ = 0, height_ = 0;
  bool ready_ = false;
  uint64_t fail_logs_ = 0;
  Breakdown bd_;

  gbm_device* gbm_ = nullptr;
  EGLDisplay dpy_ = EGL_NO_DISPLAY;
  EGLContext ctx_ = EGL_NO_CONTEXT;
  EGLSurface surf_ = EGL_NO_SURFACE;
  EGLConfig cfg_ = nullptr;
  PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR_ = nullptr;
  PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR_ = nullptr;
  PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES_ = nullptr;

  GLuint prog_ = 0;
  GLint loc_tex_ = -1;

  // Source import cache: the decoder pool is a fixed set of dma-bufs (24 at
  // 1080p), and importing one as an EGLImage + external texture costs ~2.7
  // ms per frame plus the destroy -- a fifth of the stage's budget at 60
  // fps. Keyed by (fd, dma-buf inode): MPP holds each pool buffer's fd for
  // the pool's life, so a reused fd number means the old buffer is gone and
  // its entry is replaced; a different inode on the same fd can never be
  // served the old image. Bounded by the number of live fds. The EGLImage
  // keeps its dma-buf alive, so stale entries are exactly what the fd-reuse
  // rule evicts.
  struct SrcEntry {
    int fd = -1;
    unsigned long ino = 0;
    EGLImageKHR img = EGL_NO_IMAGE_KHR;
    GLuint tex = 0;
  };
  static constexpr int kMaxSrc = 48;
  SrcEntry src_[kMaxSrc];
  int n_src_ = 0;
  GLuint src_texture(int fd, uint32_t w, uint32_t h, uint32_t hs, uint32_t vs);
  void drop_src_entry(SrcEntry& e);
  void destroy_src_cache();

  static constexpr int kTargets = 2;
  struct Target {
    gbm_bo* bo = nullptr;
    EGLImageKHR img = EGL_NO_IMAGE_KHR;
    GLuint tex = 0, fbo = 0;
    int prime_fd = -1;
    int stride_px = 0;
  };
  Target targets_[kTargets];
  int target_idx_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_FRAME_COLORTRANS_H_
