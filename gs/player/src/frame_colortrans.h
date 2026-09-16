#ifndef MABUR_PLAYER_FRAME_COLORTRANS_H_
#define MABUR_PLAYER_FRAME_COLORTRANS_H_

#include <cstdint>

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

 private:
  bool build_program();
  bool create_targets();
  void destroy_targets();

  int drm_fd_ = -1;
  uint32_t width_ = 0, height_ = 0;
  bool ready_ = false;
  uint64_t fail_logs_ = 0;

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
