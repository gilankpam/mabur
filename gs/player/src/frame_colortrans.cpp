#include "frame_colortrans.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <drm_fourcc.h>
#include <rga/im2d.h>
#include <rga/rga.h>

namespace maburplay {

namespace {

const char* kVert = R"GLSL(
attribute vec2 a_pos;
attribute vec2 a_uv;
varying vec2 v_uv;
void main() { v_uv = a_uv; gl_Position = vec4(a_pos, 0.0, 1.0); }
)GLSL";

// colortrans3.glsl, hooks removed, constants baked. MUST match kColorTrans3
// in colortrans.cpp -- test_colortrans pins the C++ side to the same
// reference values, so retune both together. GLSL mat3 is COLUMN-major:
// the three vec3s below are the matrix's columns.
const char* kFrag = R"GLSL(
#extension GL_OES_EGL_image_external : require
precision highp float;
varying vec2 v_uv;
uniform samplerExternalOES tex;
const float kYoff = 0.04887585532746823;   // 200/1023 * 0.25
const float kBlackLift = 0.020;
const mat3 kM = mat3(1.17866031, -0.11147506, 0.03243649,
                     0.17460893,  1.55408099, 0.11275820,
                     0.01571472, -0.07362197, 1.22378927);
const float kGamma = 1.0;
const float kLift = -0.15;
const float kGain = 1.75;
const vec3  kMult = vec3(1.0, 1.0, 1.0);
const float kSat = 0.775;                  // 1 + (-22.5 / 100)
void main() {
  vec3 c = texture2D(tex, v_uv).rgb;
  vec3 y = kM * (c - vec3(kYoff)) + vec3(kBlackLift);
  c = clamp(y, 0.0, 1.0);
  y = pow(c, vec3(kGamma));
  y += vec3(kLift);
  y *= kGain;
  y *= kMult;
  float l = dot(y, vec3(0.2126, 0.7152, 0.0722));
  y = mix(vec3(l), y, kSat);
  gl_FragColor = vec4(clamp(y, 0.0, 1.0), 1.0);
}
)GLSL";

GLuint compile(GLenum type, const char* src) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = GL_FALSE;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (ok != GL_TRUE) {
    char log[512];
    glGetShaderInfoLog(s, sizeof(log), nullptr, log);
    std::fprintf(stderr, "FrameColorTrans: shader compile error: %s\n", log);
    glDeleteShader(s);
    return 0;
  }
  return s;
}

}  // namespace

FrameColorTrans::~FrameColorTrans() { deinit(); }

bool FrameColorTrans::init(uint32_t width, uint32_t height) {
  deinit();
  width_ = width;
  height_ = height;
  drm_fd_ = ::open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
  if (drm_fd_ < 0) {
    std::fprintf(stderr, "FrameColorTrans: open /dev/dri/card0: %s\n", std::strerror(errno));
    return false;
  }
  gbm_ = gbm_create_device(drm_fd_);
  if (!gbm_) {
    std::fprintf(stderr, "FrameColorTrans: gbm_create_device failed\n");
    return false;
  }
  eglCreateImageKHR_ = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
  eglDestroyImageKHR_ = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
  glEGLImageTargetTexture2DOES_ =
      (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
  auto getPlatformDisplay =
      (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
  if (!eglCreateImageKHR_ || !eglDestroyImageKHR_ || !glEGLImageTargetTexture2DOES_) {
    std::fprintf(stderr, "FrameColorTrans: EGL image extensions missing\n");
    return false;
  }
  dpy_ = getPlatformDisplay ? getPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm_, nullptr)
                            : eglGetDisplay((EGLNativeDisplayType)gbm_);
  if (dpy_ == EGL_NO_DISPLAY || !eglInitialize(dpy_, nullptr, nullptr) ||
      !eglBindAPI(EGL_OPENGL_ES_API)) {
    std::fprintf(stderr, "FrameColorTrans: EGL display init failed (0x%x)\n", eglGetError());
    return false;
  }
  const EGLint cfg_attrs[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_SURFACE_TYPE,
                              EGL_PBUFFER_BIT,     EGL_RED_SIZE,        8,
                              EGL_GREEN_SIZE,      8,                   EGL_BLUE_SIZE,
                              8,                   EGL_NONE};
  EGLint n = 0;
  if (!eglChooseConfig(dpy_, cfg_attrs, &cfg_, 1, &n) || n == 0) {
    const EGLint relaxed[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE};
    if (!eglChooseConfig(dpy_, relaxed, &cfg_, 1, &n) || n == 0) {
      std::fprintf(stderr, "FrameColorTrans: eglChooseConfig failed\n");
      return false;
    }
  }
  const EGLint ctx_attrs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
  ctx_ = eglCreateContext(dpy_, cfg_, EGL_NO_CONTEXT, ctx_attrs);
  const EGLint pb_attrs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
  surf_ = eglCreatePbufferSurface(dpy_, cfg_, pb_attrs);
  if (ctx_ == EGL_NO_CONTEXT || !eglMakeCurrent(dpy_, surf_, surf_, ctx_)) {
    std::fprintf(stderr, "FrameColorTrans: context/makeCurrent failed (0x%x)\n", eglGetError());
    return false;
  }
  if (!build_program() || !create_targets()) return false;
  ready_ = true;
  std::fprintf(stderr, "FrameColorTrans: ready %ux%u (%s)\n", width_, height_,
               glGetString(GL_RENDERER) ? (const char*)glGetString(GL_RENDERER) : "?");
  return true;
}

bool FrameColorTrans::build_program() {
  GLuint vs = compile(GL_VERTEX_SHADER, kVert);
  GLuint fs = compile(GL_FRAGMENT_SHADER, kFrag);
  if (!vs || !fs) {
    glDeleteShader(vs);
    glDeleteShader(fs);
    return false;
  }
  prog_ = glCreateProgram();
  glAttachShader(prog_, vs);
  glAttachShader(prog_, fs);
  glBindAttribLocation(prog_, 0, "a_pos");
  glBindAttribLocation(prog_, 1, "a_uv");
  glLinkProgram(prog_);
  glDeleteShader(vs);
  glDeleteShader(fs);
  GLint ok = GL_FALSE;
  glGetProgramiv(prog_, GL_LINK_STATUS, &ok);
  if (ok != GL_TRUE) {
    char log[512];
    glGetProgramInfoLog(prog_, sizeof(log), nullptr, log);
    std::fprintf(stderr, "FrameColorTrans: link error: %s\n", log);
    glDeleteProgram(prog_);
    prog_ = 0;
    return false;
  }
  loc_tex_ = glGetUniformLocation(prog_, "tex");
  return true;
}

bool FrameColorTrans::create_targets() {
  for (int i = 0; i < kTargets; ++i) {
    Target& t = targets_[i];
    t.bo = gbm_bo_create(gbm_, width_, height_, GBM_FORMAT_ARGB8888,
                         GBM_BO_USE_RENDERING | GBM_BO_USE_SCANOUT);
    if (!t.bo) {
      std::fprintf(stderr, "FrameColorTrans: gbm_bo_create failed\n");
      return false;
    }
    t.prime_fd = gbm_bo_get_fd(t.bo);
    if (t.prime_fd < 0) return false;
    t.stride_px = (int)(gbm_bo_get_stride(t.bo) / 4);
    const EGLint attrs[] = {EGL_WIDTH,
                            (EGLint)width_,
                            EGL_HEIGHT,
                            (EGLint)height_,
                            EGL_LINUX_DRM_FOURCC_EXT,
                            (EGLint)DRM_FORMAT_ARGB8888,
                            EGL_DMA_BUF_PLANE0_FD_EXT,
                            t.prime_fd,
                            EGL_DMA_BUF_PLANE0_OFFSET_EXT,
                            0,
                            EGL_DMA_BUF_PLANE0_PITCH_EXT,
                            (EGLint)gbm_bo_get_stride(t.bo),
                            EGL_NONE};
    t.img = eglCreateImageKHR_(dpy_, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attrs);
    if (t.img == EGL_NO_IMAGE_KHR) {
      std::fprintf(stderr, "FrameColorTrans: target EGLImage failed (0x%x)\n", eglGetError());
      return false;
    }
    glGenTextures(1, &t.tex);
    glBindTexture(GL_TEXTURE_2D, t.tex);
    glEGLImageTargetTexture2DOES_(GL_TEXTURE_2D, t.img);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glGenFramebuffers(1, &t.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t.tex, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      std::fprintf(stderr, "FrameColorTrans: framebuffer %d incomplete\n", i);
      return false;
    }
  }
  return true;
}

bool FrameColorTrans::process(int src_fd, uint32_t w, uint32_t h, uint32_t hs, uint32_t vs,
                              int dst_fd, uint32_t dst_hs, uint32_t dst_vs) {
  if (!ready_ || w != width_ || h != height_) return false;
  const EGLint uv_off = (EGLint)((size_t)hs * vs);
  const EGLint attrs[] = {EGL_WIDTH,
                          (EGLint)w,
                          EGL_HEIGHT,
                          (EGLint)h,
                          EGL_LINUX_DRM_FOURCC_EXT,
                          (EGLint)DRM_FORMAT_NV12,
                          EGL_DMA_BUF_PLANE0_FD_EXT,
                          src_fd,
                          EGL_DMA_BUF_PLANE0_OFFSET_EXT,
                          0,
                          EGL_DMA_BUF_PLANE0_PITCH_EXT,
                          (EGLint)hs,
                          EGL_DMA_BUF_PLANE1_FD_EXT,
                          src_fd,
                          EGL_DMA_BUF_PLANE1_OFFSET_EXT,
                          uv_off,
                          EGL_DMA_BUF_PLANE1_PITCH_EXT,
                          (EGLint)hs,
                          EGL_NONE};
  EGLImageKHR src = eglCreateImageKHR_(dpy_, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attrs);
  if (src == EGL_NO_IMAGE_KHR) {
    if (fail_logs_++ % 300 == 0)
      std::fprintf(stderr, "FrameColorTrans: NV12 import failed (0x%x)\n", eglGetError());
    return false;
  }
  GLuint src_tex = 0;
  glGenTextures(1, &src_tex);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, src_tex);
  glEGLImageTargetTexture2DOES_(GL_TEXTURE_EXTERNAL_OES, src);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  target_idx_ = (target_idx_ + 1) % kTargets;
  Target& tgt = targets_[target_idx_];
  glBindFramebuffer(GL_FRAMEBUFFER, tgt.fbo);
  glViewport(0, 0, (GLsizei)width_, (GLsizei)height_);
  glUseProgram(prog_);
  glUniform1i(loc_tex_, 0);
  // Flip V so top-origin NV12 memory lands at the top of the FB.
  const GLfloat verts[] = {-1.f, -1.f, 0.f, 0.f, 1.f, -1.f, 1.f, 0.f,
                           -1.f, 1.f,  0.f, 1.f, 1.f, 1.f,  1.f, 1.f};
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), verts);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), verts + 2);
  glEnableVertexAttribArray(1);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glFinish();
  const GLenum draw_err = glGetError();
  glDeleteTextures(1, &src_tex);
  eglDestroyImageKHR_(dpy_, src);
  if (draw_err != GL_NO_ERROR) {
    if (fail_logs_++ % 300 == 0)
      std::fprintf(stderr, "FrameColorTrans: draw failed (0x%x)\n", draw_err);
    return false;
  }

  rga_buffer_t s = wrapbuffer_fd_t(tgt.prime_fd, (int)width_, (int)height_, tgt.stride_px,
                                   (int)height_, RK_FORMAT_BGRA_8888);
  rga_buffer_t d = wrapbuffer_fd_t(dst_fd, (int)w, (int)h, (int)dst_hs, (int)dst_vs,
                                   RK_FORMAT_YCbCr_420_SP);
  if (imcvtcolor(s, d, RK_FORMAT_BGRA_8888, RK_FORMAT_YCbCr_420_SP) != IM_STATUS_SUCCESS) {
    if (fail_logs_++ % 300 == 0) std::fprintf(stderr, "FrameColorTrans: RGA BGRA->NV12 failed\n");
    return false;
  }
  return true;
}

void FrameColorTrans::destroy_targets() {
  for (Target& t : targets_) {
    if (t.fbo) glDeleteFramebuffers(1, &t.fbo);
    if (t.tex) glDeleteTextures(1, &t.tex);
    if (t.img != EGL_NO_IMAGE_KHR) eglDestroyImageKHR_(dpy_, t.img);
    if (t.prime_fd >= 0) ::close(t.prime_fd);
    if (t.bo) gbm_bo_destroy(t.bo);
    t = Target{};
  }
}

void FrameColorTrans::deinit() {
  if (dpy_ != EGL_NO_DISPLAY && ctx_ != EGL_NO_CONTEXT) {
    eglMakeCurrent(dpy_, surf_, surf_, ctx_);
    glFinish();  // drain before freeing BOs (PixelPilot: panfrost crashes otherwise)
    destroy_targets();
    if (prog_) glDeleteProgram(prog_);
    prog_ = 0;
    eglMakeCurrent(dpy_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(dpy_, ctx_);
    ctx_ = EGL_NO_CONTEXT;
  }
  if (surf_ != EGL_NO_SURFACE) eglDestroySurface(dpy_, surf_);
  surf_ = EGL_NO_SURFACE;
  if (dpy_ != EGL_NO_DISPLAY) eglTerminate(dpy_);
  dpy_ = EGL_NO_DISPLAY;
  if (gbm_) gbm_device_destroy(gbm_);
  gbm_ = nullptr;
  if (drm_fd_ >= 0) ::close(drm_fd_);
  drm_fd_ = -1;
  ready_ = false;
}

}  // namespace maburplay
