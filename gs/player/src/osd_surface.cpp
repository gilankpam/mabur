#include "osd_surface.h"

#include <fcntl.h>  // O_CLOEXEC, via libdrm's DRM_CLOEXEC macro
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

namespace maburplay {

void OsdSurface::destroy() {
  for (Buf& b : buf_) {
    if (b.ptr) munmap(b.ptr, static_cast<size_t>(b.size));
    if (b.prime_fd >= 0) ::close(b.prime_fd);
    if (fd_ >= 0) {
      if (b.fb_id) drmModeRmFB(fd_, b.fb_id);
      if (b.handle) drmModeDestroyDumbBuffer(fd_, b.handle);
    }
    b = Buf{};
  }
  fd_ = -1;
  width_ = height_ = 0;
}

OsdSurface::~OsdSurface() { destroy(); }

bool OsdSurface::alloc_(Buf* b, std::string* err) {
  // drmModeCreateDumbBuffer/MapDumbBuffer/DestroyDumbBuffer rather than raw
  // DRM_IOCTL_MODE_*_DUMB: same ioctls, and it matches how the black-primary
  // backdrop is allocated next door in drm_presenter.cpp.
  uint32_t handle = 0, pitch = 0;
  uint64_t size = 0;
  if (drmModeCreateDumbBuffer(fd_, static_cast<uint32_t>(width_), static_cast<uint32_t>(height_),
                              32, 0, &handle, &pitch, &size) != 0) {
    if (err) *err = std::string("create dumb buffer failed: ") + std::strerror(errno);
    return false;
  }
  b->handle = handle;
  b->pitch = pitch;
  b->size = size;

  const uint32_t handles[4] = {b->handle, 0, 0, 0};
  const uint32_t pitches[4] = {b->pitch, 0, 0, 0};
  const uint32_t offsets[4] = {0, 0, 0, 0};
  if (drmModeAddFB2(fd_, static_cast<uint32_t>(width_), static_cast<uint32_t>(height_),
                    DRM_FORMAT_ARGB8888, handles, pitches, offsets, &b->fb_id, 0) != 0) {
    b->fb_id = 0;
    if (err) *err = std::string("AddFB2(ARGB8888) failed: ") + std::strerror(errno);
    return false;
  }

  uint64_t map_offset = 0;
  if (drmModeMapDumbBuffer(fd_, b->handle, &map_offset) != 0) {
    if (err) *err = std::string("map dumb buffer failed: ") + std::strerror(errno);
    return false;
  }
  void* p = mmap(nullptr, static_cast<size_t>(b->size), PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
                 static_cast<off_t>(map_offset));
  if (p == MAP_FAILED) {
    if (err) *err = std::string("mmap of dumb buffer failed: ") + std::strerror(errno);
    return false;
  }
  b->ptr = p;
  std::memset(p, 0, static_cast<size_t>(b->size));

  // Phase 2's RGA composite reads this fd; nothing consumes it yet, and
  // exporting it now costs one fd per buffer.
  if (drmPrimeHandleToFD(fd_, b->handle, DRM_CLOEXEC, &b->prime_fd) != 0) b->prime_fd = -1;
  return true;
}

bool OsdSurface::init(int drm_fd, int width, int height, std::string* err) {
  fd_ = drm_fd;
  width_ = width;
  height_ = height;
  if (fd_ < 0 || width_ <= 0 || height_ <= 0) {
    if (err) *err = "invalid drm fd or dimensions";
    return false;
  }
  // All-or-nothing: buffer 0 succeeding while buffer 1 fails must not leave
  // buffer 0's GEM object, FB, mapping and prime fd held for the process
  // lifetime -- the caller only ever checks the bool.
  if (!alloc_(&buf_[0], err) || !alloc_(&buf_[1], err)) {
    destroy();  // leaves the object in the default-constructed, unusable state
    return false;
  }
  return true;
}

Surface OsdSurface::cpu(int idx) const {
  const Buf& b = buf_[idx & 1];
  if (!b.ptr) return Surface{};
  Surface s;
  s.pixels = static_cast<uint32_t*>(b.ptr);
  s.width = width_;
  s.height = height_;
  s.stride_px = static_cast<int>(b.pitch / 4);
  return s;
}

}  // namespace maburplay
