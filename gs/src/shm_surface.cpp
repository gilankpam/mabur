#include "shm_surface.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace maburgs {

void ShmSurface::unmap_() {
  if (map_) { munmap(map_, map_size_); map_ = nullptr; map_size_ = 0; }
  region_ = nullptr;
}

bool ShmSurface::dims_ok_() const {
  if (!region_ || region_->width == 0 || region_->height == 0) return false;
  const size_t need = (size_t)region_->width * region_->height * 4;
  return need <= map_size_ - sizeof(ShmRegion);
}

bool ShmSurface::acquire() {
  int fd = shm_open(name_.c_str(), O_RDWR, 0);
  if (fd < 0) return ok();  // not created yet: keep any existing map
  struct stat st;
  if (fstat(fd, &st) != 0 ||
      st.st_size < (off_t)(sizeof(ShmRegion) + 4)) {
    close(fd);
    return ok();
  }
  // Reuse the current map only if it is the SAME object at the SAME size
  // (a PixelPilot restart re-creates the region with a new inode).
  if (map_ && (size_t)st.st_size == map_size_ && (uint64_t)st.st_ino == ino_) {
    close(fd);
    return dims_ok_();
  }
  unmap_();
  void* m = mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (m == MAP_FAILED) return false;
  map_ = m;
  map_size_ = st.st_size;
  ino_ = st.st_ino;
  region_ = static_cast<ShmRegion*>(m);
  if (!dims_ok_()) { unmap_(); return false; }  // region lies about its size
  return true;
}

}  // namespace maburgs
