#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace maburgs {

// PixelPilot ExternalSurfaceWidget shm layout (../PixelPilot_rk/src/osd.hpp).
struct ShmRegion {
  uint16_t width;
  uint16_t height;
  unsigned char data[];
};

// Maps PixelPilot's shm region (which PixelPilot creates and owns). acquire()
// (re)maps it, returning false when it is not present yet. Never creates or
// resizes the region.
class ShmSurface {
 public:
  explicit ShmSurface(std::string name) : name_(std::move(name)) {}
  ~ShmSurface() { unmap_(); }
  ShmSurface(const ShmSurface&) = delete;
  ShmSurface& operator=(const ShmSurface&) = delete;

  bool acquire();
  bool ok() const { return region_ != nullptr; }
  int width() const { return region_ ? region_->width : 0; }
  int height() const { return region_ ? region_->height : 0; }
  unsigned char* data() { return region_ ? region_->data : nullptr; }
  size_t data_size() const {
    return region_ ? (size_t)region_->width * region_->height * 4 : 0;
  }

 private:
  void unmap_();
  std::string name_;
  void* map_ = nullptr;
  size_t map_size_ = 0;
  uint64_t ino_ = 0;
  ShmRegion* region_ = nullptr;
};

}  // namespace maburgs
