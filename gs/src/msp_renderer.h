#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include "mabur/msp_dp.h"
#include "msp_font.h"
#include "shm_surface.h"

namespace maburgs {

struct MspRenderCfg {
  std::string shm_name = "msp";
  int x_offset = 0;
  int y_offset = 0;
};

// Parses reassembled MSP DisplayPort snapshots and scale-to-fill blits the
// character grid into PixelPilot's shm region as premultiplied ARGB32.
// Single-threaded (GS core thread).
class MspRenderer {
 public:
  MspRenderer(const MspRenderCfg& cfg, const MspFont& font)
      : cfg_(cfg), font_(font), shm_(cfg.shm_name) {}

  void on_snapshot(const uint8_t* bytes, size_t len);

  uint64_t frames_rendered() const { return frames_rendered_; }
  uint64_t frames_dropped_no_shm() const { return dropped_; }

 private:
  void render();

  MspRenderCfg cfg_;
  const MspFont& font_;
  mabur::MspParser parser_;
  mabur::MspScreen screen_;
  ShmSurface shm_;
  bool warned_ = false;
  uint64_t frames_rendered_ = 0;
  uint64_t dropped_ = 0;
};

}  // namespace maburgs
