#pragma once
// OsdScreen -- the web GS's MSP OSD screen model (spec 2026-09-27-web-msp-osd).
// Takes whole DisplayPort snapshots (MspSink's output), applies them to the
// shared MspScreen, and publishes each completed screen (DRAW_SCREEN) at most
// once per min_interval_ms -- maburplay OsdSource's gate. A screen completed
// inside the window is held and published by tick(): latest wins. The held
// cells are a copy taken at DRAW_SCREEN, so input arriving after it (an
// unfinished snapshot) never leaks into what gets published.
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "mabur/msp_dp.h"

namespace webgs {

class OsdScreen {
 public:
  // rows x cols cells, row-major, cell = char | page << 8 (MspScreen::cell).
  using PublishFn = std::function<void(int rows, int cols, const uint16_t* cells)>;

  explicit OsdScreen(PublishFn publish, uint64_t min_interval_ms = 30);

  void feed(const uint8_t* p, size_t n, uint64_t now_ms);
  void tick(uint64_t now_ms);

  uint64_t screens() const { return screens_; }   // published

 private:
  void try_publish_(uint64_t now_ms);

  PublishFn publish_;
  const uint64_t min_interval_ms_;
  mabur::MspParser parser_;
  mabur::MspScreen screen_;
  int rows_ = 0, cols_ = 0;
  std::vector<uint16_t> cells_;   // copy at the last DRAW_SCREEN
  bool pending_ = false;
  bool published_once_ = false;
  uint64_t last_pub_ms_ = 0;
  uint64_t screens_ = 0;
};

}  // namespace webgs
