#include "osd_screen.h"

#include <utility>

namespace webgs {

OsdScreen::OsdScreen(PublishFn publish, uint64_t min_interval_ms)
    : publish_(std::move(publish)), min_interval_ms_(min_interval_ms) {}

void OsdScreen::feed(const uint8_t* p, size_t n, uint64_t now_ms) {
  for (const mabur::MspMessage& m : parser_.feed(p, n)) {
    if (!screen_.apply(m)) continue;   // true only on DRAW_SCREEN
    rows_ = screen_.rows();
    cols_ = screen_.cols();
    cells_.resize(static_cast<size_t>(rows_) * static_cast<size_t>(cols_));
    for (int r = 0; r < rows_; ++r)
      for (int c = 0; c < cols_; ++c)
        cells_[static_cast<size_t>(r) * static_cast<size_t>(cols_) + c] = screen_.cell(r, c);
    pending_ = true;
  }
  try_publish_(now_ms);
}

void OsdScreen::tick(uint64_t now_ms) { try_publish_(now_ms); }

void OsdScreen::try_publish_(uint64_t now_ms) {
  if (!pending_) return;
  if (published_once_ && now_ms - last_pub_ms_ < min_interval_ms_) return;
  pending_ = false;
  published_once_ = true;
  last_pub_ms_ = now_ms;
  ++screens_;
  if (publish_) publish_(rows_, cols_, cells_.data());
}

}  // namespace webgs
