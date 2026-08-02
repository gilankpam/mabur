#include "null_backend.h"

#include <utility>

namespace maburplay {

bool NullBackend::init(const BackendCfg&, FrameSink sink) {
  sink_ = std::move(sink);
  return true;
}

void NullBackend::submit_au(const uint8_t*, size_t, uint32_t) { ++submits_; }

void NullBackend::flush() { ++flushes_; }

void NullBackend::release_frame(const DmaFrame&) {}

// Host-only factory: "null" is always available. "mpp" is compiled in only
// by the cross build (MABUR_PLAYER_HW, Task 7/8, mpp_backend.cpp) which
// registers itself elsewhere; this translation unit never sees that macro,
// so it always returns nullptr for anything but "null" — main.cpp turns
// that into a clear exit(2) error.
std::unique_ptr<VideoBackend> make_backend(const std::string& name) {
  if (name == "null") return std::make_unique<NullBackend>();
  return nullptr;
}

}  // namespace maburplay
