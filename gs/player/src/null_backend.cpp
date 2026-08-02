#include "null_backend.h"

#include <utility>

#ifdef MABUR_PLAYER_HW
#include "mpp_backend.h"
#endif

namespace maburplay {

bool NullBackend::init(const BackendCfg&, FrameSink sink) {
  sink_ = std::move(sink);
  return true;
}

void NullBackend::submit_au(const uint8_t*, size_t, uint32_t) { ++submits_; }

void NullBackend::flush() { ++flushes_; }

void NullBackend::release_frame(const DmaFrame&) {}

// Factory: "null" is always available (host and cross). "mpp" is only
// registered when MABUR_PLAYER_HW is compiled in (cross build, Task 7+;
// MppBackend is a stub returning init()=false until Task 8 fills it in) --
// on a plain host build this #ifdef is never defined, so "mpp" falls
// through to nullptr there and main.cpp turns that into a clear exit(2)
// error.
std::unique_ptr<VideoBackend> make_backend(const std::string& name) {
  if (name == "null") return std::make_unique<NullBackend>();
#ifdef MABUR_PLAYER_HW
  if (name == "mpp") return std::make_unique<MppBackend>();
#endif
  return nullptr;
}

}  // namespace maburplay
