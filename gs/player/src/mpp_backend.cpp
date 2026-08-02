#include "mpp_backend.h"

#include <cstdio>

// STUB (Task 7): proves the toolchain/link lines only -- every body here is
// replaced by real rockchip_mpp MPI calls in Task 8. No <rockchip/...>
// includes yet; see the rationale in mpp_backend.h.
namespace maburplay {

bool MppBackend::init(const BackendCfg&, FrameSink) {
  std::fprintf(stderr, "MppBackend: not implemented (Task 8)\n");
  return false;
}

void MppBackend::submit_au(const uint8_t*, size_t, uint32_t) {}  // Task 8

void MppBackend::flush() {}  // Task 8

void MppBackend::release_frame(const DmaFrame&) {}  // Task 8

void MppBackend::poll() {}  // Task 8

}  // namespace maburplay
