#include "drm_presenter.h"

#include <cstdio>

// STUB (Task 7): proves the toolchain/link lines only -- every body here is
// replaced by real libdrm KMS calls in Task 9. No <xf86drm.h>/
// <xf86drmMode.h> includes yet; see the rationale in drm_presenter.h.
namespace maburplay {

bool DrmPresenter::init(const std::string&) {
  std::fprintf(stderr, "DrmPresenter: not implemented (Task 9)\n");
  return false;
}

bool DrmPresenter::present(const DmaFrame&) { return false; }  // Task 9

void DrmPresenter::poll_events() {}  // Task 9

DrmPresenter::~DrmPresenter() {}  // Task 9

}  // namespace maburplay
