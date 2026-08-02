#ifndef MABUR_PLAYER_MPP_BACKEND_H_
#define MABUR_PLAYER_MPP_BACKEND_H_

#include "video_backend.h"

namespace maburplay {

// Rockchip MPP-backed hardware decode VideoBackend. Cross build only
// (MABUR_PLAYER_HW); registered by make_backend("mpp") in null_backend.cpp.
//
// STUB as of Task 7: init() always fails ("not implemented (Task 8)"), the
// rest are no-ops. Task 7's job is proving the aarch64 static toolchain
// (mpp + libdrm link lines, include paths) end to end, not decoding —
// Task 8 fills these bodies against rockchip_mpp's MPI
// (mpp_create/mpp_init, mpi->decode_put_packet/decode_get_frame, ...).
//
// Deliberately free of any <rockchip/...> include here: this header is
// pulled into null_backend.cpp's make_backend() dispatch on every
// MABUR_PLAYER_HW build, so it must stay independent of the mpp SDK
// itself (pure video_backend.h types only). Task 8 adds mpp includes to
// mpp_backend.cpp, not here.
class MppBackend : public VideoBackend {
 public:
  bool init(const BackendCfg&, FrameSink) override;
  void submit_au(const uint8_t*, size_t, uint32_t) override;
  void flush() override;
  void release_frame(const DmaFrame&) override;
  void poll() override;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MPP_BACKEND_H_
