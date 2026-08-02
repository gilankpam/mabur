#ifndef MABUR_PLAYER_MPP_BACKEND_H_
#define MABUR_PLAYER_MPP_BACKEND_H_

#include <cstdint>
#include <memory>

#include "video_backend.h"

namespace maburplay {

// Rockchip MPP-backed hardware decode VideoBackend (RK3566 HEVC low-delay
// decode -> NV12 dmabuf). Cross build only (MABUR_PLAYER_HW); registered by
// make_backend("mpp") in null_backend.cpp.
//
// Task 8 fills the Task-7 stub bodies against rockchip_mpp's MPI:
// mpp_create/mpp_init, MPP_DEC_SET_IMMEDIATE_OUT for low-delay output,
// decode_put_packet/decode_get_frame as the async feed/drain pair, info
// change handled by acking MPP_DEC_SET_INFO_CHANGE_READY, decoded frames
// exported as dmabuf fds (mpp_frame_get_buffer -> mpp_buffer_get_fd), and
// flush() mapped to mpi->reset().
//
// Deliberately free of any <rockchip/...> include here: this header is
// pulled into null_backend.cpp's make_backend() dispatch on every
// MABUR_PLAYER_HW build, so it must stay independent of the mpp SDK
// itself (pure video_backend.h types + std only). All mpp includes and
// the actual MppCtx/MppApi/MppFrame state live behind the Impl pimpl in
// mpp_backend.cpp.
class MppBackend : public VideoBackend {
 public:
  MppBackend();
  ~MppBackend() override;

  bool init(const BackendCfg&, FrameSink) override;
  void submit_au(const uint8_t*, size_t, uint32_t) override;
  void flush() override;
  void release_frame(const DmaFrame&) override;
  void poll() override;

  // Decoder-internal counters for diagnostics (main.cpp's --decode-only
  // hardware gate reads these via a dynamic_cast). Not part of the
  // VideoBackend seam -- MppBackend-specific extensions only.
  uint64_t info_changes() const;
  uint64_t errors() const;     // hard failures only (see .cpp counter split)
  uint64_t concealed() const;  // errinfo frames emitted for display

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MPP_BACKEND_H_
