#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include "mabur/msp_dp.h"
#include "mabur/sbi.h"
#include "mabur/sw_encoder.h"

namespace mabur {

struct MspSourceCfg {
  double update_rate_hz = 1.0;
  int symbol_size = 1312;
  int window = 16;
  double overhead = 1.0;
};

// Drone-side MSP OSD source: parses the FC MSP DisplayPort byte stream,
// maintains a screen buffer, and at most update_rate_hz forwards a full-screen
// keyframe snapshot as SBI-framed sliding-window-FEC bodies (stream_id =
// kMspStreamId). Each emitted body is one air frame (blocks_per_body = 1, so a
// source and its repair land in separate frames for erasure diversity); inject
// at the robust control modulation. Single-threaded (the serial thread).
class MspSource {
 public:
  using EmitFn = std::function<void(const uint8_t* body, size_t len)>;

  MspSource(const MspSourceCfg& cfg, EmitFn emit, uint32_t initial_seq = 0);

  void on_serial_bytes(const uint8_t* p, size_t n, uint64_t now_ms);

  uint64_t snapshots_sent() const { return snapshots_sent_; }
  uint64_t snapshots_gated() const { return snapshots_gated_; }
  uint64_t truncated() const { return truncated_; }

 private:
  void forward_snapshot(uint64_t now_ms);

  MspSourceCfg cfg_;
  EmitFn emit_;
  MspParser parser_;
  MspScreen screen_;
  SwEncoder enc_;
  SbiPacker packer_;
  uint64_t last_forward_ms_ = 0;
  bool have_forwarded_ = false;
  uint64_t snapshots_sent_ = 0, snapshots_gated_ = 0, truncated_ = 0;
};

}  // namespace mabur
