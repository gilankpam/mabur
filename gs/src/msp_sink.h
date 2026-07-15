#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include "mabur/sw_decoder.h"

namespace maburgs {

// GS-side MSP OSD sink: receives SBI bodies tagged stream_id == kMspStreamId,
// SBI-unpacks them, feeds the sliding-window decoder, and emits each recovered
// snapshot (one full MSP DisplayPort byte sequence) via emit() — typically a
// UDP datagram to the external renderer (msposd). Single-threaded (core
// thread).
class MspSink {
 public:
  using EmitFn = std::function<void(const uint8_t* data, size_t len)>;

  MspSink(int symbol_size, int window, EmitFn emit);

  void on_body(const uint8_t* body, size_t len, uint64_t now_ms);
  void tick(uint64_t now_ms);  // ~1 Hz: expire stale repair rows

  uint64_t snapshots_out() const { return snapshots_out_; }

 private:
  int block_payload_;
  mabur::SwDecoder dec_;
  EmitFn emit_;
  uint64_t snapshots_out_ = 0;
};

}  // namespace maburgs
