#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
namespace mabur {

// Splits a unit into <=usable-byte chunks, each prefixed with a 6-byte
// fragmentation header packed little-endian as <HHH>: seq (u16, increments
// once per fragment() call and wraps at 65536, shared across all calls on
// one instance), idx (u16, this chunk's index), count (u16, total chunks for
// this call).
//
// u16 idx/count (rather than the u8 pair of the pre-frame-shm format) is what
// lets a whole frame be one unit: a 100 KB IDR needs ~630 fragments, far past
// the 255-fragment / ~40 KB ceiling the narrow header imposed.
//
// Empty input still yields exactly one empty chunk — a zero-length unit still
// needs a seq/idx/count header for the receiver's reassembly path to be
// uniform.
class Fragmenter {
 public:
  std::vector<std::vector<uint8_t>> fragment(const uint8_t* pkt, size_t len, int usable);

  // Same fragments, handed out as (6-byte header, pointer into pkt, chunk
  // length) without a vector per fragment — the hot path writes both spans
  // straight into the FEC envelope (copy/alloc diet 2026-09-22). The
  // vector form is this plus a concatenating sink.
  using Sink = std::function<void(const uint8_t* hdr, const uint8_t* chunk,
                                  size_t chunk_len)>;
  void fragment(const uint8_t* pkt, size_t len, int usable, const Sink& sink);

  // Bytes every fragment spends on its header; callers size `usable` as
  // max_packet_size() - kHdrLen.
  static constexpr size_t kHdrLen = 6;

 private:
  uint16_t seq_ = 0;
};

}  // namespace mabur
