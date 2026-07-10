#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur {

// Splits a packet into <=usable-byte chunks, each prefixed with a 4-byte
// fragmentation header packed little-endian as <HBB>: seq (u16, increments
// once per fragment() call and wraps at 65536, shared across all calls on
// one instance), idx (u8, this chunk's index), count (u8, total chunks for
// this call). Byte-exact port of devourer's
// tools/precoder/svc_uep_fec.py's SvcUepEncoder._frag_packets.
//
// Empty input still yields exactly one empty chunk (mirrors Python's
// range(0, max(len(nal), 1), usable) — a zero-length NAL still needs a
// seq/idx/count header for the receiver's reassembly path to be uniform).
class Fragmenter {
 public:
  std::vector<std::vector<uint8_t>> fragment(const uint8_t* pkt, size_t len, int usable);

 private:
  uint16_t seq_ = 0;
};

}  // namespace mabur
