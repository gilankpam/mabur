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
//
// wide=false: 4-byte <u16 seq, u8 idx, u8 count> header (byte-exact
// Python-vector format, all existing streams). wide=true: 6-byte
// <u16 seq, u16 idx, u16 count> header for whole-frame units that
// exceed the 255-fragment / ~40 KB narrow ceiling (spec 2026-07-22).
class Fragmenter {
 public:
  explicit Fragmenter(bool wide = false) : wide_(wide) {}
  std::vector<std::vector<uint8_t>> fragment(const uint8_t* pkt, size_t len, int usable);

 private:
  uint16_t seq_ = 0;
  bool wide_ = false;
};

}  // namespace mabur
