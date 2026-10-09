#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <vector>

#include "au_ring.h"
#include "mabur/hevc_ps.h"

namespace maburgs {

using ChunkMap = std::map<uint16_t, std::vector<uint8_t>>;
using ByteSink = std::function<void(const uint8_t*, size_t)>;

// One access unit of a row-sliced H.265 picture, rebuilt from FrameStream's
// fragment chunks (spec 2026-10-10-h265-slices §5.2).
//
// Fragment i covers AU bytes [i*F - hdr_len, ...) where F = chunk 0's size
// (every fragment but the last is F bytes; chunk 0 also carries the
// FrameHdr). drain() streams whole NAL units of the contiguous prefix as
// they complete -- always a byte prefix of the AU. finish() emits the rest:
//  - complete AU: the remaining bytes (byte-identical output);
//  - damaged AU, salvageable: every remaining slice in order, each either
//    its original bytes (arrived complete) or a make_skip_slice() fill;
//  - damaged AU, not salvageable: the rest of the contiguous prefix,
//    exactly what FrameStream emitted before slice salvage existed.
// A slice is complete only if its start code and every byte up to the next
// start code (or the known AU end) lie in ONE run of consecutive fragments:
// a start code split by a hole is never trusted.
class SliceAssembler {
 public:
  SliceAssembler(const mabur::hevc::Sps& sps, const mabur::hevc::Pps& pps, uint8_t slice_rows,
                 uint16_t count, size_t hdr_len);

  void drain(const ChunkMap& chunks, const ByteSink& out);
  void finish(const ChunkMap& chunks, const ByteSink& out);

  const SliceSalvage& result() const { return result_; }
  uint8_t slices() const { return n_; }

 private:
  struct Sc { size_t pos, len; };
  void extend_prefix(const ChunkMap& chunks);
  void scan_prefix();
  SliceFallback plan(const ChunkMap& chunks, std::vector<std::vector<uint8_t>>* pieces,
                     SliceSalvage* res);

  mabur::hevc::Sps sps_;
  mabur::hevc::Pps pps_;
  uint8_t slice_rows_;
  uint16_t count_;
  size_t hdr_len_;
  uint8_t n_ = 0;
  size_t frag_ = 0;               // chunk 0's size (= every non-last fragment's)
  uint16_t prefix_chunks_ = 0;    // chunks [0, prefix_chunks_) are in buf_
  std::vector<uint8_t> buf_;      // the contiguous prefix, FrameHdr stripped
  std::vector<Sc> sc_;            // start codes found in buf_
  size_t scanned_ = 0;            // buf_ positions < scanned_ fully examined
  size_t emitted_ = 0;            // bytes of buf_ already handed out
  SliceSalvage result_;
};

}  // namespace maburgs
