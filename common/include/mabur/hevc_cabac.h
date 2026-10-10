#pragma once
#include <cstdint>

#include "mabur/hevc_bits.h"

namespace mabur::hevc {

// H.265 9.3.4.3 arithmetic coding, encoder side (the informative encoder of
// 9.3.5, same as H.264's). Only what a fill slice codes: regular bins and
// the terminating end_of_slice_segment_flag.
struct CabacCtx {
  uint8_t state = 0;
  uint8_t mps = 0;
};

extern const uint8_t kRangeTabLps[64][4];  // Table 9-52
extern const uint8_t kTransIdxLps[64];     // Table 9-53

// 9.3.2.2 context initialisation from an initValue and SliceQpY.
CabacCtx init_cabac_ctx(int init_value, int slice_qp_y);

class CabacEncoder {
 public:
  explicit CabacEncoder(BitWriter& w) : w_(w) {}
  void encode(CabacCtx& c, int bin);
  // bin 1 flushes; its final bit is the rbsp_stop_one_bit, so the caller
  // only zero-pads to a byte boundary afterwards.
  void terminate(int bin);

 private:
  void renorm();
  void put_bit(int b);
  BitWriter& w_;
  uint32_t low_ = 0, range_ = 510, outstanding_ = 0;
  bool first_ = true;
};

}  // namespace mabur::hevc
