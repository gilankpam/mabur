#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/hevc_bits.h"
#include "mabur/hevc_ps.h"

namespace mabur::hevc {

// One slice_segment_header (H.265 7.3.6.1), parsed far enough to rewrite
// it: the values a fill slice needs, and the RBSP bit positions of the
// fields write_slice_header() changes. Everything between those positions
// is copied verbatim, so a fill carries its picture's exact POC, RPS, QP
// and reference setup.
struct SliceHeader {
  uint8_t nal_hdr[2] = {0, 0};
  uint8_t nal_type = 0;
  std::vector<uint8_t> rbsp;      // de-escaped payload after the NAL header
  bool first = false;
  uint32_t address = 0;
  uint32_t slice_type = 0;        // 0 B, 1 P, 2 I
  int32_t slice_qp_y = 26;
  bool cabac_init_flag = false;
  uint32_t max_num_merge_cand = 5;
  bool sao_luma = false, sao_chroma = false;
  bool deblocking_disabled = false;
  bool lf_across_present = false, lf_across = false;
  size_t pos_after_pps_id = 0;    // first bit after slice_pic_parameter_set_id
  size_t pos_after_address = 0;   // first bit after slice_segment_address (== pos_after_pps_id when first)
  size_t pos_sao = 0;             // slice_sao_luma_flag (valid when Sps::sao)
  size_t pos_lf_across = 0;       // slice_loop_filter_across_slices_enabled_flag (when present)
  size_t pos_end = 0;             // first bit of byte_alignment()
  size_t data_byte = 0;           // first RBSP byte of slice_segment_data()
};

enum class SliceParse { kOk, kMalformed, kUnsupported, kDependent };

// nal: from the 2-byte NAL header, escaped. kUnsupported: a PPS id other
// than the tracked one. kDependent: a dependent slice segment (the SDK
// never emits one; salvage treats it as unsupported).
SliceParse parse_slice_header(const uint8_t* nal, size_t n, const Sps& sps, const Pps& pps,
                              SliceHeader* out);

struct HeaderEdit {
  bool first;
  uint32_t address;
  bool clear_sao;
};

// Writes t's header with e applied, then byte_alignment(). Bit-exact with
// t's own header when e == {t.first, t.address, false}.
void write_slice_header(BitWriter& w, const SliceHeader& t, const HeaderEdit& e, const Sps& sps,
                        const Pps& pps);

}  // namespace mabur::hevc
