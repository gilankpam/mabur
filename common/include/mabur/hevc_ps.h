#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mabur/hevc_bits.h"

namespace mabur::hevc {

// What slice salvage needs from the SPS/PPS (spec 2026-10-10-h265-slices
// §5.2): picture geometry and every field the slice-segment-header syntax
// depends on. Streams using a feature outside the bench encoder's subset
// parse as kUnsupported and switch salvage off rather than risk a wrong
// header.
enum class PsStatus { kNone, kOk, kMalformed, kUnsupported };

struct Sps {
  uint32_t sps_id = 0;
  uint32_t chroma_format_idc = 1;
  uint32_t width = 0, height = 0;  // coded luma samples
  uint32_t log2_max_poc_lsb = 4;
  uint32_t log2_min_cb = 3, log2_ctb = 4;
  bool sao = false;
  bool tmvp = false;
  uint32_t num_st_rps = 0;
  std::vector<uint32_t> st_num_delta_pocs;  // NumDeltaPocs[i]
  std::vector<uint32_t> st_num_used;        // used_by_curr entries of set i
  bool long_term = false;
  uint32_t num_lt_sps = 0;
  std::vector<uint8_t> lt_used_sps;
  uint32_t pic_w_ctbs() const { return (width + (1u << log2_ctb) - 1) >> log2_ctb; }
  uint32_t pic_h_ctbs() const { return (height + (1u << log2_ctb) - 1) >> log2_ctb; }
  uint32_t addr_bits() const { return ceil_log2(pic_w_ctbs() * pic_h_ctbs()); }
};

struct Pps {
  uint32_t pps_id = 0, sps_id = 0;
  bool dependent_slices = false;
  bool output_flag = false;
  uint32_t extra_bits = 0;
  bool cabac_init_present = false;
  uint32_t num_ref_idx_l0_default = 1, num_ref_idx_l1_default = 1;
  int32_t init_qp = 26;
  bool chroma_qp_offsets_present = false;
  bool deblocking_override_enabled = false;
  bool deblocking_disabled = false;
  bool loop_filter_across_slices = false;
  bool lists_modification = false;
};

// nal: from the 2-byte NAL header, emulation-prevention bytes included.
PsStatus parse_sps(const uint8_t* nal, size_t n, Sps* out);
PsStatus parse_pps(const uint8_t* nal, size_t n, Pps* out);

// st_ref_pic_set(idx) (H.265 7.3.7). idx == num_sets is a slice header's
// own set. num_delta_pocs: NumDeltaPocs of the SPS sets parsed so far.
// out_ndp = NumDeltaPocs of this set, out_used = its used_by_curr entries.
bool parse_st_rps(BitReader& r, uint32_t idx, uint32_t num_sets,
                  const std::vector<uint32_t>& num_delta_pocs, uint32_t* out_ndp,
                  uint32_t* out_used);

// Latest SPS/PPS seen in complete access units.
class ParamTracker {
 public:
  void feed(const uint8_t* au, size_t n);
  bool usable() const;
  bool unsupported() const {
    return sps_st_ == PsStatus::kUnsupported || pps_st_ == PsStatus::kUnsupported;
  }
  const Sps& sps() const { return sps_; }
  const Pps& pps() const { return pps_; }
  void reset() { sps_st_ = pps_st_ = PsStatus::kNone; }

 private:
  Sps sps_;
  Pps pps_;
  PsStatus sps_st_ = PsStatus::kNone, pps_st_ = PsStatus::kNone;
};

}  // namespace mabur::hevc
