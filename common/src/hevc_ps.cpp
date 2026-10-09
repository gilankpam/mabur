#include "mabur/hevc_ps.h"

#include "mabur/hevc_params.h"

namespace mabur::hevc {
namespace {

// profile_tier_level(1, max_sub) (7.3.3): the general part is 96 bits.
bool skip_ptl(BitReader& r, uint32_t max_sub) {
  r.u(32); r.u(32); r.u(32);
  std::vector<uint32_t> pp(max_sub), lp(max_sub);
  for (uint32_t i = 0; i < max_sub; ++i) { pp[i] = r.u(1); lp[i] = r.u(1); }
  if (max_sub > 0)
    for (uint32_t i = max_sub; i < 8; ++i) r.u(2);
  for (uint32_t i = 0; i < max_sub; ++i) {
    if (pp[i]) { r.u(32); r.u(32); r.u(24); }
    if (lp[i]) r.u(8);
  }
  return !r.bad();
}

}  // namespace

bool parse_st_rps(BitReader& r, uint32_t idx, uint32_t num_sets,
                  const std::vector<uint32_t>& num_delta_pocs, uint32_t* out_ndp,
                  uint32_t* out_used) {
  const bool inter = idx != 0 && r.u(1);
  if (inter) {
    const uint32_t delta_idx_m1 = idx == num_sets ? r.ue() : 0;
    if (delta_idx_m1 + 1 > idx || idx - (delta_idx_m1 + 1) >= num_delta_pocs.size()) return false;
    const uint32_t ref = idx - (delta_idx_m1 + 1);
    r.u(1);   // delta_rps_sign
    r.ue();   // abs_delta_rps_minus1
    uint32_t n = 0, used = 0;
    for (uint32_t j = 0; j <= num_delta_pocs[ref]; ++j) {
      const uint32_t u = r.u(1);
      uint32_t use_delta = 1;
      if (!u) use_delta = r.u(1);
      if (u || use_delta) ++n;
      if (u) ++used;
    }
    *out_ndp = n;
    *out_used = used;
  } else {
    const uint32_t neg = r.ue(), pos = r.ue();
    if (neg > 16 || pos > 16) return false;
    uint32_t used = 0;
    for (uint32_t i = 0; i < neg + pos; ++i) { r.ue(); used += r.u(1); }
    *out_ndp = neg + pos;
    *out_used = used;
  }
  return !r.bad();
}

PsStatus parse_sps(const uint8_t* nal, size_t n, Sps* out) {
  if (!nal || n < 3) return PsStatus::kMalformed;
  const auto rb = unescape(nal + 2, n - 2);
  BitReader r(rb.data(), rb.size());
  Sps s;
  r.u(4);                          // sps_video_parameter_set_id
  const uint32_t max_sub = r.u(3);
  r.u(1);                          // sps_temporal_id_nesting_flag
  if (!skip_ptl(r, max_sub)) return PsStatus::kMalformed;
  s.sps_id = r.ue();
  s.chroma_format_idc = r.ue();
  if (s.chroma_format_idc > 3) return PsStatus::kMalformed;
  if (s.chroma_format_idc == 3 && r.u(1)) return PsStatus::kUnsupported;  // separate planes
  s.width = r.ue();
  s.height = r.ue();
  if (r.u(1)) { r.ue(); r.ue(); r.ue(); r.ue(); }  // conformance window
  r.ue(); r.ue();                                  // bit depths
  s.log2_max_poc_lsb = r.ue() + 4;
  const uint32_t sub_info = r.u(1);
  for (uint32_t i = sub_info ? 0 : max_sub; i <= max_sub; ++i) { r.ue(); r.ue(); r.ue(); }
  s.log2_min_cb = r.ue() + 3;
  s.log2_ctb = s.log2_min_cb + r.ue();
  r.ue(); r.ue(); r.ue(); r.ue();  // transform block sizes, hierarchy depths
  if (r.u(1) && r.u(1)) return PsStatus::kUnsupported;  // sps_scaling_list_data
  r.u(1);                          // amp_enabled_flag
  s.sao = r.u(1);
  if (r.u(1)) { r.u(4); r.u(4); r.ue(); r.ue(); r.u(1); }  // pcm
  s.num_st_rps = r.ue();
  if (s.num_st_rps > 64) return PsStatus::kMalformed;
  for (uint32_t i = 0; i < s.num_st_rps; ++i) {
    uint32_t ndp = 0, used = 0;
    if (!parse_st_rps(r, i, s.num_st_rps, s.st_num_delta_pocs, &ndp, &used))
      return PsStatus::kMalformed;
    s.st_num_delta_pocs.push_back(ndp);
    s.st_num_used.push_back(used);
  }
  s.long_term = r.u(1);
  if (s.long_term) {
    s.num_lt_sps = r.ue();
    if (s.num_lt_sps > 32) return PsStatus::kMalformed;
    for (uint32_t i = 0; i < s.num_lt_sps; ++i) {
      r.u(static_cast<int>(s.log2_max_poc_lsb));
      s.lt_used_sps.push_back(static_cast<uint8_t>(r.u(1)));
    }
  }
  s.tmvp = r.u(1);
  if (r.bad() || s.width == 0 || s.height == 0 || s.log2_ctb < 4 || s.log2_ctb > 6 ||
      (s.width & ((1u << s.log2_min_cb) - 1)) || (s.height & ((1u << s.log2_min_cb) - 1)))
    return PsStatus::kMalformed;
  *out = s;
  return PsStatus::kOk;
}

PsStatus parse_pps(const uint8_t* nal, size_t n, Pps* out) {
  if (!nal || n < 3) return PsStatus::kMalformed;
  const auto rb = unescape(nal + 2, n - 2);
  BitReader r(rb.data(), rb.size());
  Pps p;
  p.pps_id = r.ue();
  p.sps_id = r.ue();
  p.dependent_slices = r.u(1);
  p.output_flag = r.u(1);
  p.extra_bits = r.u(3);
  r.u(1);                          // sign_data_hiding_enabled_flag
  p.cabac_init_present = r.u(1);
  p.num_ref_idx_l0_default = r.ue() + 1;
  p.num_ref_idx_l1_default = r.ue() + 1;
  p.init_qp = 26 + r.se();
  r.u(1);                          // constrained_intra_pred_flag
  r.u(1);                          // transform_skip_enabled_flag
  if (r.u(1)) r.ue();              // cu_qp_delta_enabled -> diff_cu_qp_delta_depth
  r.se(); r.se();                  // pps_cb/cr_qp_offset
  p.chroma_qp_offsets_present = r.u(1);
  const bool wp = r.u(1), wbp = r.u(1), tq = r.u(1), tiles = r.u(1), wpp = r.u(1);
  if (wp || wbp || tq || tiles || wpp) return PsStatus::kUnsupported;
  p.loop_filter_across_slices = r.u(1);
  if (r.u(1)) {                    // deblocking_filter_control_present_flag
    p.deblocking_override_enabled = r.u(1);
    p.deblocking_disabled = r.u(1);
    if (!p.deblocking_disabled) { r.se(); r.se(); }
  }
  if (r.u(1)) return PsStatus::kUnsupported;  // pps_scaling_list_data_present_flag
  p.lists_modification = r.u(1);
  r.ue();                          // log2_parallel_merge_level_minus2
  if (r.u(1)) return PsStatus::kUnsupported;  // slice_segment_header_extension_present
  if (r.u(1)) return PsStatus::kUnsupported;  // pps_extension_present
  if (r.bad()) return PsStatus::kMalformed;
  *out = p;
  return PsStatus::kOk;
}

void ParamTracker::feed(const uint8_t* au, size_t n) {
  for (const NalView& v : split_nals(au, n)) {
    if (v.type == 33) {
      Sps s;
      sps_st_ = parse_sps(v.p, v.n, &s);
      if (sps_st_ == PsStatus::kOk) sps_ = s;
    } else if (v.type == 34) {
      Pps p;
      pps_st_ = parse_pps(v.p, v.n, &p);
      if (pps_st_ == PsStatus::kOk) pps_ = p;
    }
  }
}

bool ParamTracker::usable() const {
  return sps_st_ == PsStatus::kOk && pps_st_ == PsStatus::kOk && pps_.sps_id == sps_.sps_id;
}

}  // namespace mabur::hevc
