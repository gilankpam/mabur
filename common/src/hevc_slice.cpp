#include "mabur/hevc_slice.h"

namespace mabur::hevc {

SliceParse parse_slice_header(const uint8_t* nal, size_t n, const Sps& sps, const Pps& pps,
                              SliceHeader* out) {
  if (!nal || n < 4) return SliceParse::kMalformed;
  SliceHeader h;
  h.nal_hdr[0] = nal[0];
  h.nal_hdr[1] = nal[1];
  h.nal_type = static_cast<uint8_t>((nal[0] >> 1) & 0x3F);
  if (h.nal_type > 21) return SliceParse::kMalformed;
  h.rbsp = unescape(nal + 2, n - 2);
  BitReader r(h.rbsp.data(), h.rbsp.size());
  h.first = r.u(1);
  if (h.nal_type >= 16 && h.nal_type <= 23) r.u(1);  // no_output_of_prior_pics_flag
  if (r.ue() != pps.pps_id) return SliceParse::kUnsupported;
  h.pos_after_pps_id = r.pos();
  bool dependent = false;
  if (!h.first) {
    if (pps.dependent_slices) dependent = r.u(1);
    h.address = r.u(static_cast<int>(sps.addr_bits()));
  }
  h.pos_after_address = r.pos();
  if (dependent) return SliceParse::kDependent;
  r.u(static_cast<int>(pps.extra_bits));
  h.slice_type = r.ue();
  if (h.slice_type > 2) return SliceParse::kMalformed;
  if (pps.output_flag) r.u(1);
  bool slice_tmvp = false;
  uint32_t num_pic_total_curr = 0;
  if (h.nal_type != 19 && h.nal_type != 20) {  // not IDR
    r.u(static_cast<int>(sps.log2_max_poc_lsb));
    if (!r.u(1)) {                             // short_term_ref_pic_set_sps_flag
      uint32_t ndp = 0, used = 0;
      if (!parse_st_rps(r, sps.num_st_rps, sps.num_st_rps, sps.st_num_delta_pocs, &ndp, &used))
        return SliceParse::kMalformed;
      num_pic_total_curr += used;
    } else {
      uint32_t idx = 0;
      if (sps.num_st_rps > 1) idx = r.u(static_cast<int>(ceil_log2(sps.num_st_rps)));
      if (idx >= sps.num_st_rps) return SliceParse::kMalformed;
      num_pic_total_curr += sps.st_num_used[idx];
    }
    if (sps.long_term) {
      const uint32_t n_sps = sps.num_lt_sps > 0 ? r.ue() : 0;
      const uint32_t n_pics = r.ue();
      if (n_sps > sps.num_lt_sps || n_sps + n_pics > 32) return SliceParse::kMalformed;
      for (uint32_t i = 0; i < n_sps + n_pics; ++i) {
        if (i < n_sps) {
          uint32_t li = 0;
          if (sps.num_lt_sps > 1) li = r.u(static_cast<int>(ceil_log2(sps.num_lt_sps)));
          if (li >= sps.num_lt_sps) return SliceParse::kMalformed;
          num_pic_total_curr += sps.lt_used_sps[li];
        } else {
          r.u(static_cast<int>(sps.log2_max_poc_lsb));
          num_pic_total_curr += r.u(1);
        }
        if (r.u(1)) r.ue();  // delta_poc_msb_present_flag -> cycle
      }
    }
    if (sps.tmvp) slice_tmvp = r.u(1);
  }
  if (sps.sao) {
    h.pos_sao = r.pos();
    h.sao_luma = r.u(1);
    if (sps.chroma_format_idc != 0) h.sao_chroma = r.u(1);
  }
  uint32_t nl0 = pps.num_ref_idx_l0_default, nl1 = pps.num_ref_idx_l1_default;
  if (h.slice_type != 2) {
    if (r.u(1)) {                              // num_ref_idx_active_override_flag
      nl0 = r.ue() + 1;
      if (h.slice_type == 0) nl1 = r.ue() + 1;
    }
    if (nl0 > 15 || nl1 > 15) return SliceParse::kMalformed;  // num_ref_idx_{l0,l1}_active_minus1 <= 14
    if (pps.lists_modification && num_pic_total_curr > 1) {
      const int bits = static_cast<int>(ceil_log2(num_pic_total_curr));
      if (r.u(1)) for (uint32_t i = 0; i < nl0; ++i) r.u(bits);
      if (h.slice_type == 0 && r.u(1)) for (uint32_t i = 0; i < nl1; ++i) r.u(bits);
    }
    if (h.slice_type == 0) r.u(1);             // mvd_l1_zero_flag
    if (pps.cabac_init_present) h.cabac_init_flag = r.u(1);
    if (slice_tmvp) {
      bool col_l0 = true;
      if (h.slice_type == 0) col_l0 = r.u(1);
      if ((col_l0 && nl0 > 1) || (!col_l0 && nl1 > 1)) r.ue();  // collocated_ref_idx
    }
    const uint32_t five_minus = r.ue();
    if (five_minus > 4) return SliceParse::kMalformed;
    h.max_num_merge_cand = 5 - five_minus;
  }
  h.slice_qp_y = pps.init_qp + r.se();
  if (pps.chroma_qp_offsets_present) { r.se(); r.se(); }
  bool override = false;
  if (pps.deblocking_override_enabled) override = r.u(1);
  h.deblocking_disabled = pps.deblocking_disabled;
  if (override) {
    h.deblocking_disabled = r.u(1);
    if (!h.deblocking_disabled) { r.se(); r.se(); }
  }
  if (pps.loop_filter_across_slices && (h.sao_luma || h.sao_chroma || !h.deblocking_disabled)) {
    h.lf_across_present = true;
    h.pos_lf_across = r.pos();
    h.lf_across = r.u(1);
  }
  h.pos_end = r.pos();
  if (r.u(1) != 1) return SliceParse::kMalformed;  // alignment_bit_equal_to_one
  while (r.pos() & 7)
    if (r.u(1) != 0) return SliceParse::kMalformed;
  if (r.bad()) return SliceParse::kMalformed;
  h.data_byte = r.pos() / 8;
  *out = std::move(h);
  return SliceParse::kOk;
}

void write_slice_header(BitWriter& w, const SliceHeader& t, const HeaderEdit& e, const Sps& sps,
                        const Pps& pps) {
  const uint8_t* b = t.rbsp.data();
  w.put(e.first ? 1 : 0);
  w.copy(b, 1, t.pos_after_pps_id);            // [no_output_of_prior_pics] + pps id
  if (!e.first) {
    if (pps.dependent_slices) w.put(0);        // dependent_slice_segment_flag
    w.u(e.address, static_cast<int>(sps.addr_bits()));
  }
  size_t from = t.pos_after_address;
  if (sps.sao) {
    const size_t sao_bits = sps.chroma_format_idc != 0 ? 2 : 1;
    w.copy(b, from, t.pos_sao);
    if (e.clear_sao) {
      for (size_t i = 0; i < sao_bits; ++i) w.put(0);
    } else {
      w.copy(b, t.pos_sao, t.pos_sao + sao_bits);
    }
    from = t.pos_sao + sao_bits;
  }
  const bool sao_on = !e.clear_sao && (t.sao_luma || t.sao_chroma);
  const bool lf_present = pps.loop_filter_across_slices && (sao_on || !t.deblocking_disabled);
  if (t.lf_across_present) {
    w.copy(b, from, t.pos_lf_across);
    if (lf_present) w.put(t.lf_across ? 1 : 0);
    w.copy(b, t.pos_lf_across + 1, t.pos_end);
  } else {
    // Clearing SAO never adds the flag: absent means deblocking was off too.
    w.copy(b, from, t.pos_end);
  }
  w.put(1);       // byte_alignment(): alignment_bit_equal_to_one
  w.align_zero();
}

}  // namespace mabur::hevc
