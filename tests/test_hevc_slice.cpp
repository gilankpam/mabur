#include <vector>
#include "mabur/hevc_bits.h"
#include "mabur/hevc_ps.h"
#include "mabur/hevc_slice.h"
#include "mtest.h"
#include "slice_fixture.h"
using namespace mabur::hevc;

namespace {
struct Ctx { ParamTracker t; std::vector<std::vector<uint8_t>> aus; };
Ctx ctx() {
  Ctx c;
  c.aus = mtest::load_slice_fixture();
  c.t.feed(c.aus[3].data(), c.aus[3].size());
  REQUIRE(c.t.usable());
  return c;
}
size_t body_of(const std::vector<uint8_t>& nal) { return nal[2] == 1 ? 3 : 4; }
}  // namespace

TEST(every_fixture_header_parses_with_the_bench_geometry) {
  Ctx c = ctx();
  for (size_t a = 0; a < c.aus.size(); ++a) {
    const auto sl = mtest::slice_nals(c.aus[a]);
    for (size_t k = 0; k < sl.size(); ++k) {
      SliceHeader h;
      const size_t b = body_of(sl[k]);
      REQUIRE(parse_slice_header(sl[k].data() + b, sl[k].size() - b, c.t.sps(), c.t.pps(), &h) == SliceParse::kOk);
      CHECK(h.first == (k == 0));
      CHECK(h.address == (k == 0 ? 0u : 150u * k));
      CHECK(h.slice_type == (a == 0 ? 2u : 1u));  // AU 0 is the IDR
      if (a != 0) CHECK(h.max_num_merge_cand == 2);
      CHECK(h.sao_luma && h.sao_chroma);
      CHECK(h.data_byte > 0 && h.data_byte < h.rbsp.size());
    }
  }
}

TEST(write_with_no_edit_is_bit_exact) {
  Ctx c = ctx();
  for (size_t a = 0; a < c.aus.size(); ++a) {
    for (const auto& nal : mtest::slice_nals(c.aus[a])) {
      SliceHeader h;
      const size_t b = body_of(nal);
      REQUIRE(parse_slice_header(nal.data() + b, nal.size() - b, c.t.sps(), c.t.pps(), &h) == SliceParse::kOk);
      BitWriter w;
      write_slice_header(w, h, {h.first, h.address, false}, c.t.sps(), c.t.pps());
      REQUIRE(w.bits() == h.data_byte * 8);
      CHECK(std::vector<uint8_t>(h.rbsp.begin(), h.rbsp.begin() + static_cast<long>(h.data_byte)) == w.bytes());
    }
  }
}

TEST(edited_header_reparses_with_the_edit_applied) {
  Ctx c = ctx();
  const auto sl = mtest::slice_nals(c.aus[5]);
  for (const uint32_t target : {0u, 150u, 450u}) {
    SliceHeader t;
    const size_t b = body_of(sl[1]);
    REQUIRE(parse_slice_header(sl[1].data() + b, sl[1].size() - b, c.t.sps(), c.t.pps(), &t) == SliceParse::kOk);
    BitWriter w;
    write_slice_header(w, t, {target == 0, target, true}, c.t.sps(), c.t.pps());
    std::vector<uint8_t> nal = {t.nal_hdr[0], t.nal_hdr[1]};
    const auto esc = escape(w.bytes().data(), w.bytes().size());
    nal.insert(nal.end(), esc.begin(), esc.end());
    nal.push_back(0x80);  // a stand-in slice-data byte so the parser has data
    SliceHeader e;
    REQUIRE(parse_slice_header(nal.data(), nal.size(), c.t.sps(), c.t.pps(), &e) == SliceParse::kOk);
    CHECK(e.first == (target == 0));
    CHECK(e.address == target);
    CHECK(!e.sao_luma && !e.sao_chroma);
    CHECK(e.slice_qp_y == t.slice_qp_y);
    CHECK(e.slice_type == t.slice_type);
    CHECK(e.max_num_merge_cand == t.max_num_merge_cand);
    CHECK(e.lf_across_present == t.lf_across_present);  // deblocking is on: flag stays
  }
}

TEST(pps_id_mismatch_is_unsupported) {
  Ctx c = ctx();
  Pps other = c.t.pps();
  other.pps_id = 7;
  const auto nal = mtest::slice_nals(c.aus[5])[0];
  SliceHeader h;
  CHECK(parse_slice_header(nal.data() + body_of(nal), nal.size() - body_of(nal), c.t.sps(), other, &h) ==
        SliceParse::kUnsupported);
}

// A hand-built, otherwise-complete P-slice header on an IDR NAL (nal_type
// 19/20 skips the POC/RPS section, so nothing else needs faking) with
// num_ref_idx_l0_active_minus1 = 15 -> nl0 = 16, outside the spec's 0..14.
// NumPicTotalCurr stays 0 here (POC/RPS skipped), so the dangerous
// lists_modification loop never runs even without the guard -- this pins
// just the missing bounds check: without it this fully-formed header
// parses as kOk with a garbage nl0 that a later NumPicTotalCurr > 1 header
// would turn into the GS-thread stall the guard exists to prevent.
TEST(num_ref_idx_active_override_out_of_range_is_malformed) {
  Ctx c = ctx();
  const auto& sps = c.t.sps();
  const auto& pps = c.t.pps();
  const auto idr = mtest::slice_nals(c.aus[0])[0];
  const size_t ib = body_of(idr);
  REQUIRE((idr[ib] >> 1 & 0x3F) == 19 || (idr[ib] >> 1 & 0x3F) == 20);  // IDR: no POC/RPS section

  BitWriter w;
  w.put(1);                                  // first_slice_segment_in_pic_flag
  w.put(1);                                  // no_output_of_prior_pics_flag (IRAP range)
  w.ue(pps.pps_id);                          // slice_pic_parameter_set_id
  w.u(0, static_cast<int>(pps.extra_bits));  // num_extra_slice_header_bits
  w.ue(1);                                   // slice_type = 1 (P)
  // pps.output_flag is false in this fixture; nal_type is IDR so the
  // POC/RPS section and slice_temporal_mvp_enabled_flag are both skipped.
  REQUIRE(!pps.output_flag);
  w.put(0);                                  // slice_sao_luma_flag = 0
  w.put(0);                                  // slice_sao_chroma_flag = 0
  w.put(1);                                  // num_ref_idx_active_override_flag
  w.ue(15);                                  // num_ref_idx_l0_active_minus1 = 15 -> nl0 = 16
  // lists_modification && NumPicTotalCurr > 1 -- NumPicTotalCurr is 0 here,
  // so nothing is written regardless of nl0.
  w.ue(0);                                   // five_minus_max_num_merge_cand = 0
  w.ue(0);                                   // slice_qp_delta se(0) == 0
  REQUIRE(!pps.chroma_qp_offsets_present);
  REQUIRE(!pps.deblocking_override_enabled);
  const bool lf_present =
      pps.loop_filter_across_slices && !pps.deblocking_disabled;  // sao flags both 0 above
  if (lf_present) w.put(0);                  // slice_loop_filter_across_slices_enabled_flag
  w.put(1);                                  // byte_alignment(): alignment_bit_equal_to_one
  w.align_zero();

  std::vector<uint8_t> nal = {idr[ib], idr[ib + 1]};
  const auto esc = escape(w.bytes().data(), w.bytes().size());
  nal.insert(nal.end(), esc.begin(), esc.end());
  SliceHeader h;
  CHECK(parse_slice_header(nal.data(), nal.size(), sps, pps, &h) == SliceParse::kMalformed);
}

MTEST_MAIN
