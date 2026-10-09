#include <vector>
#include "mabur/hevc_params.h"
#include "mabur/hevc_ps.h"
#include "mtest.h"
#include "slice_fixture.h"
using namespace mabur::hevc;

namespace {
const mabur::NalView* find_nal(const std::vector<mabur::NalView>& v, uint8_t type) {
  for (auto& n : v) if (n.type == type) return &n;
  return nullptr;
}
}  // namespace

TEST(fixture_has_seven_aus_with_expected_slice_counts) {
  const auto aus = mtest::load_slice_fixture();
  REQUIRE(aus.size() == 7);
  const size_t expect[7] = {4, 4, 4, 1, 4, 4, 4};
  for (size_t i = 0; i < 7; ++i) CHECK(mtest::slice_nals(aus[i]).size() == expect[i]);
}

TEST(sps_of_the_bench_encoder) {
  const auto aus = mtest::load_slice_fixture();
  const auto nals = mabur::split_nals(aus[3].data(), aus[3].size());
  const auto* s = find_nal(nals, 33);
  REQUIRE(s);
  Sps sps;
  REQUIRE(parse_sps(s->p, s->n, &sps) == PsStatus::kOk);
  CHECK(sps.width == 1920);
  CHECK(sps.height == 1080);
  CHECK(sps.chroma_format_idc == 1);
  CHECK(sps.log2_ctb == 6);
  CHECK(sps.log2_min_cb == 3);
  CHECK(sps.pic_w_ctbs() == 30);
  CHECK(sps.pic_h_ctbs() == 17);
  CHECK(sps.addr_bits() == 9);
  CHECK(sps.sao);
  CHECK(sps.tmvp);
  CHECK(sps.log2_max_poc_lsb == 16);
  CHECK(sps.num_st_rps == 3);
  CHECK(!sps.long_term);
}

TEST(pps_of_the_bench_encoder) {
  const auto aus = mtest::load_slice_fixture();
  const auto nals = mabur::split_nals(aus[3].data(), aus[3].size());
  const auto* p = find_nal(nals, 34);
  REQUIRE(p);
  Pps pps;
  REQUIRE(parse_pps(p->p, p->n, &pps) == PsStatus::kOk);
  CHECK(pps.dependent_slices);          // enabled, though the SDK never uses them
  CHECK(pps.lists_modification);        // present: the header parser must handle it
  CHECK(pps.loop_filter_across_slices);
  CHECK(!pps.cabac_init_present);
  CHECK(pps.init_qp == 26);
  CHECK(!pps.deblocking_override_enabled);
}

TEST(tracker_becomes_usable_from_a_parameter_set_au) {
  const auto aus = mtest::load_slice_fixture();
  ParamTracker t;
  CHECK(!t.usable());
  t.feed(aus[4].data(), aus[4].size());   // plain P: nothing to learn
  CHECK(!t.usable());
  t.feed(aus[3].data(), aus[3].size());   // refresh start carries VPS/SPS/PPS
  CHECK(t.usable());
  CHECK(t.sps().width == 1920);
  t.reset();
  CHECK(!t.usable());
}

TEST(tracker_reports_unsupported_streams) {
  // A PPS with tiles_enabled_flag set: hand-built RBSP, escaped.
  // pps_id ue(0)=1, sps_id ue(0)=1, dep 0, out 0, extra 000, sdh 0, cabac_init 0,
  // l0 ue(0)=1, l1 ue(0)=1, init_qp se(0)=1, cip 0, tskip 0, cuqpd 0,
  // cb se(0)=1, cr se(0)=1, chroma_off 0, wp 0, wbp 0, tq 0, tiles 1, ...
  mabur::hevc::BitWriter w;
  w.ue(0); w.ue(0); w.u(0, 1); w.u(0, 1); w.u(0, 3); w.u(0, 1); w.u(0, 1);
  w.ue(0); w.ue(0); w.ue(0); w.u(0, 1); w.u(0, 1); w.u(0, 1);
  w.ue(0); w.ue(0); w.u(0, 1); w.u(0, 1); w.u(0, 1); w.u(0, 1); w.u(1, 1);
  w.put(1); w.align_zero();
  std::vector<uint8_t> nal = {0x44, 0x01};
  const auto esc = mabur::hevc::escape(w.bytes().data(), w.bytes().size());
  nal.insert(nal.end(), esc.begin(), esc.end());
  Pps pps;
  CHECK(parse_pps(nal.data(), nal.size(), &pps) == PsStatus::kUnsupported);
}

MTEST_MAIN
