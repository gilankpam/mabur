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

MTEST_MAIN
