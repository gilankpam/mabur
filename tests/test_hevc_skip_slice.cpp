// The writer's bins are checked by an independent arithmetic DECODER built
// from the same tables (catches renorm/flush/termination bugs); the syntax
// walk is checked structurally here and against ffmpeg (the only oracle) by
// tools/slices/slicefill_check.py (catches table/context/semantic bugs). The
// GS hardware decoder is not part of it: that is checked on the bench.
#include <vector>
#include "mabur/hevc_cabac.h"
#include "mabur/hevc_ps.h"
#include "mabur/hevc_skip_slice.h"
#include "mabur/hevc_slice.h"
#include "mtest.h"
#include "slice_fixture.h"
using namespace mabur::hevc;

namespace {
struct Dec {
  const std::vector<uint8_t>& b;
  size_t pos;
  uint32_t range = 510, off = 0;
  int bit() { const int v = (b[pos >> 3] >> (7 - (pos & 7))) & 1; ++pos; return v; }
  void start() { for (int i = 0; i < 9; ++i) off = (off << 1) | static_cast<uint32_t>(bit()); }
  int decode(CabacCtx& c) {
    const uint32_t lps = kRangeTabLps[c.state][(range >> 6) & 3];
    range -= lps;
    int bin;
    if (off >= range) {
      bin = !c.mps; off -= range; range = lps;
      if (c.state == 0) c.mps = static_cast<uint8_t>(1 - c.mps);
      c.state = kTransIdxLps[c.state];
    } else {
      bin = c.mps;
      if (c.state < 62) ++c.state;
    }
    while (range < 256) { range <<= 1; off = (off << 1) | static_cast<uint32_t>(bit()); }
    return bin;
  }
  int terminate() {
    range -= 2;
    if (off >= range) return 1;
    while (range < 256) { range <<= 1; off = (off << 1) | static_cast<uint32_t>(bit()); }
    return 0;
  }
};

struct Env { ParamTracker t; std::vector<std::vector<uint8_t>> aus; SliceHeader tmpl; };
Env env(size_t au, size_t slice) {
  Env e;
  e.aus = mtest::load_slice_fixture();
  e.t.feed(e.aus[3].data(), e.aus[3].size());
  const auto nal = mtest::slice_nals(e.aus[au])[slice];
  const size_t b = nal[2] == 1 ? 3 : 4;
  REQUIRE(parse_slice_header(nal.data() + b, nal.size() - b, e.t.sps(), e.t.pps(), &e.tmpl) == SliceParse::kOk);
  return e;
}
}  // namespace

TEST(steps_for_a_full_ctu_row_and_the_bottom_edge) {
  Env e = env(5, 1);
  // CTU row 1 (30 CTUs, CTB 64): split=0, skip=1, merge_idx=0, end per CTU.
  auto s = skip_fill_steps(e.t.sps(), 2, 30, 60);
  REQUIRE(s.size() == 30 * 4);
  CHECK(s[0].kind == FillBin::kSplit && s[0].value == 0 && s[0].ctx_inc == 0);
  CHECK(s[1].kind == FillBin::kSkip && s[1].value == 1 && s[1].ctx_inc == 0);   // no left, no above in-slice
  CHECK(s[2].kind == FillBin::kMergeIdx && s[2].value == 0);
  CHECK(s[3].kind == FillBin::kEndOfSlice && s[3].value == 0);
  CHECK(s[4 + 1].kind == FillBin::kSkip && s[4 + 1].ctx_inc == 1);              // left skip CU
  CHECK(s.back().kind == FillBin::kEndOfSlice && s.back().value == 1);
  // Two rows: the second row sees its above neighbour in the same slice.
  auto two = skip_fill_steps(e.t.sps(), 2, 30, 90);
  CHECK(two[30 * 4 + 4 + 1].ctx_inc == 2);
  // Bottom row 16 (y 1024..1079): 64x64 crosses the edge -> implicit split;
  // per CTU 2 coded 32x32 splits, 4 coded 16x16 splits, 2+4+8 skip CUs.
  auto bot = skip_fill_steps(e.t.sps(), 2, 16 * 30, 16 * 30 + 1);
  size_t splits = 0, skips = 0, merges = 0;
  for (auto& st : bot) {
    splits += st.kind == FillBin::kSplit;
    skips += st.kind == FillBin::kSkip;
    merges += st.kind == FillBin::kMergeIdx;
  }
  CHECK(splits == 6);
  CHECK(skips == 14);
  CHECK(merges == 14);
}

TEST(fill_header_and_bins_decode_back) {
  Env e = env(5, 1);
  for (const auto& range : std::vector<std::pair<uint32_t, uint32_t>>{{0, 150}, {150, 300}, {450, 510}}) {
    auto fill = make_skip_slice(e.t.sps(), e.t.pps(), e.tmpl, range.first, range.second);
    REQUIRE(fill.has_value());
    REQUIRE(fill->size() > 8);
    CHECK((*fill)[0] == 0 && (*fill)[1] == 0 && (*fill)[2] == 0 && (*fill)[3] == 1);
    SliceHeader h;
    REQUIRE(parse_slice_header(fill->data() + 4, fill->size() - 4, e.t.sps(), e.t.pps(), &h) == SliceParse::kOk);
    CHECK(h.first == (range.first == 0));
    CHECK(h.address == range.first);
    CHECK(!h.sao_luma && !h.sao_chroma);
    CHECK(h.nal_type == e.tmpl.nal_type);
    // Decode every bin with the same contexts.
    const auto steps = skip_fill_steps(e.t.sps(), h.max_num_merge_cand, range.first, range.second);
    const int it = h.slice_type == 1 ? (h.cabac_init_flag ? 2 : 1) : (h.cabac_init_flag ? 1 : 2);
    const int split_init[3] = {107, 139, 126}, skip_init[3] = {197, 185, 201};
    const int merge_init = it == 1 ? 122 : 137;
    CabacCtx split[3], skip[3], merge = init_cabac_ctx(merge_init, h.slice_qp_y);
    for (int i = 0; i < 3; ++i) {
      split[i] = init_cabac_ctx(split_init[i], h.slice_qp_y);
      skip[i] = init_cabac_ctx(skip_init[i], h.slice_qp_y);
    }
    Dec d{h.rbsp, h.data_byte * 8};
    d.start();
    for (const auto& st : steps) {
      int v = 0;
      switch (st.kind) {
        case FillBin::kSplit: v = d.decode(split[st.ctx_inc]); break;
        case FillBin::kSkip: v = d.decode(skip[st.ctx_inc]); break;
        case FillBin::kMergeIdx: v = d.decode(merge); break;
        case FillBin::kEndOfSlice: v = d.terminate(); break;
      }
      CHECK(v == st.value);
    }
    // After end_of_slice_segment_flag = 1 the last bit read is the
    // rbsp_stop_one_bit; then zero bits to the end of the RBSP.
    CHECK(((h.rbsp[(d.pos - 1) >> 3] >> (7 - ((d.pos - 1) & 7))) & 1) == 1);
    while (d.pos & 7) CHECK(d.bit() == 0);
    CHECK(d.pos / 8 == h.rbsp.size());
  }
}

TEST(i_slices_are_refused) {
  Env e = env(0, 1);  // IDR slice
  CHECK(e.tmpl.slice_type == 2);
  CHECK(!make_skip_slice(e.t.sps(), e.t.pps(), e.tmpl, 150, 300).has_value());
}

TEST(bad_ranges_are_refused) {
  Env e = env(5, 1);
  CHECK(!make_skip_slice(e.t.sps(), e.t.pps(), e.tmpl, 300, 300).has_value());
  CHECK(!make_skip_slice(e.t.sps(), e.t.pps(), e.tmpl, 450, 511).has_value());
}

MTEST_MAIN
