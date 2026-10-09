#include <algorithm>
#include <cstring>
#include <vector>
#include "mabur/hevc_ps.h"
#include "mabur/hevc_skip_slice.h"
#include "mabur/hevc_slice.h"
#include "mtest.h"
#include "slice_assembler.h"
#include "slice_fixture.h"
using namespace maburgs;
using namespace mabur::hevc;

namespace {
constexpr size_t kHdr = 8;  // FrameHdr bytes at the front of chunk 0

struct Au {
  ParamTracker t;
  std::vector<uint8_t> bytes;                 // the original Annex-B AU
  std::vector<std::vector<uint8_t>> slices;   // its slice NALs
  ChunkMap chunks;                            // all fragments
  uint16_t count = 0;
  size_t frag = 0;
};

// Fixture AU `index` cut into fragments of `frag` payload bytes (production:
// 324, the 332-byte symbol geometry), FrameHdr stand-in at the front.
Au au(size_t index, size_t frag = 324) {
  Au a;
  const auto aus = mtest::load_slice_fixture();
  a.t.feed(aus[3].data(), aus[3].size());
  a.bytes = aus[index];
  a.slices = mtest::slice_nals(a.bytes);
  a.frag = frag;
  std::vector<uint8_t> unit(kHdr, 0xEE);
  unit.insert(unit.end(), a.bytes.begin(), a.bytes.end());
  for (size_t off = 0, i = 0; off < unit.size(); off += frag, ++i)
    a.chunks[static_cast<uint16_t>(i)].assign(unit.begin() + static_cast<long>(off),
                                              unit.begin() + static_cast<long>(std::min(unit.size(), off + frag)));
  a.count = static_cast<uint16_t>(a.chunks.size());
  return a;
}

size_t offset_of(const Au& a, size_t slice) {   // AU byte offset of slice's start code
  const auto it = std::search(a.bytes.begin(), a.bytes.end(), a.slices[slice].begin(), a.slices[slice].end());
  return static_cast<size_t>(it - a.bytes.begin());
}
uint16_t chunk_of(const Au& a, size_t au_off) { return static_cast<uint16_t>((au_off + kHdr) / a.frag); }
// A fragment strictly inside slice k: neither the one holding k's start code
// nor the one holding k+1's.
uint16_t inner_chunk(const Au& a, size_t k) {
  const uint16_t c = static_cast<uint16_t>(chunk_of(a, offset_of(a, k)) + 1);
  if (k + 1 < a.slices.size()) REQUIRE(c < chunk_of(a, offset_of(a, k + 1)));
  return c;
}

struct Out {
  std::vector<uint8_t> b;
  ByteSink sink() { return [this](const uint8_t* p, size_t n) { b.insert(b.end(), p, p + n); }; }
};

void check_salvaged_picture(const Au& a, const std::vector<uint8_t>& out, const std::vector<bool>& expect_kept) {
  const auto got = mtest::slice_nals(out);
  REQUIRE(got.size() == a.slices.size());
  for (size_t k = 0; k < got.size(); ++k) {
    if (expect_kept[k]) {
      CHECK(got[k] == a.slices[k]);
    } else {
      CHECK(got[k] != a.slices[k]);
      SliceHeader h;
      REQUIRE(parse_slice_header(got[k].data() + 4, got[k].size() - 4, a.t.sps(), a.t.pps(), &h) == SliceParse::kOk);
      CHECK(h.address == 150u * k);
      CHECK(h.first == (k == 0));
      // Exactly the fill the assembler must build: the template is the first
      // slice that arrived whole. A kept-but-damaged slice (truncated, or
      // with a foreign tail) differs from this and fails here.
      const size_t t = static_cast<size_t>(std::find(expect_kept.begin(), expect_kept.end(), true) - expect_kept.begin());
      REQUIRE(t < a.slices.size());
      SliceHeader tmpl;
      REQUIRE(parse_slice_header(a.slices[t].data() + 4, a.slices[t].size() - 4, a.t.sps(), a.t.pps(), &tmpl) == SliceParse::kOk);
      const auto fill = make_skip_slice(a.t.sps(), a.t.pps(), tmpl, static_cast<uint32_t>(150 * k),
                                        static_cast<uint32_t>(std::min<size_t>(150 * (k + 1), 510)));
      REQUIRE(fill.has_value());
      CHECK(got[k] == *fill);
    }
  }
}

SliceSalvage run(Au& a, std::vector<uint8_t>* out, uint8_t slice_rows = 5) {
  SliceAssembler sa(a.t.sps(), a.t.pps(), slice_rows, a.count, kHdr);
  Out o;
  sa.finish(a.chunks, o.sink());
  *out = o.b;
  return sa.result();
}
}  // namespace

TEST(complete_au_streams_identical_bytes) {
  Au a = au(5);
  SliceAssembler sa(a.t.sps(), a.t.pps(), 5, a.count, kHdr);
  Out o;
  ChunkMap partial;
  for (auto& [i, c] : a.chunks) { partial[i] = c; sa.drain(partial, o.sink()); }
  sa.finish(partial, o.sink());
  CHECK(o.b == a.bytes);
  CHECK(!sa.result().salvaged);
  CHECK(sa.result().slices == 4 && sa.result().kept == 4);
}

TEST(hole_in_slice_1_keeps_0_2_3_and_fills_1) {
  Au a = au(5);
  a.chunks.erase(inner_chunk(a, 1));
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out);
  REQUIRE(r.salvaged);
  CHECK(r.kept == 3);
  CHECK(r.filled == 1);
  CHECK(r.kept_after_hole == 2);
  check_salvaged_picture(a, out, {true, false, true, true});
}

TEST(tail_lost_fills_the_tail) {
  Au a = au(5);
  const uint16_t keep = chunk_of(a, offset_of(a, 2));     // slice 2's start code stays
  REQUIRE(chunk_of(a, offset_of(a, 2) + 3) == keep);      // whole code in that fragment
  for (uint16_t i = static_cast<uint16_t>(keep + 1); i < a.count; ++i) a.chunks.erase(i);
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out);
  REQUIRE(r.salvaged);
  CHECK(r.kept_after_hole == 0);
  check_salvaged_picture(a, out, {true, true, false, false});
}

TEST(slice_0_lost_is_filled_from_a_later_template) {
  Au a = au(5);
  a.chunks.erase(inner_chunk(a, 0));
  std::vector<uint8_t> out;
  REQUIRE(run(a, &out).salvaged);
  check_salvaged_picture(a, out, {false, true, true, true});
}

TEST(start_code_split_by_hole_is_not_a_slice) {       // Review Focus 1
  // Pick a fragment size whose boundary falls 2 bytes into slice 2's
  // 00 00 00 01: "00 00" ends one run, "00 01" opens the missing fragment.
  const size_t sc = offset_of(au(5), 2);
  size_t F = 0;
  for (size_t f = 200; f <= 600 && !F; ++f)
    if ((sc + kHdr + 2) % f == 0) F = f;
  REQUIRE(F != 0);
  Au a = au(5, F);
  const uint16_t c = static_cast<uint16_t>((sc + kHdr + 2) / F);
  REQUIRE(chunk_of(a, offset_of(a, 3)) > c);              // slice 3 untouched
  a.chunks.erase(c);
  std::vector<uint8_t> out;
  REQUIRE(run(a, &out).salvaged);
  // Slice 1's end is unknown (its successor's code is cut), slice 2's start
  // is cut: both filled, never "kept" from a half-seen start code.
  check_salvaged_picture(a, out, {true, false, false, true});
}

TEST(slice_ending_at_hole_boundary_is_filled) {       // Review Focus 2
  // Production geometry: lose the fragment holding slice 2's start code
  // (it also holds slice 1's tail) -> 1 and 2 are filled.
  {
    Au a = au(5);
    const uint16_t c = chunk_of(a, offset_of(a, 2));
    REQUIRE((offset_of(a, 2) + kHdr) % a.frag != 0);      // tail of slice 1 in c
    a.chunks.erase(c);
    std::vector<uint8_t> out;
    REQUIRE(run(a, &out).salvaged);
    check_salvaged_picture(a, out, {true, false, false, true});
  }
  // The exact edge: a fragment size where slice 1's start code OPENS a
  // fragment, so slice 0 ends on the last byte of the fragment before it.
  // Lose the code's fragment: slice 0 arrived byte-complete, but its end is
  // unknowable (nothing says the run end is a NAL end) -> 0 and 1 filled.
  const size_t sc = offset_of(au(5), 1);
  size_t F = 0;
  for (size_t f = 200; f <= 800 && !F; ++f)
    if ((sc + kHdr) % f == 0) F = f;
  REQUIRE(F != 0);
  Au a = au(5, F);
  const uint16_t c = static_cast<uint16_t>((sc + kHdr) / F);
  REQUIRE(chunk_of(a, offset_of(a, 2)) > c);              // slice 2 untouched
  a.chunks.erase(c);
  std::vector<uint8_t> out;
  REQUIRE(run(a, &out).salvaged);
  check_salvaged_picture(a, out, {false, false, true, true});
}

TEST(start_code_tail_opening_a_run_is_not_trusted) {  // Review Focus 1, other half
  // A fragment size whose boundary falls 1 byte into slice 1's 00 00 00 01
  // and the fragment before it is lost: the run after the hole opens with
  // "00 00 01" -- a 3-byte code that is really the tail of a 4-byte one.
  // Its slice did not arrive whole (a start-code byte is lost); keeping it
  // would emit non-original bytes. Filled, like any other damaged slice.
  const size_t sc = offset_of(au(5), 1);
  size_t F = 0;
  for (size_t f = 100; f <= 800 && !F; ++f)
    if ((sc + kHdr + 1) % f == 0) F = f;
  REQUIRE(F != 0);
  Au a = au(5, F);
  const uint16_t c = static_cast<uint16_t>((sc + kHdr + 1) / F);
  REQUIRE(a.chunks.at(c)[0] == 0 && a.chunks.at(c)[1] == 0 && a.chunks.at(c)[2] == 1);
  REQUIRE(chunk_of(a, offset_of(a, 2)) > c);
  a.chunks.erase(static_cast<uint16_t>(c - 1));
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out);
  REQUIRE(r.salvaged);
  CHECK(r.kept == 2);
  check_salvaged_picture(a, out, {false, false, true, true});
}

TEST(start_code_straddling_drains_is_recorded_once) {
  // Every fragment size 200..600, drained one fragment at a time, with a
  // hole inside slice 3. Some of these sizes cut a slice's 00 00 00 01
  // across two drains (+1/+2/+3 bytes in) and some end it exactly at a
  // fragment end (+4): the drain that sees it whole must record it once,
  // and the next drain's 3-byte rescan window ("00 00 01") must not record
  // a second, 3-byte code one byte later -- that miscounts the slices
  // drain() already streamed and throws the salvage away.
  int cut[5] = {0, 0, 0, 0, 0};
  for (size_t F = 200; F <= 600; ++F) {
    Au a = au(5, F);
    for (size_t k = 1; k < 4; ++k) {
      const size_t into = (offset_of(a, k) + kHdr + 4) % F;
      if (into < 4) ++cut[into == 0 ? 4 : into];
    }
    a.chunks.erase(inner_chunk(a, 3));
    SliceAssembler s1(a.t.sps(), a.t.pps(), 5, a.count, kHdr);
    Out o1;
    ChunkMap partial;
    for (auto& [i, c] : a.chunks) { partial[i] = c; s1.drain(partial, o1.sink()); }
    s1.finish(partial, o1.sink());
    REQUIRE(s1.result().salvaged);
    check_salvaged_picture(a, o1.b, {true, true, true, false});
    std::vector<uint8_t> o2;
    run(a, &o2);
    CHECK(o1.b == o2);
  }
  for (int i = 1; i <= 4; ++i) REQUIRE(cut[i] > 0);   // every straddle shape exercised
}

TEST(missing_last_fragment_fills_last_slice) {        // Review Focus 3
  Au a = au(5);
  REQUIRE(chunk_of(a, offset_of(a, 3)) < a.count - 1);
  a.chunks.erase(static_cast<uint16_t>(a.count - 1));
  std::vector<uint8_t> out;
  REQUIRE(run(a, &out).salvaged);
  check_salvaged_picture(a, out, {true, true, true, false});
}

TEST(drained_prefix_plus_finish_equals_one_shot_finish) {
  Au a = au(5);
  a.chunks.erase(inner_chunk(a, 2));
  SliceAssembler s1(a.t.sps(), a.t.pps(), 5, a.count, kHdr);
  Out o1;
  ChunkMap partial;
  for (auto& [i, c] : a.chunks) { partial[i] = c; s1.drain(partial, o1.sink()); }
  s1.finish(partial, o1.sink());
  std::vector<uint8_t> o2;
  run(a, &o2);
  CHECK(o1.b == o2);
}

TEST(i_slice_picture_passes_through) {
  Au a = au(0);  // IDR
  const uint16_t c = chunk_of(a, offset_of(a, 2));
  a.chunks.erase(c);
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out);
  CHECK(!r.salvaged);
  CHECK(r.fallback == kSliceFbISlice);
  // Passthrough = today's truncated output: the contiguous prefix.
  CHECK(out == std::vector<uint8_t>(a.bytes.begin(), a.bytes.begin() + static_cast<long>(c * a.frag - kHdr)));
}

TEST(no_complete_slice_passes_through) {
  Au a = au(5);
  for (uint16_t i = 1; i < a.count; i += 2) a.chunks.erase(i);   // every other fragment
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out);
  CHECK(!r.salvaged);
  CHECK(r.fallback == kSliceFbNoTemplate);
}

TEST(geometry_mismatch_passes_through) {             // Review Focus 5
  Au a = au(5);
  a.chunks.erase(inner_chunk(a, 1));
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out, /*slice_rows=*/4);   // real geometry is 5
  CHECK(!r.salvaged);
  CHECK(r.fallback == kSliceFbGeometry);
}

TEST(short_prefix_fragment_passes_through) {
  // A non-last fragment shorter than F inside the contiguous prefix: the
  // byte offsets of everything after it are wrong, and slice 0 (which
  // spans it) is missing bytes -- never "kept". Passthrough.
  Au a = au(5);
  a.chunks.at(1).resize(a.frag - 10);
  a.chunks.erase(inner_chunk(a, 2));
  std::vector<uint8_t> out;
  const SliceSalvage r = run(a, &out);
  CHECK(!r.salvaged);
  CHECK(r.fallback == kSliceFbGeometry);
}

MTEST_MAIN
