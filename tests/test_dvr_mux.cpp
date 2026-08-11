#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "dvr_mux.h"
#include "mtest.h"

using maburplay::DvrMux;

namespace {

// ---- tiny fake-AU builder -------------------------------------------
// Fake AU: a single HEVC NAL (type-1 TRAIL_R body — content is
// irrelevant, DvrMux never inspects it) behind a 3-byte start code, so
// annexb_to_length_prefixed inside the mux has real work to do.
std::vector<uint8_t> fake_au(uint8_t tag_byte) {
  return {0x00, 0x00, 0x01, /*hdr*/ 0x02, 0x01, /*payload*/ tag_byte, tag_byte, tag_byte};
}

// The mux's own conversion (mirrors annexb_to_length_prefixed on the one
// NAL above: 2-byte header + 3 payload bytes = 5 bytes -> 4+5=9 mdat
// bytes) — used by the test to predict expected mdat sizes without
// re-including hevc_params.h logic.
size_t fake_au_mdat_bytes() { return 4 + 5; }

std::string scratch_path(const char* name) {
  const char* dir = std::getenv("TMPDIR");
  std::string base = dir ? dir : "/tmp";
  return base + "/" + name;
}

// ---- minimal in-test box walker --------------------------------------
struct Box {
  std::string type;
  size_t off;   // offset of the 4-byte size field
  size_t size;  // total box size, including header
  size_t payload_off() const { return off + 8; }
  size_t payload_size() const { return size - 8; }
};

uint32_t read_u32(const std::vector<uint8_t>& f, size_t p) {
  return (static_cast<uint32_t>(f[p]) << 24) | (static_cast<uint32_t>(f[p + 1]) << 16) |
         (static_cast<uint32_t>(f[p + 2]) << 8) | static_cast<uint32_t>(f[p + 3]);
}

uint64_t read_u64(const std::vector<uint8_t>& f, size_t p) {
  return (static_cast<uint64_t>(read_u32(f, p)) << 32) | static_cast<uint64_t>(read_u32(f, p + 4));
}

// Parses a flat run of size+fourcc boxes in [begin, end). Does not
// recurse — callers narrow the range (skipping any fullbox header first)
// and call again for nested containers.
std::vector<Box> parse_boxes(const std::vector<uint8_t>& f, size_t begin, size_t end) {
  std::vector<Box> out;
  size_t p = begin;
  while (p + 8 <= end) {
    uint32_t sz = read_u32(f, p);
    REQUIRE(sz >= 8);
    REQUIRE(p + sz <= end);
    std::string type(reinterpret_cast<const char*>(&f[p + 4]), 4);
    out.push_back(Box{type, p, sz});
    p += sz;
  }
  REQUIRE(p == end);  // no trailing garbage / truncated box
  return out;
}

const Box* find(const std::vector<Box>& boxes, const char* type) {
  for (auto& b : boxes)
    if (b.type == type) return &b;
  return nullptr;
}

std::vector<uint8_t> read_whole_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  REQUIRE(in.good());
  std::vector<uint8_t> out((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return out;
}

// ISO/IEC 14496-12 VisualSampleEntry fixed layout: SampleEntry(8) +
// VisualSampleEntry fixed fields(70) = 78 bytes precede any extension
// boxes (here, hvcC).
const size_t kVisualSampleEntryFixedBytes = 78;

}  // namespace

TEST(dvr_mux_box_tree_and_fragment_cut) {
  std::string path = scratch_path("dvr_mux_test1.mp4");
  std::remove(path.c_str());

  // Synthetic 23-byte hvcc blob — content is opaque to DvrMux, it must
  // come back out byte-for-byte inside hvcC.
  std::vector<uint8_t> hvcc;
  for (int i = 0; i < 23; ++i) hvcc.push_back(static_cast<uint8_t>(0x40 + i));

  DvrMux mux;
  REQUIRE(mux.open(path, hvcc, 1920, 1080));

  // 5 samples. Sample 0 is the first key AU (starts fragment 1, no cut —
  // nothing pending yet). Sample 2 is a second key AU, forcing a cut:
  // fragment 1 = samples {0,1}, fragment 2 = samples {2,3,4}.
  uint32_t pts[5] = {0, 16683, 33366, 50049, 66732};
  bool key[5] = {true, false, true, false, false};
  for (int i = 0; i < 5; ++i) {
    std::vector<uint8_t> au = fake_au(static_cast<uint8_t>(0x10 + i));
    mux.write_sample(au.data(), au.size(), pts[i], key[i]);
  }
  mux.close();

  CHECK(mux.samples() == 5);
  CHECK(mux.fragments() == 2);

  std::vector<uint8_t> f = read_whole_file(path);

  // --- top level: ftyp, moov, moof, mdat, moof, mdat -------------------
  std::vector<Box> top = parse_boxes(f, 0, f.size());
  REQUIRE(top.size() == 6);
  const char* expect_top[6] = {"ftyp", "moov", "moof", "mdat", "moof", "mdat"};
  for (int i = 0; i < 6; ++i) CHECK(top[i].type == expect_top[i]);

  const Box& ftyp = top[0];
  CHECK(ftyp.payload_size() >= 12);
  CHECK(std::memcmp(&f[ftyp.payload_off()], "iso5", 4) == 0);  // major_brand

  // --- moov: trak, mvex ------------------------------------------------
  const Box& moov = top[1];
  std::vector<Box> moov_kids = parse_boxes(f, moov.payload_off(), moov.off + moov.size);
  const Box* trak = find(moov_kids, "trak");
  const Box* mvex = find(moov_kids, "mvex");
  REQUIRE(trak != nullptr);
  REQUIRE(mvex != nullptr);

  // --- walk down to stsd -------------------------------------------
  std::vector<Box> trak_kids = parse_boxes(f, trak->payload_off(), trak->off + trak->size);
  const Box* mdia = find(trak_kids, "mdia");
  REQUIRE(mdia != nullptr);
  std::vector<Box> mdia_kids = parse_boxes(f, mdia->payload_off(), mdia->off + mdia->size);
  const Box* minf = find(mdia_kids, "minf");
  REQUIRE(minf != nullptr);
  std::vector<Box> minf_kids = parse_boxes(f, minf->payload_off(), minf->off + minf->size);
  const Box* stbl = find(minf_kids, "stbl");
  REQUIRE(stbl != nullptr);
  std::vector<Box> stbl_kids = parse_boxes(f, stbl->payload_off(), stbl->off + stbl->size);
  const Box* stsd = find(stbl_kids, "stsd");
  REQUIRE(stsd != nullptr);

  // stsd is a FullBox with an entry_count before its entries: skip the
  // 4-byte version/flags + 4-byte entry_count.
  std::vector<Box> stsd_entries = parse_boxes(f, stsd->payload_off() + 8, stsd->off + stsd->size);
  const Box* hvc1 = find(stsd_entries, "hvc1");
  REQUIRE(hvc1 != nullptr);

  std::vector<Box> hvc1_kids =
      parse_boxes(f, hvc1->payload_off() + kVisualSampleEntryFixedBytes, hvc1->off + hvc1->size);
  const Box* hvcC = find(hvc1_kids, "hvcC");
  REQUIRE(hvcC != nullptr);
  CHECK(hvcC->payload_size() == hvcc.size());
  CHECK(std::memcmp(&f[hvcC->payload_off()], hvcc.data(), hvcc.size()) == 0);

  // --- fragments: moof/mdat pairs --------------------------------------
  const Box& moof1 = top[2];
  const Box& mdat1 = top[3];
  const Box& moof2 = top[4];
  const Box& mdat2 = top[5];

  auto check_fragment = [&](const Box& moof, const Box& mdat, uint32_t expect_seq,
                             size_t expect_sample_count, uint64_t expect_tfdt,
                             uint32_t expect_dur_per_sample) {
    std::vector<Box> moof_kids = parse_boxes(f, moof.payload_off(), moof.off + moof.size);
    const Box* mfhd = find(moof_kids, "mfhd");
    const Box* traf = find(moof_kids, "traf");
    REQUIRE(mfhd != nullptr);
    REQUIRE(traf != nullptr);
    CHECK(read_u32(f, mfhd->payload_off() + 4) == expect_seq);

    std::vector<Box> traf_kids = parse_boxes(f, traf->payload_off(), traf->off + traf->size);
    const Box* tfhd = find(traf_kids, "tfhd");
    const Box* tfdt = find(traf_kids, "tfdt");
    const Box* trun = find(traf_kids, "trun");
    REQUIRE(tfhd != nullptr);
    REQUIRE(tfdt != nullptr);
    REQUIRE(trun != nullptr);

    // tfhd: version0, flags 0x020000 (default-base-is-moof), track_id 1.
    CHECK(read_u32(f, tfhd->payload_off()) == 0x00020000u);
    CHECK(read_u32(f, tfhd->payload_off() + 4) == 1u);

    // tfdt: version1 -> 64-bit baseMediaDecodeTime.
    CHECK(read_u32(f, tfdt->payload_off()) == 0x01000000u);
    CHECK(read_u64(f, tfdt->payload_off() + 4) == expect_tfdt);

    // trun: flags 0x000701 (data-offset|duration|size|flags present).
    CHECK(read_u32(f, trun->payload_off()) == 0x00000701u);
    uint32_t sample_count = read_u32(f, trun->payload_off() + 4);
    CHECK(sample_count == expect_sample_count);
    uint32_t data_offset = read_u32(f, trun->payload_off() + 8);
    CHECK(data_offset == moof.size + 8);  // first mdat payload byte, moof-relative

    size_t p = trun->payload_off() + 12;
    size_t mdat_expect = 0;
    for (size_t i = 0; i < sample_count; ++i) {
      // Every sample's duration, including the last one (which has no
      // next sample in this fragment to measure against, so it must
      // reuse the last real delta — never 0).
      uint32_t dur = read_u32(f, p + 0);
      CHECK(dur == expect_dur_per_sample);
      uint32_t size = read_u32(f, p + 4);
      CHECK(size == fake_au_mdat_bytes());
      mdat_expect += size;
      p += 12;
    }
    CHECK(mdat.payload_size() == mdat_expect);
  };

  // All five samples are spaced by a constant 16683us delta, so every
  // trun entry in both fragments — including each fragment's last
  // sample — must carry that same duration.
  check_fragment(moof1, mdat1, 1, 2, 0, 16683);
  check_fragment(moof2, mdat2, 2, 3, pts[2], 16683);  // sample 2 starts fragment 2, no wrap yet

  // Explicit sample_flags check, keyed off which global sample each trun
  // entry corresponds to (fragment1: samples 0,1; fragment2: samples 2,3,4).
  auto flags_at = [&](const Box& moof, size_t sample_index_in_fragment) -> const uint8_t* {
    std::vector<Box> moof_kids = parse_boxes(f, moof.payload_off(), moof.off + moof.size);
    const Box* traf = find(moof_kids, "traf");
    std::vector<Box> traf_kids = parse_boxes(f, traf->payload_off(), traf->off + traf->size);
    const Box* trun = find(traf_kids, "trun");
    size_t p = trun->payload_off() + 12 + sample_index_in_fragment * 12 + 8;
    return &f[p];
  };
  CHECK(std::memcmp(flags_at(moof1, 0), "\x02\x00\x00\x00", 4) == 0);  // sample0 key
  CHECK(std::memcmp(flags_at(moof1, 1), "\x01\x01\x00\x00", 4) == 0);  // sample1 non-key
  CHECK(std::memcmp(flags_at(moof2, 0), "\x02\x00\x00\x00", 4) == 0);  // sample2 key
  CHECK(std::memcmp(flags_at(moof2, 1), "\x01\x01\x00\x00", 4) == 0);  // sample3 non-key
  CHECK(std::memcmp(flags_at(moof2, 2), "\x01\x01\x00\x00", 4) == 0);  // sample4 non-key
}

TEST(dvr_mux_pts_wrap_tfdt_strictly_increasing) {
  std::string path = scratch_path("dvr_mux_test2.mp4");
  std::remove(path.c_str());

  std::vector<uint8_t> hvcc(23, 0xAB);

  DvrMux mux;
  REQUIRE(mux.open(path, hvcc, 1920, 1080));

  // Sample 0: pts near the u32 ceiling, key (fragment 1 start).
  // Sample 1: pts wrapped to a tiny value, non-key (still fragment 1 —
  // the true elapsed time is small, well under the 1s default cut).
  // Sample 2: pts continuing forward, key (forces the cut into
  // fragment 2). tfdt must reflect unwrapped (monotonic) time, not the
  // raw wrapped u32.
  std::vector<uint8_t> au = fake_au(0xEE);
  mux.write_sample(au.data(), au.size(), 0xFFFFFFF0u, true);
  mux.write_sample(au.data(), au.size(), 0x00000005u, false);
  mux.write_sample(au.data(), au.size(), 0x00004145u, true);
  mux.close();

  CHECK(mux.fragments() == 2);

  std::vector<uint8_t> f = read_whole_file(path);
  std::vector<Box> top = parse_boxes(f, 0, f.size());
  REQUIRE(top.size() == 6);
  const Box& moof1 = top[2];
  const Box& moof2 = top[4];

  auto read_tfdt = [&](const Box& moof) -> uint64_t {
    std::vector<Box> moof_kids = parse_boxes(f, moof.payload_off(), moof.off + moof.size);
    const Box* traf = find(moof_kids, "traf");
    std::vector<Box> traf_kids = parse_boxes(f, traf->payload_off(), traf->off + traf->size);
    const Box* tfdt = find(traf_kids, "tfdt");
    return read_u64(f, tfdt->payload_off() + 4);
  };

  uint64_t tfdt1 = read_tfdt(moof1);
  uint64_t tfdt2 = read_tfdt(moof2);

  // The recording timeline is REBASED to zero at the first sample (the
  // raw capture pts is encoder-session-relative; absolute tfdt made
  // players front-pad the seekbar with the session's age).
  CHECK(tfdt1 == 0);
  // Unwrap across the u32 wrap: delta(0x00000005, 0xFFFFFFF0) = 0x15 = 21
  // -> t=21. Then delta(0x00004145, 0x00000005) = 0x4140 = 16704
  // -> t = 21 + 16704.
  uint64_t expect_tfdt2 = 21ull + 16704ull;
  CHECK(tfdt2 == expect_tfdt2);
  CHECK(tfdt2 > tfdt1);  // strictly increasing despite the raw u32 wrap

  // Duration coverage: fragment 1 has 2 samples (real delta 21us, from
  // the wrap-unwrap above), so both its trun entries — including the
  // last — must read 21. Fragment 2 has exactly one sample (sample 2
  // forces the cut and close() flushes immediately after), so its lone
  // trun entry has no next-sample delta to measure and must fall back
  // to the last real delta carried over from fragment 1: also 21, never
  // 0.
  auto read_trun_durations = [&](const Box& moof) -> std::vector<uint32_t> {
    std::vector<Box> moof_kids = parse_boxes(f, moof.payload_off(), moof.off + moof.size);
    const Box* traf = find(moof_kids, "traf");
    std::vector<Box> traf_kids = parse_boxes(f, traf->payload_off(), traf->off + traf->size);
    const Box* trun = find(traf_kids, "trun");
    uint32_t sample_count = read_u32(f, trun->payload_off() + 4);
    std::vector<uint32_t> out;
    size_t p = trun->payload_off() + 12;
    for (uint32_t i = 0; i < sample_count; ++i) {
      out.push_back(read_u32(f, p));
      p += 12;
    }
    return out;
  };

  std::vector<uint32_t> durs1 = read_trun_durations(moof1);
  REQUIRE(durs1.size() == 2);
  CHECK(durs1[0] == 21u);
  CHECK(durs1[1] == 21u);  // last sample of fragment 1 reuses the real delta

  std::vector<uint32_t> durs2 = read_trun_durations(moof2);
  REQUIRE(durs2.size() == 1);
  CHECK(durs2[0] == 21u);  // lone-sample fragment: carried, never 0
}

// A second recording on the SAME DvrMux must be an independent file. The
// toggle re-opens the mux on every press, and before the per-file reset
// the first file's queued samples flushed into the second one's first
// fragment and its PTS origin came along with them.
TEST(reopen_starts_a_clean_file) {
  const std::string p1 = scratch_path("dvr_reopen_1.mp4");
  const std::string p2 = scratch_path("dvr_reopen_2.mp4");
  std::vector<uint8_t> hvcc(23, 0xCD);

  DvrMux m;
  REQUIRE(m.open(p1, hvcc, 1920, 1080, 1000));
  // Three samples, none of which cuts a fragment: key only on the first,
  // and 3 x 16.7 ms is far short of fragment_ms. Two therefore sit in
  // pending_ when close() flushes.
  std::vector<uint8_t> a1 = fake_au(0xA1);
  std::vector<uint8_t> a2 = fake_au(0xA2);
  std::vector<uint8_t> a3 = fake_au(0xA3);
  m.write_sample(a1.data(), a1.size(), 0, true);
  m.write_sample(a2.data(), a2.size(), 16667, false);
  m.write_sample(a3.data(), a3.size(), 33334, false);
  m.close();
  CHECK(m.samples() == 3);

  // Second file: one sample, key, pts restarting from 0 as a fresh
  // session's would.
  REQUIRE(m.open(p2, hvcc, 1920, 1080, 1000));
  CHECK(m.samples() == 0);      // counters are per file
  CHECK(m.fragments() == 0);
  std::vector<uint8_t> b1 = fake_au(0xB1);
  m.write_sample(b1.data(), b1.size(), 0, true);
  m.close();
  CHECK(m.samples() == 1);

  // File 2 must contain exactly ONE sample's worth of mdat payload. With
  // the leak it carried file 1's two pending samples as well.
  const std::vector<uint8_t> f2 = read_whole_file(p2);
  std::vector<Box> top2 = parse_boxes(f2, 0, f2.size());
  size_t mdat_bytes = 0;
  for (const Box& b : top2)
    if (b.type == "mdat") mdat_bytes += b.payload_size();
  CHECK(mdat_bytes == fake_au_mdat_bytes());

  // And none of file 1's payload tags may appear anywhere in file 2.
  bool leaked = false;
  for (size_t i = 0; i < f2.size(); ++i)
    if (f2[i] == 0xA1 || f2[i] == 0xA2 || f2[i] == 0xA3) leaked = true;
  CHECK(!leaked);

  std::remove(p1.c_str());
  std::remove(p2.c_str());
}

MTEST_MAIN
