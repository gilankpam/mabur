#include <cstring>
#include <vector>
#include "hevc_params.h"
#include "mtest.h"

using maburplay::annexb_to_length_prefixed;
using maburplay::au_is_irap;
using maburplay::HevcParams;
using maburplay::NalView;
using maburplay::split_nals;

namespace {

std::vector<uint8_t> cat(std::initializer_list<std::vector<uint8_t>> parts) {
  std::vector<uint8_t> out;
  for (auto& p : parts) out.insert(out.end(), p.begin(), p.end());
  return out;
}

const std::vector<uint8_t> kSc3 = {0x00, 0x00, 0x01};
const std::vector<uint8_t> kSc4 = {0x00, 0x00, 0x00, 0x01};

// Minimal synthetic NAL payloads. Header byte0 = (type << 1), byte1 = 1
// (temporal_id_plus1 = 1, layer_id = 0) — matches (byte0>>1)&0x3F decode.
std::vector<uint8_t> nal_header(uint8_t type) { return {static_cast<uint8_t>(type << 1), 0x01}; }

// VPS(32): header + a few arbitrary payload bytes (contents don't matter —
// HevcParams only stores/re-emits VPS verbatim).
std::vector<uint8_t> make_vps() {
  return cat({nal_header(32), {0x0C, 0x0D, 0x0E}});
}

// PPS(34): header + arbitrary payload.
std::vector<uint8_t> make_pps() {
  return cat({nal_header(34), {0xCA, 0xFE}});
}

// SPS(33): header, then byte[2] (vps_id/sublayers/nesting, arbitrary),
// then the 12-byte profile_tier_level at byte[3..14] with a distinctive
// sequential pattern so the hvcC copy is easy to assert on, then a couple
// of trailing bytes (rest of the real SPS — irrelevant to hvcC).
std::vector<uint8_t> make_sps() {
  return cat({nal_header(33), {0xAB},
              {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B},
              {0xFF, 0xFF}});
}

}  // namespace

TEST(split_nals_counts_types_both_start_codes) {
  // IDR_W_RADL (19), TRAIL_R (1), back-to-back with no gap, mixing 3- and
  // 4-byte start codes, plus a trailing start code with nothing after it
  // (must be skipped, not emitted as an empty NAL).
  std::vector<uint8_t> idr = cat({nal_header(19), {0xAA, 0xBB, 0xCC}});
  std::vector<uint8_t> trail = cat({nal_header(1), {0x01, 0x02}});
  std::vector<uint8_t> au = cat({kSc3, idr, kSc4, trail, kSc3});

  auto nals = split_nals(au.data(), au.size());
  REQUIRE(nals.size() == 2);
  CHECK(nals[0].type == 19);
  CHECK(nals[0].n == idr.size());
  CHECK(std::memcmp(nals[0].p, idr.data(), idr.size()) == 0);
  CHECK(nals[1].type == 1);
  CHECK(nals[1].n == trail.size());
  CHECK(std::memcmp(nals[1].p, trail.data(), trail.size()) == 0);
}

TEST(split_nals_skips_start_code_at_buffer_end) {
  // A lone start code with nothing after it anywhere in the buffer.
  std::vector<uint8_t> au = kSc3;
  auto nals = split_nals(au.data(), au.size());
  CHECK(nals.empty());
}

TEST(split_nals_back_to_back_start_codes_no_gap_nal) {
  std::vector<uint8_t> a = cat({nal_header(1), {0x01}});
  std::vector<uint8_t> b = cat({nal_header(2), {0x02}});
  // Two start codes with zero bytes between them.
  std::vector<uint8_t> au = cat({kSc3, a, kSc3, kSc4, b});
  auto nals = split_nals(au.data(), au.size());
  REQUIRE(nals.size() == 2);
  CHECK(nals[0].type == 1);
  CHECK(nals[1].type == 2);
}

TEST(au_is_irap_true_for_idr_false_for_trail) {
  std::vector<uint8_t> idr = cat({nal_header(19), {0xAA}});
  std::vector<uint8_t> au_irap = cat({kSc3, idr});
  CHECK(au_is_irap(au_irap.data(), au_irap.size()));

  std::vector<uint8_t> trail = cat({nal_header(1), {0xAA}});
  std::vector<uint8_t> au_not = cat({kSc4, trail});
  CHECK(!au_is_irap(au_not.data(), au_not.size()));
}

TEST(hevc_params_feed_across_aus_flips_complete_on_second) {
  std::vector<uint8_t> vps = make_vps();
  std::vector<uint8_t> sps = make_sps();
  std::vector<uint8_t> pps = make_pps();

  std::vector<uint8_t> au1 = cat({kSc4, vps, kSc3, sps});  // VPS+SPS only
  std::vector<uint8_t> au2 = cat({kSc3, pps});             // PPS only

  HevcParams hp;
  CHECK(!hp.complete());
  bool r1 = hp.feed(au1.data(), au1.size());
  CHECK(!r1);
  CHECK(!hp.complete());
  CHECK(hp.vps() == vps);
  CHECK(hp.sps() == sps);

  bool r2 = hp.feed(au2.data(), au2.size());
  CHECK(r2);
  CHECK(hp.complete());
  CHECK(hp.pps() == pps);
}

TEST(hvcc_layout_matches_field_plan) {
  std::vector<uint8_t> vps = make_vps();
  std::vector<uint8_t> sps = make_sps();
  std::vector<uint8_t> pps = make_pps();

  HevcParams hp;
  std::vector<uint8_t> au = cat({kSc4, vps, kSc3, sps, kSc3, pps});
  REQUIRE(hp.feed(au.data(), au.size()));

  std::vector<uint8_t> h = hp.hvcc();

  // Byte-for-byte expected header (23 bytes) per the field plan.
  std::vector<uint8_t> expect_header = {
      0x01,                                            // configurationVersion
      0x10, 0x11, 0x12, 0x13, 0x14, 0x15,               // profile_tier_level[0..5]
      0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,               // profile_tier_level[6..11]
      0xF0, 0x00,                                       // min_spatial_segmentation=0
      0xFC,                                             // parallelismType=0
      0xFD,                                             // chromaFormat=1 (0xFC|1)
      0xF8,                                             // bitDepthLumaMinus8=0
      0xF8,                                             // bitDepthChromaMinus8=0
      0x00, 0x00,                                       // avgFrameRate=0
      0x13,                                              // constFR=0/numTemporal=2/nested=0/lenMinus1=3
      0x03,                                              // numOfArrays=3
  };
  REQUIRE(h.size() >= expect_header.size());
  CHECK(std::memcmp(h.data(), expect_header.data(), expect_header.size()) == 0);
  CHECK(h[0] == 0x01);
  CHECK(h[22] == 0x03);  // numOfArrays at the documented offset

  size_t off = expect_header.size();
  // Array 1: VPS.
  REQUIRE(h.size() >= off + 5);
  CHECK(h[off + 0] == (0x80 | 32));
  CHECK(h[off + 1] == 0x00);
  CHECK(h[off + 2] == 0x01);
  uint16_t vps_len = static_cast<uint16_t>((h[off + 3] << 8) | h[off + 4]);
  CHECK(vps_len == vps.size());
  off += 5;
  REQUIRE(h.size() >= off + vps_len);
  CHECK(std::memcmp(h.data() + off, vps.data(), vps.size()) == 0);
  off += vps_len;

  // Array 2: SPS.
  REQUIRE(h.size() >= off + 5);
  CHECK(h[off + 0] == (0x80 | 33));
  CHECK(h[off + 1] == 0x00);
  CHECK(h[off + 2] == 0x01);
  uint16_t sps_len = static_cast<uint16_t>((h[off + 3] << 8) | h[off + 4]);
  CHECK(sps_len == sps.size());
  off += 5;
  REQUIRE(h.size() >= off + sps_len);
  CHECK(std::memcmp(h.data() + off, sps.data(), sps.size()) == 0);
  off += sps_len;

  // Array 3: PPS.
  REQUIRE(h.size() >= off + 5);
  CHECK(h[off + 0] == (0x80 | 34));
  CHECK(h[off + 1] == 0x00);
  CHECK(h[off + 2] == 0x01);
  uint16_t pps_len = static_cast<uint16_t>((h[off + 3] << 8) | h[off + 4]);
  CHECK(pps_len == pps.size());
  off += 5;
  REQUIRE(h.size() >= off + pps_len);
  CHECK(std::memcmp(h.data() + off, pps.data(), pps.size()) == 0);
  off += pps_len;

  CHECK(off == h.size());  // nothing trailing/missing
}

TEST(hvcc_deescapes_emulation_prevention_in_ptl) {
  // A realistic Main-profile SPS: compat flags 0x60000000 and zero
  // constraint flags put two 00 00 pairs inside the PTL, which the
  // ESCAPED wire bitstream renders with 00 00 03 emulation-prevention
  // bytes. The hvcC copy must de-escape (final-review finding: copying
  // wire bytes shifted every PTL field after the first EPB; ffmpeg-family
  // gates can't see it because they parse the in-band SPS, not hvcC).
  //
  // RBSP:    AB | 01 60 00 00 00 90 00 00 00 00 00 00  (hdr | 12-byte PTL)
  // Escaped: AB   01 60 00 00 03 00 90 00 00 03 00 00 03 00 00 03 00 ...
  std::vector<uint8_t> sps_escaped =
      cat({nal_header(33),
           {0xAB, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00, 0x03,
            0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x5D, 0xFF}});
  HevcParams hp;
  std::vector<uint8_t> au =
      cat({kSc4, make_vps(), kSc3, sps_escaped, kSc3, make_pps()});
  REQUIRE(hp.feed(au.data(), au.size()));

  std::vector<uint8_t> h = hp.hvcc();
  const std::vector<uint8_t> expect_ptl = {0x01, 0x60, 0x00, 0x00, 0x00, 0x90,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  REQUIRE(h.size() >= 13);
  CHECK(std::memcmp(h.data() + 1, expect_ptl.data(), expect_ptl.size()) == 0);
  // The stored SPS array payload stays ESCAPED (decoders de-escape it
  // themselves); only the hvcC header fields are RBSP.
  CHECK(hp.sps() == sps_escaped);
}

TEST(annexb_to_length_prefixed_roundtrip_mixed_start_codes) {
  std::vector<uint8_t> a = cat({nal_header(1), {0x01, 0x02, 0x03}});
  std::vector<uint8_t> b = cat({nal_header(19), {0xAA, 0xBB}});
  std::vector<uint8_t> c = cat({nal_header(34), {0xCC}});
  // Mixed 3-byte / 4-byte start codes across three NALs.
  std::vector<uint8_t> au = cat({kSc3, a, kSc4, b, kSc3, c});

  auto out = annexb_to_length_prefixed(au.data(), au.size());

  size_t expect_size = 4 * 3 + a.size() + b.size() + c.size();
  CHECK(out.size() == expect_size);

  size_t off = 0;
  auto check_one = [&](const std::vector<uint8_t>& nal) {
    REQUIRE(out.size() >= off + 4);
    uint32_t len = (static_cast<uint32_t>(out[off]) << 24) |
                   (static_cast<uint32_t>(out[off + 1]) << 16) |
                   (static_cast<uint32_t>(out[off + 2]) << 8) |
                   static_cast<uint32_t>(out[off + 3]);
    CHECK(len == nal.size());
    off += 4;
    REQUIRE(out.size() >= off + len);
    CHECK(std::memcmp(out.data() + off, nal.data(), nal.size()) == 0);
    off += len;
  };
  check_one(a);
  check_one(b);
  check_one(c);
  CHECK(off == out.size());
}

TEST(annexb_to_length_prefixed_skips_trailing_empty_start_code) {
  std::vector<uint8_t> a = cat({nal_header(1), {0x01}});
  std::vector<uint8_t> au = cat({kSc3, a, kSc4});  // trailing start code, no NAL after
  auto out = annexb_to_length_prefixed(au.data(), au.size());
  CHECK(out.size() == 4 + a.size());
}

MTEST_MAIN
