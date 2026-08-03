#include "mtest.h"
#include "in_formats.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace maburplay;

// in_formats_has_linear() parses struct drm_format_modifier_blob (see
// <drm/drm_mode.h>): a 24-byte header (version, flags, count_formats,
// formats_offset, count_modifiers, modifiers_offset) followed by a u32
// fourcc array and a struct drm_format_modifier array (each 24 bytes:
// u64 formats bitmask, u32 offset, u32 pad, u64 modifier), all at
// driver-controlled byte offsets. These tests build that byte layout by
// hand -- no libdrm, no ioctls -- so the parser (and specifically the exact
// Cluster0-win0 AFBC-only shape that shipped the bench regression F1 in
// commit d397eb1 fixed) is checked on the host, every run.
namespace {

// DRM_FORMAT_ARGB8888 ('AR24' as a little-endian fourcc, drm_fourcc.h) and
// DRM_FORMAT_MOD_LINEAR (0, drm_fourcc.h) -- hardcoded rather than pulled
// from the driver header, so this test depends only on the wire values the
// parser's contract is defined over, not on which header happens to define
// them.
constexpr uint32_t kArgb8888 = 0x34325241u;  // fourcc('A','R','2','4')
constexpr uint64_t kModLinear = 0ull;
// A stand-in for an ARM AFBC modifier: real ones set high vendor/type bits
// (DRM_FORMAT_MOD_ARM_AFBC(...)); the exact value doesn't matter to the
// parser, only that it is present, nonzero, and not kModLinear.
constexpr uint64_t kModAfbcLike = 0x0800000000000001ull;

void put_u32_at(std::vector<uint8_t>& b, size_t off, uint32_t v) {
  if (b.size() < off + 4) b.resize(off + 4, 0);
  std::memcpy(b.data() + off, &v, 4);
}
void put_u64_at(std::vector<uint8_t>& b, size_t off, uint64_t v) {
  if (b.size() < off + 8) b.resize(off + 8, 0);
  std::memcpy(b.data() + off, &v, 8);
}

// Writes just the 24-byte drm_format_modifier_blob header; does NOT touch
// or reserve space for the format/modifier arrays -- callers place those
// explicitly (or don't, for the "offset past the end" cases).
std::vector<uint8_t> make_header(uint32_t version, uint32_t count_formats,
                                 uint32_t formats_offset, uint32_t count_modifiers,
                                 uint32_t modifiers_offset) {
  std::vector<uint8_t> b(24, 0);
  put_u32_at(b, 0, version);
  put_u32_at(b, 4, 0);  // flags: unused by the parser
  put_u32_at(b, 8, count_formats);
  put_u32_at(b, 12, formats_offset);
  put_u32_at(b, 16, count_modifiers);
  put_u32_at(b, 20, modifiers_offset);
  return b;
}

void put_format(std::vector<uint8_t>& b, size_t formats_offset, uint32_t index, uint32_t fourcc) {
  put_u32_at(b, formats_offset + 4u * index, fourcc);
}

// One struct drm_format_modifier entry: `formats` is the sliding-window
// bitmask, `window_offset` is which 64-format window it covers (format
// index i is covered iff window_offset <= i < window_offset + 64, and its
// bit within the mask is i - window_offset).
void put_modifier(std::vector<uint8_t>& b, size_t modifiers_offset, uint32_t index,
                  uint64_t formats_bitmask, uint32_t window_offset, uint64_t modifier) {
  const size_t base = modifiers_offset + 24u * index;
  put_u64_at(b, base + 0, formats_bitmask);
  put_u32_at(b, base + 8, window_offset);
  put_u32_at(b, base + 12, 0);  // pad
  put_u64_at(b, base + 16, modifier);
}

}  // namespace

TEST(accept_linear_argb8888) {
  auto b = make_header(/*version=*/1, /*count_formats=*/1, /*formats_offset=*/24,
                       /*count_modifiers=*/1, /*modifiers_offset=*/28);
  put_format(b, 24, 0, kArgb8888);
  put_modifier(b, 28, 0, /*formats=*/0x1ull, /*window_offset=*/0, kModLinear);
  std::string why;
  CHECK(in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

// The actual defect this whole commit fixed: a plane (vop2's Cluster0-win0)
// that lists ARGB8888 in its plain format list AND in IN_FORMATS, but with
// only an AFBC modifier -- no LINEAR entry anywhere. A CPU-written dumb
// buffer can never scan out there; this must be rejected.
TEST(reject_afbc_only_cluster0_win0_shape) {
  auto b = make_header(1, 1, 24, 1, 28);
  put_format(b, 24, 0, kArgb8888);
  put_modifier(b, 28, 0, 0x1ull, 0, kModAfbcLike);
  std::string why;
  CHECK(!in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
  CHECK(!why.empty());
}

TEST(reject_truncated_shorter_than_header) {
  std::vector<uint8_t> b(10, 0);  // < sizeof(drm_format_modifier_blob) == 24
  std::string why;
  CHECK(!in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

TEST(reject_formats_offset_past_end) {
  // Header only (24 bytes); formats_offset points 100000 bytes past the end
  // of the (24-byte) buffer actually supplied. fend must exceed blob_len.
  auto b = make_header(1, /*count_formats=*/1, /*formats_offset=*/100000, /*count_modifiers=*/0,
                       /*modifiers_offset=*/0);
  std::string why;
  CHECK(!in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

// count_formats/count_modifiers at UINT32_MAX must not overflow the
// bounds computation (fend/mend) back under blob_len -- the multiplication
// has to happen in 64 bits. Buffer stays tiny; if the guard overflowed, a
// wrapped fend/mend could slip under `len` and the parser would read wildly
// out of bounds (or, best case here, just misbehave) instead of rejecting.
TEST(reject_count_uint32_max_no_overflow) {
  auto b = make_header(1, UINT32_MAX, 24, UINT32_MAX, 28);
  std::string why;
  CHECK(!in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

// Sliding-modifier-window: the format index (70) is beyond bit 63 of the
// FIRST modifier entry's window (offset 0, covers indices 0-63) and only
// matches in a SECOND entry whose window starts at offset 64.
TEST(accept_format_index_beyond_first_window) {
  const uint32_t kIndex = 70;
  auto b = make_header(1, /*count_formats=*/kIndex + 1, /*formats_offset=*/24,
                       /*count_modifiers=*/2,
                       /*modifiers_offset=*/24 + 4u * (kIndex + 1));
  for (uint32_t i = 0; i < kIndex; ++i) put_format(b, 24, i, 0x30303030u + i);  // filler, != ARGB8888
  put_format(b, 24, kIndex, kArgb8888);
  const size_t mods_off = 24 + 4u * (kIndex + 1);
  // First window [0,63]: idx(70) - offset(0) = 70 >= 64 -> must be skipped
  // by the parser, even though this entry claims LINEAR.
  put_modifier(b, mods_off, 0, /*formats=*/~0ull, /*window_offset=*/0, kModLinear);
  // Second window [64,127]: idx(70) - offset(64) = 6 -> bit 6 set, LINEAR.
  put_modifier(b, mods_off, 1, /*formats=*/(1ull << 6), /*window_offset=*/64, kModLinear);
  std::string why;
  CHECK(in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

// Byte offsets that are not 4/8-byte aligned. The parser is documented to
// use memcpy throughout for exactly this reason (driver-supplied offsets
// cannot be assumed aligned) -- prove it actually tolerates them rather
// than merely claiming to.
TEST(accept_unaligned_offsets) {
  auto b = make_header(1, 1, /*formats_offset=*/25, 1, /*modifiers_offset=*/33);
  put_format(b, 25, 0, kArgb8888);
  put_modifier(b, 33, 0, 0x1ull, 0, kModLinear);
  std::string why;
  CHECK(in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

// A hypothetical v2 blob: layout this parser does not understand. Rejected
// even though the bytes that follow are otherwise a perfectly well-formed
// v1-shaped blob (F3: hdr.version was never checked before this commit).
TEST(reject_unsupported_version) {
  auto b = make_header(/*version=*/2, 1, 24, 1, 28);
  put_format(b, 24, 0, kArgb8888);
  put_modifier(b, 28, 0, 0x1ull, 0, kModLinear);
  std::string why;
  CHECK(!in_formats_has_linear(b.data(), static_cast<uint32_t>(b.size()), kArgb8888, &why));
}

TEST(reject_null_blob) {
  std::string why;
  CHECK(!in_formats_has_linear(nullptr, 0, kArgb8888, &why));
}

MTEST_MAIN
