#pragma once
// tests/fixtures/slices4_1080p.h265: 7 access units cut from the bench
// drone's 4-slice 1080p60 capture (tools/slices/make_slice_fixture.py):
//   0 IDR: VPS/SPS/PPS + 4 I slices      1 P TRAIL_N     2 P TRAIL_R
//   3 refresh start: VPS/SPS/PPS + 1 TRAIL_R slice (the SDK leaves it whole)
//   4 P TRAIL_N   5 P TRAIL_R   6 P TRAIL_N   (4 slices each, CTU 0/150/300/450)
#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "mabur/hevc_params.h"

namespace mtest {

inline size_t start_code_pos(const std::vector<uint8_t>& b, const uint8_t* body) {
  const size_t p = static_cast<size_t>(body - b.data());
  return (p >= 4 && b[p - 4] == 0 && b[p - 3] == 0 && b[p - 2] == 0 && b[p - 1] == 1) ? p - 4 : p - 3;
}

inline std::vector<std::vector<uint8_t>> load_slice_fixture() {
  std::ifstream f(std::string(MABUR_FIXTURE_DIR) + "/slices4_1080p.h265", std::ios::binary);
  if (!f) throw std::runtime_error("missing tests/fixtures/slices4_1080p.h265");
  const std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::vector<std::vector<uint8_t>> aus;
  size_t au_start = 0, nonvcl_after_vcl = SIZE_MAX;
  bool seen_vcl = false;
  for (const mabur::NalView& v : mabur::split_nals(b.data(), b.size())) {
    const size_t sc = start_code_pos(b, v.p);
    if (v.type >= 32) {
      if (seen_vcl && nonvcl_after_vcl == SIZE_MAX) nonvcl_after_vcl = sc;
      continue;
    }
    const bool first = v.n > 2 && (v.p[2] & 0x80);
    if (first && seen_vcl) {
      const size_t cut = nonvcl_after_vcl != SIZE_MAX ? nonvcl_after_vcl : sc;
      aus.emplace_back(b.begin() + static_cast<long>(au_start), b.begin() + static_cast<long>(cut));
      au_start = cut;
    }
    nonvcl_after_vcl = SIZE_MAX;
    seen_vcl = true;
  }
  aus.emplace_back(b.begin() + static_cast<long>(au_start), b.end());
  return aus;
}

// VCL NAL units of one AU, start code included, in bitstream order.
inline std::vector<std::vector<uint8_t>> slice_nals(const std::vector<uint8_t>& au) {
  std::vector<std::vector<uint8_t>> out;
  const auto nals = mabur::split_nals(au.data(), au.size());
  for (size_t i = 0; i < nals.size(); ++i) {
    if (nals[i].type >= 32) continue;
    const size_t s = start_code_pos(au, nals[i].p);
    const size_t e = static_cast<size_t>(nals[i].p - au.data()) + nals[i].n;
    out.emplace_back(au.begin() + static_cast<long>(s), au.begin() + static_cast<long>(e));
  }
  return out;
}

}  // namespace mtest
