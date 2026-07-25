#include <vector>
#include <cstdint>
#include "mtest.h"
#include "vectors.h"
#include "mabur/nal.h"
using namespace mabur;

TEST(parse_hevc_nal_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/nal.json");
  for (auto& c : j["cases"]) {
    auto nal = mtest::unhex(c["in"].get<std::string>());
    NalInfo info = parse_hevc_nal(nal.data(), nal.size());
    CHECK(info.type == static_cast<uint8_t>(c["type"].get<int>()));
    CHECK(info.tid == static_cast<uint8_t>(c["tid"].get<int>()));
    CHECK(info.critical == c["critical"].get<bool>());
  }
}

TEST(parse_hevc_nal_too_short_is_default) {
  std::vector<uint8_t> one_byte = {0x40};
  NalInfo info = parse_hevc_nal(one_byte.data(), one_byte.size());
  CHECK(info.type == 0);
  CHECK(info.tid == 0);
  CHECK(info.critical == false);

  NalInfo info_empty = parse_hevc_nal(nullptr, 0);
  CHECK(info_empty.type == 0);
  CHECK(info_empty.tid == 0);
  CHECK(info_empty.critical == false);
}

namespace {
// Builds one Annex-B NAL: 4-byte start code + 2-byte HEVC header + payload.
std::vector<uint8_t> mk_nal(uint8_t type, uint8_t tid, size_t payload = 8) {
  std::vector<uint8_t> v = {0x00, 0x00, 0x00, 0x01,
                            static_cast<uint8_t>(type << 1),
                            static_cast<uint8_t>(tid + 1)};
  v.resize(v.size() + payload, 0x55);
  return v;
}
std::vector<uint8_t> cat(std::initializer_list<std::vector<uint8_t>> parts) {
  std::vector<uint8_t> v;
  for (auto& p : parts) v.insert(v.end(), p.begin(), p.end());
  return v;
}
}  // namespace

TEST(classify_frame_replays_nal_vectors_as_single_nal_frames) {
  // Every nal.json case as a one-NAL Annex-B frame: critical -> stream 0,
  // otherwise 1 + min(tid, 2). Keeps the layer mapping pinned to the Python
  // reference now that the RTP classifier it also covered is gone.
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/nal.json");
  for (auto& c : j["cases"]) {
    auto nal = mtest::unhex(c["in"].get<std::string>());
    const bool critical = c["critical"].get<bool>();
    const int type = c["type"].get<int>();
    const int tid = c["tid"].get<int>();
    // Non-VCL, non-critical NALs (e.g. SEI 39, type 63) pick no layer on
    // their own; classify_frame's no-parseable-VCL fallback is stream 0.
    const int expect = critical ? 0 : (type < 16 ? 1 + std::min(tid, 2) : 0);

    std::vector<uint8_t> frame = {0x00, 0x00, 0x00, 0x01};
    frame.insert(frame.end(), nal.begin(), nal.end());
    CHECK(classify_frame(frame.data(), frame.size()) == expect);
  }
}

TEST(classify_frame_idr_with_param_sets_is_critical) {
  // VPS(32) + SPS(33) + PPS(34) + IDR_W_RADL(19) — the typical IDR frame.
  auto f = cat({mk_nal(32, 0), mk_nal(33, 0), mk_nal(34, 0), mk_nal(19, 0, 5000)});
  CHECK(classify_frame(f.data(), f.size()) == 0);
}

TEST(classify_frame_p_frame_by_tid) {
  CHECK(classify_frame(mk_nal(1, 0, 2000).data(), mk_nal(1, 0, 2000).size()) == 1);
  CHECK(classify_frame(mk_nal(1, 1, 2000).data(), mk_nal(1, 1, 2000).size()) == 2);
  CHECK(classify_frame(mk_nal(1, 2, 2000).data(), mk_nal(1, 2, 2000).size()) == 3);
  CHECK(classify_frame(mk_nal(1, 5, 2000).data(), mk_nal(1, 5, 2000).size()) == 3);  // tid clamp
}

TEST(classify_frame_sei_then_slice_uses_slice_tid) {
  // Prefix SEI (39, non-VCL, non-critical) must not pick the layer.
  auto f = cat({mk_nal(39, 0), mk_nal(1, 2, 2000)});
  CHECK(classify_frame(f.data(), f.size()) == 3);
}

TEST(classify_frame_three_byte_start_code) {
  std::vector<uint8_t> f = {0x00, 0x00, 0x01, static_cast<uint8_t>(1 << 1), 0x02};
  f.resize(f.size() + 100, 0x55);
  CHECK(classify_frame(f.data(), f.size()) == 2);  // tid 1
}

TEST(classify_frame_garbage_protects_up) {
  std::vector<uint8_t> junk(64, 0xFF);
  CHECK(classify_frame(junk.data(), junk.size()) == 0);
  CHECK(classify_frame(nullptr, 0) == 0);
}

MTEST_MAIN
