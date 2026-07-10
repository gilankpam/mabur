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

// Builds a fixed 12-byte RTP header (no CSRC, no extension) with pt=0x61
// (arbitrary dynamic payload type), followed by the given HEVC payload.
std::vector<uint8_t> wrap_rtp(const std::vector<uint8_t>& payload,
                               uint8_t first_byte = 0x80) {
  std::vector<uint8_t> pkt;
  pkt.push_back(first_byte);  // version=2, no padding, no extension (unless overridden), CSRC=0
  pkt.push_back(0x61);        // marker=0, payload type=0x61
  pkt.push_back(0x00);        // seq hi
  pkt.push_back(0x01);        // seq lo
  pkt.push_back(0x00);        // ts
  pkt.push_back(0x00);
  pkt.push_back(0x00);
  pkt.push_back(0x01);
  pkt.push_back(0xDE);        // ssrc
  pkt.push_back(0xAD);
  pkt.push_back(0xBE);
  pkt.push_back(0xEF);
  pkt.insert(pkt.end(), payload.begin(), payload.end());
  return pkt;
}

}  // namespace

TEST(classify_rtp_replays_nal_vectors_via_single_nal_payload) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/nal.json");
  for (auto& c : j["cases"]) {
    auto nal = mtest::unhex(c["in"].get<std::string>());
    bool critical = c["critical"].get<bool>();
    int tid = c["tid"].get<int>();
    int expect = critical ? 0 : 1 + std::min(tid, 2);

    auto pkt = wrap_rtp(nal);
    int stream = classify_rtp(pkt.data(), pkt.size());
    CHECK(stream == expect);
  }
}

TEST(classify_rtp_fu_wrapping_type19_is_critical_stream0) {
  // FU header: nal type 49 (0x62 0x01), FU header byte: start bit set (0x80)
  // | real type 19 (IDR_W_RADL) in low 6 bits -> 0x93. tid byte (nal[1]) = 1
  // (tid encoded as (nal1 & 7)-1 = 0).
  std::vector<uint8_t> payload = {
      static_cast<uint8_t>((49 << 1) & 0xFE),  // b0: type=49
      0x01,                                     // b1: tid field -> tid 0
      static_cast<uint8_t>(0x80 | 19),          // FU header: start | type 19
      0xAA, 0xBB,                                // fu payload bytes
  };
  auto pkt = wrap_rtp(payload);
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 0);
}

TEST(classify_rtp_fu_wrapping_type1_tid2_is_stream3) {
  // FU header nal: type=49, tid byte -> (nal1&7)-1 == 2 => nal1 & 7 == 3.
  std::vector<uint8_t> payload = {
      static_cast<uint8_t>((49 << 1) & 0xFE),  // b0: type=49
      0x03,                                     // b1: tid field -> tid 2
      static_cast<uint8_t>(0x80 | 1),           // FU header: start | type 1
      0xCC, 0xDD,
  };
  auto pkt = wrap_rtp(payload);
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 3);
}

TEST(classify_rtp_ap_type48_is_stream0) {
  std::vector<uint8_t> payload = {
      static_cast<uint8_t>((48 << 1) & 0xFE),  // b0: type=48 (AP)
      0x01,                                     // b1
      0x00, 0x08,                                // aggregation unit size (unused by classifier)
      0x26, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
  };
  auto pkt = wrap_rtp(payload);
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 0);
}

TEST(classify_rtp_csrc_count_shifts_payload_offset) {
  // pkt[0] = 0x83 -> version 2, CSRC count = 3 -> payload offset 12 + 4*3 = 24.
  std::vector<uint8_t> pkt;
  pkt.push_back(0x83);
  pkt.push_back(0x61);
  pkt.push_back(0x00);
  pkt.push_back(0x01);
  pkt.push_back(0x00);
  pkt.push_back(0x00);
  pkt.push_back(0x00);
  pkt.push_back(0x01);
  pkt.push_back(0xDE);
  pkt.push_back(0xAD);
  pkt.push_back(0xBE);
  pkt.push_back(0xEF);
  // 3 CSRC entries, 4 bytes each = 12 bytes of filler.
  for (int i = 0; i < 12; ++i) pkt.push_back(0x00);
  // Payload: non-critical slice type 1, tid 0 -> stream 1.
  std::vector<uint8_t> nal = {0x02, 0x01, 0x18, 0x37, 0x56, 0x75, 0x94, 0xB3};
  pkt.insert(pkt.end(), nal.begin(), nal.end());

  CHECK(classify_rtp(pkt.data(), pkt.size()) == 1);
}

TEST(classify_rtp_truncated_garbage_is_stream0) {
  std::vector<uint8_t> pkt(10, 0xFF);  // shorter than min RTP header (14)
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 0);
}

TEST(classify_rtp_non_rtp_version_is_stream0) {
  // Fixed-size 12-byte header + payload, but version bits (top 2 bits of
  // byte 0) are not 2.
  auto pkt = wrap_rtp(std::vector<uint8_t>{0x02, 0x01, 0x18, 0x37}, 0x40);
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 0);
}

MTEST_MAIN
