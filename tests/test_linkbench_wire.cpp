#include "mtest.h"
#include "bench_wire.h"
#include <cstring>
using namespace linkbench;

TEST(bench_packet_round_trip) {
  auto p = build_bench_packet(0x01020304u, 62);
  REQUIRE(p.size() == 62);
  uint32_t seq = 0; bool pattern_ok = false;
  REQUIRE(parse_bench_packet(p.data(), p.size(), &seq, &pattern_ok));
  CHECK(seq == 0x01020304u);
  CHECK(pattern_ok);
}

TEST(bench_packet_pattern_corruption_detected) {
  auto p = build_bench_packet(7, 62);
  p[10] ^= 0xFF;  // flip one fill byte
  uint32_t seq = 0; bool pattern_ok = true;
  REQUIRE(parse_bench_packet(p.data(), p.size(), &seq, &pattern_ok));
  CHECK(seq == 7);
  CHECK(!pattern_ok);
}

TEST(bench_packet_rejects_short_and_len_mismatch) {
  uint32_t seq; bool ok;
  uint8_t tiny[3] = {0, 0, 0};
  CHECK(!parse_bench_packet(tiny, sizeof tiny, &seq, &ok));
  auto p = build_bench_packet(1, 30);
  CHECK(!parse_bench_packet(p.data(), p.size() - 5, &seq, &ok));  // truncated
}

TEST(bench_packet_min_size_is_header_only) {
  auto p = build_bench_packet(42, kBenchPktHeader);  // no fill bytes
  REQUIRE(p.size() == kBenchPktHeader);
  uint32_t seq = 0; bool pattern_ok = false;
  REQUIRE(parse_bench_packet(p.data(), p.size(), &seq, &pattern_ok));
  CHECK(seq == 42);
  CHECK(pattern_ok);
}

TEST(dot11_header_layout) {
  auto h = build_dot11_header(0x123);
  REQUIRE(h.size() == kDot11HeaderLen);
  CHECK(h[0] == 0x40);  // probe-req, matches maburd/maburgs canonical frames
  for (int i = 4; i < 10; ++i) CHECK(h[i] == 0xff);       // DA broadcast
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  CHECK(std::memcmp(h.data() + 10, sa, 6) == 0);          // SA
  CHECK(std::memcmp(h.data() + 16, sa, 6) == 0);          // BSSID
  uint16_t seq_ctl = static_cast<uint16_t>(h[22] | (h[23] << 8));
  CHECK((seq_ctl >> 4) == 0x123);
}

TEST(range_payload_roundtrip_with_and_without_fcs) {
  RangeFrameInfo in;
  in.bw = 40; in.mcs = 5; in.rel = -17; in.cycle = 0x1234; in.seq = 0xBEEF;
  auto p = build_range_payload(in, 1400);
  REQUIRE(p.size() == 1400);
  RangeFrameInfo out;
  REQUIRE(parse_range_payload(p.data(), p.size(), &out));
  CHECK(out.bw == 40); CHECK(out.mcs == 5); CHECK(out.rel == -17);
  CHECK(out.cycle == 0x1234); CHECK(out.seq == 0xBEEF);
  p.insert(p.end(), {0xde, 0xad, 0xbe, 0xef});  // trailing FCS as RX delivers it
  REQUIRE(parse_range_payload(p.data(), p.size(), &out));
  CHECK(out.rel == -17);
}

TEST(range_payload_rejects_corruption_and_foreign) {
  RangeFrameInfo in;
  in.bw = 20; in.mcs = 0; in.rel = 3;
  auto p = build_range_payload(in, 200);
  RangeFrameInfo out;
  auto bad = p; bad[100] ^= 0x01;                 // fill byte flipped
  CHECK(!parse_range_payload(bad.data(), bad.size(), &out));
  bad = p; bad[4] = 30;                           // impossible width
  CHECK(!parse_range_payload(bad.data(), bad.size(), &out));
  bad = p; bad[0] = 'X';                          // not ours
  CHECK(!parse_range_payload(bad.data(), bad.size(), &out));
  auto bp = build_bench_packet(7, 200);           // a bench packet is not one
  CHECK(!parse_range_payload(bp.data(), bp.size(), &out));
}

TEST(parse_rate_bps_forms) {
  CHECK(parse_rate_bps("8M") == 8000000ull);
  CHECK(parse_rate_bps("1.5M") == 1500000ull);
  CHECK(parse_rate_bps("800k") == 800000ull);
  CHECK(parse_rate_bps("800K") == 800000ull);
  CHECK(parse_rate_bps("12345") == 12345ull);
  CHECK(parse_rate_bps("") == 0ull);
  CHECK(parse_rate_bps("junk") == 0ull);
  CHECK(parse_rate_bps("-5M") == 0ull);
}

MTEST_MAIN
