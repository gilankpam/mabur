#include <cstdint>
#include <vector>

#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
#include "mtest.h"

using namespace mabur;

namespace {
std::vector<uint8_t> pat(size_t n, int seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i)
    v[i] = static_cast<uint8_t>((i * 31 + static_cast<size_t>(seed) * 17 + 7) & 0xFF);
  return v;
}
sw::SwHeader hdr(const std::vector<uint8_t>& env) {
  sw::SwHeader h;
  CHECK(sw::parse_header(env.data(), env.size(), &h));
  return h;
}
// Feed packets sized to seal exactly one symbol each (symbol_size 64:
// payload 62 + 2B length prefix fills the symbol; the NEXT packet seals it).
std::vector<std::vector<uint8_t>> feed(SwEncoder& e, int n_packets) {
  std::vector<std::vector<uint8_t>> out;
  for (int i = 0; i < n_packets; ++i) {
    auto p = pat(62, i);
    for (auto& env : e.add_packet(p.data(), p.size())) out.push_back(std::move(env));
  }
  return out;
}
}  // namespace

TEST(source_envelopes_carry_monotonic_seq) {
  SwEncoder e(SwConfig{64, 8, 0.0});
  auto envs = feed(e, 5);          // 5 full-size packets: symbols 0..3 sealed
  auto tail = e.flush();           // seals the 5th
  for (auto& t : tail) envs.push_back(std::move(t));
  CHECK(envs.size() == 5);
  for (uint32_t i = 0; i < envs.size(); ++i) {
    auto h = hdr(envs[i]);
    CHECK(!h.repair);
    CHECK(h.seq == i);
    CHECK(h.symbol_size == 64);
    CHECK(envs[i].size() == sw::kSwHeaderLen + 64);
  }
}

TEST(credit_emits_one_repair_per_source_at_overhead_1) {
  SwEncoder e(SwConfig{64, 8, 1.0});
  auto envs = feed(e, 5);          // 4 seals -> 4 source + 4 repair
  int src = 0, rep = 0;
  for (auto& env : envs) (hdr(env).repair ? rep : src)++;
  CHECK(src == 4);
  CHECK(rep == 4);
  CHECK(e.sources_out() == 4);
  CHECK(e.repairs_out() == 4);
}

TEST(credit_emits_every_other_at_overhead_half) {
  SwEncoder e(SwConfig{64, 8, 0.5});
  auto envs = feed(e, 9);          // 8 seals -> 8 source + 4 repair
  int rep = 0;
  for (auto& env : envs) if (hdr(env).repair) ++rep;
  CHECK(rep == 4);
}

TEST(repair_window_slides_and_caps) {
  SwEncoder e(SwConfig{64, 4, 1.0});
  auto envs = feed(e, 7);          // 6 seals; last repair after seq 5 sealed
  sw::SwHeader last{};
  for (auto& env : envs) { auto h = hdr(env); if (h.repair) last = h; }
  // After sealing seqs 0..5 with window 4, the newest repair covers [2, 6).
  CHECK(last.window_len == 4);
  CHECK(last.seq == 2);
}

TEST(repair_keys_increment) {
  SwEncoder e(SwConfig{64, 8, 1.0});
  auto envs = feed(e, 4);
  std::vector<uint32_t> keys;
  for (auto& env : envs) { auto h = hdr(env); if (h.repair) keys.push_back(h.repair_key); }
  CHECK(keys.size() == 3);
  CHECK(keys[0] == 0);
  CHECK(keys[1] == 1);
  CHECK(keys[2] == 2);
}

TEST(flush_seals_partial_and_emits_tail_repair_once) {
  SwEncoder e(SwConfig{64, 8, 0.25});
  auto p = pat(10, 1);
  CHECK(e.add_packet(p.data(), p.size()).empty());  // partial symbol pending
  CHECK(e.has_pending());
  auto f1 = e.flush();
  // source (sealed partial) + exactly one tail repair
  CHECK(f1.size() == 2);
  CHECK(!hdr(f1[0]).repair);
  CHECK(hdr(f1[1]).repair);
  CHECK(!e.has_pending());
  auto f2 = e.flush();               // idle flush: nothing new, no repair spam
  CHECK(f2.empty());
}

TEST(oversize_packet_dropped_counted) {
  SwEncoder e(SwConfig{64, 8, 0.5});
  auto p = pat(63, 0);               // > max_packet_size() == 62
  CHECK(e.add_packet(p.data(), p.size()).empty());
  CHECK(e.oversize_drops() == 1);
}

TEST(packets_never_span_symbols) {
  // 40B packet then 30B packet: 2+40=42 used, 2+30 doesn't fit in the
  // remaining 22 -> first symbol seals padded, second packet starts fresh.
  SwEncoder e(SwConfig{64, 8, 0.0});
  auto a = pat(40, 0), b = pat(30, 1);
  CHECK(e.add_packet(a.data(), a.size()).empty());
  auto envs = e.add_packet(b.data(), b.size());
  CHECK(envs.size() == 1);           // sealed symbol 0
  const uint8_t* sym = envs[0].data() + sw::kSwHeaderLen;
  CHECK(sym[0] == 40 && sym[1] == 0);                 // len prefix
  for (int i = 0; i < 40; ++i) CHECK(sym[2 + i] == a[static_cast<size_t>(i)]);
  for (int i = 42; i < 64; ++i) CHECK(sym[i] == 0);   // zero pad, no spill
}

MTEST_MAIN
