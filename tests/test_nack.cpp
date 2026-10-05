// SPIKE 2026-10-05 (fec-nack): wire frame, decoder erasure view, tracker
// state machine, retransmit ring.
#include <vector>

#include "mabur/link_key.h"
#include "mabur/rc_proto.h"
#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
#include "mtest.h"
#include "nack_tracker.h"
#include "retx_ring.h"

using namespace mabur;
using namespace mabur::rc;
using namespace maburgs;

static LinkKey key() { return *parse_key_hex("3f9a1c77e04b5d2290ab6ef1c8d34e5a"); }

TEST(nack_pack_parse_verify_roundtrip) {
  Nack n;
  n.seq32 = 77;
  n.n = 2;
  n.e[0] = NackEntry{0, 1000, 0x5u};
  n.e[1] = NackEntry{1, 4000000000u, 0x80000000u};
  TagCtx ctx{11, 22, 77};
  auto f = pack_nack(n, key(), ctx);
  CHECK(frame_type(f.data(), f.size()) == T_NACK);
  auto p = parse_nack(f.data(), f.size());
  REQUIRE(p.has_value());
  CHECK(p->seq32 == 77);
  CHECK(p->n == 2);
  CHECK(p->e[0].sid == 0 && p->e[0].first_seq == 1000 && p->e[0].bitmap == 0x5u);
  CHECK(p->e[1].sid == 1 && p->e[1].first_seq == 4000000000u && p->e[1].bitmap == 0x80000001u);
  CHECK(verify_control(f.data(), f.size(), key(), ctx));
  CHECK(!verify_control(f.data(), f.size(), key(), TagCtx{11, 22, 78}));
  CHECK(!verify_control(f.data(), f.size(), kDefaultLinkKey, ctx));
  // corrupt a body byte: parse fails on CRC
  auto g = f; g[12] ^= 0xFF;
  CHECK(!parse_nack(g.data(), g.size()).has_value());
}

static std::vector<uint8_t> pat(size_t n, int seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>((i * 31 + seed * 17 + 7) & 0xFF);
  return v;
}

TEST(decoder_missing_sources_and_state) {
  SwConfig cfg{64, 8, 0.0};  // no repairs: sources only
  SwEncoder e(cfg, 500);
  std::vector<std::vector<uint8_t>> envs;
  for (int i = 0; i < 12; ++i) {
    auto p = pat(62, i);
    for (auto& env : e.add_packet(p.data(), p.size())) envs.push_back(env);
  }
  for (auto& env : e.flush()) envs.push_back(env);
  // sources only: flush() also emits a tail repair, which would recover one
  // of the two holes this test punches
  std::vector<std::vector<uint8_t>> srcs;
  for (auto& env : envs) {
    sw::SwHeader h;
    if (sw::parse_header(env.data(), env.size(), &h) && !h.repair) srcs.push_back(env);
  }
  envs = srcs;
  REQUIRE(envs.size() == 12);
  SwDecoder d(cfg);
  // feed all but seqs 503 and 504 (index 3, 4)
  for (size_t i = 0; i < envs.size(); ++i) {
    if (i == 3 || i == 4) continue;
    d.add_symbol(envs[i].data(), envs[i].size(), 1000);
  }
  auto m = d.missing_sources(64);
  REQUIRE(m.size() == 2);
  CHECK(m[0] == 503 && m[1] == 504);
  CHECK(d.source_state(503) == SwDecoder::SourceState::kUnknown);
  CHECK(d.source_state(502) == SwDecoder::SourceState::kDirect);
  // the "retransmit" lands
  d.add_symbol(envs[3].data(), envs[3].size(), 1010);
  CHECK(d.source_state(503) == SwDecoder::SourceState::kDirect);
  CHECK(d.missing_sources(64).size() == 1);
}

TEST(tracker_settle_repeat_and_fill) {
  NackCfg c; c.enable = true; c.settle_ms = 5; c.repeat_ms = 10; c.max_tries = 2;
  NackTracker t(c);
  std::vector<uint32_t> missing{100, 101, 140};
  std::map<uint32_t, SwDecoder::SourceState> st;
  for (auto s : missing) st[s] = SwDecoder::SourceState::kUnknown;
  auto mf = [&](int sid) { return sid == 0 ? missing : std::vector<uint32_t>{}; };
  auto sf = [&](int sid, uint32_t seq) {
    if (sid != 0) return SwDecoder::SourceState::kUnknown;
    auto it = st.find(seq); return it == st.end() ? SwDecoder::SourceState::kDirect : it->second;
  };
  CHECK(!t.poll(1000, mf, sf).has_value());  // admitted, not yet settled
  CHECK(!t.poll(1004, mf, sf).has_value());
  auto n = t.poll(1005, mf, sf);
  REQUIRE(n.has_value());
  CHECK(n->n == 2);  // {100,101} in one run, 140 in another (>= 32 apart)
  CHECK(n->e[0].first_seq == 100 && n->e[0].bitmap == 0x3u);
  CHECK(n->e[1].first_seq == 140 && n->e[1].bitmap == 0x1u);
  CHECK(t.stats().sent == 1 && t.stats().syms_requested == 3);
  CHECK(!t.poll(1010, mf, sf).has_value());  // repeat not due
  // 100 filled by a direct copy, 101 recovered by a repair
  st[100] = SwDecoder::SourceState::kDirect;
  st[101] = SwDecoder::SourceState::kRecovered;
  missing = {140};
  auto r = t.poll(1016, mf, sf);
  REQUIRE(r.has_value());  // repeat for 140
  CHECK(r->n == 1 && r->e[0].first_seq == 140);
  CHECK(t.stats().filled == 1 && t.stats().wasted == 1 && t.stats().repeats == 1);
  CHECK(t.stats().fill_max_ms == 11);
  CHECK(!t.poll(1040, mf, sf).has_value());  // max_tries reached
  st[140] = SwDecoder::SourceState::kBelowFloor; missing = {};
  t.poll(1050, mf, sf);
  CHECK(t.stats().abandoned == 1 && t.outstanding() == 0);
}

TEST(tracker_disabled_is_inert) {
  NackTracker t(NackCfg{});
  CHECK(!t.poll(1, [](int) { return std::vector<uint32_t>{1, 2}; },
                [](int, uint32_t) { return SwDecoder::SourceState::kUnknown; }).has_value());
}

TEST(retx_ring_put_get_evict) {
  RetxRing r(4);
  uint8_t e[3] = {1, 2, 3};
  for (uint32_t s = 10; s < 16; ++s) { e[0] = static_cast<uint8_t>(s); r.put(0, s, e, 3); }
  CHECK(!r.get(0, 10).has_value());
  CHECK(!r.get(0, 11).has_value());
  auto g = r.get(0, 15);
  REQUIRE(g.has_value());
  CHECK((*g)[0] == 15 && g->size() == 3);
  CHECK(!r.get(1, 15).has_value());
  CHECK(!r.get(7, 15).has_value());
}

MTEST_MAIN
