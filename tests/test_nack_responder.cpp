// NackResponder: a verified T_NACK -> ring lookups -> retx SBI bodies in
// push order, under the air-time token bucket.
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>
#include "mtest.h"
#include "air_rate.h"
#include "nack_responder.h"
using namespace mabur;

constexpr size_t kEnv = 32;

static std::vector<uint8_t> env_of(uint32_t seq) {
  std::vector<uint8_t> e(kEnv, 0xA5);
  e[0] = static_cast<uint8_t>(seq);
  return e;
}

struct Rig {
  RetxRing ring{64, kEnv};
  NackResponder r{kEnv, 1};
  AirClockCfg air;
  NackDroneCfg cfg;
  std::vector<std::vector<uint8_t>> out;
  void put(uint32_t first, uint32_t count) {
    for (uint32_t s = first; s < first + count; ++s) ring.put(s, env_of(s).data(), kEnv);
  }
  // Requests `count` seqs from `first` (up to 4 x 32).
  NackResponder::Tally ask(uint32_t first, uint32_t count, uint8_t flags = 0, const AppliedOp* op = nullptr) {
    rc::Nack n;
    n.flags = flags;
    n.n = 0;
    for (uint32_t s = 0; s < count; s += 32, ++n.n) {
      const uint32_t k = std::min<uint32_t>(32, count - s);
      n.e[n.n] = {first + s, k == 32 ? 0xFFFFFFFFu : (1u << k) - 1};
    }
    return r.answer(n, op, air, cfg, 1000, ring, &out);
  }
  // The TxQueue head after the hook push_fronts every body in order.
  std::deque<std::vector<uint8_t>> queue() const {
    std::deque<std::vector<uint8_t>> q;
    for (const auto& b : out) q.push_front(b);
    return q;
  }
};

static bool carries(const std::vector<uint8_t>& body, uint32_t seq) {
  const auto e = env_of(seq);
  return std::search(body.begin(), body.end(), e.begin(), e.end()) != body.end();
}

TEST(miss_counts_absent_seq) {
  Rig r; r.put(100, 1);
  const auto t = r.ask(100, 2);
  CHECK(t.syms == 1); CHECK(t.miss == 1); CHECK(t.refused == 0);
  REQUIRE(r.out.size() == 1);
  CHECK(carries(r.out[0], 100));
}

TEST(answers_in_request_order) {
  Rig r; r.put(100, 4);
  const auto t = r.ask(100, 4);
  CHECK(t.syms == 4);
  const auto q = r.queue();
  REQUIRE(q.size() == 4);
  for (uint32_t i = 0; i < 4; ++i) CHECK(carries(q[i], 100 + i));
}

TEST(repeat_sends_twice_in_order) {
  Rig r; r.put(100, 4);
  const auto t = r.ask(100, 4, rc::kNackFlagRepeat);
  CHECK(t.syms == 4);
  const auto q = r.queue();
  REQUIRE(q.size() == 8);
  for (uint32_t i = 0; i < 8; ++i) CHECK(carries(q[i], 100 + i % 4));
}

TEST(push_order_is_cleared_each_call) {
  Rig r; r.put(100, 4);
  r.ask(100, 4);
  r.ask(102, 1);
  REQUIRE(r.out.size() == 1);
  CHECK(carries(r.out[0], 102));
}

TEST(no_op_spends_initial_tokens_then_refuses) {
  {
    Rig r; r.put(100, 40);
    const auto t = r.ask(100, 40);
    CHECK(t.syms == 40); CHECK(t.refused == 0); CHECK(t.miss == 0);
  }
  {
    Rig r; r.put(100, 40);
    const auto t = r.ask(100, 40, rc::kNackFlagRepeat);
    CHECK(t.syms == 32); CHECK(t.refused == 8);
    CHECK(r.out.size() == 64);
  }
}

TEST(op_sets_bucket_depth) {
  Rig r; r.put(100, 40);
  r.cfg.burst_ms = 1;
  AppliedOp op;
  op.ladder = rc::ladder_from(rc::PhyMode::HT, 0, 20);
  const double sym_per_s = delivered_mbps(op.ladder[0], r.air) * 1e6 / 8.0 / static_cast<double>(kEnv);
  const double depth = TokenBucket::depth_for(sym_per_s, r.cfg.burst_ms);
  REQUIRE(depth < 40);
  const auto t = r.ask(100, 40, 0, &op);
  CHECK(t.syms == static_cast<uint64_t>(std::floor(depth)));
  CHECK(t.refused == 40 - t.syms);
}

MTEST_MAIN
