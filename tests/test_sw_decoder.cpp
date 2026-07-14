#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
#include "mtest.h"
#include "vectors.h"

using namespace mabur;

namespace {
std::vector<uint8_t> pat(size_t n, int seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i)
    v[i] = static_cast<uint8_t>((i * 31 + static_cast<size_t>(seed) * 17 + 7) & 0xFF);
  return v;
}

// Encode n_packets 62-byte packets (one per symbol at symbol_size 64) and
// return the envelope stream (flush() included).
std::vector<std::vector<uint8_t>> encode_stream(const SwConfig& cfg, int n_packets,
                                                std::vector<std::vector<uint8_t>>* pkts) {
  SwEncoder e(cfg);
  std::vector<std::vector<uint8_t>> envs;
  for (int i = 0; i < n_packets; ++i) {
    auto p = pat(62, i);
    if (pkts) pkts->push_back(p);
    for (auto& env : e.add_packet(p.data(), p.size())) envs.push_back(std::move(env));
  }
  for (auto& env : e.flush()) envs.push_back(std::move(env));
  return envs;
}

std::set<std::vector<uint8_t>> to_set(const std::vector<std::vector<uint8_t>>& v) {
  return {v.begin(), v.end()};
}
}  // namespace

TEST(clean_stream_delivers_all_sources_immediately) {
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 20, &pkts);
  SwDecoder d(cfg);
  std::vector<std::vector<uint8_t>> got;
  for (auto& env : envs)
    for (auto& p : d.add_symbol(env.data(), env.size(), 1000)) got.push_back(std::move(p));
  CHECK(got.size() == pkts.size());
  CHECK(to_set(got) == to_set(pkts));
  CHECK(d.syms_delivered() == 20);
  CHECK(d.syms_recovered() == 0);     // repairs were all redundant
  CHECK(d.rows_in_flight() == 0);     // redundant repairs must not pile up
}

TEST(single_loss_recovered_from_repair) {
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 10, &pkts);
  SwDecoder d(cfg);
  std::vector<std::vector<uint8_t>> got;
  for (size_t i = 0; i < envs.size(); ++i) {
    if (i == 2) continue;  // drop one source envelope
    for (auto& p : d.add_symbol(envs[i].data(), envs[i].size(), 1000)) got.push_back(std::move(p));
  }
  CHECK(to_set(got) == to_set(pkts));
  CHECK(d.syms_recovered() == 1);
}

TEST(burst_loss_within_budget_recovered) {
  // window 16, overhead 1.0: drop 4 consecutive sources; the next repairs
  // cover them.
  SwConfig cfg{64, 16, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 30, &pkts);
  SwDecoder d(cfg);
  std::vector<std::vector<uint8_t>> got;
  int dropped_sources = 0;
  for (auto& env : envs) {
    sw::SwHeader h;
    CHECK(sw::parse_header(env.data(), env.size(), &h));
    if (!h.repair && h.seq >= 5 && h.seq <= 8) { ++dropped_sources; continue; }
    for (auto& p : d.add_symbol(env.data(), env.size(), 1000)) got.push_back(std::move(p));
  }
  CHECK(dropped_sources == 4);
  CHECK(to_set(got) == to_set(pkts));
  CHECK(d.syms_recovered() == 4);
}

TEST(duplicate_sources_and_repairs_idempotent) {
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 10, &pkts);
  SwDecoder d(cfg);
  std::vector<std::vector<uint8_t>> got;
  for (int pass = 0; pass < 2; ++pass)  // every envelope twice (two cards)
    for (auto& env : envs)
      for (auto& p : d.add_symbol(env.data(), env.size(), 1000)) got.push_back(std::move(p));
  CHECK(got.size() == pkts.size());
  CHECK(d.symbols_dropped_stale() >= 10);  // dup sources counted
}

TEST(loss_beyond_horizon_counts_abandoned) {
  SwConfig cfg{64, 4, 0.0};  // no repairs at all
  auto envs = encode_stream(cfg, 60, nullptr);
  SwDecoder d(cfg, /*seq_horizon=*/16);
  uint64_t delivered = 0;
  for (size_t i = 0; i < envs.size(); ++i) {
    if (i == 3) continue;  // lost forever
    delivered += d.add_symbol(envs[i].data(), envs[i].size(), 1000).size();
  }
  CHECK(delivered == 59);
  CHECK(d.syms_abandoned() == 1);  // counted once seq 3 fell behind horizon
}

TEST(encoder_restart_resets_decoder) {
  // A restart is a seq jump beyond kResetSpan (2^20) — a nearby jump must
  // read as dup/stale, not reset. Simulate the long first flight with a
  // hand-crafted far-future source, then a rebooted encoder from seq 0.
  SwConfig cfg{64, 8, 0.5};
  auto s1 = encode_stream(cfg, 20, nullptr);
  SwDecoder d(cfg);
  for (auto& env : s1) d.add_symbol(env.data(), env.size(), 1000);
  CHECK(d.resets() == 0);

  auto make_source = [](uint32_t seq) {
    sw::SwHeader h;
    h.repair = false; h.symbol_size = 64; h.seq = seq;
    std::vector<uint8_t> env;
    sw::pack_header(env, h);
    std::vector<uint8_t> sym(64, 0);
    sym[0] = 2; sym[1] = 0; sym[2] = 0xAB; sym[3] = 0xCD;  // one 2-byte packet
    env.insert(env.end(), sym.begin(), sym.end());
    return env;
  };
  auto jump = make_source(3'000'000);  // far ahead: reset #1
  d.add_symbol(jump.data(), jump.size(), 1500);
  CHECK(d.resets() == 1);

  // Rebooted encoder: seq restarts at 0 — 3M behind newest: reset #2.
  std::vector<std::vector<uint8_t>> pkts2;
  auto s2 = encode_stream(cfg, 20, &pkts2);
  std::vector<std::vector<uint8_t>> got;
  for (auto& env : s2)
    for (auto& p : d.add_symbol(env.data(), env.size(), 2000)) got.push_back(std::move(p));
  CHECK(d.resets() == 2);
  CHECK(to_set(got) == to_set(pkts2));
}

TEST(deadline_expires_stuck_rows) {
  SwConfig cfg{64, 8, 1.0};
  auto envs = encode_stream(cfg, 6, nullptr);
  SwDecoder d(cfg);
  // Anchor with the first source, then feed ONE late repair whose window
  // spans several still-missing sources: it cannot reduce to a single
  // unknown, so it parks as a GE row until the deadline.
  bool fed_source = false;
  std::vector<uint8_t> last_repair;
  for (auto& env : envs) {
    sw::SwHeader h;
    CHECK(sw::parse_header(env.data(), env.size(), &h));
    if (!h.repair) {
      if (!fed_source) {
        d.add_symbol(env.data(), env.size(), 1000);
        fed_source = true;
      }
    } else {
      last_repair = env;
    }
  }
  CHECK(!last_repair.empty());
  CHECK(d.add_symbol(last_repair.data(), last_repair.size(), 1000).empty());
  CHECK(d.rows_in_flight() == 1);
  CHECK(d.expire_rows_older_than(200, 1500) == 1);
  CHECK(d.rows_in_flight() == 0);
}

TEST(vectors_decode_scenarios) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/sw.json");
  for (const auto& c : j["cases"]) {
    std::vector<std::vector<uint8_t>> envs;
    for (const auto& s : c["stream"]) envs.push_back(mtest::unhex(s.get<std::string>()));
    for (const auto& s : c["flush"]) envs.push_back(mtest::unhex(s.get<std::string>()));
    for (const auto& d : c["decode"]) {
      std::set<size_t> drop;
      for (const auto& x : d["drop"]) drop.insert(x.get<size_t>());
      SwDecoder dec(SwConfig{c["symbol_size"], c["window"], c["overhead"]});
      std::vector<std::string> got;
      for (size_t i = 0; i < envs.size(); ++i) {
        if (drop.count(i)) continue;
        for (auto& p : dec.add_symbol(envs[i].data(), envs[i].size(), 1000))
          got.push_back(mtest::hex(p));
      }
      std::sort(got.begin(), got.end());
      size_t k = 0;
      for (const auto& s : d["recovered_sorted"]) CHECK(got.at(k++) == s.get<std::string>());
      CHECK(k == got.size());
    }
  }
}

TEST(bad_cfg_and_garbage_counted_dropped) {
  SwConfig cfg{64, 8, 0.5};
  auto envs = encode_stream(SwConfig{128, 8, 0.5}, 4, nullptr);  // wrong symbol_size
  SwDecoder d(cfg);
  for (auto& env : envs) CHECK(d.add_symbol(env.data(), env.size(), 1000).empty());
  CHECK(d.symbols_dropped_bad_cfg() == envs.size());
  const uint8_t junk[14] = {0};
  CHECK(d.add_symbol(junk, sizeof junk, 1000).empty());
}

MTEST_MAIN
