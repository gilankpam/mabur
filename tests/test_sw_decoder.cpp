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

TEST(late_source_copy_of_recovered_symbol_counted_arrived) {
  // Repair-vs-arrival race (bench 2026-07-27): a repair processed before a
  // reordered source symbol solves it ("recovered"); the direct copy landing
  // moments later must count syms_recovered_arrived, so the ladder's pre-FEC
  // loss metric can tell a race win from a symbol that truly never arrived.
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 10, &pkts);
  SwDecoder d(cfg);
  std::vector<uint8_t> held;  // source seq 2, delayed past its repairs
  std::vector<std::vector<uint8_t>> got;
  for (auto& env : envs) {
    sw::SwHeader h;
    CHECK(sw::parse_header(env.data(), env.size(), &h));
    if (!h.repair && h.seq == 2) { held = env; continue; }
    for (auto& p : d.add_symbol(env.data(), env.size(), 1000)) got.push_back(std::move(p));
  }
  CHECK(d.syms_recovered() == 1);          // repair won the race
  CHECK(d.syms_recovered_arrived() == 0);  // copy not seen yet
  CHECK(d.add_symbol(held.data(), held.size(), 1001).empty());  // dup payload
  CHECK(d.syms_recovered_arrived() == 1);
  CHECK(d.syms_delivered() == 9);  // reclassification never touches delivered
  CHECK(to_set(got) == to_set(pkts));

  // A SECOND copy of the same symbol is a plain stale dup again.
  const auto stale_before = d.symbols_dropped_stale();
  CHECK(d.add_symbol(held.data(), held.size(), 1002).empty());
  CHECK(d.syms_recovered_arrived() == 1);
  CHECK(d.symbols_dropped_stale() == stale_before + 1);
}

TEST(dup_of_delivered_symbol_not_counted_recovered_arrived) {
  SwConfig cfg{64, 8, 1.0};
  auto envs = encode_stream(cfg, 10, nullptr);
  SwDecoder d(cfg);
  for (int pass = 0; pass < 2; ++pass)  // every envelope twice (two cards)
    for (auto& env : envs) d.add_symbol(env.data(), env.size(), 1000);
  CHECK(d.syms_recovered() == 0);
  CHECK(d.syms_recovered_arrived() == 0);
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

TEST(encoder_restart_with_random_seq_recovers) {
  // A rebooted encoder must land >kResetSpan away so the decoder resets
  // instead of swallowing the new stream as stale (final-review Critical:
  // seq-0 restart after N<2^20 symbols silently dropped N+1 packets).
  SwConfig cfg{64, 8, 0.5};
  SwEncoder e1(cfg, 0);
  SwDecoder d(cfg);
  std::vector<uint8_t> p(62, 0xAA);
  for (int i = 0; i < 200; ++i)
    for (auto& env : e1.add_packet(p.data(), p.size())) d.add_symbol(env.data(), env.size(), 1000);

  // Restart lands far away (simulated random draw): decoder re-anchors.
  SwEncoder e2(cfg, 0x40000000u);
  std::vector<std::vector<uint8_t>> pkts2, got;
  for (int i = 0; i < 50; ++i) {
    auto q = p;
    q[0] = static_cast<uint8_t>(i);
    pkts2.push_back(q);
    for (auto& env : e2.add_packet(q.data(), q.size()))
      for (auto& out : d.add_symbol(env.data(), env.size(), 2000)) got.push_back(std::move(out));
  }
  for (auto& env : e2.flush())
    for (auto& out : d.add_symbol(env.data(), env.size(), 2000)) got.push_back(std::move(out));
  CHECK(d.resets() == 1);
  CHECK(got.size() == pkts2.size());
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

TEST(watermark_pre_transition_abandonment_books_stale) {
  // 62-byte packets, one per symbol at symbol_size 64: pkt index == seq.
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 40, &pkts);
  SwDecoder d(cfg);
  // Feed the first half, dropping sources for seqs 4..6 AND all repairs
  // (so the hole can never be recovered and must be abandoned).
  size_t src_i = 0;
  std::vector<std::vector<uint8_t>> fed_tail;
  for (auto& env : envs) {
    sw::SwHeader h;
    REQUIRE(sw::parse_header(env.data(), env.size(), &h));
    if (h.repair) continue;
    if (src_i >= 20) { fed_tail.push_back(env); ++src_i; continue; }
    const bool drop = src_i >= 4 && src_i <= 6;
    if (!drop) d.add_symbol(env.data(), env.size(), 1000);
    ++src_i;
  }
  CHECK(d.syms_abandoned() == 0);  // hole still inside horizon (8*4=32)
  // Transition: everything so far is pre-transition.
  d.mark_transition();
  CHECK(d.boundary_open());
  // Feed the tail as kPost sources: first one closes the boundary; their
  // seq advance pushes the hole past the horizon -> abandonment.
  for (auto& env : fed_tail) d.add_symbol(env.data(), env.size(), 1001, SwBoundary::kPost);
  CHECK(!d.boundary_open());
  CHECK(d.syms_abandoned() == 3);
  CHECK(d.syms_abandoned_stale() == 3);  // all three below the watermark
}

TEST(watermark_post_transition_abandonment_books_current) {
  // 60 sources so the hole at 12..14 falls past the horizon (8*4=32) by the
  // end: floor = 59 - 32 = 27 > 14.
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 60, &pkts);
  SwDecoder d(cfg);
  std::vector<std::vector<uint8_t>> sources;
  for (auto& env : envs) {
    sw::SwHeader h;
    REQUIRE(sw::parse_header(env.data(), env.size(), &h));
    if (!h.repair) sources.push_back(env);
  }
  REQUIRE(sources.size() == 60);
  // Clean pre-transition run: seqs 0..9.
  for (size_t i = 0; i < 10; ++i) d.add_symbol(sources[i].data(), sources[i].size(), 1000);
  d.mark_transition();
  // Post-transition: seq 10 closes the boundary (wm = 9); then drop 12..14
  // and feed the rest — a CURRENT-rung hole.
  for (size_t i = 10; i < 60; ++i) {
    if (i >= 12 && i <= 14) continue;
    d.add_symbol(sources[i].data(), sources[i].size(), 1001, SwBoundary::kPost);
  }
  CHECK(d.syms_abandoned() == 3);
  CHECK(d.syms_abandoned_stale() == 0);  // above the watermark = current
}

TEST(watermark_open_boundary_books_stale_and_pre_advances) {
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 60, &pkts);
  SwDecoder d(cfg);
  std::vector<std::vector<uint8_t>> sources;
  for (auto& env : envs) {
    sw::SwHeader h;
    REQUIRE(sw::parse_header(env.data(), env.size(), &h));
    if (!h.repair) sources.push_back(env);
  }
  REQUIRE(sources.size() == 60);
  for (size_t i = 0; i < 5; ++i) d.add_symbol(sources[i].data(), sources[i].size(), 1000);
  d.mark_transition();  // wm = 4, open
  // In-flight old-op stragglers arrive AFTER the transition with kPre:
  // 5..7 heard, 8..9 lost (killed in flight). Boundary never closes here
  // (no kPost source ever arrives — e.g. the new-op stream is not heard
  // yet). Advance far with unhinted (kNone) sources 10..59 so the hole
  // 8..9 falls past the horizon (floor = 59 - 32 = 27).
  for (size_t i = 5; i < 8; ++i) d.add_symbol(sources[i].data(), sources[i].size(), 1001, SwBoundary::kPre);
  for (size_t i = 10; i < 60; ++i) d.add_symbol(sources[i].data(), sources[i].size(), 1002);
  CHECK(d.boundary_open());
  CHECK(d.syms_abandoned() == 2);  // exactly the killed-in-flight 8..9
  CHECK(d.syms_abandoned_stale() == d.syms_abandoned());  // open => all stale
  d.close_boundary();
  CHECK(!d.boundary_open());
}

TEST(watermark_inactive_decoder_books_all_current) {
  // No mark_transition() ever: stale counter must stay 0 (legacy behavior).
  SwConfig cfg{64, 8, 1.0};
  std::vector<std::vector<uint8_t>> pkts;
  auto envs = encode_stream(cfg, 40, &pkts);
  SwDecoder d(cfg);
  size_t src_i = 0;
  for (auto& env : envs) {
    sw::SwHeader h;
    REQUIRE(sw::parse_header(env.data(), env.size(), &h));
    if (h.repair) continue;
    const bool drop = src_i >= 4 && src_i <= 6;
    if (!drop) d.add_symbol(env.data(), env.size(), 1000);
    ++src_i;
  }
  CHECK(d.syms_abandoned() == 3);
  CHECK(d.syms_abandoned_stale() == 0);
}

// --- ArrivalTracker feed (spec 2026-09-05-arrival-loss-design.md §4.1) ----
// Sources only (overhead 0.0) so pkt index == seq and no repairs move the
// expectation; default guard 32.

TEST(arrival_counts_clean_stream) {
  SwConfig cfg{64, 4, 0.0};
  auto envs = encode_stream(cfg, 100, nullptr);
  SwDecoder d(cfg, /*seq_horizon=*/512);
  for (auto& env : envs) d.add_symbol(env.data(), env.size(), 1000);
  CHECK(d.arr_expected() == 68);  // newest 99 - guard 32 -> seqs 0..67
  CHECK(d.arr_arrived() == 68);
  CHECK(d.arr_expected_stale() == 0);
  CHECK(d.arr_late() == 0);
  CHECK(d.syms_abandoned() == 0);  // decode accounting untouched
}

TEST(arrival_counts_loss_pattern_exactly) {
  SwConfig cfg{64, 4, 0.0};
  auto envs = encode_stream(cfg, 100, nullptr);
  SwDecoder d(cfg, 512);
  for (size_t i = 0; i < envs.size(); ++i) {
    if (i == 3 || (i >= 40 && i <= 42)) continue;
    d.add_symbol(envs[i].data(), envs[i].size(), 1000);
  }
  CHECK(d.arr_expected() == 68);
  CHECK(d.arr_arrived() == 64);
}

TEST(arrival_duplicate_source_is_idempotent) {
  SwConfig cfg{64, 4, 0.0};
  auto envs = encode_stream(cfg, 100, nullptr);
  SwDecoder d(cfg, 512);
  for (auto& env : envs) {
    d.add_symbol(env.data(), env.size(), 1000);
    d.add_symbol(env.data(), env.size(), 1000);  // second card heard it too
  }
  CHECK(d.arr_expected() == 68);
  CHECK(d.arr_arrived() == 68);
  CHECK(d.arr_late() == 0);
}

TEST(arrival_behind_settle_line_counts_late) {
  SwConfig cfg{64, 4, 0.0};
  auto envs = encode_stream(cfg, 100, nullptr);
  SwDecoder d(cfg, 512);
  for (size_t i = 0; i < envs.size(); ++i) {
    if (i == 3) continue;
    d.add_symbol(envs[i].data(), envs[i].size(), 1000);
  }
  d.add_symbol(envs[3].data(), envs[3].size(), 1001);  // 96 seqs late
  CHECK(d.arr_late() == 1);
  CHECK(d.arr_arrived() == 67);  // not un-booked
}

TEST(arrival_stale_split_follows_the_watermark) {
  // Mirrors watermark_pre_transition_abandonment_books_stale: drop 4..6
  // before the transition, close the boundary with kPost sources.
  SwConfig cfg{64, 4, 0.0};
  auto envs = encode_stream(cfg, 100, nullptr);
  SwDecoder d(cfg, 512);
  for (size_t i = 0; i < 20; ++i) {
    if (i >= 4 && i <= 6) continue;
    d.add_symbol(envs[i].data(), envs[i].size(), 1000);
  }
  d.mark_transition();
  for (size_t i = 20; i < 100; ++i) {
    if (i == 30) continue;  // a CURRENT-rung loss
    d.add_symbol(envs[i].data(), envs[i].size(), 1001, SwBoundary::kPost);
  }
  CHECK(!d.boundary_open());
  CHECK(d.arr_expected() == 68);
  CHECK(d.arr_arrived() == 64);
  CHECK(d.arr_expected_stale() == 20);   // seqs 0..19 (wm = 19)
  CHECK(d.arr_arrived_stale() == 17);    // 4,5,6 missing
  CHECK(d.arr_expected() - d.arr_expected_stale() == 48);
  CHECK(d.arr_arrived() - d.arr_arrived_stale() == 47);  // only seq 30
}

TEST(arrival_repair_advances_expectation) {
  // overhead 1.0: repairs follow every window; drop ALL sources after 10
  // and feed only repairs -> expectation advances, nothing arrives.
  SwConfig cfg{64, 8, 1.0};
  auto envs = encode_stream(cfg, 60, nullptr);
  SwDecoder d(cfg, 512);
  size_t src = 0;
  for (auto& env : envs) {
    sw::SwHeader h;
    REQUIRE(sw::parse_header(env.data(), env.size(), &h));
    if (!h.repair) { if (src++ < 10) d.add_symbol(env.data(), env.size(), 1000); continue; }
    d.add_symbol(env.data(), env.size(), 1000);
  }
  CHECK(d.arr_expected() > 10);
  CHECK(d.arr_arrived() == 10);
}

TEST(arrival_counters_survive_encoder_restart) {
  SwConfig cfg{64, 4, 0.0};
  SwDecoder d(cfg, 512);
  auto a = encode_stream(cfg, 100, nullptr);
  for (auto& env : a) d.add_symbol(env.data(), env.size(), 1000);
  const auto e0 = d.arr_expected();
  REQUIRE(e0 == 68);
  SwEncoder e2(cfg, /*initial_seq=*/5000000);  // far jump -> decoder reset
  std::vector<std::vector<uint8_t>> b;
  for (int i = 0; i < 100; ++i) { auto p = pat(62, i); for (auto& env : e2.add_packet(p.data(), p.size())) b.push_back(env); }
  for (auto& env : e2.flush()) b.push_back(env);
  for (auto& env : b) d.add_symbol(env.data(), env.size(), 2000);
  CHECK(d.resets() == 1);
  CHECK(d.arr_expected() == e0 + 68);
  CHECK(d.arr_arrived() == e0 + 68);
}

MTEST_MAIN
