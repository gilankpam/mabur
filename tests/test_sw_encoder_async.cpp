// Async-mode contract (spec 2026-07-17): with a FecWorker attached,
// SwEncoder emits the SAME envelope SET as sync mode for an identical feed
// — every envelope byte-exact, only emission order relaxes (repairs surface
// at a later add_packet/flush drain). flush() joins the worker, so counts
// are comparable at every flush boundary.
#include <algorithm>
#include <random>
#include <vector>

#include "mabur/fec_worker.h"
#include "mabur/sw_encoder.h"
#include "mtest.h"

using namespace mabur;

static void set_equality(int symbol_size, int window, double ov, int n_pkts,
                         int flush_every, uint32_t queue_slots) {
  SwConfig cfg{symbol_size, window, ov};
  SwEncoder sync_enc(cfg, 777);
  FecWorker worker(-1, queue_slots);
  SwEncoder async_enc(cfg, 777, &worker);
  std::vector<std::vector<uint8_t>> ea, eb;
  auto sink = [](std::vector<std::vector<uint8_t>> envs,
                 std::vector<std::vector<uint8_t>>& into) {
    for (auto& e : envs) into.push_back(std::move(e));
  };
  std::mt19937 rng(99);
  for (int i = 0; i < n_pkts; ++i) {
    const size_t len = 1 + rng() % static_cast<size_t>(cfg.max_packet_size());
    std::vector<uint8_t> p(len);
    for (auto& v : p) v = static_cast<uint8_t>(rng());
    sink(sync_enc.add_packet(p.data(), p.size()), ea);
    sink(async_enc.add_packet(p.data(), p.size()), eb);
    if (flush_every && i % flush_every == flush_every - 1) {
      sink(sync_enc.flush(), ea);
      sink(async_enc.flush(), eb);
      CHECK(ea.size() == eb.size());  // flush joined: counts equal HERE
    }
  }
  sink(sync_enc.flush(), ea);
  sink(async_enc.flush(), eb);
  std::sort(ea.begin(), ea.end());
  std::sort(eb.begin(), eb.end());
  CHECK(ea == eb);
  CHECK(sync_enc.sources_out() == async_enc.sources_out());
  CHECK(sync_enc.repairs_out() == async_enc.repairs_out());
}

TEST(async_set_equals_sync_scalar_geometry) {
  set_equality(164, 64, 0.375, 2000, 97, 256);
}
TEST(async_set_equals_sync_big_symbols) {
  set_equality(1312, 128, 1.0, 1200, 61, 256);
}
// queue_slots=1 forces the try_enqueue-full inline fallback on most repairs;
// whichever path each repair takes, the output set must not change.
TEST(async_tiny_queue_inline_fallback_still_exact) {
  set_equality(164, 16, 2.0, 1500, 0, 1);
}
// Sustained max-overhead load across many window wraps: any stale-row read
// (slack bound / join backstop bug) corrupts a repair payload and breaks
// set equality with overwhelming probability.
TEST(async_sustained_load_no_stale_rows) {
  set_equality(164, 128, 2.0, 6000, 0, 256);
}

TEST(sync_mode_unaffected_by_worker_param_default) {
  // worker == nullptr must be today's exact behavior: repairs inline, in
  // order — ORDER-sensitive equality between the 2-arg and 3-arg forms.
  SwConfig cfg{64, 8, 1.0};
  SwEncoder a(cfg, 5);
  SwEncoder b(cfg, 5, nullptr);
  std::mt19937 rng(3);
  for (int i = 0; i < 200; ++i) {
    std::vector<uint8_t> p(40);
    for (auto& v : p) v = static_cast<uint8_t>(rng());
    CHECK(a.add_packet(p.data(), p.size()) == b.add_packet(p.data(), p.size()));
  }
  CHECK(a.flush() == b.flush());
}

MTEST_MAIN
