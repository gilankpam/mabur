// Async-mode contract (as amended by fec-join-delete):
// with a FecWorker attached, every envelope SwEncoder emits is byte-exact
// to one the sync encoder emits for the same feed — sources all of them,
// repairs a subset: a repair credited while the worker queue already holds
// kMaxBacklogJobs is skipped and booked in SwFecGauge::backlog_drops, so
// sync repairs_out == async repairs_out + backlog_drops always, and the
// sets are equal whenever backlog_drops is 0. Only emission order relaxes
// (repairs surface at a later add_packet/flush/collect drain). flush() does
// NOT join; finish() does, so counts are comparable at every finish
// boundary.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

#include "mabur/fec_worker.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
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
      sink(async_enc.finish(), eb);
      // finish joined: counts equal HERE, up to repairs the backlog cap
      // skipped (a fast host can outrun the worker even between
      // checkpoints).
      CHECK(ea.size() == eb.size() + async_enc.take_fec_gauge().backlog_drops);
    }
  }
  sink(sync_enc.flush(), ea);
  sink(async_enc.flush(), eb);
  sink(async_enc.finish(), eb);
  std::sort(ea.begin(), ea.end());
  std::sort(eb.begin(), eb.end());
  const uint64_t drops = async_enc.take_fec_gauge().backlog_drops;
  CHECK(sync_enc.sources_out() == async_enc.sources_out());
  CHECK(sync_enc.repairs_out() == async_enc.repairs_out() + drops);
  CHECK(eb.size() == async_enc.sources_out() + async_enc.repairs_out());
  if (drops == 0)
    CHECK(ea == eb);
  else
    CHECK(std::includes(ea.begin(), ea.end(), eb.begin(), eb.end()));
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

// Gauge contract (fec-compute handover): in async mode every
// repair passes through execute_repair_job (worker or inline fallback), so
// after a joining flush the cumulative job count equals repairs_out. The
// tiny queue forces inline fallbacks, which must be counted too.
TEST(async_gauge_jobs_equals_repairs_out) {
  SwConfig cfg{164, 16, 1.0};
  FecWorker worker(-1, 1);
  SwEncoder enc(cfg, 42, &worker);
  std::mt19937 rng(7);
  for (int i = 0; i < 800; ++i) {
    std::vector<uint8_t> p(1 + rng() % static_cast<size_t>(cfg.max_packet_size()));
    for (auto& v : p) v = static_cast<uint8_t>(rng());
    enc.add_packet(p.data(), p.size());
  }
  enc.flush();
  enc.finish();  // joins: nothing outstanding when we read the gauge
  const auto g = enc.take_fec_gauge();
  CHECK(enc.repairs_out() > 0);
  CHECK(g.jobs == enc.repairs_out());
  CHECK(g.inline_full <= g.jobs);
  // Window maxima reset on take; cumulative fields don't.
  const auto g2 = enc.take_fec_gauge();
  CHECK(g2.jobs == g.jobs);
  CHECK(g2.join_wait_max_us == 0);
  CHECK(g2.enq_depth_max == 0);
}

TEST(sync_gauge_stays_zero) {
  SwConfig cfg{64, 8, 1.0};
  SwEncoder enc(cfg, 5);
  std::mt19937 rng(11);
  for (int i = 0; i < 200; ++i) {
    std::vector<uint8_t> p(40);
    for (auto& v : p) v = static_cast<uint8_t>(rng());
    enc.add_packet(p.data(), p.size());
  }
  enc.flush();
  CHECK(enc.repairs_out() > 0);
  const auto g = enc.take_fec_gauge();
  CHECK(g.jobs == 0);
  CHECK(g.build_us == 0);
  CHECK(g.inline_full == 0);
  CHECK(g.join_waits == 0);
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

// ---- fec-join-delete --------------------------------------------------
// Test helpers: a held worker (FecWorker::set_held, test surface) parks
// the consumer so the producer's behaviour with an arbitrarily slow worker
// is deterministic. Feeds stay well under kSlackRows seals so the ring-row
// backstop join — the one join that MUST remain — is never the thing
// under test.

namespace {
std::vector<uint8_t> full_packet(const SwConfig& cfg, std::mt19937& rng) {
  std::vector<uint8_t> p(static_cast<size_t>(cfg.max_packet_size()));
  for (auto& v : p) v = static_cast<uint8_t>(rng());
  return p;
}
// Spins until the worker has built every repair the encoder handed it.
bool wait_worker_idle(SwEncoder& enc, int timeout_ms) {
  const auto t0 = std::chrono::steady_clock::now();
  while (enc.repairs_outstanding()) {
    if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(timeout_ms))
      return false;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}
}  // namespace

// The frame-end flush must return while repairs are still queued: the
// hot thread no longer spin-waits for the worker (the 64 %-of-core join,
// docs/findings/2026-09-21-bitrate-ceiling.md). A watchdog releases the
// worker after 3 s so a regression fails instead of hanging.
TEST(flush_returns_without_joining_a_held_worker) {
  SwConfig cfg{164, 16, 1.0};
  FecWorker worker(-1, 256);
  worker.set_held(true);
  SwEncoder enc(cfg, 5, &worker);
  std::atomic<bool> watchdog_fired{false};
  std::thread watchdog([&] {
    for (int i = 0; i < 300 && !watchdog_fired.load(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!watchdog_fired.exchange(true)) worker.set_held(false);
  });
  std::mt19937 rng(1);
  std::vector<std::vector<uint8_t>> got;
  for (int i = 0; i < 40; ++i) {
    auto p = full_packet(cfg, rng);
    for (auto& e : enc.add_packet(p.data(), p.size())) got.push_back(std::move(e));
  }
  for (auto& e : enc.flush()) got.push_back(std::move(e));
  const bool returned_while_held = !watchdog_fired.load();
  watchdog_fired.store(true);  // disarm
  watchdog.join();
  CHECK(returned_while_held);
  CHECK(enc.repairs_outstanding());
  CHECK(enc.take_fec_gauge().join_waits == 0);
  // Sources shipped, every credited repair is still queued.
  CHECK(got.size() == enc.sources_out());
  worker.set_held(false);
  for (auto& e : enc.finish()) got.push_back(std::move(e));
  CHECK(got.size() == enc.sources_out() + enc.repairs_out());
}

// collect() hands over what the worker has finished, never waits for the
// rest — the hot loop's ≤5 ms harvest between frames.
TEST(collect_returns_finished_repairs_without_join) {
  SwConfig cfg{164, 16, 1.0};
  FecWorker worker(-1, 256);
  worker.set_held(true);
  SwEncoder enc(cfg, 5, &worker);
  std::mt19937 rng(2);
  for (int i = 0; i < 40; ++i) {
    auto p = full_packet(cfg, rng);
    enc.add_packet(p.data(), p.size());
  }
  enc.flush();
  CHECK(enc.collect().empty());  // nothing finished yet
  worker.set_held(false);
  REQUIRE(wait_worker_idle(enc, 3000));
  const auto repairs = enc.collect();
  CHECK(repairs.size() == enc.repairs_out());
  CHECK(enc.collect().empty());  // drained
  CHECK(enc.take_fec_gauge().join_waits == 0);
}

// Backlog bound: once the worker queue holds kMaxBacklogJobs, further
// credited repairs are skipped and booked in backlog_drops — the encoder
// degrades to fewer repairs, never blocks, never drops a source. What it
// DOES emit stays a byte-exact subset of the sync encoder's output.
TEST(backlog_cap_skips_repairs_never_sources) {
  SwConfig cfg{164, 16, 2.0};
  SwEncoder sync_enc(cfg, 9);
  FecWorker worker(-1, 256);
  worker.set_held(true);
  SwEncoder enc(cfg, 9, &worker);
  std::mt19937 rng(3);
  std::vector<std::vector<uint8_t>> ea, eb;
  for (int i = 0; i < 300; ++i) {  // 300 seals × ov 2 = 600 credits
    auto p = full_packet(cfg, rng);
    for (auto& e : sync_enc.add_packet(p.data(), p.size())) ea.push_back(std::move(e));
    for (auto& e : enc.add_packet(p.data(), p.size())) eb.push_back(std::move(e));
  }
  for (auto& e : sync_enc.flush()) ea.push_back(std::move(e));
  for (auto& e : enc.flush()) eb.push_back(std::move(e));
  CHECK(worker.depth() <= SwEncoder::kMaxBacklogJobs);
  CHECK(worker.depth() == SwEncoder::kMaxBacklogJobs);
  const auto g = enc.take_fec_gauge();
  CHECK(g.backlog_drops > 0);
  CHECK(g.join_waits == 0);
  CHECK(g.inline_full == 0);
  CHECK(enc.sources_out() == sync_enc.sources_out());
  CHECK(enc.repairs_out() + g.backlog_drops == sync_enc.repairs_out());
  worker.set_held(false);
  for (auto& e : enc.finish()) eb.push_back(std::move(e));
  CHECK(eb.size() == enc.sources_out() + enc.repairs_out());
  std::sort(ea.begin(), ea.end());
  std::sort(eb.begin(), eb.end());
  CHECK(std::includes(ea.begin(), ea.end(), eb.begin(), eb.end()));
}

// Sync mode has no worker and therefore no backlog: the gauge stays zero
// and every credited repair is built.
TEST(sync_mode_never_books_backlog_drops) {
  SwConfig cfg{164, 16, 2.0};
  SwEncoder enc(cfg, 9);
  std::mt19937 rng(4);
  for (int i = 0; i < 300; ++i) {
    auto p = full_packet(cfg, rng);
    enc.add_packet(p.data(), p.size());
  }
  enc.flush();
  CHECK(enc.take_fec_gauge().backlog_drops == 0);
  CHECK(enc.repairs_out() == 600);
}

// begin_frame's frame-end batch is built inline even with a worker
// attached, so it ships in the SAME flush() call rather than waiting on a
// harvest — unlike the front repairs credited earlier in the frame, which
// are enqueued and may or may not have finished by the time flush() runs.
TEST(async_frame_end_batch_is_built_inline_and_counted) {
  FecWorker worker(-1, 256);
  SwConfig cfg{64, 32, 0.25};
  SwEncoder e(cfg, 0, &worker);
  e.begin_frame(50, 16);
  std::mt19937 rng(1);
  for (int i = 0; i < 50; ++i) {
    auto p = full_packet(cfg, rng);
    e.add_packet(p.data(), p.size());
  }
  std::vector<std::vector<uint8_t>> during_flush;
  e.flush([&](const uint8_t* p, size_t n) { during_flush.emplace_back(p, p + n); });
  int frame_end = 0;
  for (auto& env : during_flush) {
    sw::SwHeader h;
    REQUIRE(sw::parse_header(env.data(), env.size(), &h));
    if (h.repair && h.seq + h.window_len == 50) ++frame_end;
  }
  CHECK(frame_end == 5);
  CHECK(e.take_fec_gauge().frame_end_repairs == 5);
  // The front's 8 repairs (seals 4,8,...,32) went to the worker, not
  // inline: an enqueued frame-end batch would show up here instead of in
  // frame_end_repairs above.
  (void)e.finish();
  CHECK(e.take_fec_gauge().jobs == 8);
}

// ---- ring-row backstop -------------------------------------------------
// The one wait left on the feed path: the producer waits for the worker
// only when the seal it is about to write could overwrite a row an
// unfinished queued job still reads, and only until that job is built.

namespace {
// Releases a held worker after delay_ms unless disarmed first, so a
// producer that waits when it must not fails its test instead of hanging.
struct ReleaseAfter {
  ReleaseAfter(FecWorker& w, int delay_ms) : t([this, &w, delay_ms] {
    for (int i = 0; i < delay_ms && !done.load(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!done.exchange(true)) { fired.store(true); w.set_held(false); }
  }) {}
  // True when the producer got here before the release.
  bool disarm() {
    const bool in_time = !done.exchange(true);
    t.join();
    return in_time && !fired.load();
  }
  std::atomic<bool> done{false}, fired{false};
  std::thread t;
};
bool wait_depth_at_most(FecWorker& w, uint32_t depth, int timeout_ms) {
  const auto t0 = std::chrono::steady_clock::now();
  while (w.depth() > depth) {
    if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(timeout_ms))
      return false;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}
}  // namespace

// A worker that always has something queued, but is never further behind
// than a few hundred seals, is not waited for however long the feed runs:
// rounds of 200 seals, the worker let through about half its queue between
// rounds, so a job is outstanding at every enqueue for more seals than the
// reach while the oldest unfinished one stays far inside it.
TEST(busy_worker_inside_the_slack_is_not_joined) {
  SwConfig cfg{164, 16, 1.0};
  SwEncoder sync_enc(cfg, 11);
  FecWorker worker(-1, 512);
  worker.set_held(true);
  SwEncoder enc(cfg, 11, &worker);
  REQUIRE(enc.job_row_reach() > 500);
  const int rounds = static_cast<int>(enc.job_row_reach() / 200) + 2;
  std::mt19937 rng(4);
  std::vector<std::vector<uint8_t>> ea, eb;
  bool never_waited = true;
  for (int round = 0; round < rounds; ++round) {
    if (round > 0) {
      worker.set_held(false);
      REQUIRE(wait_depth_at_most(worker, 100, 3000));
      worker.set_held(true);
      // Parked before its next dequeue: let the build in flight finish.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      REQUIRE(enc.repairs_outstanding());
    }
    ReleaseAfter guard(worker, 3000);
    for (int i = 0; i < 200; ++i) {
      auto p = full_packet(cfg, rng);
      for (auto& e : sync_enc.add_packet(p.data(), p.size())) ea.push_back(std::move(e));
      for (auto& e : enc.add_packet(p.data(), p.size())) eb.push_back(std::move(e));
    }
    never_waited = guard.disarm() && never_waited;
  }
  CHECK(never_waited);
  const auto g = enc.take_fec_gauge();
  CHECK(g.join_waits == 0);
  worker.set_held(false);
  for (auto& e : sync_enc.flush()) ea.push_back(std::move(e));
  for (auto& e : enc.flush()) eb.push_back(std::move(e));
  for (auto& e : enc.finish()) eb.push_back(std::move(e));
  CHECK(eb.size() == enc.sources_out() + enc.repairs_out());
  std::sort(ea.begin(), ea.end());
  std::sort(eb.begin(), eb.end());
  CHECK(std::includes(ea.begin(), ea.end(), eb.begin(), eb.end()));
}

// The backstop itself: one job queued behind a held worker reads the first
// row. The producer seals for job_row_reach() seals past that row without
// waiting, waits at the next seal until the worker has built the job, and
// the repair is still the sync encoder's, byte for byte.
TEST(backstop_waits_exactly_at_the_reach) {
  SwConfig cfg{164, 16, 1.0};
  SwEncoder sync_enc(cfg, 21);
  FecWorker worker(-1, 256);
  worker.set_held(true);
  SwEncoder enc(cfg, 21, &worker);
  std::mt19937 rng(5);
  std::vector<std::vector<uint8_t>> ea, eb;
  auto feed = [&] {
    auto p = full_packet(cfg, rng);
    for (auto& e : sync_enc.add_packet(p.data(), p.size())) ea.push_back(std::move(e));
    for (auto& e : enc.add_packet(p.data(), p.size())) eb.push_back(std::move(e));
  };
  // A full packet seals when the next one needs the room: the second feed
  // seals row 0, whose credit is one repair over that row: queued, not built.
  feed();
  feed();
  REQUIRE(worker.depth() == 1);
  sync_enc.set_overhead(0.0);
  enc.set_overhead(0.0);
  const size_t reach = enc.job_row_reach();
  {
    ReleaseAfter guard(worker, 3000);
    for (size_t k = 1; k < reach; ++k) feed();
    CHECK(guard.disarm());
  }
  CHECK(enc.take_fec_gauge().join_waits == 0);
  CHECK(worker.depth() == 1);
  {
    ReleaseAfter guard(worker, 50);
    feed();  // the seal at age == reach: waits for the release
    CHECK(!guard.disarm());
  }
  worker.set_held(false);  // a producer that did not wait must not hang below
  CHECK(enc.take_fec_gauge().join_waits == 1);
  for (int k = 0; k < 140; ++k) feed();  // past the row the job read
  for (auto& e : sync_enc.flush()) ea.push_back(std::move(e));
  for (auto& e : enc.flush()) eb.push_back(std::move(e));
  for (auto& e : enc.finish()) eb.push_back(std::move(e));
  std::sort(ea.begin(), ea.end());
  std::sort(eb.begin(), eb.end());
  CHECK(ea == eb);
}

// Windows differ in length, so a job queued later can read rows from before
// the oldest unfinished job's first row: here a 16-row job, then a 128-row
// job whose window starts 111 rows earlier. The producer has to wait before
// the longer job's first row is overwritten, which is 18 seals after the
// reach measured from the older job and 109 before that job's own row.
TEST(backstop_covers_a_later_job_with_a_longer_window) {
  SwConfig cfg{164, 16, 1.0};
  SwEncoder sync_enc(cfg, 31);
  FecWorker worker(-1, 256);
  worker.set_held(true);
  SwEncoder enc(cfg, 31, &worker);
  std::mt19937 rng(6);
  std::vector<std::vector<uint8_t>> ea, eb;
  auto feed = [&] {
    auto p = full_packet(cfg, rng);
    for (auto& e : sync_enc.add_packet(p.data(), p.size())) ea.push_back(std::move(e));
    for (auto& e : enc.add_packet(p.data(), p.size())) eb.push_back(std::move(e));
  };
  auto overhead = [&](double o) {
    sync_enc.set_overhead(o);
    enc.set_overhead(o);
  };
  // 200 rows with no repairs, so a 128-row window has its rows.
  overhead(0.0);
  for (int k = 0; k < 200; ++k) feed();
  REQUIRE(worker.depth() == 0);
  overhead(1.0);
  feed();  // seals a row: one repair over the configured 16 rows
  REQUIRE(worker.depth() == 1);
  sync_enc.begin_frame(size_t{1} << 30, 0, 128);
  enc.begin_frame(size_t{1} << 30, 0, 128);
  feed();  // seals the next row: one repair over 128 rows
  REQUIRE(worker.depth() == 2);
  overhead(0.0);
  const size_t reach = enc.job_row_reach();
  {
    // The 16-row job's first row is 15 back from its seal and one more seal
    // went in since: the feed that seals at age == reach is number reach - 16.
    ReleaseAfter guard(worker, 3000);
    for (size_t k = 1; k < reach - 16; ++k) feed();
    CHECK(guard.disarm());
  }
  CHECK(enc.take_fec_gauge().join_waits == 0);
  CHECK(worker.depth() == 2);
  {
    ReleaseAfter guard(worker, 50);
    feed();  // waits for the release
    CHECK(!guard.disarm());
  }
  {
    // 30 more seals run over the 128-row job's first rows. A producer that
    // did not wait above still has the worker held here, and its repair
    // comes out of overwritten rows.
    ReleaseAfter guard(worker, 3000);
    for (int k = 0; k < 30; ++k) feed();
    (void)guard.disarm();
  }
  worker.set_held(false);
  for (auto& e : sync_enc.flush()) ea.push_back(std::move(e));
  for (auto& e : enc.flush()) eb.push_back(std::move(e));
  for (auto& e : enc.finish()) eb.push_back(std::move(e));
  std::sort(ea.begin(), ea.end());
  std::sort(eb.begin(), eb.end());
  CHECK(ea == eb);
}

MTEST_MAIN
