// FeedLoop (design-reader-thread.md, Task 11c): the decoder-input thread
// against a real ring + doorbell and a thread-safe fake decoder.
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "au_doorbell.h"
#include "au_ring.h"
#include "au_router.h"
#include "feed_loop.h"
#include "mabur/frame_wire.h"
#include "mtest.h"
#include "ring_client.h"
#include "stream_feeder.h"
#include "video_backend.h"

using namespace std::chrono_literals;
using mabur::framewire::FrameHdr;
using maburplay::AuEvent;
using maburplay::DrainResult;
using maburplay::FeedLoop;
using maburplay::FeedNote;

namespace {

uint64_t mono_us() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

struct FakeBackend : maburplay::VideoBackend, maburplay::StreamDecoder {
  struct Call { char op; uint32_t pts; bool last; };
  mutable std::mutex mu;
  std::condition_variable cv;
  std::vector<Call> calls;
  std::atomic<bool> block_submit{false};
  const std::atomic<bool>* cancel = nullptr;
  void log(Call c) {
    std::lock_guard<std::mutex> lk(mu);
    calls.push_back(c);
    cv.notify_all();
  }
  bool init(const maburplay::BackendCfg&, FrameSink) override { return true; }
  void submit_au(const uint8_t*, size_t, uint32_t pts) override {
    // A wedged decoder: BUFFER_FULL until the feed is told to give way.
    while (block_submit.load() && !(cancel && cancel->load())) std::this_thread::sleep_for(1ms);
    log({'W', pts, false});
  }
  void flush() override { log({'F', 0, false}); }
  void release_frame(const maburplay::DmaFrame&) override {}
  bool probe() override { return true; }
  bool start(const uint8_t*, size_t, uint8_t, uint32_t pts) override { log({'S', pts, false}); return true; }
  bool append(const uint8_t*, size_t, uint32_t pts, bool last) override { log({'A', pts, last}); return true; }
  void abort(uint32_t pts) override { log({'X', pts, true}); }
  bool wait_for(char op, uint32_t pts, std::chrono::milliseconds lim, bool last = false) {
    std::unique_lock<std::mutex> lk(mu);
    return cv.wait_for(lk, lim, [&] {
      for (const Call& c : calls)
        if (c.op == op && c.pts == pts && (!last || c.last)) return true;
      return false;
    });
  }
  size_t count() const { std::lock_guard<std::mutex> lk(mu); return calls.size(); }
  std::vector<Call> snapshot() const { std::lock_guard<std::mutex> lk(mu); return calls; }
};

std::vector<uint8_t> make_au(int n, std::vector<size_t>* starts) {
  std::vector<uint8_t> au;
  for (int k = 0; k < n; ++k) {
    starts->push_back(au.size());
    const uint8_t sc[] = {0, 0, 0, 1, 0x02, 0x01};
    au.insert(au.end(), sc, sc + sizeof sc);
    au.insert(au.end(), static_cast<size_t>(20 + 7 * k), static_cast<uint8_t>(0x40 + k));
  }
  return au;
}

struct Rig {
  std::string ring_path, sock_path;
  maburgs::AuRingWriter w;
  maburgs::AuDoorbell db;
  std::unique_ptr<maburplay::VideoBackend> backend;
  FakeBackend* fake = nullptr;
  maburplay::StreamFeeder feeder{true};
  std::unique_ptr<maburplay::AuRouter> router;
  std::unique_ptr<maburplay::RingClient> ring;
  std::unique_ptr<FeedLoop> feed;
  // What the feed thread's callbacks use: set once before start() and never
  // written again. feed.reset() nulls `feed` BEFORE ~FeedLoop stops the
  // thread, so a callback reading `feed` would race it (and could see null).
  FeedLoop* fl = nullptr;
  std::vector<FeedNote> notes;  // the "main thread"'s view

  explicit Rig(const char* tag, uint32_t drain_ms = 100) {
    const std::string pid = std::to_string(getpid());
    ring_path = std::string("/tmp/test_feed_loop_") + tag + "_" + pid;
    sock_path = ring_path + ".sock";
    REQUIRE(w.open(ring_path, {4096, 16}));
    REQUIRE(db.open(sock_path, {4096, 16}));
    auto fb = std::make_unique<FakeBackend>();
    fake = fb.get();
    backend = std::move(fb);
    feeder.set_decoder(fake);
    router = std::make_unique<maburplay::AuRouter>(feeder, backend);
    ring = std::make_unique<maburplay::RingClient>(
        maburplay::RingClient::Cfg{ring_path, sock_path}, [this](AuEvent&& ev) {
          if (fl && !fl->checkpoint()) return;
          router->on_event(std::move(ev));
        });
    REQUIRE(ring->open());
    feed = std::make_unique<FeedLoop>(*ring, *router, feeder, FeedLoop::Cfg{2, drain_ms, 256});
    fl = feed.get();
    router->set_hooks({[this](FeedNote&& n) { fl->push_note(std::move(n)); },
                       [this]() { return fl->park_for_flush(); }, [] { return mono_us(); }});
    fake->cancel = &fl->cancel_flag();
  }
  ~Rig() {
    feed.reset();
    ring.reset();
    unlink(ring_path.c_str());
    unlink(sock_path.c_str());
  }
  void start_and_connect() {
    feed->start();
    for (int i = 0; i < 500 && !db.client_connected(); ++i) {
      db.poll();
      std::this_thread::sleep_for(1ms);
    }
    REQUIRE(db.client_connected());
    std::this_thread::sleep_for(10ms);  // the hello lands; the feed validates it
  }
  void publish_whole(uint32_t pts, uint8_t sid, bool complete, uint8_t flags = 0) {
    std::vector<size_t> s;
    const auto au = make_au(1, &s);
    FrameHdr h;
    h.pts_us = pts;
    h.flags = flags;
    w.begin(h, sid, 1);
    w.append(au.data(), au.size());
    w.finish(complete, maburgs::AuLatMeta{});
    db.notify(0);
  }
  // maburplay's service_feed(), minus the frame holders.
  bool service() {
    feed->take_notes(&notes);
    if (!feed->flush_waiting()) return false;
    feed->take_notes(&notes);
    feeder.on_flush();
    backend->flush();
    router->disarm();
    feed->resume();
    return true;
  }
};

}  // namespace

TEST(a_slice_reaches_the_decoder_while_the_main_thread_is_busy) {
  Rig r("busy");
  r.start_and_connect();
  r.publish_whole(100, 0, true);  // complete sid-0: arms
  REQUIRE(r.fake->wait_for('W', 100, 500ms));
  std::vector<size_t> s;
  const auto au = make_au(4, &s);
  FrameHdr h;
  h.pts_us = 200;
  r.w.begin(h, 0, 4);
  r.w.append(au.data(), s[1]);  // slice 0 whole (valid ends on a NAL boundary)
  r.db.notify(0);
  // The "main thread" is busy (OSD compose, SD write): it neither pumps nor
  // services anything. The feed thread alone STARTs the picture.
  const auto t0 = std::chrono::steady_clock::now();
  REQUIRE(r.fake->wait_for('S', 200, 500ms));
  CHECK(std::chrono::steady_clock::now() - t0 < 100ms);
  r.w.append(au.data() + s[1], s[2] - s[1]);
  r.db.notify(0);
  REQUIRE(r.fake->wait_for('A', 200, 500ms));
  r.w.append(au.data() + s[2], au.size() - s[2]);
  r.w.finish(true, maburgs::AuLatMeta{});
  r.db.notify(0);
  REQUIRE(r.fake->wait_for('A', 200, 500ms, /*last=*/true));
  std::this_thread::sleep_for(10ms);
  r.service();
  bool submit_seen = false;
  for (const FeedNote& n : r.notes)
    if (n.kind == FeedNote::Kind::kSubmit && n.meta.pts_us == 200u) submit_seen = true;
  CHECK(submit_seen);
}

TEST(flush_parks_the_feed_until_the_main_thread_resumes) {
  Rig r("flush");
  r.start_and_connect();
  r.publish_whole(100, 0, true);
  REQUIRE(r.fake->wait_for('W', 100, 500ms));
  r.publish_whole(300, 0, true, mabur::framewire::kFlagDiscont);  // flush_before
  for (int i = 0; i < 500 && !r.feed->flush_waiting(); ++i) std::this_thread::sleep_for(1ms);
  REQUIRE(r.feed->flush_waiting());
  const size_t calls = r.fake->count();
  r.publish_whole(400, 0, true);  // lands while parked: it waits too
  std::this_thread::sleep_for(30ms);
  CHECK(r.fake->count() == calls);  // nothing reached the decoder while parked
  CHECK(r.service());
  REQUIRE(r.fake->wait_for('W', 400, 500ms));
  const auto c = r.fake->snapshot();
  int f = -1, w3 = -1, w4 = -1;
  for (int i = 0; i < static_cast<int>(c.size()); ++i) {
    if (c[i].op == 'F') f = i;
    if (c[i].op == 'W' && c[i].pts == 300u) w3 = i;
    if (c[i].op == 'W' && c[i].pts == 400u) w4 = i;
  }
  CHECK(f >= 0);
  CHECK(f < w3);
  CHECK(w3 < w4);
  CHECK(r.feed->parks() == 1);
}

TEST(a_watchdog_park_breaks_a_wedged_submit_and_holds_the_feed) {
  Rig r("wd");
  r.start_and_connect();
  r.fake->block_submit = true;
  r.publish_whole(100, 0, true);  // the feed sits in submit_au (BUFFER_FULL)
  std::this_thread::sleep_for(20ms);
  const auto t0 = std::chrono::steady_clock::now();
  REQUIRE(r.feed->park(1000ms));
  CHECK(std::chrono::steady_clock::now() - t0 < 200ms);
  r.fake->block_submit = false;
  const size_t calls = r.fake->count();
  r.publish_whole(200, 0, true);
  std::this_thread::sleep_for(30ms);
  CHECK(r.fake->count() == calls);  // parked: the new record waits
  r.backend->flush();               // the watchdog's work, feed parked
  r.router->disarm();
  r.feed->resume();
  REQUIRE(r.fake->wait_for('W', 200, 500ms));
}

TEST(drain_finishes_the_streaming_picture_on_its_close) {
  Rig r("drain");
  r.start_and_connect();
  r.publish_whole(100, 0, true);
  REQUIRE(r.fake->wait_for('W', 100, 500ms));
  std::vector<size_t> s;
  const auto au = make_au(4, &s);
  FrameHdr h;
  h.pts_us = 200;
  r.w.begin(h, 0, 4);
  r.w.append(au.data(), s[2]);
  r.db.notify(0);
  REQUIRE(r.fake->wait_for('A', 200, 500ms));  // streaming: S + slice 1
  r.feed->begin_drain();
  std::this_thread::sleep_for(10ms);
  r.w.append(au.data() + s[2], au.size() - s[2]);
  r.w.finish(true, maburgs::AuLatMeta{});
  r.db.notify(0);
  for (int i = 0; i < 500 && !r.feed->finished(); ++i) {
    r.service();
    std::this_thread::sleep_for(1ms);
  }
  REQUIRE(r.feed->finished());
  r.feed->stop_and_join();
  CHECK(r.feed->drained());
  const DrainResult d = r.feed->drain_result();
  CHECK(d.kind == DrainResult::kFinished);
  CHECK(d.pts == 200u);
  CHECK(r.fake->wait_for('A', 200, 0ms, /*last=*/true));
  for (const auto& c : r.fake->snapshot()) CHECK(c.op != 'X');
}

TEST(drain_ends_the_picture_once_when_its_close_never_comes) {
  Rig r("drain2", /*drain_ms=*/30);
  r.start_and_connect();
  r.publish_whole(100, 0, true);
  REQUIRE(r.fake->wait_for('W', 100, 500ms));
  std::vector<size_t> s;
  const auto au = make_au(4, &s);
  FrameHdr h;
  h.pts_us = 200;
  r.w.begin(h, 0, 4);
  r.w.append(au.data(), s[2]);
  r.db.notify(0);
  REQUIRE(r.fake->wait_for('A', 200, 500ms));
  r.feed->begin_drain();
  for (int i = 0; i < 500 && !r.feed->finished(); ++i) std::this_thread::sleep_for(1ms);
  r.feed->stop_and_join();
  CHECK(r.feed->drain_result().kind == DrainResult::kAborted);
  CHECK(r.fake->wait_for('X', 200, 0ms));
}

TEST(periodic_runs_on_the_feed_thread) {
  Rig r("periodic");
  std::atomic<int> n{0};
  std::thread::id id;
  std::mutex m;
  r.feed->set_periodic([&] { std::lock_guard<std::mutex> lk(m); id = std::this_thread::get_id(); ++n; }, 5);
  r.start_and_connect();
  std::this_thread::sleep_for(50ms);
  CHECK(n.load() >= 2);
  std::lock_guard<std::mutex> lk(m);
  CHECK(id != std::this_thread::get_id());
}

// Design §4 rule 1 across back-to-back park gates: maburplay's loop resumes a
// flush park (service_feed) and may call the watchdog's park() right after.
// park() must not return while the resumed feed is still leaving its park --
// on its way out it submits the flush record -- and a served flush must not
// read as waiting again.
TEST(park_right_after_resume_waits_for_the_feed_to_leave_its_park) {
  Rig r("reresume");
  r.start_and_connect();
  r.publish_whole(100, 0, true);
  REQUIRE(r.fake->wait_for('W', 100, 500ms));
  int raced = 0, stale_flush = 0;
  for (uint32_t i = 0; i < 500; ++i) {
    r.publish_whole(1000 + i, 0, true, mabur::framewire::kFlagDiscont);  // flush_before
    for (int k = 0; k < 5000 && !r.feed->flush_waiting(); ++k) std::this_thread::sleep_for(100us);
    REQUIRE(r.feed->flush_waiting());
    r.feeder.on_flush();  // service_feed(), feed parked for the flush
    r.backend->flush();
    r.router->disarm();
    r.feed->resume();
    if (r.feed->flush_waiting()) ++stale_flush;
    REQUIRE(r.feed->park(2000ms));  // the watchdog, at once
    const size_t calls = r.fake->count();
    std::this_thread::sleep_for(300us);  // a feed wrongly still running lands its submit
    if (r.fake->count() != calls) ++raced;
    r.feed->resume();
  }
  CHECK(raced == 0);
  CHECK(stale_flush == 0);
}

TEST(destroying_a_parked_loop_does_not_hang) {
  // The whole Rig lives and dies on another thread so the destruction is
  // bounded and asserted: ~FeedLoop with the feed parked must stop + join.
  auto fut = std::async(std::launch::async, [] {
    Rig r("dtor");
    r.start_and_connect();
    REQUIRE(r.feed->park(1000ms));
  });  // ~Rig -> ~FeedLoop, parked
  const bool ready = fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  CHECK(ready);
  if (!ready) {
    // A hung destructor would also hang ~future (std::async's shared state
    // joins its thread), so leave without unwinding: the FAIL line above is
    // already printed. ctest's timeout stays the backstop.
    std::fflush(stdout);
    std::_Exit(1);
  }
}

MTEST_MAIN
