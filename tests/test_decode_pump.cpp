// DecodePump: the decode-output thread against fake decoders whose poll()
// emits the frames a test queued, plus the feed rig for the submit-note path.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "decode_pump.h"
#include "feed_rig.h"
#include "frame_regulator.h"
#include "lat_tracker.h"
#include "poll_wait.h"
#include "wake_queue.h"

using maburplay::DecodePump;
using maburplay::DmaFrame;
using maburplay::FrameRegulator;
using maburplay::HandedFrame;
using maburplay::LatTracker;
using maburplay::WakeQueue;

namespace {

DmaFrame frame(uint32_t pts, int w = 1920, int h = 1080) {
  DmaFrame f;
  f.pts_us = pts;
  f.width = w;
  f.height = h;
  f.opaque = reinterpret_cast<void*>(static_cast<uintptr_t>(pts | 1u));
  return f;
}

// Polls cond until it holds or the deadline passes.
bool wait_until(const std::function<bool()>& cond, std::chrono::milliseconds lim = 1000ms) {
  const auto end = std::chrono::steady_clock::now() + lim;
  while (!cond()) {
    if (std::chrono::steady_clock::now() >= end) return cond();
    std::this_thread::sleep_for(200us);
  }
  return true;
}

// The decoder's output side: poll() hands every queued frame to the sink
// (on the pump thread), release_frame() records what came back.
struct PumpFake : maburplay::VideoBackend {
  mutable std::mutex mu;
  std::condition_variable cv;
  std::deque<DmaFrame> pending;
  std::vector<uint32_t> released;
  std::atomic<uint64_t> polls{0};
  FrameSink sink;

  bool init(const maburplay::BackendCfg&, FrameSink s) override {
    sink = std::move(s);
    return true;
  }
  void submit_au(const uint8_t*, size_t, uint32_t) override {}
  void flush() override {}
  void release_frame(const DmaFrame& f) override {
    std::lock_guard<std::mutex> lk(mu);
    released.push_back(f.pts_us);
  }
  void poll() override {
    polls.fetch_add(1);
    emit_pending_();
  }
  void emit(const DmaFrame& f) {
    std::lock_guard<std::mutex> lk(mu);
    pending.push_back(f);
    cv.notify_all();
  }
  std::vector<uint32_t> released_snapshot() const {
    std::lock_guard<std::mutex> lk(mu);
    return released;
  }

 protected:
  // true = at least one frame went to the sink.
  bool emit_pending_() {
    std::deque<DmaFrame> got;
    {
      std::lock_guard<std::mutex> lk(mu);
      got.swap(pending);
    }
    for (const DmaFrame& f : got) sink(f);
    return !got.empty();
  }
};

// A decoder whose poll() blocks up to its PollWait timeout when nothing is
// ready, like MPP with MPP_SET_OUTPUT_TIMEOUT.
struct BlockingFake : PumpFake, maburplay::PollWait {
  std::atomic<int> timeout_ms{0};
  bool refuse = false;  // like MppBackend before init()
  bool set_poll_timeout_ms(int ms) override {
    if (refuse) return false;
    timeout_ms.store(ms);
    return true;
  }
  void poll() override {
    polls.fetch_add(1);
    if (emit_pending_()) return;
    std::unique_lock<std::mutex> lk(mu);
    cv.wait_for(lk, std::chrono::milliseconds(timeout_ms.load()), [this] { return !pending.empty(); });
    lk.unlock();
    emit_pending_();
  }
};

// Each test declares its fakes before its Pump, so ~DecodePump stops the
// thread before the backend it polls goes away.
struct Pump {
  LatTracker lat;
  FrameRegulator reg;
  WakeQueue<HandedFrame> out;
  WakeQueue<DmaFrame> rel;
  DecodePump pump;
  Pump() : reg(0), pump(lat, reg, out, rel) {}
  Pump(int regulate_ms, bool vsync, int lead, int chain, int cap)
      : reg(regulate_ms, vsync, lead, chain, cap), pump(lat, reg, out, rel) {}
};

// LatTracker's PtsAnchor warm-up (test_lat_tracker.cpp's warm()): 33
// submissions on a constant arrival offset kC, so a later frame on the same
// offset joins the completed window.
constexpr uint32_t kFrameStep = 16'684;
constexpr uint64_t kC = 100'000;
void warm_lat(LatTracker& lat) {
  for (int i = 0; i < 33; ++i) {
    maburgs::AuRecordMeta m;
    m.pts_us = static_cast<uint32_t>(i * kFrameStep);
    m.t_first_us = m.pts_us + kC;
    m.t_complete_us = m.t_first_us + 1000;
    lat.on_submit(m, 0);
  }
}

// FrameRegulator servo warm-up (test_frame_regulator.cpp's warm/flips_until).
constexpr uint64_t kT0 = 5'000'000;
constexpr uint64_t kVb = 16'667;
void warm_reg(FrameRegulator& reg) {
  for (int i = 0; i < 8; ++i) reg.on_flip(kT0 + i * kVb, true);
}
void flips_until(FrameRegulator& reg, uint64_t* next, uint64_t t) {
  while (kT0 + *next * kVb <= t) reg.on_flip(kT0 + (*next)++ * kVb, true);
}

// Offers 20 clean frames on the capture grid, then pts 20*kFrameStep 30 ms
// late; returns the allowance right after that late frame. A late frame the
// regulator was told is incomplete stays out of the lateness window (the
// allowance stays at the clean spread + 1 ms); otherwise it alone widens the
// allowance to the cap.
double allowance_after_late_frame(FrameRegulator& reg) {
  warm_reg(reg);
  FrameRegulator::Displaced disp;
  DmaFrame out;
  uint64_t next_flip = 8;
  for (uint32_t i = 0; i <= 20; ++i) {
    const uint32_t pts = i * kFrameStep;
    const uint64_t t = kT0 + 8 * kVb + uint64_t{i} * kFrameStep + (i == 20 ? 30'000 : 0);
    flips_until(reg, &next_flip, t);
    reg.offer(frame(pts), t, t, &disp);
    while (reg.release_due(t, &out)) {}
  }
  return reg.allowance_ms();
}

}  // namespace

TEST(a_decoded_frame_is_stamped_and_handed_over_then_primed) {
  PumpFake fake;
  Pump p;
  std::mutex om;
  std::vector<std::string> order;
  p.pump.set_prime([&](const DmaFrame&) {
    std::lock_guard<std::mutex> lk(om);
    if (p.out.size() == 1) order.push_back("handover");
    order.push_back("prime");
  });
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  CHECK(!p.pump.have_frame());
  p.pump.start();
  const uint64_t t0 = mono_us();
  fake.emit(frame(100, 1280, 720));
  REQUIRE(wait_until([&] { std::lock_guard<std::mutex> lk(om); return order.size() >= 2; }));
  p.pump.stop_and_join();
  std::vector<HandedFrame> h;
  CHECK(p.out.drain(&h) == 1);
  REQUIRE(h.size() == 1);
  CHECK(h[0].f.pts_us == 100);
  CHECK(h[0].t_decoded_us > 0);
  CHECK(h[0].t_decoded_us >= t0);
  CHECK(order == (std::vector<std::string>{"handover", "prime"}));
  CHECK(p.pump.frames() == 1);
  CHECK(p.pump.have_frame());
  CHECK(p.pump.last_pts() == 100u);
  CHECK(p.pump.last_frame_us() == h[0].t_decoded_us);
  CHECK(p.pump.video_w() == 1280);
  CHECK(p.pump.video_h() == 720);
  CHECK(fake.released_snapshot().empty());  // the pump never releases on its own
}

TEST(every_decoded_frames_pts_reaches_the_pts_sink_after_the_handover) {
  PumpFake fake;
  Pump p;
  std::mutex om;
  std::vector<uint32_t> pts;
  bool handed_first = true;
  p.pump.set_pts_sink([&](uint32_t v) {
    std::lock_guard<std::mutex> lk(om);
    if (p.out.size() < pts.size() + 1) handed_first = false;
    pts.push_back(v);
  });
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  p.pump.start();
  fake.emit(frame(100));
  fake.emit(frame(16767));
  REQUIRE(wait_until([&] { std::lock_guard<std::mutex> lk(om); return pts.size() >= 2; }));
  p.pump.stop_and_join();
  CHECK(pts == (std::vector<uint32_t>{100, 16767}));
  CHECK(handed_first);  // the sink never delays the hand-over
}

TEST(releases_queued_by_another_thread_reach_the_backend) {
  PumpFake fake;
  Pump p;
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  p.pump.start();
  std::thread other([&] { p.rel.push(frame(5)); });
  other.join();
  CHECK(wait_until([&] { return fake.released_snapshot().size() == 1; }));
  CHECK(fake.released_snapshot() == std::vector<uint32_t>{5});
  CHECK(p.rel.size() == 0);
}

TEST(the_source_rate_is_measured_from_decoded_pts) {
  PumpFake fake;
  Pump p;
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  p.pump.start();
  CHECK(p.pump.source_hz() == 0.0);
  for (uint32_t i = 0; i < 12; ++i) fake.emit(frame(i * kFrameStep));
  REQUIRE(wait_until([&] { return p.pump.frames() == 12; }));
  std::this_thread::sleep_for(1010ms);  // SourceRate's window is 1 s of mono time
  fake.emit(frame(12 * kFrameStep));
  REQUIRE(wait_until([&] { return p.pump.frames() == 13; }));
  const double hz = p.pump.source_hz();
  CHECK(hz > 59.9 && hz < 60.0);
}

TEST(a_submit_note_landing_before_its_frame_gives_the_frame_a_row) {
  Rig r("pump_row");
  PumpFake fake;
  Pump p;
  warm_lat(p.lat);
  p.pump.set_feed(r.fl);
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  r.start_and_connect();
  p.pump.start();
  const uint32_t pts = 33 * kFrameStep;  // the warm-up's next step
  maburgs::AuLatMeta m;
  m.t_first_us = pts + kC;
  m.t_complete_us = m.t_first_us + 1000;
  r.publish_whole(pts, 0, true, 0, m);  // complete sid-0: arms, note, submit
  REQUIRE(r.fake->wait_for('W', pts, 500ms));  // the note precedes the submit
  fake.emit(frame(pts));
  REQUIRE(wait_until([&] { return p.out.size() == 1; }));
  const uint64_t t = mono_us();
  p.lat.on_present(pts, t);
  p.lat.on_flip(pts, t + 1000, true);
  const LatTracker::Line L = p.lat.flush_line();
  CHECK(L.n == 1);
  REQUIRE(L.rows.size() == 1);
  CHECK(L.rows[0].pts_us == pts);
  std::vector<HandedFrame> h;
  p.out.drain(&h);
  REQUIRE(h.size() == 1);
  CHECK(L.rows[0].t_decoded_us == h[0].t_decoded_us);  // on_decoded = the hand-over stamp
  CHECK(L.rows[0].t_submit_us > 0);                     // on_submit took the note's own stamp
}

TEST(an_incomplete_note_reaches_the_regulator_before_its_frame) {
  // Control: the late frame, untold, widens the allowance to the cap.
  {
    FrameRegulator control(12, true, 3, 3, 16);
    CHECK(allowance_after_late_frame(control) > 10.0);
  }
  Rig r("pump_incomplete");
  PumpFake fake;
  Pump p(12, true, 3, 3, 16);
  p.pump.set_feed(r.fl);
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  r.start_and_connect();
  p.pump.start();
  const uint32_t late = 20 * kFrameStep;
  r.publish_whole(19 * kFrameStep, 0, true);  // complete sid-0: arms
  REQUIRE(r.fake->wait_for('W', 19 * kFrameStep, 500ms));
  maburgs::AuLatMeta salvaged;
  salvaged.slice.salvaged = true;
  r.publish_whole(late, 1, false, 0, salvaged);  // decodable, not complete
  REQUIRE(r.fake->wait_for('W', late, 500ms));
  fake.emit(frame(late));
  REQUIRE(wait_until([&] { return p.out.size() == 1; }));
  CHECK(allowance_after_late_frame(p.reg) < 3.0);
}

TEST(submit_notes_are_drained_every_round_with_no_frame_decoded) {
  // The decoder emits nothing (a cold start before the first picture, or
  // a wedged decoder): the feed's submit notes must not pile up waiting
  // for a frame -- the pump drains them every poll round.
  Rig r("pump_nofr");
  PumpFake fake;
  Pump p;
  p.pump.set_feed(r.fl);
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  r.start_and_connect();
  p.pump.start();
  for (uint32_t i = 0; i < 8; ++i) {
    r.publish_whole(i * kFrameStep, 0, true);
    REQUIRE(r.fake->wait_for('W', i * kFrameStep, 500ms));
  }
  std::this_thread::sleep_for(100ms);  // a hundred idle rounds or more
  std::vector<maburplay::FeedNote> left;
  r.fl->take_submit_notes(&left);
  CHECK(left.empty());
  CHECK(p.pump.frames() == 0);
  CHECK(p.out.size() == 0);
}

TEST(park_drains_pending_releases_into_the_backend_they_belong_to) {
  PumpFake old_, new_;
  Pump p;
  old_.init({}, p.pump.sink());
  new_.init({}, p.pump.sink());
  p.pump.set_backend(&old_);
  p.pump.start();
  p.rel.push(frame(7));  // a frame of the OLD decoder
  REQUIRE(p.pump.park(1000ms));
  CHECK(old_.released_snapshot() == std::vector<uint32_t>{7});  // drained before parked
  p.pump.set_backend(&new_);
  p.pump.resume();
  new_.emit(frame(8));
  REQUIRE(wait_until([&] { return p.pump.frames() == 1; }));  // runs on the new decoder
  CHECK(old_.released_snapshot().size() == 1);
  CHECK(new_.released_snapshot().empty());
  CHECK(p.pump.releases_discarded() == 0);
}

TEST(set_backend_discards_a_release_queued_after_park) {
  PumpFake old_, new_;
  Pump p;
  old_.init({}, p.pump.sink());
  new_.init({}, p.pump.sink());
  p.pump.set_backend(&old_);
  p.pump.start();
  REQUIRE(p.pump.park(1000ms));
  p.rel.push(frame(9));  // queued while parked: the old decoder's frame
  p.pump.set_backend(&new_);
  CHECK(p.rel.size() == 0);
  CHECK(p.pump.releases_discarded() == 1);
  p.pump.resume();
  new_.emit(frame(10));
  // A frame handed over after resume: the loop has drained releases since.
  REQUIRE(wait_until([&] { return p.pump.frames() == 1; }));
  CHECK(old_.released_snapshot().empty());
  CHECK(new_.released_snapshot().empty());
}

TEST(set_backend_discards_even_for_the_same_backend_address) {
  // A watchdog recreate: the old decoder destroyed, the new one allocated at
  // the same address. What was queued since the park belongs to the old one.
  PumpFake fake;
  Pump p;
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  p.pump.start();
  REQUIRE(p.pump.park(1000ms));
  p.rel.push(frame(11));
  p.pump.set_backend(&fake);
  CHECK(p.rel.size() == 0);
  CHECK(p.pump.releases_discarded() == 1);
  p.pump.resume();
  fake.emit(frame(12));
  REQUIRE(wait_until([&] { return p.pump.frames() == 1; }));  // releases drained since
  CHECK(fake.released_snapshot().empty());
}

TEST(a_refused_poll_timeout_falls_back_to_the_idle_wait) {
  BlockingFake fake;
  fake.refuse = true;  // poll() then returns at once, like an uninitialised MppBackend
  Pump p;
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  p.pump.start();
  std::this_thread::sleep_for(50ms);
  p.pump.stop_and_join();
  const uint64_t n = fake.polls.load();
  CHECK(n >= 1);
  CHECK(n <= 500);  // ~1 per ms when idle; a spin would be orders more
}

TEST(a_backend_without_poll_wait_is_not_spun) {
  PumpFake fake;  // poll() returns at once with nothing
  Pump p;
  fake.init({}, p.pump.sink());
  p.pump.set_backend(&fake);
  p.pump.start();
  std::this_thread::sleep_for(50ms);
  p.pump.stop_and_join();
  const uint64_t n = fake.polls.load();
  CHECK(n >= 1);    // it does poll
  CHECK(n <= 500);  // ~1 per ms when idle; a spin would be orders more
}

TEST(park_completes_within_the_poll_timeout_and_stop_joins) {
  auto fut = std::async(std::launch::async, [] {
    BlockingFake fake;
    Pump p;
    fake.init({}, p.pump.sink());
    p.pump.set_backend(&fake);
    CHECK(fake.timeout_ms.load() == 4);
    p.pump.start();
    REQUIRE(wait_until([&] { return fake.polls.load() >= 3; }));
    const auto t0 = std::chrono::steady_clock::now();
    CHECK(p.pump.park(1000ms));
    // Bounded by the poll timeout, not by park's limit (slack for a loaded host).
    CHECK(std::chrono::steady_clock::now() - t0 < 200ms);
    const uint64_t polls = fake.polls.load();
    std::this_thread::sleep_for(20ms);
    CHECK(fake.polls.load() == polls);  // parked: outside every backend call
    p.pump.resume();
    REQUIRE(wait_until([&] { return fake.polls.load() > polls; }));
    p.pump.stop_and_join();
  });
  const bool ready = fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  CHECK(ready);
  if (!ready) {
    std::fflush(stdout);
    std::_Exit(1);
  }
}

TEST(destroying_a_parked_pump_does_not_hang) {
  auto fut = std::async(std::launch::async, [] {
    BlockingFake fake;
    Pump p;
    fake.init({}, p.pump.sink());
    p.pump.set_backend(&fake);
    p.pump.start();
    REQUIRE(p.pump.park(1000ms));
  });  // ~DecodePump, parked
  const bool ready = fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  CHECK(ready);
  if (!ready) {
    std::fflush(stdout);
    std::_Exit(1);
  }
}

MTEST_MAIN
