#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "au_doorbell.h"
#include "au_ring.h"
#include "mtest.h"
#include "ring_client.h"

using mabur::framewire::FrameHdr;
using mabur::framewire::kFlagDiscont;
using mabur::framewire::kFlagIdr;
using maburplay::AuEvent;
using maburplay::RingClient;

namespace {

std::string tmp_path(const char* tag) {
  return "/tmp/test_ring_client_" + std::string(tag) + "_" + std::to_string(getpid());
}

std::vector<uint8_t> au_bytes(size_t n, uint8_t seed) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(seed + i * 7);
  return v;
}

// Publishes one AU. 2-stream space (spec 2026-08-29-airtime-balance-uep):
// sid 0 = base, sid 1 = enhance. sid==1 records rely on
// maburgs::AuRingWriter's own contract: finish(complete, lat) ORs
// kRecFlagComplete into flags iff complete.
void publish(maburgs::AuRingWriter& w, uint8_t sid, bool complete, size_t bytes,
             uint8_t seed, uint8_t extra_flags = 0, uint32_t pts_us = 0) {
  FrameHdr h;
  h.pts_us = pts_us;
  h.flags = extra_flags;
  const auto au = au_bytes(bytes, seed);
  w.begin(h, sid);
  w.append(au.data(), au.size());
  w.finish(complete, maburgs::AuLatMeta{});
}

// Collects delivered events into a vector via the Sink callback.
struct Collector {
  std::vector<AuEvent> events;
  RingClient::Sink sink() {
    return [this](AuEvent&& ev) { events.push_back(std::move(ev)); };
  }
};

}  // namespace

TEST(incomplete_sid1_dropped_whole_and_counted) {
  const std::string ring = tmp_path("ring1");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  publish(w, /*sid=*/1, /*complete=*/false, 100, 1);

  Collector c;
  RingClient rc({ring, tmp_path("nosock1")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) == 0);
  CHECK(c.events.empty());
  CHECK(rc.dropped_enhance_incomplete() == 1);
  CHECK(rc.delivered() == 0);
  CHECK(rc.truncated_base() == 0);
  unlink(ring.c_str());
}

TEST(incomplete_sid0_delivered_and_counted_truncated) {
  const std::string ring = tmp_path("ring2");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  publish(w, /*sid=*/0, /*complete=*/false, 120, 2);

  Collector c;
  RingClient rc({ring, tmp_path("nosock2")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) == 1);
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].meta.sid == 0);
  CHECK((c.events[0].meta.flags & maburgs::kRecFlagComplete) == 0);
  CHECK(c.events[0].au == au_bytes(120, 2));
  CHECK(rc.delivered() == 1);
  CHECK(rc.truncated_base() == 1);
  CHECK(rc.dropped_enhance_incomplete() == 0);
  unlink(ring.c_str());
}

TEST(discont_flag_sets_flush_before_once) {
  const std::string ring = tmp_path("ring3");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  publish(w, /*sid=*/0, /*complete=*/true, 50, 3, kFlagDiscont);
  publish(w, /*sid=*/0, /*complete=*/true, 60, 4, /*extra_flags=*/0);

  Collector c;
  RingClient rc({ring, tmp_path("nosock3")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) == 2);
  REQUIRE(c.events.size() == 2);
  CHECK((c.events[0].meta.flags & kFlagDiscont) != 0);
  CHECK(c.events[0].flush_before == true);
  CHECK(c.events[1].flush_before == false);
  unlink(ring.c_str());
}

TEST(writer_restart_sets_flush_before_and_counts_resync) {
  const std::string ring = tmp_path("ring4");
  {
    maburgs::AuRingWriter w1;
    REQUIRE(w1.open(ring, {4096, 4}));
    for (int i = 0; i < 3; ++i) publish(w1, 0, true, 40, static_cast<uint8_t>(i));
  }  // w1 closed

  Collector c;
  RingClient rc({ring, tmp_path("nosock4")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) >= 1);  // drains whatever the first writer's session retained
  const size_t before = c.events.size();
  CHECK(rc.resyncs() == 0);

  maburgs::AuRingWriter w2;
  REQUIRE(w2.open(ring, {4096, 4}));  // epoch change: new writer, same geometry
  publish(w2, 1, true, 30, 9);  // distinct sid confirms this record, not a leftover, arrived

  size_t got = 0;
  for (int attempt = 0; attempt < 50 && got == 0; ++attempt) got = rc.pump(5);
  REQUIRE(got >= 1);
  REQUIRE(c.events.size() > before);
  CHECK(rc.resyncs() >= 1);
  CHECK(c.events[before].flush_before == true);
  CHECK(c.events[before].meta.sid == 1);
  unlink(ring.c_str());
}

TEST(oneshot_drain_applies_policy_on_quiescent_ring) {
  const std::string ring = tmp_path("ring5");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  publish(w, 0, true, 30, 1);                 // delivered, complete
  publish(w, 0, false, 30, 2);                // delivered, truncated_base++
  publish(w, 1, true, 30, 3);                 // delivered (complete enhance)
  publish(w, 1, false, 30, 4);                // dropped, dropped_enhance_incomplete++
  publish(w, 0, true, 30, 5, kFlagDiscont);   // delivered, itself carries flush_before

  Collector c;
  RingClient rc({ring, tmp_path("nosock5")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.oneshot_drain() == true);
  CHECK(!rc.dead());
  // 5 published, 1 dropped whole (incomplete sid1) -> 4 delivered.
  CHECK(rc.delivered() == 4);
  CHECK(rc.truncated_base() == 1);
  CHECK(rc.dropped_enhance_incomplete() == 1);
  REQUIRE(c.events.size() == 4);
  CHECK(c.events[0].meta.sid == 0);
  CHECK(c.events[1].meta.sid == 0);
  CHECK((c.events[1].meta.flags & maburgs::kRecFlagComplete) == 0);
  CHECK(c.events[2].meta.sid == 1);
  CHECK(c.events[3].meta.sid == 0);
  CHECK((c.events[3].meta.flags & kFlagDiscont) != 0);
  // The kFlagDiscont AU itself is the rebase point: it carries
  // flush_before, not whatever AU would come after it.
  CHECK(c.events[3].flush_before == true);
  unlink(ring.c_str());
}

// -------- doorbell-less operation: every behavior above still holds with
// the socket path pointing at a file that was never created. -------------
TEST(doorbell_less_pump_uses_timeout_path) {
  const std::string ring = tmp_path("ring6");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  publish(w, 1, false, 20, 1);   // dropped
  publish(w, 0, false, 20, 2);   // delivered, truncated
  publish(w, 0, true, 20, 3, kFlagDiscont);

  const std::string nosock = tmp_path("nosock6");
  unlink(nosock.c_str());  // guarantee it never existed

  Collector c;
  RingClient rc({ring, nosock}, c.sink());
  REQUIRE(rc.open());
  const auto t0 = std::chrono::steady_clock::now();
  CHECK(rc.pump(10) == 2);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  // Plain-sleep timeout path: pump should not return near-instantly, and
  // should not hang either.
  CHECK(elapsed >= std::chrono::milliseconds(8));
  CHECK(elapsed < std::chrono::milliseconds(500));

  REQUIRE(c.events.size() == 2);
  CHECK(rc.dropped_enhance_incomplete() == 1);
  CHECK(rc.truncated_base() == 1);
  CHECK(c.events[1].flush_before == true);
  CHECK(!rc.dead());
  unlink(ring.c_str());
}

// -------- doorbell client wire behavior (Step 3 implementation detail,
// tested here since ring_client owns the client-side socket logic). ------
TEST(doorbell_hello_validates_and_notify_wakes_pump_early) {
  const std::string ring = tmp_path("ring7");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));

  const std::string sock = tmp_path("sock7");
  maburgs::AuDoorbell db;
  REQUIRE(db.open(sock, {4096, 8}));

  Collector c;
  RingClient rc({ring, sock}, c.sink());
  REQUIRE(rc.open());

  // First pump: connects, but the server hasn't accepted/sent hello yet
  // (that only happens inside AuDoorbell::poll()), so this call falls
  // through the not-yet-ready path and just times out quickly.
  rc.pump(5);
  db.poll();  // server accepts the connection, sends hello
  // Give the hello a moment to land, then let the client consume it.
  rc.pump(5);

  publish(w, 0, true, 40, 7);
  db.notify(0);
  // A large per-call timeout: if the notify wakeup were NOT working, this
  // single call would have to sit out the full poll(2) timeout before
  // drain_ring_() ever runs (state lives in the ring, so it would still
  // eventually succeed — just slowly). A tight elapsed bound below is what
  // actually proves the doorbell shortened the wait.
  const auto t0 = std::chrono::steady_clock::now();
  const size_t got = rc.pump(2000);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  REQUIRE(got == 1);
  CHECK(elapsed < std::chrono::milliseconds(300));
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].meta.sid == 0);
  unlink(ring.c_str());
  unlink(sock.c_str());
}

TEST(doorbell_hello_mismatch_closes_without_breaking_ring_reads) {
  const std::string ring = tmp_path("ring8");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));

  // Fake "server": a raw listening seqpacket socket that sends a
  // deliberately wrong hello (mismatched geometry) instead of a real
  // AuDoorbell, to exercise the client's validation path.
  const std::string sock = tmp_path("sock8");
  unlink(sock.c_str());
  int listen_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
  REQUIRE(listen_fd >= 0);
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  std::strncpy(a.sun_path, sock.c_str(), sizeof(a.sun_path) - 1);
  REQUIRE(bind(listen_fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
  REQUIRE(listen(listen_fd, 1) == 0);

  Collector c;
  RingClient rc({ring, sock}, c.sink());
  REQUIRE(rc.open());
  rc.pump(5);  // client connects; nothing in the ring yet, nothing to accept yet

  int client_fd = accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK);
  REQUIRE(client_fd >= 0);
  uint8_t bad_hello[16];
  uint32_t magic = maburgs::kAuRingMagic, ver = maburgs::kAuRingVersion;
  uint32_t sb = 9999, sc = 9999;  // wrong geometry
  std::memcpy(bad_hello, &magic, 4);
  std::memcpy(bad_hello + 4, &ver, 4);
  std::memcpy(bad_hello + 8, &sb, 4);
  std::memcpy(bad_hello + 12, &sc, 4);
  REQUIRE(send(client_fd, bad_hello, sizeof(bad_hello), 0) == 16);

  // The client should consume and reject the bad hello (closing its side
  // of the socket) without disturbing ring reads at all — doorbell is
  // wakeup-only. Publish AFTER the mismatch exchange so this call's
  // delivery count unambiguously proves the ring path still works.
  publish(w, 0, true, 25, 5);
  CHECK(rc.pump(5) == 1);
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].meta.sid == 0);
  CHECK(!rc.dead());

  close(client_fd);
  close(listen_fd);
  unlink(ring.c_str());
  unlink(sock.c_str());
}

// -------- flush-flag survival: the flag must never be silently eaten by
// an AU that policy drops between the discontinuity and the next AU that
// actually reaches the sink. --------------------------------------------
TEST(flush_flag_survives_intervening_dropped_enhance_au) {
  const std::string ring = tmp_path("ring9");
  auto w1 = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w1->open(ring, {4096, 8}));
  publish(*w1, 0, true, 30, 1);                // (a) clean base
  publish(*w1, 0, true, 30, 2, kFlagDiscont);  // (b) discont base

  Collector c;
  RingClient rc({ring, tmp_path("nosock9")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) == 2);
  REQUIRE(c.events.size() == 2);
  CHECK(c.events[0].flush_before == false);
  // (b) itself carries the flag and consumes it right there.
  CHECK((c.events[1].meta.flags & kFlagDiscont) != 0);
  CHECK(c.events[1].flush_before == true);

  // Re-signal via a writer restart (epoch change): the reader reports
  // kResync with NO AU delivered, so the flag has nothing to attach to
  // yet. The very next record is an incomplete sid-1 (enhance) AU that
  // policy drops whole. The flag must survive THAT drop too, landing on
  // the first AU that actually reaches the sink afterward.
  w1.reset();
  auto w2 = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w2->open(ring, {4096, 8}));
  publish(*w2, 1, false, 25, 9);   // dropped whole; must NOT eat the flag
  publish(*w2, 0, true, 25, 10);   // first AU to actually reach the sink

  size_t got = 0;
  for (int attempt = 0; attempt < 50 && got == 0; ++attempt) got = rc.pump(5);
  REQUIRE(got >= 1);
  CHECK(rc.resyncs() >= 1);
  CHECK(rc.dropped_enhance_incomplete() == 1);
  REQUIRE(c.events.size() == 3);  // (a), (b), and the post-restart clean base
  CHECK(c.events[2].meta.sid == 0);
  CHECK(c.events[2].au == au_bytes(25, 10));
  CHECK(c.events[2].flush_before == true);
  for (const auto& ev : c.events) CHECK(ev.meta.sid != 1);  // dropped AU never sank
  unlink(ring.c_str());
}

TEST(incomplete_sid1_with_discont_flag_still_flushes_next_base) {
  const std::string ring = tmp_path("ring10");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  publish(w, 0, true, 20, 1);                        // clean base
  publish(w, 1, false, 20, 2, kFlagDiscont);          // drop + discont, SAME record
  publish(w, 0, true, 20, 3);                         // clean base after

  Collector c;
  RingClient rc({ring, tmp_path("nosock10")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) == 2);
  REQUIRE(c.events.size() == 2);
  CHECK(c.events[0].meta.sid == 0);
  CHECK(c.events[0].flush_before == false);
  CHECK(c.events[1].meta.sid == 0);
  CHECK(c.events[1].au == au_bytes(20, 3));
  CHECK(c.events[1].flush_before == true);
  CHECK(rc.dropped_enhance_incomplete() == 1);
  CHECK(rc.truncated_base() == 0);  // the drop is on the enhance path, not base-truncation
  for (const auto& ev : c.events) CHECK(ev.meta.sid != 1);
  unlink(ring.c_str());
}

// The GS boot race (2026-09-08): S97maburplay starts within milliseconds of
// S96maburgs, so maburplay can reach the ring open before maburgs has created
// the ring. A waiting open lets the player sit on its splash until the ring
// shows up instead of exiting.
TEST(open_waits_for_a_ring_created_after_the_call) {
  const std::string ring = tmp_path("ring_late");
  unlink(ring.c_str());

  std::unique_ptr<maburgs::AuRingWriter> w;
  std::thread writer([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    w = std::make_unique<maburgs::AuRingWriter>();
    REQUIRE(w->open(ring, {4096, 8}));
    publish(*w, 0, true, 20, 1);
  });

  Collector c;
  RingClient rc({ring, tmp_path("nosock_late")}, c.sink());
  const bool ok = rc.open(5000);
  writer.join();

  CHECK(ok);
  CHECK(rc.pump(50) == 1);
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].au == au_bytes(20, 1));
  unlink(ring.c_str());
}

// --oneshot (host e2e) must still fail fast on a missing ring rather than
// hang: the ring is the fixture there, and its absence is a real failure.
TEST(open_without_wait_fails_immediately_on_missing_ring) {
  const std::string ring = tmp_path("ring_absent");
  unlink(ring.c_str());

  Collector c;
  RingClient rc({ring, tmp_path("nosock_absent")}, c.sink());
  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = rc.open(0);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  CHECK(!ok);
  CHECK(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() < 200);
}

// A bounded wait that never sees a ring gives up and says so.
TEST(open_with_bounded_wait_times_out_on_missing_ring) {
  const std::string ring = tmp_path("ring_never");
  unlink(ring.c_str());

  Collector c;
  RingClient rc({ring, tmp_path("nosock_never")}, c.sink());
  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = rc.open(300);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  CHECK(!ok);
  const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  CHECK(ms >= 300);
  CHECK(ms < 3000);
}

// Slice salvage (spec 2026-10-10-h265-slices §5.5): a salvaged AU (sid 0 or
// sid 1) is a legal, gap-free picture (kept slices + skip-slice fills) --
// deliver it like any other AU, just counted separately from a plain
// truncation.
TEST(salvaged_sid1_and_sid0_are_delivered_and_counted) {
  const std::string ring = tmp_path("ring_salv");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  auto publish_salvaged = [&](uint8_t sid, uint8_t seed) {
    FrameHdr h;
    const auto au = au_bytes(100, seed);
    w.begin(h, sid);
    w.append(au.data(), au.size());
    maburgs::AuLatMeta lat;
    lat.slice.salvaged = true;
    w.finish(false, lat);
  };
  publish_salvaged(1, 1);
  publish_salvaged(0, 2);

  Collector c;
  RingClient rc({ring, tmp_path("nosock_salv")}, c.sink());
  REQUIRE(rc.open());
  CHECK(rc.pump(5) == 2);
  CHECK(c.events.size() == 2);
  CHECK(rc.dropped_enhance_incomplete() == 0);
  CHECK(rc.salvaged_enhance() == 1);
  CHECK(rc.salvaged_base() == 1);
  CHECK(rc.truncated_base() == 0);
  unlink(ring.c_str());
}

// --- Ring v4: open records (spec 2026-10-10-h265-slices §6.2-6.3) ---

using maburplay::AuEventKind;

TEST(open_record_surfaces_open_grow_then_close) {
  const std::string ring = tmp_path("open1");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  Collector c;
  RingClient rc({ring, tmp_path("nosock_open1")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.pts_us = 111;
  const auto au = au_bytes(300, 9);
  w.begin(h, 0, 4);
  w.append(au.data(), 100);
  CHECK(rc.pump(0) == 0);                        // pump counts closed records only
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].kind == AuEventKind::kOpen);
  CHECK(c.events[0].meta.nslices == 4);
  CHECK(c.events[0].meta.pts_us == 111u);
  CHECK(c.events[0].au == std::vector<uint8_t>(au.begin(), au.begin() + 100));
  CHECK(!c.events[0].flush_before);
  w.append(au.data() + 100, 150);
  rc.pump(0);
  REQUIRE(c.events.size() == 2);
  CHECK(c.events[1].kind == AuEventKind::kGrow);
  CHECK(c.events[1].au == std::vector<uint8_t>(au.begin(), au.begin() + 250));
  rc.pump(0);
  CHECK(c.events.size() == 2);                   // nothing new, no event
  w.append(au.data() + 250, 50);
  w.finish(true, maburgs::AuLatMeta{});
  CHECK(rc.pump(0) == 1);
  REQUIRE(c.events.size() == 3);
  CHECK(c.events[2].kind == AuEventKind::kClose);
  CHECK(!c.events[2].aborted);
  CHECK(c.events[2].au == au);
  CHECK(c.events[2].meta.nslices == 4);
  CHECK(rc.opened() == 1);
  CHECK(rc.open_aborted() == 0);
  CHECK(rc.delivered() == 1);
  unlink(ring.c_str());
}

TEST(writer_restart_mid_open_emits_aborted_close) {   // Review Focus 4
  const std::string ring = tmp_path("open_restart");
  Collector c;
  auto w1 = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w1->open(ring, {4096, 8}));
  RingClient rc({ring, tmp_path("nosock_restart")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.pts_us = 7;
  const auto au = au_bytes(200, 3);
  w1->begin(h, 0, 4);
  w1->append(au.data(), au.size());
  rc.pump(0);
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].kind == AuEventKind::kOpen);
  w1.reset();                                    // maburgs dies mid-AU
  maburgs::AuRingWriter w2;
  REQUIRE(w2.open(ring, {4096, 8}));             // restart: new epoch
  publish(w2, 0, true, 64, 5, 0, 99);
  CHECK(rc.pump(0) == 1);
  REQUIRE(c.events.size() == 3);
  CHECK(c.events[1].kind == AuEventKind::kClose);
  CHECK(c.events[1].aborted);                    // the open one ends first ...
  CHECK(c.events[1].meta.pts_us == 7u);
  CHECK(c.events[1].au.empty());
  CHECK(c.events[2].kind == AuEventKind::kClose);
  CHECK(!c.events[2].aborted);                   // ... then the new writer's record
  CHECK(c.events[2].meta.pts_us == 99u);
  CHECK(c.events[2].flush_before);
  CHECK(rc.open_aborted() == 1);
  unlink(ring.c_str());
}

TEST(writer_restart_with_new_au_open_never_grows_the_old_view) {
  const std::string ring = tmp_path("open_restart2");
  Collector c;
  auto w1 = std::make_unique<maburgs::AuRingWriter>();
  REQUIRE(w1->open(ring, {4096, 8}));
  RingClient rc({ring, tmp_path("nosock_restart2")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.pts_us = 7;
  const auto au1 = au_bytes(200, 3);
  w1->begin(h, 0, 4);
  w1->append(au1.data(), au1.size());
  rc.pump(0);
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].kind == AuEventKind::kOpen);
  CHECK(c.events[0].meta.pts_us == 7u);

  w1.reset();                                    // maburgs dies mid-AU
  maburgs::AuRingWriter w2;
  REQUIRE(w2.open(ring, {4096, 8}));             // restart: new epoch
  FrameHdr h2;
  h2.pts_us = 99;
  const auto au2 = au_bytes(300, 200);           // a DIFFERENT byte pattern
  w2.begin(h2, 0, 4);
  w2.append(au2.data(), au2.size());             // leave it OPEN; do not finish

  rc.pump(0);
  REQUIRE(c.events.size() == 3);
  CHECK(c.events[0].kind == AuEventKind::kOpen);
  CHECK(c.events[0].meta.pts_us == 7u);
  CHECK(c.events[1].kind == AuEventKind::kClose);
  CHECK(c.events[1].aborted);
  CHECK(c.events[1].meta.pts_us == 7u);
  CHECK(c.events[1].au.empty());
  CHECK(c.events[2].kind == AuEventKind::kOpen);
  CHECK(c.events[2].meta.pts_us == 99u);
  CHECK(c.events[2].au == au2);
  for (const auto& ev : c.events) CHECK(ev.kind != AuEventKind::kGrow);
  CHECK(rc.open_aborted() == 1);
  unlink(ring.c_str());
  // Revert check: deleting end_open_() in drain_ring_'s kResync branch must fail this test.
}

TEST(overflowing_open_record_ends_with_aborted_close) {   // Review Focus 2
  const std::string ring = tmp_path("open_over");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {1024, 4}));
  Collector c;
  RingClient rc({ring, tmp_path("nosock_over")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.pts_us = 1;
  const auto big = au_bytes(1500, 1);
  w.begin(h, 0, 4);
  w.append(big.data(), 600);
  rc.pump(0);
  REQUIRE(c.events.size() == 1);                 // kOpen, 600 B
  w.append(big.data() + 600, 600);               // 1200 > 1024: aborted mid-stream
  CHECK(rc.pump(0) == 0);
  REQUIRE(c.events.size() == 2);
  CHECK(c.events[1].kind == AuEventKind::kClose);
  CHECK(c.events[1].aborted);
  CHECK(c.events[1].au.empty());
  CHECK(w.finish(true, maburgs::AuLatMeta{}) == UINT64_MAX);
  publish(w, 0, true, 100, 2, 0, 2);             // the next AU reuses rec_no 0
  CHECK(rc.pump(0) == 1);
  REQUIRE(c.events.size() == 3);                 // its close only: never a grow of old bytes
  CHECK(c.events[2].kind == AuEventKind::kClose);
  CHECK(!c.events[2].aborted);
  CHECK(c.events[2].au == au_bytes(100, 2));
  CHECK(rc.open_aborted() == 1);
  unlink(ring.c_str());
}

TEST(close_of_a_reused_rec_no_aborts_the_followed_record) {   // Review Focus 1, client side
  const std::string ring = tmp_path("open_reuse");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {1024, 4}));
  Collector c;
  RingClient rc({ring, tmp_path("nosock_reuse")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.pts_us = 1;
  const auto big = au_bytes(1500, 1);
  w.begin(h, 0, 4);
  w.append(big.data(), 600);
  rc.pump(0);
  REQUIRE(c.events.size() == 1);                 // following rec 0, pts 1
  w.append(big.data() + 600, 600);               // overflow ...
  CHECK(w.finish(true, maburgs::AuLatMeta{}) == UINT64_MAX);
  publish(w, 0, true, 100, 2, 0, 2);             // ... and rec 0 again, closed, unseen
  CHECK(rc.pump(0) == 1);
  REQUIRE(c.events.size() == 3);
  CHECK(c.events[1].aborted);
  CHECK(c.events[1].meta.pts_us == 1u);
  CHECK(!c.events[2].aborted);
  CHECK(c.events[2].meta.pts_us == 2u);
  CHECK(c.events[2].au == au_bytes(100, 2));
  unlink(ring.c_str());
}

TEST(enhance_dropped_by_policy_after_open_still_ends_its_stream) {
  const std::string ring = tmp_path("open_enh");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  Collector c;
  RingClient rc({ring, tmp_path("nosock_enh")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.pts_us = 3;
  const auto au = au_bytes(200, 4);
  w.begin(h, 1, 4);
  w.append(au.data(), 100);
  rc.pump(0);
  REQUIRE(c.events.size() == 1);
  w.append(au.data() + 100, 100);
  w.finish(false, maburgs::AuLatMeta{});         // truncated enhance: policy drops it
  CHECK(rc.pump(0) == 0);
  REQUIRE(c.events.size() == 2);
  CHECK(c.events[1].kind == AuEventKind::kClose);
  CHECK(c.events[1].aborted);
  CHECK(rc.dropped_enhance_incomplete() == 1);
  CHECK(rc.delivered() == 0);
  CHECK(rc.open_aborted() == 0);                 // a policy drop, not a ring abort
  unlink(ring.c_str());
}

TEST(open_preview_carries_the_discont_flush) {
  const std::string ring = tmp_path("open_disc");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  Collector c;
  RingClient rc({ring, tmp_path("nosock_disc")}, c.sink());
  REQUIRE(rc.open());
  FrameHdr h;
  h.flags = kFlagDiscont;
  const auto au = au_bytes(80, 6);
  w.begin(h, 0, 4);
  w.append(au.data(), au.size());
  rc.pump(0);
  REQUIRE(c.events.size() == 1);
  CHECK(c.events[0].kind == AuEventKind::kOpen);
  CHECK(c.events[0].flush_before);               // preview: its close will flush
  w.finish(true, maburgs::AuLatMeta{});
  rc.pump(0);
  REQUIRE(c.events.size() == 2);
  CHECK(c.events[1].flush_before);
  unlink(ring.c_str());
}

TEST(wake_window_is_delivery_minus_t_complete) {
  const std::string ring = tmp_path("wake");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  Collector c;
  RingClient rc({ring, tmp_path("nosock_wake")}, c.sink());
  REQUIRE(rc.open());
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const uint64_t now_us = static_cast<uint64_t>(ts.tv_sec) * 1000000ull +
                          static_cast<uint64_t>(ts.tv_nsec) / 1000ull;
  FrameHdr h;
  const auto au = au_bytes(64, 1);
  w.begin(h, 0);
  w.append(au.data(), au.size());
  maburgs::AuLatMeta lat;
  lat.t_complete_us = now_us - 3000;
  w.finish(true, lat);
  CHECK(rc.pump(0) == 1);
  const auto wk = rc.take_wake();
  CHECK(wk.n == 1);
  CHECK(wk.p50 >= 3000);
  CHECK(wk.p50 < 1000000);
  CHECK(rc.take_wake().n == 0);                  // the window was cleared
  unlink(ring.c_str());
}

TEST(wake_fd_ends_a_pump_wait_early) {
  const std::string ring = tmp_path("wake1");
  maburgs::AuRingWriter w;
  REQUIRE(w.open(ring, {4096, 8}));
  const std::string nosock = tmp_path("nosock_wake1");
  unlink(nosock.c_str());
  Collector c;
  RingClient rc({ring, nosock}, c.sink());
  REQUIRE(rc.open());
  const int efd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  REQUIRE(efd >= 0);
  rc.set_wake_fd(efd);
  std::thread t([efd] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const uint64_t one = 1;
    (void)!::write(efd, &one, sizeof one);
  });
  const auto t0 = std::chrono::steady_clock::now();
  rc.pump(2000);
  const auto el = std::chrono::steady_clock::now() - t0;
  t.join();
  CHECK(el >= std::chrono::milliseconds(15));
  CHECK(el < std::chrono::milliseconds(500));
  ::close(efd);
  unlink(ring.c_str());
}

MTEST_MAIN
