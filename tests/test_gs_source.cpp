#include "mtest.h"
#include "gs_source.h"
#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>
using namespace maburplay;

static const char* kA = R"({"v":1,"cards":[],
  "link":{"air_pct":10.0,"ctl":{"rung":{"mcs":1,"ov":0.5}}}})";
static const char* kB = R"({"v":1,"cards":[],
  "link":{"air_pct":80.0,"ctl":{"rung":{"mcs":7,"ov":0.1}}}})";

// Sends one datagram to 127.0.0.1:port.
static void send_to(int port, const char* s) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  REQUIRE(::sendto(fd, s, std::strlen(s), 0, (sockaddr*)&a, sizeof(a)) > 0);
  ::close(fd);
}

// poll() is non-blocking and a real loopback datagram may not have landed
// on the socket by the very next line, so every test that depends on actual
// socket delivery retries poll() in a bounded loop rather than sleeping a
// fixed interval or trusting a single call. The loop is bounded (never
// hangs) and each iteration advances now_ms by 1, so the timestamps fed to
// GsSource stay monotonic and predictable once delivery lands.

// Retries poll() until it returns true (a fresh decode) or the budget runs
// out. Returns the last poll() result.
static bool poll_until_fresh(GsSource* src, uint64_t start_ms) {
  bool ready = false;
  for (int i = 0; i < 200 && !ready; ++i) ready = src->poll(start_ms + i);
  return ready;
}

// Retries poll() until `datagrams()` has counted `want` datagrams (i.e. the
// backlog has actually been drained off the kernel socket buffer), returning
// the poll() call's own return value from the call that reached `want`. This
// is what lets a test assert on the return value of the SPECIFIC poll() call
// that processed a particular datagram, even though delivery timing on a
// real socket is not guaranteed to land within one call.
static bool poll_until_count(GsSource* src, uint64_t start_ms, uint64_t want) {
  bool last = false;
  for (int i = 0; i < 200 && src->datagrams() < want; ++i) {
    last = src->poll(start_ms + i);
  }
  return last;
}

// Waits (via ::poll(2) on the raw socket, which only checks readability and
// never removes anything from the queue) until the socket has data pending,
// or the timeout expires. This is the drain-free counterpart to the helpers
// above: it lets a test synchronize on "the datagram(s) have genuinely
// landed" and THEN make exactly one GsSource::poll() call, so that call is
// what's actually under test -- not rescued by a retry loop that would mask
// a poll() which only recv()s once per invocation instead of draining to
// EAGAIN.
static bool wait_readable(int fd, int timeout_ms) {
  struct pollfd pfd {};
  pfd.fd = fd;
  pfd.events = POLLIN;
  const int r = ::poll(&pfd, 1, timeout_ms);
  return r > 0 && (pfd.revents & POLLIN);
}

// Number of open descriptors held by this process. Counting /proc/self/fd is
// what makes a leak observable rather than inferred: "still works
// afterwards" would pass just as happily with a missing close() -- only the
// fd count moves. Declared up top since more than one test below needs it.
static int open_fd_count() {
  DIR* d = ::opendir("/proc/self/fd");
  if (!d) return -1;
  int n = 0;
  while (::readdir(d)) ++n;
  ::closedir(d);
  return n;  // includes . / .. / the dirfd itself: constant offsets, we diff
}

TEST(open_binds_an_ephemeral_port_and_reports_it) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  CHECK(src.port() > 0);
  CHECK(err.empty());
}

TEST(poll_delivers_a_datagram_and_counts_it) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  send_to(src.port(), kA);
  CHECK(poll_until_fresh(&src, 1000));
  CHECK(src.datagrams() == 1);
  CHECK(src.snapshots() == 1);
  CHECK(src.have_any());
  REQUIRE(src.snapshot().mcs.has_value());
  CHECK(*src.snapshot().mcs == 1);
}

// A backlog must collapse to the NEWEST sample: the OSD shows now, not a
// queue replayed one frame at a time. This is the test for poll()'s "drains
// every pending datagram" contract, so it must actually exercise a SINGLE
// poll() call draining two -- a retry-loop helper here would let a poll()
// that only recv()s once per call (and relies on being called again) pass
// just as happily, silently losing the coverage. wait_readable() is the
// drain-free synchronization: it confirms the kernel has data queued
// without consuming it, so the ONE src.poll() below is the real subject.
TEST(poll_drains_the_backlog_and_keeps_the_newest) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  send_to(src.port(), kA);
  send_to(src.port(), kB);
  REQUIRE(wait_readable(src.debug_fd(), 2000));
  CHECK(src.poll(1000));
  CHECK(src.datagrams() == 2);
  REQUIRE(src.snapshot().mcs.has_value());
  CHECK(*src.snapshot().mcs == 7);  // kB, the last one
}

TEST(poll_with_nothing_pending_returns_false) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  CHECK(!src.poll(1000));
  CHECK(src.datagrams() == 0);
  CHECK(!src.have_any());
}

// A malformed datagram is counted and dropped; the last good snapshot and
// the freshness clock both survive it. The specific assertion that matters
// is on the RETURN VALUE of the poll() call that actually drained the
// malformed datagram (nothing renderable arrived in that call) -- so we
// wait for datagrams() to reach 2 (proof the bad one was received) and
// check the return value from that exact call, rather than a single
// poll() immediately after sendto which could trivially return false just
// because the datagram hadn't landed yet.
TEST(malformed_datagram_is_counted_and_the_last_good_one_survives) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  src.set_stale_ms(3000);
  send_to(src.port(), kB);
  REQUIRE(poll_until_fresh(&src, 1000));
  send_to(src.port(), "{not json");
  CHECK(!poll_until_count(&src, 1500, 2));  // nothing renderable arrived
  CHECK(src.datagrams() == 2);
  CHECK(src.parse_errors() == 1);
  CHECK(src.snapshots() == 1);
  REQUIRE(src.snapshot().mcs.has_value());
  CHECK(*src.snapshot().mcs == 7);  // still kB
  // stale_ms=3000, last good decode at (approximately) 1000. Pick a check
  // time past 1000+3000 but still short of a wrongly-advanced clock's
  // 1500+3000: this window is the one place the two clocks disagree, so it
  // is what actually proves the clock did NOT advance on the bad datagram.
  CHECK(src.stale(4300));
}

TEST(stale_only_after_a_first_snapshot_and_the_full_interval) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  src.set_stale_ms(3000);
  CHECK(!src.stale(999999));  // never heard anything: nothing to call stale
  send_to(src.port(), kA);
  REQUIRE(poll_until_fresh(&src, 1000));
  CHECK(!src.stale(1000));
  CHECK(!src.stale(3999));
  CHECK(src.stale(4000));  // 1000 + 3000
}

TEST(stale_ms_zero_disables_staleness) {
  GsSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  src.set_stale_ms(0);
  send_to(src.port(), kA);
  REQUIRE(poll_until_fresh(&src, 1000));
  CHECK(!src.stale(10000000));
}

// feed_open() is the socket-free path --gs-render uses.
TEST(feed_open_accepts_bytes_without_a_socket) {
  GsSource src;
  REQUIRE(src.feed_open());
  CHECK(src.port() == 0);
  CHECK(src.feed((const uint8_t*)kB, std::strlen(kB), 500));
  REQUIRE(src.snapshot().mcs.has_value());
  CHECK(*src.snapshot().mcs == 7);
}

TEST(binding_a_port_twice_fails_with_a_reason) {
  GsSource a;
  std::string err;
  REQUIRE(a.open(0, &err));
  const int before = open_fd_count();
  REQUIRE(before > 0);  // /proc must be mounted for this to mean anything
  GsSource b;
  std::string err2;
  CHECK(!b.open(a.port(), &err2));
  CHECK(!err2.empty());
  // A failed bind() must close the socket() it just opened right there in
  // open()'s failure path, not defer it to b's eventual destructor: check
  // the count with `b` still in scope (undestructed), which is the only
  // moment that distinguishes "closed immediately" from "closed late".
  const int after = open_fd_count();
  CHECK(after - before == 0);
}

TEST(destroying_many_sources_does_not_leak_descriptors) {
  const int before = open_fd_count();
  REQUIRE(before > 0);  // /proc must be mounted for this to mean anything
  for (int i = 0; i < 64; ++i) {
    GsSource src;
    std::string err;
    REQUIRE(src.open(0, &err));
    send_to(src.port(), kA);
    poll_until_fresh(&src, 1000);
    // src goes out of scope here -- the socket must close with it.
  }
  const int after = open_fd_count();
  CHECK(after - before == 0);
}

MTEST_MAIN
