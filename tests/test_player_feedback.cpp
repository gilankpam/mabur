#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>
#include "mabur/player_fb.h"
#include "mtest.h"
#include "player_feedback.h"

namespace {
// Sends a raw datagram to 127.0.0.1:port. Returns true on success.
bool send_to(int port, const std::string& s) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return false;
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
  const ssize_t n = sendto(fd, s.data(), s.size(), 0,
                           reinterpret_cast<sockaddr*>(&a), sizeof(a));
  close(fd);
  return n == static_cast<ssize_t>(s.size());
}

std::string dgram(bool idr, const char* reason, uint64_t joins) {
  mabur::playerfb::Msg m;
  m.idr = idr;
  m.reason = std::string(reason) == "join" ? 2 : 1;
  m.joins = joins;
  char buf[192];
  const size_t n = mabur::playerfb::format(m, buf, sizeof(buf));
  return std::string(buf, n);
}
}  // namespace

TEST(open_binds_an_ephemeral_port) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  CHECK(fb.ok());
  CHECK(fb.port() > 0);
  CHECK(!fb.have_any());
}

TEST(level_edge_is_reported_once) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  CHECK(fb.poll(1000) == true);   // clear -> set edge
  CHECK(fb.want());
  REQUIRE(send_to(fb.port(), dgram(true, "join", 2)));
  CHECK(fb.poll(1500) == false);  // still set: NOT a new episode
  CHECK(fb.want());
  CHECK(fb.msg().joins == 2);
}

TEST(level_clears_and_can_re_edge) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(1000));
  REQUIRE(send_to(fb.port(), dgram(false, "flush", 0)));
  CHECK(!fb.poll(1100));
  CHECK(!fb.want());
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(1200));  // new episode
}

TEST(backlog_collapses_to_the_newest) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  REQUIRE(send_to(fb.port(), dgram(true, "join", 9)));
  CHECK(fb.poll(1000));
  CHECK(fb.msg().joins == 9);
  CHECK(fb.datagrams() == 2);
}

TEST(malformed_is_counted_and_ignored) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), "garbage not a datagram"));
  CHECK(!fb.poll(1000));
  CHECK(fb.malformed() == 1);
  CHECK(!fb.have_any());
  CHECK(!fb.want());
}

TEST(silence_expires_the_level_without_a_new_edge) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(1000));
  CHECK(fb.want());
  fb.expire(5000, 3000);  // 4 s of silence, stale_ms 3000
  CHECK(!fb.want());
  CHECK(fb.age_ms(5000) == 4000);
}

TEST(zero_length_datagram_does_not_stall_the_drain) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), ""));  // zero-length datagram, arrives first
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  CHECK(fb.poll(1000));  // the valid datagram behind it must still be seen
  CHECK(fb.want());
  CHECK(fb.msg().joins == 1);
  CHECK(fb.malformed() == 1);
  CHECK(fb.datagrams() == 2);
}

// --------------------------------------------------------------------------
// reconcile_player_idr(): the GS-side level reconciliation. This is the ONLY
// retry mechanism in the whole feature -- the stream carries no IRAP after
// session start and GDR never repaints, so a request that is never re-made
// means a permanently broken picture. All four shapes below are asserted
// against the LEVEL, not the received edge.
// --------------------------------------------------------------------------

// C2: the GS emitted the granted IDR (its latch cleared) but the player never
// received it -- ring lap, oversize drop. The player keeps asserting the same
// level, so there is no clear->set edge ever again. The request must still be
// re-raised, which is what the spec's failure table promises ("latch stays
// set, request repeats, bounded by the drone's 1 s cooldown").
// REVERT CHECK: fails if the raise is gated on poll()'s edge return.
TEST(held_level_re_raises_after_the_latch_clears) {
  maburgs::PlayerFeedback fb;
  maburgs::IdrRequester q;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(maburgs::reconcile_player_idr(fb, q, 1000, 3000));
  CHECK(q.want());
  CHECK(q.episodes_player() == 1);

  // The IDR is emitted on the wire; our latch clears. The player did not see
  // it, so no new datagram arrives -- only the held level remains.
  CHECK(q.on_frame_emitted(true, true, 1100));
  CHECK(!q.want());
  CHECK(fb.want());  // player still asserting

  CHECK(maburgs::reconcile_player_idr(fb, q, 1200, 3000));  // re-raised
  CHECK(q.want());
  CHECK(q.episodes_player() == 2);
}

// C1: the player dies (watchdog exit, crash) and the init wrapper respawns it
// in ~1 s -- inside player_fb_stale_ms. Our shadow level is still set from the
// dead instance, so the fresh instance's first idr=1 is NOT an edge, while our
// latch may already have cleared. Without level reconciliation the new
// player's non-IRAP join is never repaired.
// REVERT CHECK: fails if the raise is gated on poll()'s edge return.
TEST(respawned_player_inside_the_stale_window_re_raises) {
  maburgs::PlayerFeedback fb;
  maburgs::IdrRequester q;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "watchdog", 0)));
  CHECK(maburgs::reconcile_player_idr(fb, q, 1000, 3000));
  CHECK(q.on_frame_emitted(true, true, 1100));  // repaired that instance
  CHECK(!q.want());

  // Respawned instance, 1 s later: joins at a non-IRAP sid0 and asserts. Its
  // counters restart from 0, and the level never went clear in between.
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  CHECK(!fb.poll(2000));  // no edge: the shadow level was already set
  CHECK(fb.want());
  CHECK(maburgs::reconcile_player_idr(fb, q, 2000, 3000));
  CHECK(q.episodes_player() == 2);
}

// Ordering guard: expire() must run BEFORE the level is consulted, or a dead
// player's stale assertion raises a request nobody is asking for. REVERT
// CHECK: fails if expire() is moved after the want() check.
TEST(stale_expired_level_does_not_raise) {
  maburgs::PlayerFeedback fb;
  maburgs::IdrRequester q;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(maburgs::reconcile_player_idr(fb, q, 1000, 3000));
  CHECK(q.on_frame_emitted(true, true, 1100));
  CHECK(!q.want());
  // 4 s of silence with the level still notionally set: the player is gone.
  CHECK(!maburgs::reconcile_player_idr(fb, q, 5000, 3000));
  CHECK(!q.want());
  CHECK(q.episodes_player() == 1);
  CHECK(!fb.want());
}

// Runaway bound: on a healthy link the player's level is 0, so reconciling
// every core-loop iteration raises nothing at all. The only way to re-raise is
// for the latch to clear, which requires an IDR to have been emitted -- which
// the drone rate-limits to one per waybeam.idr_cooldown_ms.
TEST(healthy_level_never_raises_however_often_reconciled) {
  maburgs::PlayerFeedback fb;
  maburgs::IdrRequester q;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(false, "none", 0)));
  for (uint64_t t = 1000; t < 1100; ++t)
    CHECK(!maburgs::reconcile_player_idr(fb, q, t, 3000));
  CHECK(!q.want());
  CHECK(q.episodes_player() == 0);
}

// expire() must clamp like age_ms(): a caller clock behind the receive stamp
// must not underflow into a huge age and drop a level that just arrived.
// REVERT CHECK: fails (level wrongly expired) if the now_ms <= last_rx_ms_
// guard is removed from expire(). Same hazard class as commit f9c898b.
TEST(expire_clamps_when_caller_clock_lags_the_receive_stamp) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(5000));
  fb.expire(4990, 3000);  // caller clock behind the stamp
  CHECK(fb.want());
  fb.expire(5000, 3000);  // equal: age 0, nothing to expire
  CHECK(fb.want());
  fb.expire(9000, 3000);  // genuinely stale
  CHECK(!fb.want());
}

MTEST_MAIN
