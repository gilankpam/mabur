#include "mtest.h"
#include "cal_control.h"
#include "cal_session.h"

#include <string>

using namespace maburgs;

TEST(start_replies_ok_and_begins) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  CHECK(CalControl::apply("start", s, &reply));
  CHECK(reply.find("ok") == 0);
  CHECK(s.state() != CalSession::State::Idle);
}

TEST(start_reports_why_it_refused) {
  CalSession s(CalSessionCfg{});
  s.set_peer(/*linked=*/false, /*cal_capable=*/true);
  std::string reply;
  CHECK(!CalControl::apply("start", s, &reply));
  CHECK(reply.find("err") == 0);
  CHECK(reply.find("link") != std::string::npos);
}

TEST(start_accepts_a_margin_override) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  CHECK(CalControl::apply("start margin=2.0", s, &reply));
  CHECK(s.margin_db() == 2.0);
}

TEST(status_never_changes_state) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  CHECK(!CalControl::apply("status", s, &reply));
  CHECK(reply.find("idle") != std::string::npos);
  CHECK(s.state() == CalSession::State::Idle);
}

TEST(abort_stops_a_running_session) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  REQUIRE(CalControl::apply("start", s, &reply));
  CHECK(CalControl::apply("abort", s, &reply));
  CHECK(s.state() == CalSession::State::Idle);
}

TEST(abort_while_idle_is_a_no_op_and_is_not_logged) {
  // The stderr line poll() emits on a true return is the post-mortem record
  // that a calibration happened. An operator's idle sanity-check abort must
  // not write one.
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  CHECK(!CalControl::apply("abort", s, &reply));
  CHECK(s.state() == CalSession::State::Idle);
}

TEST(unknown_and_empty_commands_are_errors_not_crashes) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string reply;
  CHECK(!CalControl::apply("", s, &reply));
  CHECK(reply.find("err") == 0);
  CHECK(!CalControl::apply("frobnicate", s, &reply));
  CHECK(reply.find("err") == 0);
}

TEST(binds_loopback_only) {
  // The GS answers on 10.18.0.1; this port must never be reachable from the
  // drone or a bench laptop.
  CalControl c;
  REQUIRE(c.open(18400));
  CHECK(c.ok());
  CHECK(std::string(c.bound_address()) == "127.0.0.1");
}

MTEST_MAIN
