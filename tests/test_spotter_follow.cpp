// SpotterFollow (web/src/spotter_follow.h): the spotter's channel follower,
// spec 2026-10-04 web-gs-channel-core §6. Pure: every test drives the clock.
#include <string>
#include "mtest.h"
#include "spotter_follow.h"
using namespace webgs;

namespace {
SpotterFollowCfg cfg(uint8_t start = 40) {
  SpotterFollowCfg c;
  c.channels = {40, 64, 112, 144};
  c.start = start;
  return c;
}
// A follower locked on `ch` at t=1000 ms (sweep found it on the first frame).
SpotterFollow locked_on(uint8_t ch, double t = 1000) {
  SpotterFollow f(cfg(ch));
  f.tick(t);
  f.on_card_channel(ch, t);
  f.on_frame(ch, t);
  return f;
}
}  // namespace

TEST(starts_sweeping_at_start_and_falls_back_to_first_member) {
  SpotterFollow f(cfg(112));
  CHECK(f.state() == FollowState::Sweeping);
  CHECK(f.desired() == 112);
  CHECK(f.follows() == 0);
  SpotterFollow g(cfg(136));          // not a member
  CHECK(g.desired() == 40);
  CHECK(std::string(to_string(FollowState::Sweeping)) == "sweeping");
  CHECK(std::string(to_string(FollowState::Locked)) == "locked");
  CHECK(std::string(to_string(FollowState::Following)) == "following");
}

TEST(sweep_dwells_from_card_report_then_advances_round_robin) {
  SpotterFollow f(cfg(112));
  f.tick(1000);
  f.tick(1200);                       // 200 ms with no card report: dwell clock not started
  CHECK(f.desired() == 112);
  f.on_card_channel(112, 1200);
  f.tick(1349);
  CHECK(f.desired() == 112);          // 149 ms on the member
  f.tick(1350);
  CHECK(f.desired() == 144);          // dwell_ms reached: next member
  f.on_card_channel(144, 1350);
  f.tick(1500);
  CHECK(f.desired() == 40);           // wraps
  CHECK(f.state() == FollowState::Sweeping);
}

TEST(silence_sweep_locks_on_first_frame) {
  SpotterFollow f(cfg(40));
  f.tick(1000);
  f.on_card_channel(40, 1000);
  f.tick(1150);                       // nothing on 40
  CHECK(f.desired() == 64);
  f.on_card_channel(64, 1160);
  f.on_frame(64, 1200);               // the link is here
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 64);
  CHECK(f.follows() == 0);            // a sweep find is not a follow
}

TEST(locked_sweeps_after_silence_starting_at_next_member) {
  SpotterFollow f = locked_on(64);
  f.on_frame(64, 1500);
  f.tick(2500);                       // 1000 ms since the last frame: not yet (strictly greater)
  CHECK(f.state() == FollowState::Locked);
  f.tick(2501);
  CHECK(f.state() == FollowState::Sweeping);
  CHECK(f.desired() == 112);          // the member after cur
}

TEST(follow_an_order_then_confirm) {
  SpotterFollow f = locked_on(40);
  f.on_rcf(64, 1, 40, 1100);
  CHECK(f.state() == FollowState::Following);
  CHECK(f.desired() == 64);
  CHECK(f.follows() == 1);
  f.on_frame(40, 1150);               // a late frame on the old channel changes nothing
  CHECK(f.state() == FollowState::Following);
  f.on_frame(64, 1400);
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 64);
  // the same pair again while locked on the target: nothing
  f.on_rcf(64, 1, 64, 1500);
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.follows() == 1);
}

TEST(timeout_and_return) {
  SpotterFollow f = locked_on(40);
  f.on_rcf(64, 1, 40, 1100);
  f.tick(3100);                       // confirm_ms exactly: still following (strictly greater)
  CHECK(f.state() == FollowState::Following);
  f.tick(3101);
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 40);
  // one full silence window to hear 40 again before sweeping
  f.tick(4101);
  CHECK(f.state() == FollowState::Locked);
  f.tick(4102);
  CHECK(f.state() == FollowState::Sweeping);
  CHECK(f.desired() == 64);           // after 40 in the set
}

// Review Focus 2
TEST(standing_order_after_timeout_return_is_ignored) {
  SpotterFollow f = locked_on(40);
  f.on_rcf(64, 1, 40, 1100);
  f.tick(3200);                       // gave up, back on 40
  REQUIRE(f.state() == FollowState::Locked);
  REQUIRE(f.desired() == 40);
  f.on_frame(40, 3300);
  f.on_rcf(64, 1, 40, 3300);          // the GS still says (64, 1)
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 40);
  CHECK(f.follows() == 1);
  f.on_rcf(64, 2, 40, 3400);          // a NEW epoch is a new order
  CHECK(f.state() == FollowState::Following);
  CHECK(f.follows() == 2);
}

TEST(non_member_order_is_recorded_and_ignored) {
  SpotterFollow f = locked_on(40);
  f.on_rcf(136, 1, 40, 1100);
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 40);
  CHECK(f.follows() == 0);
  f.on_rcf(136, 1, 40, 1200);         // repeated: still nothing (recorded as seen)
  CHECK(f.follows() == 0);
  f.on_rcf(0, 1, 40, 1300);           // hop_ch 0: no order ever issued
  CHECK(f.state() == FollowState::Locked);
}

TEST(mid_flight_join_does_nothing) {
  SpotterFollow f = locked_on(144);
  f.on_rcf(144, 7, 144, 1100);        // the standing truth: the GS is where we are
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 144);
  CHECK(f.follows() == 0);
  f.on_rcf(144, 7, 144, 1200);
  CHECK(f.follows() == 0);
}

TEST(relay_style_delayed_card_report) {
  SpotterFollow f = locked_on(40);
  f.on_rcf(64, 1, 40, 1100);
  // the relay takes 200 ms to report 64; a frame on 64 before that is impossible,
  // and the confirm clock runs from the order, not from the report
  f.tick(1300);
  f.on_card_channel(64, 1300);
  f.on_frame(64, 1350);
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 64);
  // sweeping with a slow card: the dwell counts from the report
  SpotterFollow s(cfg(40));
  s.tick(1000);
  s.on_card_channel(40, 1000);
  s.tick(1150);                       // -> 64 requested
  REQUIRE(s.desired() == 64);
  s.tick(1500);                       // 350 ms, no report yet: still 64
  CHECK(s.desired() == 64);
  s.on_card_channel(64, 1500);
  s.tick(1649);
  CHECK(s.desired() == 64);
  s.tick(1650);
  CHECK(s.desired() == 112);
}

// Review Focus 1
TEST(withdrawal_heard_on_back_returns_at_once) {
  SpotterFollow f = locked_on(40);
  f.on_rcf(64, 1, 40, 1100);
  REQUIRE(f.state() == FollowState::Following);
  f.on_rcf(40, 2, 40, 1150);          // withdrawn before the card moved
  CHECK(f.state() == FollowState::Locked);
  CHECK(f.desired() == 40);
  CHECK(f.follows() == 1);
  // a retarget mid-follow (order 64 then order 112 before we arrived)
  SpotterFollow g = locked_on(40);
  g.on_rcf(64, 1, 40, 1100);
  g.on_rcf(112, 2, 40, 1150);
  CHECK(g.state() == FollowState::Following);
  CHECK(g.desired() == 112);
  CHECK(g.follows() == 2);
  g.tick(3151);                       // nothing on 112 within confirm_ms of the LAST order
  CHECK(g.desired() == 40);
}

// Review Focus 3
TEST(rcf_on_sweep_member_locks_then_follows) {
  SpotterFollow f(cfg(40));
  f.tick(1000);
  f.on_card_channel(40, 1000);
  // wiring order: on_frame then on_rcf for the same body
  f.on_frame(40, 1050);
  f.on_rcf(64, 1, 40, 1050);
  CHECK(f.state() == FollowState::Following);
  CHECK(f.desired() == 64);
  // on_rcf alone while sweeping (a unit-level caller): same result
  SpotterFollow g(cfg(40));
  g.tick(1000);
  g.on_card_channel(40, 1000);
  g.on_rcf(64, 1, 40, 1050);
  CHECK(g.state() == FollowState::Following);
  CHECK(g.desired() == 64);
  CHECK(g.follows() == 1);
}

// Review Focus 4
TEST(sweep_skips_a_member_the_card_never_reports) {
  SpotterFollow f(cfg(40));
  f.tick(1000);
  f.on_card_channel(40, 1000);
  f.tick(1150);
  REQUIRE(f.desired() == 64);
  f.tick(2149);                       // 999 ms without on_card_channel(64)
  CHECK(f.desired() == 64);
  f.tick(2150);
  CHECK(f.desired() == 112);          // tune_timeout_ms: skipped
  CHECK(f.state() == FollowState::Sweeping);
}

// Review Focus 5
TEST(rx_channel_zero_is_ignored_everywhere) {
  SpotterFollow f = locked_on(40);
  f.on_frame(0, 1900);                // mid-retune stamp: no refresh
  f.tick(2001);
  CHECK(f.state() == FollowState::Sweeping);
  SpotterFollow g(cfg(40));
  g.tick(1000);
  g.on_card_channel(40, 1000);
  g.on_frame(0, 1050);                // no lock
  CHECK(g.state() == FollowState::Sweeping);
  g.on_rcf(64, 1, 0, 1060);           // not acted on, not recorded
  CHECK(g.state() == FollowState::Sweeping);
  CHECK(g.follows() == 0);
  g.on_frame(40, 1070);
  g.on_rcf(64, 1, 40, 1070);          // the same pair, now heard properly: follows
  CHECK(g.state() == FollowState::Following);
}

TEST(single_member_set_keeps_sweeping_itself) {
  SpotterFollowCfg c;
  c.channels = {136};
  c.start = 136;
  SpotterFollow f(c);
  f.tick(1000);
  f.on_card_channel(136, 1000);
  f.tick(1200);
  CHECK(f.desired() == 136);
  f.on_frame(136, 1300);
  CHECK(f.state() == FollowState::Locked);
  f.tick(2400);
  CHECK(f.state() == FollowState::Sweeping);
  CHECK(f.desired() == 136);
}

MTEST_MAIN
