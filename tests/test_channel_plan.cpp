#include "mtest.h"
#include "channel_plan.h"
using namespace maburgs;

static ChannelPlanCfg C(int n) { ChannelPlanCfg c; c.home = 136; c.n_cards = n; c.split_after_ms = 5000; c.home_window_ms = 300; c.beacon_period_ms = 20; return c; }

TEST(before_freeze_everything_is_home_and_selector_beacons) {
  ChannelPlan p(C(2));
  CHECK(!p.frozen()); CHECK(p.op() == 136);
  CHECK(p.desired(0) == 136 && p.desired(1) == 136);
  CHECK(!p.beacon_cards().has_value());
  p.tick(0, false); p.tick(60000, false);          // no split before freeze
  CHECK(!p.split());
  CHECK(p.take_events().empty());
}

TEST(ack_commits_and_freezes) {
  ChannelPlan p(C(2));
  p.on_ack(1000, 149, 149);
  CHECK(p.frozen() && p.op() == 149);
  CHECK(p.desired(0) == 149 && p.desired(1) == 149);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].card == -1 && ev[0].from == 136 && ev[0].to == 149 && ev[0].reason == MoveReason::Commit);
  CHECK(ev[0].t_ms == 1000);
}

TEST(ack_disagreeing_with_proposal_is_authoritative) {
  ChannelPlan p(C(2));
  p.on_ack(1000, 136, 149);                         // drone said home
  CHECK(p.frozen() && p.op() == 136);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::AckOverride && ev[0].from == 136 && ev[0].to == 136);
}

TEST(two_card_split_after_loss_then_reunite_on_ack) {
  ChannelPlan p(C(2));
  p.on_ack(0, 149, 149); p.take_events();
  p.tick(100, true);
  p.tick(1100, false);                              // link lost at 1100
  p.tick(6000, false);
  CHECK(!p.split());                                // 4900 < 5000
  p.tick(6100, false);
  CHECK(p.split());
  CHECK(p.desired(0) == 136 && p.desired(1) == 149);
  auto bc = p.beacon_cards();
  REQUIRE(bc.has_value()); REQUIRE(bc->size() == 2);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::SplitHome && ev[0].card == 0 && ev[0].to == 136);
  p.on_ack(7000, 149, 149);                         // same op: reunite, no commit
  CHECK(!p.split());
  CHECK(p.desired(0) == 149 && p.desired(1) == 149);
  CHECK(!p.beacon_cards().has_value());
  ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::Reunite && ev[0].card == 0 && ev[0].from == 136 && ev[0].to == 149);
}

TEST(video_reunites_and_short_fade_never_splits) {
  ChannelPlan p(C(2));
  p.on_ack(0, 149, 149); p.take_events();
  p.tick(1000, false); p.tick(3000, false); p.tick(3500, true);   // 2.5 s fade
  CHECK(!p.split()); CHECK(p.take_events().empty());
  p.tick(4000, false); p.tick(9100, false);
  CHECK(p.split()); p.take_events();
  p.tick(9200, true);                               // video on the op channel
  CHECK(!p.split());
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1); CHECK(ev[0].reason == MoveReason::Reunite);
}

TEST(one_card_interleaves_with_quiet_gap) {
  ChannelPlan p(C(1));
  p.on_ack(0, 149, 149); p.take_events();
  p.tick(1000, false); p.tick(6000, false);
  CHECK(p.split());
  // Window 0 (home): [6000, 6300). Beacons allowed except the last 20 ms.
  CHECK(p.desired(0) == 136);
  auto bc = p.beacon_cards(); REQUIRE(bc.has_value()); CHECK(bc->size() == 1 && (*bc)[0] == 0);
  p.tick(6285, false);
  CHECK(p.desired(0) == 136);
  bc = p.beacon_cards(); REQUIRE(bc.has_value()); CHECK(bc->empty());   // quiet gap
  p.tick(6300, false);
  CHECK(p.desired(0) == 149);                        // op window
  bc = p.beacon_cards(); REQUIRE(bc.has_value()); CHECK(bc->size() == 1);
  p.tick(6600, false);
  CHECK(p.desired(0) == 136);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);                            // only the split, not the hops
  CHECK(ev[0].reason == MoveReason::SplitHome);
  p.on_ack(6650, 149, 149);
  CHECK(!p.split() && p.desired(0) == 149);
}
MTEST_MAIN
