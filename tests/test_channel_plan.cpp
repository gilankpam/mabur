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

// An ack that agrees to home freezes the plan on home: there is nothing to
// split toward. Entering the split would fan the same DISC out on both cards
// on one channel and log a from==to move, so tick() must refuse it forever.
TEST(op_equal_home_never_splits) {
  ChannelPlan p(C(2));
  p.on_ack(0, 136, 136);                            // drone agreed to home
  CHECK(p.frozen() && p.op() == 136);
  p.take_events();
  p.tick(100, true);
  p.tick(1100, false);                              // link lost
  p.tick(60000, false);                             // far past split_after_ms
  CHECK(!p.split());
  CHECK(p.desired(0) == 136 && p.desired(1) == 136);
  CHECK(!p.beacon_cards().has_value());
  CHECK(p.take_events().empty());
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
TEST(two_card_hop_lead_then_confirm) {
  ChannelPlan p(C(2));
  p.on_ack(0, 136, 136); p.take_events(); p.tick(100, true);
  p.hop_order(1000, 149, /*lead_card=*/1);
  CHECK(p.hopping() && p.hop_target() == 149 && p.hop_lead() == 1);
  CHECK(p.desired(0) == 136 && p.desired(1) == 149);
  CHECK(!p.split());
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::HopLead && ev[0].card == 1 && ev[0].from == 136 && ev[0].to == 149);
  p.tick(1200, false);            // FEC starved during the split: no SplitHome
  CHECK(!p.split() && p.desired(0) == 136);
  p.hop_confirmed(1250);
  CHECK(!p.hopping() && p.op() == 149);
  CHECK(p.desired(0) == 149 && p.desired(1) == 149);
  ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::HopFollow && ev[0].card == -1 && ev[0].from == 136 && ev[0].to == 149);
}
TEST(two_card_hop_withdraw_returns_lead) {
  ChannelPlan p(C(2));
  p.on_ack(0, 136, 136); p.take_events();
  p.hop_order(1000, 149, 1); p.take_events();
  p.hop_withdraw(1500);
  CHECK(!p.hopping() && p.op() == 136 && p.desired(1) == 136);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::HopWithdraw && ev[0].card == 1 && ev[0].from == 149 && ev[0].to == 136);
}
TEST(one_card_hop_moves_all_and_confirms_or_withdraws) {
  ChannelPlan p(C(1));
  p.on_ack(0, 136, 136); p.take_events();
  p.hop_order(1000, 165, -1);
  CHECK(p.desired(0) == 165 && p.op() == 136);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::HopOneCard && ev[0].card == -1 && ev[0].to == 165);
  p.hop_withdraw(1600);
  CHECK(p.desired(0) == 136 && p.op() == 136);
  p.hop_order(2000, 165, -1); p.take_events();
  p.hop_confirmed(2100);
  CHECK(p.op() == 165 && p.desired(0) == 165);
}
TEST(hop_after_a_hop_uses_the_new_op_as_from) {
  ChannelPlan p(C(2));
  p.on_ack(0, 136, 136); p.take_events();
  p.hop_order(1000, 149, 1); p.hop_confirmed(1100); p.take_events();
  p.hop_order(3000, 165, 0);            // lead is whichever card is not TX now
  CHECK(p.desired(0) == 165 && p.desired(1) == 149);
  auto ev = p.take_events();
  CHECK(ev[0].from == 149 && ev[0].to == 165 && ev[0].card == 0);
}
TEST(hop_withdraw_resets_loss_timer_for_split) {
  ChannelPlan p(C(2));
  p.on_ack(0, 149, 149); p.take_events();
  p.tick(1000, false);              // lost_since_ms_ = 1000 (pre-hop loss bookkeeping)
  p.tick(2000, false);              // 1000 ms of pre-hop loss accumulated, well under 5000
  CHECK(!p.split());
  p.hop_order(2000, 165, 1);
  p.tick(2200, false);              // during the hop: no bookkeeping, no split
  p.hop_withdraw(2500);             // lead card returns, op_ unchanged
  p.tick(2500, false);              // continuous loss resumes right at withdraw
  CHECK(!p.split());
  p.tick(6000, false);              // OLD buggy split point: 1000 + split_after_ms(5000)
  CHECK(!p.split());                // must NOT have split yet: pre-hop loss must not count
  p.tick(7499, false);              // withdraw_time(2500) + split_after_ms(5000) - 1
  CHECK(!p.split());
  p.tick(7500, false);              // withdraw_time + split_after_ms
  CHECK(p.split());
}
TEST(hop_order_while_hopping_emits_withdraw_for_abandoned_lead) {
  ChannelPlan p(C(2));
  p.on_ack(0, 136, 136); p.take_events();
  p.hop_order(1000, 149, 1);
  p.hop_order(2000, 165, 0);        // second order abandons the first hop's lead
  CHECK(p.hopping() && p.hop_target() == 165 && p.hop_lead() == 0);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 3);
  CHECK(ev[0].reason == MoveReason::HopLead && ev[0].card == 1 && ev[0].from == 136 && ev[0].to == 149);
  CHECK(ev[1].reason == MoveReason::HopWithdraw && ev[1].card == 1 && ev[1].from == 149 && ev[1].to == 136);
  CHECK(ev[2].reason == MoveReason::HopLead && ev[2].card == 0 && ev[2].from == 136 && ev[2].to == 165);
}
MTEST_MAIN
