#include "mtest.h"
#include "channel_plan.h"
using namespace maburgs;

static ChannelPlanCfg C(int n, uint8_t start = 136) {
  ChannelPlanCfg c; c.start = start; c.channels = {136, 149, 161}; c.n_cards = n; c.search_after_ms = 5000; return c;
}

TEST(starts_on_start_channel_and_releases_the_scout_before_any_link) {
  ChannelPlan p(C(2));
  CHECK(p.op() == 136);
  CHECK(p.desired(0) == 136 && p.desired(1) == 136);
  p.tick(0, false);
  CHECK(p.release_scout());               // never linked: nothing to protect, search at once
  CHECK(p.take_events().empty());
}

TEST(commit_moves_op_and_logs) {
  ChannelPlan p(C(2));
  p.commit(1000, 149);
  CHECK(p.op() == 149 && p.desired(0) == 149);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::Commit && ev[0].from == 136 && ev[0].to == 149 && ev[0].card == -1);
  p.commit(1100, 149);                     // same channel: no event
  CHECK(p.take_events().empty());
}

TEST(ack_agreeing_with_op_changes_nothing) {
  ChannelPlan p(C(2));
  p.on_ack(1000, 136, 136);
  CHECK(p.op() == 136);
  CHECK(p.take_events().empty());
}

TEST(ack_override_to_a_member_is_followed_non_member_ignored) {
  ChannelPlan p(C(2));
  p.on_ack(1000, 149, 136);                // drone insisted on 149
  CHECK(p.op() == 149);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::AckOverride && ev[0].from == 136 && ev[0].to == 149);
  p.on_ack(1100, 112, 149);                // 112 not in the set
  CHECK(p.op() == 149);
  CHECK(p.take_events().empty());
  CHECK(!p.member(112) && p.member(161));
}

TEST(loss_holds_op_for_search_after_ms_then_releases_the_scout) {
  ChannelPlan p(C(2));
  p.tick(100, true);
  CHECK(!p.release_scout());
  p.tick(1100, false);                     // lost at 1100
  p.tick(6000, false);
  CHECK(!p.release_scout());               // 4900 < 5000: short fade, keep diversity
  CHECK(p.desired(0) == 136 && p.desired(1) == 136);   // every card stays on op
  p.tick(6100, false);
  CHECK(p.release_scout());
  p.tick(6200, true);                      // drone found / video back
  CHECK(!p.release_scout());
  CHECK(p.take_events().empty());          // search is not a move
}

TEST(hop_window_does_not_count_toward_the_search_timer) {
  ChannelPlan p(C(2));
  p.tick(100, true);
  p.tick(1100, false);                     // lost at 1100
  p.hop_order(1200, 149, 1);
  CHECK(!p.release_scout());               // hopping: never release
  p.hop_withdraw(4200);                    // 3 s hop window excluded
  p.tick(6500, false);                     // 5400 since loss, minus 3000 hop = 2400 < 5000
  CHECK(!p.release_scout());
  p.tick(9200, false);
  CHECK(p.release_scout());
}

TEST(two_card_hop_lead_then_follow_moves_op) {
  ChannelPlan p(C(2));
  p.tick(0, true);
  p.hop_order(1000, 149, 1);
  CHECK(p.hopping() && p.desired(1) == 149 && p.desired(0) == 136);
  CHECK(p.is_link_video(149) && p.is_link_video(136) && !p.is_link_video(161));
  p.hop_confirmed(1300);
  CHECK(!p.hopping() && p.op() == 149 && p.desired(0) == 149);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].reason == MoveReason::HopLead && ev[1].reason == MoveReason::HopFollow);
}

TEST(one_card_hop_moves_the_only_card_and_withdraw_restores) {
  ChannelPlan p(C(1));
  p.tick(0, true);
  p.hop_order(1000, 149, -1);
  CHECK(p.desired(0) == 149);
  p.hop_withdraw(1600);
  CHECK(p.op() == 136 && p.desired(0) == 136);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 2);
  CHECK(ev[0].reason == MoveReason::HopOneCard && ev[1].reason == MoveReason::HopWithdraw);
}

TEST(to_string_covers_every_reason) {
  CHECK(std::string(to_string(MoveReason::Commit)) == "commit");
  CHECK(std::string(to_string(MoveReason::AckOverride)) == "ack_override");
  CHECK(std::string(to_string(MoveReason::HopLead)) == "hop_lead");
  CHECK(std::string(to_string(MoveReason::HopFollow)) == "hop_follow");
  CHECK(std::string(to_string(MoveReason::HopWithdraw)) == "hop_withdraw");
  CHECK(std::string(to_string(MoveReason::HopOneCard)) == "hop_one_card");
  CHECK(std::string(to_string(MoveReason::LinkFound)) == "link_found");
}

// Carried over from the pre-split-deletion file (hop-specific, not home-
// specific): a second order mid-hop abandons the first lead and must log
// its withdrawal before starting the new one.
TEST(hop_order_while_hopping_emits_withdraw_for_abandoned_lead) {
  ChannelPlan p(C(2));
  p.tick(0, true);
  p.hop_order(1000, 149, 1);
  p.hop_order(2000, 165, 0);        // second order abandons the first hop's lead
  CHECK(p.hopping() && p.hop_target() == 165 && p.hop_lead() == 0);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 3);
  CHECK(ev[0].reason == MoveReason::HopLead && ev[0].card == 1 && ev[0].from == 136 && ev[0].to == 149);
  CHECK(ev[1].reason == MoveReason::HopWithdraw && ev[1].card == 1 && ev[1].from == 149 && ev[1].to == 136);
  CHECK(ev[2].reason == MoveReason::HopLead && ev[2].card == 0 && ev[2].from == 136 && ev[2].to == 165);
}

// Carried over: hop_withdraw() with no hop in flight (the one-card Order
// path, main.cpp never calls hop_order() until OneCardRetune) must not
// perturb the search timer -- pre-fix it ran the shift with hop_start_ms_
// still 0, pushing lost_since_ms_ a whole session into the future.
TEST(withdraw_with_no_hop_in_flight_leaves_the_search_timer_alone) {
  ChannelPlan p(C(1));
  p.tick(0, true);
  p.tick(30000, false);             // lost_since_ms_ = 30000 (link down, deep into the flight)
  p.hop_withdraw(30100);            // controller timed out; the plan never started hopping
  CHECK(!p.hopping());
  CHECK(p.take_events().empty());   // nothing moved, so nothing to log
  p.tick(34999, false);
  CHECK(!p.release_scout());
  p.tick(35000, false);             // 30000 + search_after_ms(5000)
  CHECK(p.release_scout());         // pre-fix: lost_since_ms_ = 60100, no release for another 30 s
}

// Carried over: hop_confirmed() with no hop in flight is a no-op (pre-fix:
// op_ = hop_target_ = 0).
TEST(confirm_with_no_hop_in_flight_is_a_no_op) {
  ChannelPlan p(C(1));
  p.tick(0, true);
  p.tick(30000, false);
  p.hop_confirmed(30100);
  CHECK(p.op() == 136);
  CHECK(p.desired(0) == 136);
  CHECK(p.take_events().empty());
  p.tick(35000, false);
  CHECK(p.release_scout());
}

// ---- final review C1 (redirected): link where the drone is found, then
// relocate with a hop order. want = where the link SHOULD live; op = where
// it does. Every move of a linked link is a hop order; a DISC only finds.

TEST(want_starts_on_the_start_channel) {
  ChannelPlan p(C(2, 149));
  CHECK(p.want() == 149 && p.op() == 149);
  p.tick(0, true);
  CHECK(!p.relocate_due());
}

// Revert (link_found leaves op): the link card stays on the old op while
// the drone sits on X and the session lapses (the C1 hole).
TEST(link_found_moves_op_for_every_card_and_logs) {
  ChannelPlan p(C(2));
  p.tick(0, false);
  p.link_found(10, 149);
  CHECK(p.op() == 149 && p.desired(0) == 149 && p.desired(1) == 149);
  CHECK(p.want() == 136);                     // where the link should be is unchanged
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::LinkFound && ev[0].card == -1 && ev[0].from == 136 &&
        ev[0].to == 149);
  p.link_found(20, 149);                      // already there
  p.link_found(20, 112);                      // not a member
  CHECK(p.op() == 149 && p.take_events().empty());
}

TEST(relocate_due_truth_table) {
  ChannelPlan p(C(2));
  p.tick(0, false);
  p.link_found(10, 149);
  CHECK(!p.relocate_due());                   // not in session
  p.tick(20, true);
  CHECK(p.relocate_due());                    // in session, op 149 != want 136
  p.hop_order(30, 136, 1);
  CHECK(!p.relocate_due());                   // the relocation is in flight
  p.hop_confirmed(200);
  CHECK(p.op() == 136 && !p.relocate_due());  // landed: op == want
}

TEST(commit_without_session_moves_op_and_want) {
  ChannelPlan p(C(2));
  p.tick(0, false);
  p.commit(100, 149);
  CHECK(p.op() == 149 && p.want() == 149);
  auto ev = p.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].reason == MoveReason::Commit);
}

// In session the link moves only by a hop order: commit sets want and the
// caller relocates. Revert (commit moves op in session): the cards retune
// under a live link with no order to the drone.
TEST(commit_in_session_only_sets_want) {
  ChannelPlan p(C(2));
  p.tick(0, true);
  p.commit(100, 149);
  CHECK(p.op() == 136 && p.want() == 149 && p.desired(0) == 136);
  CHECK(p.take_events().empty());
  CHECK(p.relocate_due());
  p.commit(110, 112);                         // not a member: ignored
  CHECK(p.want() == 149);
}

TEST(set_want_accepts_members_only) {
  ChannelPlan p(C(2));
  p.tick(0, true);
  p.link_found(5, 161);
  p.set_want(10, 161);                        // accept where the link is
  CHECK(p.want() == 161 && !p.relocate_due());
  p.set_want(20, 112);
  CHECK(p.want() == 161);
}

// A reactive hop decides where the link lives: want follows the confirmed
// target, so the next link-up does not relocate back onto the channel the
// hop fled. Revert (want untouched): relocate_due reads true after the hop.
TEST(hop_confirm_moves_want_with_op) {
  ChannelPlan p(C(2));
  p.tick(0, true);
  p.hop_order(100, 161, 1);
  p.hop_confirmed(300);
  CHECK(p.op() == 161 && p.want() == 161 && !p.relocate_due());
}

MTEST_MAIN
