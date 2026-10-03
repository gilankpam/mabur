#include <string>

#include "mtest.h"
#include "boot_pick.h"
using namespace maburgs;

// Final review I3: the boot pick + relocation state machine, extracted from
// main.cpp so every exit is tested. Each test drives BootPick the way the
// core loop does: one tick() per control tick, note_hop_action() for every
// HopAction the controller returned.

static BootPickIn base() {
  BootPickIn in;
  in.now_ms = 1000;
  in.since_start_ms = 1000;
  in.max_ms = 30000;
  in.op = 40;
  in.want = 40;
  in.proposal = 40;
  in.scout_owns = true;          // auto, pick open: the scout holds its card
  in.hop_idle_or_hold = true;
  return in;
}
static BootPickIn linked(BootPickIn in) {
  in.in_session = true;
  in.hop_active = true;
  return in;
}
static bool is(const BootPickOut& o, BootPickOut::Kind k) { return o.kind == k; }
static bool frozen(const BootPickOut& o, const char* why) {
  return o.kind == BootPickOut::Freeze && o.reason && std::string(o.reason) == why;
}

TEST(nothing_before_maturity) {
  BootPick b(true);
  CHECK(is(b.tick(base()), BootPickOut::None));
  CHECK(is(b.tick(linked(base())), BootPickOut::None));
  CHECK(b.open());
}

TEST(unlinked_maturity_commits_and_freezes_once) {
  BootPick b(true);
  BootPickIn in = base();
  in.scout_mature = true; in.scout_op_ranked = true; in.proposal = 144;
  auto o = b.tick(in);
  CHECK(is(o, BootPickOut::Commit) && o.ch == 144 && std::string(o.reason) == "commit");
  CHECK(!b.open());
  for (int i = 0; i < 3; ++i) CHECK(is(b.tick(in), BootPickOut::None));   // fires once
}

TEST(linked_maturity_on_op_freezes_in_place_and_accepts_op) {
  BootPick b(true);
  BootPickIn in = linked(base());
  in.scout_mature = true; in.scout_op_ranked = true; in.op = 64; in.proposal = 64;
  auto o = b.tick(in);
  CHECK(frozen(o, "in place") && o.accept_op);
}

// Final review I1: op only has its pre-link visits; a working link is not
// moved on a one-sided comparison. Revert (no op_ranked check): WantPick.
TEST(linked_maturity_with_op_unmeasured_freezes_in_place) {
  BootPick b(true);
  BootPickIn in = linked(base());
  in.scout_mature = true; in.scout_op_ranked = false; in.proposal = 144;
  auto o = b.tick(in);
  CHECK(frozen(o, "op unmeasured") && o.accept_op);
}

TEST(calibration_running_at_maturity_freezes_in_place) {
  BootPick b(true);
  BootPickIn in = base();
  in.cal_running = true;
  in.scout_mature = true; in.scout_op_ranked = true; in.proposal = 144;
  auto o = b.tick(in);
  CHECK(frozen(o, "calibration running") && o.accept_op);
}

// The boot hop is the relocation: WantPick sets want, the scout parks, one
// relocate order goes out, VerifyPass freezes "relocated".
TEST(linked_maturity_on_another_pair_relocates_and_freezes_relocated) {
  BootPick b(true);
  BootPickIn in = linked(base());
  in.scout_mature = true; in.scout_op_ranked = true; in.proposal = 144;
  auto o = b.tick(in);
  CHECK(is(o, BootPickOut::WantPick) && o.ch == 144);
  CHECK(b.open() && b.pick_wanted());
  in.want = 144; in.relocate_due = true;
  CHECK(is(b.tick(in), BootPickOut::None));      // the scout still owns its card (parking)
  in.scout_owns = false;
  o = b.tick(in);
  CHECK(is(o, BootPickOut::Relocate) && o.ch == 144);
  b.note_hop_action(HopAction::Order, /*relocate_tick=*/true);
  CHECK(b.relocating());
  in.hop_idle_or_hold = false; in.relocate_due = false; in.plan_hopping = true;
  CHECK(is(b.tick(in), BootPickOut::None));
  b.note_hop_action(HopAction::Confirm, false);
  in.plan_hopping = false; in.op = 144;
  CHECK(is(b.tick(in), BootPickOut::None));
  b.note_hop_action(HopAction::VerifyPass, false);
  in.hop_idle_or_hold = true;
  o = b.tick(in);
  CHECK(frozen(o, "relocated") && !o.accept_op);
  CHECK(!b.open() && !b.relocating());
}

static BootPick placed(BootPickIn& in) {
  BootPick b(true);
  in = linked(base());
  in.scout_mature = true; in.scout_op_ranked = true; in.proposal = 144;
  b.tick(in);                                     // WantPick
  in.want = 144; in.relocate_due = true; in.scout_owns = false;
  b.tick(in);                                     // Relocate
  b.note_hop_action(HopAction::Order, true);
  in.hop_idle_or_hold = false;
  b.tick(in);
  return b;
}

TEST(relocate_withdrawn_accepts_the_channel_the_link_is_on) {
  BootPickIn in;
  BootPick b = placed(in);
  b.note_hop_action(HopAction::Withdraw, false);
  in.hop_idle_or_hold = true;
  auto o = b.tick(in);
  CHECK(frozen(o, "relocate failed, staying") && o.accept_op);
}

// verify fail with nothing to retry: the controller holds.
TEST(relocate_verify_fail_hold_accepts_op) {
  BootPickIn in;
  BootPick b = placed(in);
  b.note_hop_action(HopAction::Confirm, false);
  b.note_hop_action(HopAction::Hold, false);
  in.hop_idle_or_hold = true;
  CHECK(frozen(b.tick(in), "relocate failed, staying"));
}

// Confirmed, then the session dropped mid-verify (Verifying -> Idle with no
// action): only a VerifyPass counts as landed.
TEST(relocate_cut_short_counts_as_failed) {
  BootPickIn in;
  BootPick b = placed(in);
  b.note_hop_action(HopAction::Confirm, false);
  in.hop_idle_or_hold = true; in.in_session = false;
  CHECK(frozen(b.tick(in), "relocate failed, staying"));
}

// The hop cap refuses the order on the spot (Hold on the relocate tick).
TEST(relocate_refused_by_the_hop_cap_fails_next_tick) {
  BootPick b(true);
  BootPickIn in = linked(base());
  in.scout_mature = true; in.scout_op_ranked = true; in.proposal = 144;
  b.tick(in);
  in.want = 144; in.relocate_due = true; in.scout_owns = false;
  CHECK(is(b.tick(in), BootPickOut::Relocate));
  b.note_hop_action(HopAction::Hold, true);
  CHECK(!b.relocating());
  CHECK(frozen(b.tick(in), "relocate failed, staying"));
}

TEST(link_lost_before_the_relocation_is_placed_keeps_the_pick) {
  BootPick b(true);
  BootPickIn in = linked(base());
  in.scout_mature = true; in.scout_op_ranked = true; in.proposal = 144;
  b.tick(in);                                     // WantPick
  in.in_session = false; in.hop_active = false; in.want = 144;
  auto o = b.tick(in);
  CHECK(frozen(o, "link lost before relocate") && !o.accept_op);   // want stays the pick
}

TEST(max_ms_freezes_unmeasured_but_waits_for_a_placed_relocation) {
  BootPick b(true);
  BootPickIn in = base();
  in.since_start_ms = 30000;
  auto o = b.tick(in);
  CHECK(frozen(o, "max_ms") && o.accept_op);
  BootPickIn in2;
  BootPick c = placed(in2);
  in2.since_start_ms = 30000;
  CHECK(is(c.tick(in2), BootPickOut::None));      // relocation in flight
  c.note_hop_action(HopAction::VerifyPass, false);
  in2.hop_idle_or_hold = true;
  CHECK(frozen(c.tick(in2), "relocated"));
}

TEST(scout_card_died_freezes) {
  BootPick b(true);
  BootPickIn in = base();
  in.scout_card_died = true;
  auto o = b.tick(in);
  CHECK(frozen(o, "scout card died") && !o.accept_op);
}

TEST(one_card_prelude_commits_once_only_when_unlinked) {
  BootPick b(true);
  BootPickIn in = base();
  in.one_card = true; in.scout_prelude_done = true; in.proposal = 112;
  auto o = b.tick(in);
  CHECK(is(o, BootPickOut::AckPrelude) && o.ch == 112);
  CHECK(is(b.tick(in), BootPickOut::None));       // once
  CHECK(b.open());                                // measuring continues between op windows
  BootPick c(true);
  BootPickIn l = linked(in);
  o = c.tick(l);
  CHECK(is(o, BootPickOut::AckPrelude) && o.ch == 0);   // linked: nothing committed
  BootPick d(true);
  in.proposal = 40;
  CHECK(is(d.tick(in), BootPickOut::AckPrelude) && d.tick(in).ch == 0);
}

// One card: the scout owns the only card while the pick is open, so the
// pick freezes on the link edge and the relocation (if due) follows.
TEST(one_card_link_edge_freezes_then_relocates) {
  BootPick b(true);
  BootPickIn in = linked(base());
  in.one_card = true; in.scout_prelude_done = true;
  b.tick(in);                                     // AckPrelude
  in.op = 112; in.want = 40; in.relocate_due = true;   // drone found on 112
  CHECK(is(b.tick(in), BootPickOut::None));       // scout owns, no edge yet
  in.link_edge = true;
  CHECK(frozen(b.tick(in), "one-card linked"));
  in.link_edge = false; in.scout_owns = false;    // the frozen scout parked
  auto o = b.tick(in);
  CHECK(is(o, BootPickOut::Relocate) && o.ch == 40);
}

// Pinned: no pick at all, but a drone found off the pin links there and is
// relocated to the pin on the link-up; a failure accepts op (no freeze --
// there is no pick to close).
TEST(pinned_found_off_pin_relocates_to_the_pin) {
  BootPick b(false);
  BootPickIn in = linked(base());
  in.scout_owns = false; in.op = 112; in.want = 40; in.relocate_due = true;
  auto o = b.tick(in);
  CHECK(is(o, BootPickOut::Relocate) && o.ch == 40);
  b.note_hop_action(HopAction::Order, true);
  in.hop_idle_or_hold = false;
  b.tick(in);
  b.note_hop_action(HopAction::Withdraw, false);
  in.hop_idle_or_hold = true;
  CHECK(is(b.tick(in), BootPickOut::AcceptOp));
  BootPick c(false);
  c.tick(in);
  c.note_hop_action(HopAction::Order, true);
  c.note_hop_action(HopAction::VerifyPass, false);
  CHECK(is(c.tick(in), BootPickOut::None));       // landed: nothing to do
}

// Once per link-up: a relocation that did not land is not retried in the
// same session even if the caller still reports it due; the next link-up
// re-arms it.
TEST(relocation_is_tried_once_per_link_up) {
  BootPick b(false);
  BootPickIn in = linked(base());
  in.scout_owns = false; in.op = 112; in.want = 40; in.relocate_due = true;
  CHECK(is(b.tick(in), BootPickOut::Relocate));
  b.note_hop_action(HopAction::Hold, true);
  CHECK(is(b.tick(in), BootPickOut::AcceptOp));
  CHECK(is(b.tick(in), BootPickOut::None));       // still "due": not again
  in.in_session = false; in.hop_active = false;
  b.tick(in);
  in.in_session = true; in.hop_active = true;
  CHECK(is(b.tick(in), BootPickOut::Relocate));
}

TEST(relocation_waits_for_the_cards_and_the_controller) {
  BootPickIn in = linked(base());
  in.op = 112; in.want = 40; in.relocate_due = true;
  BootPick b(false);
  CHECK(is(b.tick(in), BootPickOut::None));       // the scout owns a card
  in.scout_owns = false; in.plan_hopping = true;
  CHECK(is(b.tick(in), BootPickOut::None));       // a hop is in flight
  in.plan_hopping = false; in.hop_idle_or_hold = false;
  CHECK(is(b.tick(in), BootPickOut::None));       // the controller is busy
  in.hop_idle_or_hold = true; in.hop_active = false;
  CHECK(is(b.tick(in), BootPickOut::None));       // calibration run: no hops
  in.hop_active = true;
  CHECK(b.relocation_pending(in));
  CHECK(is(b.tick(in), BootPickOut::Relocate));
  CHECK(!b.relocation_pending(in));
}

MTEST_MAIN
