#pragma once
#include <cstdint>

#include "hop_controller.h"

namespace maburgs {

// What the boot pick needs from the rest of the core loop this tick. Plain
// values only: main.cpp fills it from ChannelScout / ChannelPlan /
// HopController / VrxController, so every exit below is unit-testable.
struct BootPickIn {
  double now_ms = 0;
  uint64_t since_start_ms = 0;   // since GS start
  int max_ms = 30000;            // radio.scan.max_ms
  bool in_session = false;
  bool cal_running = false;      // a calibration run counts as linked
  bool one_card = false;
  // ChannelScout
  bool scout_mature = false;
  bool scout_op_ranked = false;
  bool scout_prelude_done = false;
  bool scout_owns = false;       // main.cpp's scout_owns(): the scout holds its card
  uint8_t proposal = 0;
  // ChannelPlan
  uint8_t op = 0;
  uint8_t want = 0;
  bool relocate_due = false;     // in session, no hop in flight, op != want
  bool plan_hopping = false;
  // HopController / the hop gate
  bool hop_active = false;       // hop_active(): in session, no calibration run
  bool hop_idle_or_hold = true;  // controller state Idle or Hold
  // Edges seen this tick
  bool link_edge = false;        // the drone's move edge was acted on (one card: linked)
  bool scout_card_died = false;  // the scout card died while working
};

// One decision per tick. ch / reason / accept_op qualify the kind:
//  - Commit:     plan.commit(ch), then close the pick with `reason`.
//  - AckPrelude: one card: plan.commit(ch) when ch != 0, then
//                scout->ack_prelude(plan.op()). The pick stays open.
//  - WantPick:   linked at maturity on another pair: plan.set_want(ch) and
//                scout->freeze() (it parks and frees the lead card); the
//                relocation below then moves the link -- this IS the boot hop.
//  - Relocate:   place ONE relocate order toward ch (= want) this tick
//                (HopTick::relocate, best = ch, synthetic trigger), then
//                report what the controller did through note_hop_action().
//  - Freeze:     close the pick with `reason`; accept_op: plan.set_want(op)
//                first (stay where the link is).
//  - AcceptOp:   plan.set_want(op) only (a relocation failed after the pick
//                had already closed).
struct BootPickOut {
  enum Kind { None, Commit, AckPrelude, WantPick, Relocate, Freeze, AcceptOp } kind = None;
  uint8_t ch = 0;
  const char* reason = nullptr;
  bool accept_op = false;
};

// The boot pick (docs/channel-select.md "The pick") and the relocation that
// moves the link to ChannelPlan::want() (final review C1/I3, 2026-10-04).
//
// The pick is open from start (auto mode) until it freezes, exactly once:
// "commit", "in place", "op unmeasured", "calibration running", "relocated",
// "relocate failed, staying", "link lost before relocate", "one-card
// linked", "scout card died", "max_ms".
//
// The relocation runs for the whole process: on every link-up (and when
// the pick wants another pair) where the link is not where it should be
// (relocate_due), one relocate order toward want, placed only once the
// scout owns no card, no hop is in flight and the controller is Idle/Hold.
// VerifyPass: done. Anything else (verify fail, withdraw, a session lost
// before or after the confirm, the hop cap): accept the channel the link is
// on (set_want(op)) -- no retry until the next link-up.
//
// Pure: no I/O, the caller's clock.
class BootPick {
 public:
  explicit BootPick(bool pick_open) : open_(pick_open) {}

  BootPickOut tick(const BootPickIn& in);
  // Every HopAction the controller returned this tick (both the tick's own
  // and the session-lost one). relocate_tick: this is the tick a Relocate
  // was handed to the controller.
  void note_hop_action(HopAction::Kind kind, bool relocate_tick);

  bool open() const { return open_; }
  // A relocate order is in flight (placed, not yet resolved): the caller
  // keeps HopTick::best/escape empty so a verify fail holds instead of
  // wandering, and places no reactive order.
  bool relocating() const { return relocating_; }
  // A relocation is due and has not been tried since this link-up: no
  // reactive order may take the controller first.
  bool relocation_pending(const BootPickIn& in) const {
    return in.in_session && in.relocate_due && !tried_ && !relocating_;
  }
  bool pick_wanted() const { return pick_wanted_; }   // WantPick issued, relocation not resolved

 private:
  BootPickOut freeze(const char* why, bool accept_op);

  bool open_;
  bool one_card_committed_ = false;
  bool one_card_linked_ = false;  // one card: a link edge was seen (latched)
  bool pick_wanted_ = false;
  bool was_session_ = false;
  bool tried_ = false;          // a Relocate was handed out since this link-up
  bool relocating_ = false;     // the controller took it (Order)
  bool resolve_now_ = false;    // the controller refused it on the spot (Hold: the hop cap)
  bool landed_ = false;         // VerifyPass seen for the in-flight relocation
};

}  // namespace maburgs
