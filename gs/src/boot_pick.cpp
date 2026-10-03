#include "boot_pick.h"

namespace maburgs {

BootPickOut BootPick::freeze(const char* why, bool accept_op) {
  open_ = false;
  pick_wanted_ = false;
  BootPickOut o;
  o.kind = BootPickOut::Freeze;
  o.reason = why;
  o.accept_op = accept_op;
  return o;
}

BootPickOut BootPick::tick(const BootPickIn& in) {
  // Every link-up re-arms the relocation ("once per link-up").
  if (in.in_session && !was_session_) tried_ = false;
  was_session_ = in.in_session;
  if (in.one_card && in.link_edge) one_card_linked_ = true;
  const bool linked = in.in_session || in.cal_running;

  // 1. A relocation resolved: the controller is back in Idle/Hold.
  if ((relocating_ || resolve_now_) && in.hop_idle_or_hold) {
    const bool ok = relocating_ && landed_;
    relocating_ = resolve_now_ = landed_ = false;
    if (open_) return freeze(ok ? "relocated" : "relocate failed, staying", !ok);
    pick_wanted_ = false;
    if (!ok) {
      BootPickOut o;
      o.kind = BootPickOut::AcceptOp;
      return o;
    }
  }

  // 2. The pick. Decisions are latched: mature()/proposal() are read only
  // until one is taken.
  if (open_) {
    if (in.scout_card_died) return freeze("scout card died", false);
    if (in.one_card && in.scout_prelude_done && !one_card_committed_) {
      // One card: the prelude ranking commits BEFORE the first DISC; the
      // scout holds its first op window until ack_prelude().
      one_card_committed_ = true;
      BootPickOut o;
      o.kind = BootPickOut::AckPrelude;
      o.ch = (!linked && in.proposal != in.op) ? in.proposal : 0;
      return o;
    }
    // One card: the sole card carries the link and cannot measure. The
    // scout owns that card while the pick is open, so the freeze has to
    // come first -- the relocation (if any) follows once it has parked.
    if (one_card_linked_) return freeze("one-card linked", false);
    if (in.since_start_ms >= static_cast<uint64_t>(in.max_ms)) {
      // A placed relocation is never yanked mid-flight: it freezes the pick
      // when it resolves (1. above). Unmeasured, the link stays where it is.
      if (!relocating_) return freeze("max_ms", !pick_wanted_);
    } else if (pick_wanted_ && !in.in_session && !relocating_) {
      // The link dropped before the relocation could be placed: the pick
      // stands (want), the next link-up relocates there.
      return freeze("link lost before relocate", false);
    } else if (!in.one_card && !pick_wanted_ && in.scout_mature) {
      // Final review I1: a linked scout never measures op's own pair, so a
      // drone found at once leaves op with only its pre-link visits. A
      // working link is not moved on a one-sided comparison.
      if (!in.scout_op_ranked) return freeze("op unmeasured", true);
      if (!linked) {
        BootPickOut o;
        o.kind = BootPickOut::Commit;
        o.ch = in.proposal;
        o.reason = "commit";
        open_ = false;
        return o;
      }
      if (in.proposal == in.op) return freeze("in place", true);
      if (in.cal_running) return freeze("calibration running", true);
      pick_wanted_ = true;
      tried_ = false;
      BootPickOut o;
      o.kind = BootPickOut::WantPick;
      o.ch = in.proposal;
      return o;
    }
  }

  // 3. The relocation: once per link-up, when the link is not where it
  // should be and nothing else holds the cards or the controller.
  if (in.in_session && in.relocate_due && !tried_ && !relocating_ && !resolve_now_ &&
      in.hop_active && !in.scout_owns && !in.plan_hopping && in.hop_idle_or_hold) {
    tried_ = true;
    BootPickOut o;
    o.kind = BootPickOut::Relocate;
    o.ch = in.want;
    return o;
  }
  return BootPickOut{};
}

void BootPick::note_hop_action(HopAction::Kind kind, bool relocate_tick) {
  if (relocate_tick) {
    // The controller either took the order or held on the spot (the hop
    // cap): the latter resolves as a failure on the next tick.
    if (kind == HopAction::Order) {
      relocating_ = true;
      landed_ = false;
    } else {
      resolve_now_ = true;
    }
    return;
  }
  if (!relocating_) return;
  switch (kind) {
    case HopAction::VerifyPass:
      landed_ = true;
      break;
    case HopAction::Order:      // a verify-fail retry: a fresh attempt
    case HopAction::Confirm:    // landed; only a VerifyPass counts
    case HopAction::Withdraw:
    case HopAction::Hold:
      landed_ = false;
      break;
    case HopAction::OneCardRetune:
    case HopAction::None:
      break;
  }
}

}  // namespace maburgs
