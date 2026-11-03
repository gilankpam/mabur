#pragma once
#include <optional>

#include "hop_controller.h"
#include "hop_verdict.h"

namespace maburgs {

// The existing post-transition settle-blank (LadderController's own
// 150 ms), spelled once so the two hop call sites agree.
constexpr double kHopSettleBlankMs = 150.0;

// Pure: how far ahead the ladder's rung store should be blanked, given the
// verdict window that just closed: the store's residual/util EWMAs are not
// updated from a jam's first blocked window until hop confirmation plus the
// existing 150 ms post-transition settle-blank.
//
// Extracted like hop_burst_gate.h so the one decision has a unit test
// instead of only ever running inside ChannelCore's hardware-touching
// verdict block. The call site (gs/src/channel_core.cpp, right after
// HopVerdict::window()) feeds the result to VrxController::blank_store(),
// which keeps the LATER of the deadlines it is given -- so this composes
// with, and never shortens, the deadline HopAction::Order sets.
//
// Three gates, each load-bearing:
//
//  - `reactive`: the reactive layer is armed (auto mode; ChannelCore passes
//    !pinned). Pinned, nothing is ever ordered and the rung is never
//    restored, so there is no hop for the store to be protected from --
//    blanking anyway would silently change what a pinned flight records.
//    HopAction::Order's own blank_store() call never fires there (the
//    controller is never fed a trigger), so this keeps the two consistent.
//
//  - VerdictOut::first_blocked: the first impaired, non-weak, `blocked`
//    window of the frozen episode, NOT ref_frozen and NOT the first
//    `interfered` window. ref_frozen is keyed on `impaired` alone, and a
//    fade (impaired AND weak) or an unknown window with no blocked reading
//    is impaired too -- keying on it blanked the store through every fade
//    and every unexplained loss, which must leave the ladder alone. The
//    first `interfered` window is too late: a jam classifies interfered
//    only occupied_windows windows in (hop.verdict.occupied_windows), and
//    the lossy windows before it -- already blocked -- are exactly where
//    the ladder takes the "demote or two, each an IDR" that
//    docs/inflight-channel-hop.md §4 accepts; arming at interfered wrote
//    them all into the per-rung store against the jammed channel. A zero-loss blocked window is not
//    impaired, so it arms nothing: there is no loss to demote on.
//
//  - One edge per frozen episode, which is what BOUNDS the blank. The
//    deadline is computed once, at the episode's first blocked window,
//    and spans exactly confirm_ms + the settle -- the same span the
//    Order-time call uses, so detection and the hop itself are scoped
//    alike. It is not re-extended by later blocked or interfered windows,
//    so a jam that runs for seconds (or alternates blocked and healthy
//    windows inside one frozen episode) cannot roll it forward
//    indefinitely; the store resumes writing confirm_ms + 150 ms after the
//    edge whether or not a hop was ever ordered (the order, when it comes,
//    sets its own deadline). Re-arming needs a genuine thaw: 3 consecutive
//    healthy windows, or HopVerdict::reset() after a verify window ends.
inline std::optional<double> hop_store_blank_until(const VerdictOut& vo, bool reactive,
                                                   int confirm_ms) {
  if (!reactive || !vo.first_blocked) return std::nullopt;
  return vo.t_ms + static_cast<double>(confirm_ms) + kHopSettleBlankMs;
}

// Pure: when the hop verdict's own link-loss window (a 500 ms trailing
// S1LossWindow, ChannelCore::s1_hop_loss_) must be blanked. At Confirm -- the
// instant the link lands on the new channel -- plus the same settle as the
// store blank, so the verify that follows is judged on loss measured on the
// channel it is verifying. Unblanked, the first eligible verify window still
// carried the jam on the old channel and the hop's own retune gap: 128 read
// 50.9 % loss with zero foreign frames and failed (bench, GS session
// 0207), which sent the retry back into the jam. The verdict gets
// its own window rather than blanking s1_loss, which also feeds the
// sideport/ctl-log/OSD loss gauge the pilot reads. Every other action keeps
// the link where it was (Order/OneCardRetune: the confirm has not happened;
// Withdraw: the trailing card never left op), so it blanks nothing.
inline std::optional<double> hop_verdict_loss_blank_until(const HopAction& act, double now_ms) {
  if (act.kind != HopAction::Confirm) return std::nullopt;
  return now_ms + kHopSettleBlankMs;
}

}  // namespace maburgs
