#pragma once
#include <algorithm>
#include <optional>

#include "hop_controller.h"
#include "hop_verdict.h"
#include "s1_loss.h"

namespace maburgs {

// The existing post-transition settle-blank (LadderController's own
// 150 ms), spelled once so the two hop call sites agree.
constexpr double kHopSettleBlankMs = 150.0;

// Pure: how far ahead the ladder's rung store should be blanked, given the
// verdict window that just closed: the store's residual/util EWMAs are not
// updated from an interferer's lead-in window until hop confirmation plus
// the existing 150 ms post-transition settle-blank.
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
//  - VerdictOut::first_blocked: the first impaired, non-weak window of the
//    frozen episode that is the lead-in of either route to `interfered` --
//    `blocked` (the occupied route), or loss-impaired with its own foreign
//    reading at or above hop.verdict.detect_pct (the earliest window a
//    detect run can start from), or `detected` itself (a detect run whose
//    loss window read under detect_pct has no earlier lead-in, so its
//    first detected window arms). NOT ref_frozen and NOT, in general, the
//    first `interfered` window. ref_frozen is keyed on `impaired` alone,
//    and a fade (impaired AND weak) or a lossy window with no foreign
//    reading behind it is impaired too -- keying on it blanked the store
//    through every fade and every unexplained loss, which must leave the
//    ladder alone. The first `interfered` window is too late: either route
//    classifies interfered only windows after its lead-in (the blocked
//    streak reaching hop.verdict.occupied_windows, the detect ring
//    filling), and the lossy windows before it are exactly where the
//    ladder takes the "demote or two, each an IDR" that
//    docs/inflight-channel-hop.md §4 accepts; arming at interfered wrote
//    them all into the per-rung store against the jammed channel. A
//    zero-loss blocked window is not impaired, so it arms nothing: there
//    is no loss to demote on.
//
//  - One edge per frozen episode, which is what BOUNDS the blank. The
//    deadline is computed once, at the episode's lead-in window,
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

// Pure: the pre-FEC loss the verdict confirms on -- the worse of the base and
// enhancement layers' own trailing windows. The ladder demotes on loss in
// either layer, and UEP gives the enhancement layer the thinner FEC, so a
// jam often shows there first; a base-only reading stays clean while the
// ladder sinks. A layer whose window is not valid (blanked after a landing,
// or a rung without an enhancement stream) contributes nothing.
inline double hop_verdict_link_loss(S1LossWindow::Sample base, S1LossWindow::Sample enh) {
  double loss = 0.0;
  if (base.valid) loss = std::max(loss, base.loss);
  if (enh.valid) loss = std::max(loss, enh.loss);
  return loss;
}

}  // namespace maburgs
