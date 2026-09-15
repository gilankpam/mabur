#pragma once
#include <optional>

#include "hop_verdict.h"

namespace maburgs {

// The existing post-transition settle-blank (LadderController's own
// 150 ms), spelled once so the two hop call sites agree.
constexpr double kHopSettleBlankMs = 150.0;

// Pure: how far ahead the ladder's rung store should be blanked, given the
// verdict window that just closed (spec section 4: the store's residual/
// util EWMAs are not updated "from the first impaired window until hop
// confirmation plus the existing 150 ms post-transition settle-blank").
//
// Extracted like hop_burst_gate.h so the one decision has a unit test
// instead of only ever running inside run_radio()'s hardware-touching
// verdict block. The call site (gs/src/main.cpp, right after
// HopVerdict::window()) feeds the result to VrxController::blank_store(),
// which keeps the LATER of the deadlines it is given -- so this composes
// with, and never shortens, the deadline HopAction::Order sets.
//
// Three gates, each load-bearing:
//
//  - `enable`. With hop.enable = false nothing is ever ordered and the
//    rung is never restored, so there is no hop for the store to be
//    protected from -- and blanking anyway would silently change what the
//    observe-only flights record versus every recording made before this
//    branch, with nothing in the log marking it. That is exactly the
//    pre/post divergence docs/data-provenance.md exists to prevent, on the
//    flights this branch exists to produce. HopAction::Order's own
//    blank_store() call is already dead while disabled (tick() zeroes the
//    action), so this keeps the two consistent.
//
//  - VerdictOut::first_interfered, NOT ref_frozen. ref_frozen is keyed on
//    `impaired`, and Fade (impaired AND weak) and Unknown (impaired
//    otherwise) are impaired too -- so keying on it blanked the store
//    through every fade and every unknown window, while section 4's last
//    bullet is "fade/unknown: unchanged ladder behaviour". Only
//    interference hops, so only interference blanks.
//
//  - One edge per frozen episode, which is what BOUNDS the blank. The
//    deadline is computed once, at the episode's first interfered window,
//    and spans exactly confirm_ms + the settle -- the same span the
//    Order-time call uses, so detection and the hop itself are scoped
//    alike. It is not re-extended by later interfered windows, so a jam
//    that runs for seconds (or alternates interfered and healthy windows
//    inside one frozen episode) cannot roll it forward indefinitely; the
//    store resumes writing confirm_ms + 150 ms after onset whether or not
//    a hop was ever ordered. Re-arming needs a genuine thaw: 3 consecutive
//    healthy windows, or HopVerdict::reset() after a verify window ends.
//
// Starting at the first interfered window rather than at the order is the
// point: the order is 2-3 persistence windows (300-450 ms) later, so every
// detection window -- including the demotes section 4 explicitly expects,
// "a demote or two, each an IDR" -- was being written into the per-rung
// store against the interfered channel.
inline std::optional<double> hop_store_blank_until(const VerdictOut& vo, bool enable,
                                                   int confirm_ms) {
  if (!enable || !vo.first_interfered) return std::nullopt;
  return vo.t_ms + static_cast<double>(confirm_ms) + kHopSettleBlankMs;
}

}  // namespace maburgs
