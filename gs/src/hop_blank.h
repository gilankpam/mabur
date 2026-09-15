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
// which keeps the LATER of the deadlines it is given -- so this rolling
// per-window extension composes with, and never shortens, the longer
// deadline HopAction::Order sets (confirm_ms + the same settle).
//
// Keyed on VerdictOut::ref_frozen, which is exactly "an impaired episode
// is open": it turns on at the first impaired window (the instant the
// spec names) and off when the references thaw -- 3 healthy windows, or
// HopVerdict::reset() after a hop's verify window ends. Before this, the
// blank started at the ORDER, which is 2-3 persistence windows later, so
// every detection window -- including the demotes the spec explicitly
// expects, "a demote or two, each an IDR" -- was written into the
// per-rung store against the interfered channel.
//
// window_ms of lead means the blank never lapses between windows; the
// settle constant is what carries it past the last impaired window.
inline std::optional<double> hop_store_blank_until(const VerdictOut& vo, int window_ms) {
  if (!vo.ref_frozen) return std::nullopt;
  return vo.t_ms + static_cast<double>(window_ms) + kHopSettleBlankMs;
}

}  // namespace maburgs
