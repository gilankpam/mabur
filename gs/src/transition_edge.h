#pragma once
// Rung-transition edge-detect for loss attribution, extracted from the
// control step in gs/src/main.cpp (2026-09-05) so the wiring is testable:
// the dry-run e2e never runs the ladder, so before this nothing on the
// host could pin which decision windows get settle-blanked at an edge.
//
// On a COMMANDED op change this (1) marks the decoder's per-sid transition
// watermark and (2) settle-blanks the two residual decision windows; the
// util windows stopped being blanked on 2026-09-05 when they moved to the
// ArrivalTracker (bench ctl-0287's s3_util re-fire was completion-counter
// debris that the tracker never books against the new rung) -- the base
// (s1) residual window since 2026-09-02 (flight ctl-0160: one residual
// event walked the ladder to rung 0 at 50 ms/rung), and the enh (s3)
// residual window since 2026-09-05 (flights 20/21: 13 of 14 s3_residual
// cascades took a second step at exactly s3_settle_ms and were promoted
// straight back). The controller's s3_blank_until only gates READING for
// s3_settle_ms; the 500 ms window behind it kept the abandonment-horizon's
// ~80 ms late booking of old-rung loss, so the tick the gate opened it
// demoted again on a 0 ms confirm. Observability windows (pool_resid*,
// s1_loss, s3_loss) stay untouched -- they report, they don't decide.
#include "mabur/profile.h"
#include "mabur/uep_decoder.h"
#include "op_point.h"
#include "s1_loss.h"

namespace maburgs {

struct TransitionEdge {
  // Post-transition settle for the instant-demote windows: one 50 ms
  // edge-detect tick + the ~80 ms abandonment-horizon booking lag + margin.
  // Deliberately half of s3_settle_ms (300): a genuine continuing fade then
  // steps ~200 ms/rung on s1, and s3 still reads only after its own gate,
  // by which time the window holds nothing older than settle_ms.
  static constexpr double kResidSettleMs = 150.0;

  int last_op_mcs = -1;
  double last_op_ov = -1.0;
  int last_enh_mcs = -1;

  // Returns true when any edge fired this tick. Only the two RESIDUAL
  // (post-FEC) windows are settle-blanked: abandonment books ~80 ms late.
  // The util (pre-FEC) windows read the ArrivalTracker (2026-09-05 spec),
  // which classifies every missing symbol by the watermark at booking
  // time, so they carry no transition debris and are not blanked.
  bool on_tick(const OpPoint& op, mabur::UepDecoder& dec,
               S1LossWindow& s1_resid_cur, S1LossWindow& s3_resid_cur,
               double now_ms) {
    bool fired = false;
    // sid 0 (base) mirrors the drone's mcs-1 rule (rc::ladder_from(...)[0]
    // .mcs) and always tracks the op. Overhead-only steps mark sid 0 too
    // (FEC re-key debris exists without a PHY change; the decoder then uses
    // the plain same-MCS fallback). Static-pin mode: nothing ever arms.
    if (op.mcs != last_op_mcs || op.overhead_base != last_op_ov) {
      const auto base_spec = mabur::rc::ladder_from(
          op.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
          static_cast<uint8_t>(op.mcs), static_cast<uint8_t>(op.bw))[0];
      dec.mark_transition(0, static_cast<uint8_t>(base_spec.mcs),
                          static_cast<uint64_t>(now_ms));
      s1_resid_cur.blank_until(now_ms + kResidSettleMs);
      last_op_mcs = op.mcs;
      last_op_ov = op.overhead_base;
      fired = true;
    }
    // sid 1 (enh) runs the op MCS too -- since the continuous probe stream
    // replaced the discrete probe attempt (2026-09-04) the enh layer is
    // never diverted to a candidate rate.
    if (op.mcs != last_enh_mcs) {
      dec.mark_transition(1, static_cast<uint8_t>(op.mcs),
                          static_cast<uint64_t>(now_ms));
      s3_resid_cur.blank_until(now_ms + kResidSettleMs);
      last_enh_mcs = op.mcs;
      fired = true;
    }
    return fired;
  }
};

}  // namespace maburgs
