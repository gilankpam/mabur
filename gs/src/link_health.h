#pragma once
// LinkHealthAssembler — the ladder's per-tick input (LinkHealth), assembled
// from the Aggregator's decoder counters, its per-card RF pools and the
// probe stream. Extracted verbatim from maburgs main.cpp's control loop
// 2026-09-27 (spec 2026-09-27-web-gs) so the web GS feeds the SAME windows
// the same way and its ladder behaves exactly like maburgs's.
//
// Call order per core-loop iteration, as main.cpp does it:
//   tick(now_ms, agg, inputs)  -- after the drain + fstream.poll
//   ctl.step(now_ms, health)   -- the caller's controller
//   on_step_sent(agg)          -- ONLY after a non-DISC step: it is the
//                                 window boundary the starvation gate and
//                                 the RF staleness gate diff against.
// on_au_begin / on_probe_body are the FrameStream begin and the
// Aggregator probe-sink hooks. Single-threaded (core loop).
//
// s1_hop_loss (the hop verdict's window) and the fec.log episode drain stay
// with maburgs: they read the same decoder counters but are not ladder
// inputs.
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "aggregator.h"
#include "ladder_controller.h"
#include "mabur/node.h"
#include "mabur/rc_proto.h"
#include "op_point.h"
#include "probe_track.h"
#include "rf_labels.h"
#include "s1_loss.h"
#include "transition_edge.h"

namespace maburgs {

struct LinkHealthCfg {
  int n_cards = 1;
  int probe_bpb = 4;             // cfg.uep_layers()[1].blocks_per_body
  int probe_block_payload = 0;   // kSwHeaderLen + uep_layers()[1].fec.symbol_size
  std::vector<bool> snr_ok;      // per card; empty = all true
};

struct LinkHealthInputs {        // the commanded state this tick
  OpPoint op;
  uint8_t probe_profile = mabur::rc::kNoProbeProfile;
  int probe_rung = -1;
};

class LinkHealthAssembler {
 public:
  // A commanded-profile change blanks the windows: bodies already in flight
  // carry the OLD profile and ProbeTrack stops scoring them, so the window
  // must refill from the new profile's bodies only (RCF lag + finalize).
  static constexpr double kProbeSwitchBlankMs = 150.0;

  explicit LinkHealthAssembler(const LinkHealthCfg& cfg);

  // FrameStream begin_frame: one probe expectation per video AU.
  void on_au_begin(uint8_t sid, uint16_t frame_id, double now_ms);
  // Aggregator probe sink (SBI stream 5), minus the slotter's
  // on_probe_tail() release, which stays with the caller.
  void on_probe_body(uint8_t card, const mabur::node::RxBody& m);

  struct Tick {
    LinkHealth health;
    std::optional<int> probe_tail_ms;   // set only on a commanded-probe edge
  };
  Tick tick(double now_ms, Aggregator& agg, const LinkHealthInputs& in);
  // After a NON-DISC step(): the feedback window boundary. Aggregator& (not
  // const) because Aggregator::decoder() has no const overload.
  void on_step_sent(Aggregator& agg);

  // read-only views (all reflect the last tick())
  uint8_t probe_commanded() const { return probe_cmd_last_; }
  std::optional<double> residual() const { return residual_; }          // pool_resid
  std::optional<double> residual_cur() const { return residual_cur_; }  // pool_resid_cur
  S1LossWindow::Sample pre_all() const { return pre_all_; }             // pre_loss_all
  std::array<uint8_t, 2> layer_delivery_pct() const { return ld_; }
  const LinkHealth& last_health() const { return last_health_; }
  const ProbeTrack& probe_track() const { return probe_track_; }
  S1LossWindow::Sample probe_sample() const { return probe_sample_; }
  uint64_t probe_expected_in_window(double now_ms) const {
    return probe_loss_.expected_in_window(now_ms);
  }
  S1LossWindow::Sample probe_card_sample(int card, double now_ms) const {
    return probe_card_loss_[static_cast<size_t>(card)].sample(now_ms);
  }
  // THIS tick's finalized probe rows only (ProbeTrack is drained every tick).
  const std::vector<ProbeFinalized>& probe_finalized() const { return probe_rows_; }

 private:
  bool snr_ok(int card) const {
    return cfg_.snr_ok.empty() || cfg_.snr_ok[static_cast<size_t>(card)];
  }

  LinkHealthCfg cfg_;
  // Probe stream (spec 2026-09-04 section 3): scored by ProbeTrack against
  // the enh AU count; the ENH layer's geometry gives bpb/block_payload, so
  // the same array the Aggregator was built from decides how a probe body
  // is parsed. Core-thread-owned like every other window in this loop.
  ProbeTrack probe_track_;
  S1LossWindow probe_loss_;  // union, commanded profile
  std::vector<S1LossWindow> probe_card_loss_;
  uint8_t probe_cmd_last_ = mabur::rc::kNoProbeProfile;
  std::vector<ProbeFinalized> probe_rows_;
  // Measured-loss ladder feedback: stream 1 (base layer)'s cumulative
  // (expected, arrived) symbol totals, pre-FEC-repair. expected = source
  // symbols ever seen by seq framing (delivered directly + recovered by FEC
  // + abandoned as unrecoverable); arrived = delivered directly PLUS
  // recovered symbols whose direct copy landed afterwards
  // (syms_recovered_arrived) — a repair that merely wins an arrival race is
  // not channel loss. Without that term, rung 0's 2x parity read a clean
  // bench as 19-26% pre-FEC loss and pinned u above up_util forever (stuck
  // at mcs0, 2026-07-27). This is the PRE-FEC window; the post-FEC residual
  // below is a different question over the same symbol counters (what FEC
  // could not repair at all) -- see gs/src/ladder_residual.cpp.
  //   s1_loss_
  // Sideport/OSD gauge (2026-09-23): BOTH video layers' current-only
  // arrival-tracker counts pooled in one window, so the LOSS row reads the
  // whole downlink; the ladder keeps deciding on s1_loss_cur (base only).
  //   pre_loss_all_
  // s3 probe-before-promote feedback (same windowing machinery as s1_loss,
  // stream 3): pre-FEC loss for probe/s3-demote decisions. The steady-state
  // demote path's residual (abandoned/expected) window is s3_resid_cur
  // below -- attribution is unconditional, so there is no total-based
  // sibling left to keep here.
  //   s3_loss_
  S1LossWindow s1_loss_, pre_loss_all_, s3_loss_;
  // Current-rung-only siblings (transition attribution, spec 2026-08-14):
  // same machinery, fed from the attributed counters. s1_loss/s3_loss stay
  // as the observability totals (residual_loss, the artifact-rate meter).
  S1LossWindow s1_loss_cur_, s1_resid_cur_, s3_loss_cur_, s3_resid_cur_;
  // Post-transition settle for s1_resid_cur (the block-4 instant-demote
  // input; see the blank_until call at the sid-0 transition edge-detect).
  // Budget: one 50 ms edge-detect tick + the ~80 ms abandonment-horizon
  // booking lag + margin. Deliberately half of s3_settle_ms: a genuine
  // continuing fade then steps ~200 ms/rung, inside the ~410-440 ms/rung
  // cadence flight-validated 2026-08-14, and nowhere near the broken
  // 50 ms/rung debris cascade this exists to stop.
  // (settle constant now lives in TransitionEdge::kResidSettleMs)
  // Observability siblings of s1_resid_cur, pooled over both layers: the
  // sideport's link.residual_loss / link.attrib.residual_cur and the ctl
  // log's S-line resid/resid_cur. These MUST be windowed, not cumulative --
  // the decoder's abandonment counters are monotonic since boot, and a
  // lifetime average never returns to zero, which would break
  // flightreport.py's residual-episode detection and turn the player OSD's
  // post-loss row into a number that barely moves during a real burst.
  S1LossWindow pool_resid_, pool_resid_cur_;
  // Per-layer delivery percent (sideport link.layer_delivery_pct), windowed
  // for the same reason.
  std::array<S1LossWindow, 2> layer_resid_;
  // Fade-trigger staleness gate (spec 2026-08-14 §3, source repointed to
  // the s1+s3 pool 2026-08-15): per-card pooled frame counts snapshotted
  // once per feedback window (same cadence as the prev_pkts_out snapshot
  // below), so "zero
  // s1+s3 frames this window" can NaN the RF labels before they reach the
  // controller. A frozen EMA must never read as a live signal. Snapshotted
  // for EVERY card, not just the chosen one: the choice itself is
  // freshness-gated, so which card is chosen can change between windows
  // and each needs its own baseline.
  std::vector<uint64_t> prev_pool_frames_;
  // Scratch for select_label_card(), hoisted so the per-window refill
  // allocates nothing after the first pass.
  std::vector<CardLabelInput> label_card_inputs_;
  // Completed-packet high-water mark at the controller's last non-disc step;
  // the starvation gate diffs against it (see the control step below).
  uint64_t prev_pkts_out_ = 0;
  // This tick's completed-packet count: main.cpp snapshotted exactly this
  // value (computed before step()) into prev_pkts_out, so on_step_sent()
  // copies it rather than re-reading the decoder.
  uint64_t pkts_now_ = 0;
  // Commanded-MCS edge detect for boundary marking. -1 forces a first mark
  // (which finds cur_mcs unknown -> closed plain-fallback boundary).
  TransitionEdge edge_;
  // last-tick views
  std::optional<double> residual_, residual_cur_;
  S1LossWindow::Sample pre_all_{false, 0.0}, probe_sample_{false, 0.0};
  std::array<uint8_t, 2> ld_{};
  LinkHealth last_health_;
};

}  // namespace maburgs
