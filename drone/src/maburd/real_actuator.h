#pragma once
// RealActuator: bridges RcAgent's Actuator interface to the radio, the UEP
// encoder handoff and the in-process venc core, plus the AppliedOp helpers
// the hot loop calls.
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <vector>

#include "air_clock.h"
#include "air_rate.h"
#include "ampdu_policy.h"
#include "live_power_table.h"
#include "radio_glue.h"
#include "radio_tx.h"
#include "rc_agent.h"
#include "vtx_recorder.h"
#include "mabur/uep_encoder.h"

#include "AmpduMode.h"
#include "IRtlRadio.h"


namespace mabur::maburd {

using namespace mabur;

// RealActuator — bridges RcAgent to the radio/UEP/encoder world.
// ---------------------------------------------------------------------------

// The recorder's encoder side: drone/venc/venc_record.c on the drone, a
// channel that is never there on host builds (the recorder then reports
// Disabled on a start, which a host build can never be asked for anyway).
struct VencRecordChannel : mabur::RecordChannel {
  bool start() override;
  void stop() override;
  void request_idr() override;
};

// RealActuator bridges RcAgent's Actuator interface to the radio (RadioTx +
// FrameSink), the hot-thread-owned UepEncoder (via the shared_op handoff),
// and the in-process encoder (venc_core.h's verbs, called directly — the
// HTTP control plane and its WaybeamClient went away with the fold-in).
//
// Its only TX-power duty is the per-width table: when an op moves the rung
// to the other width, the plan for that width goes live (LivePowerTable).
// There is no per-op power step, no thermal derate, no global offset move.
//
// Threading: apply_op()/send_control()/set_bitrate_kbps()/set_roi_qp()/
// request_idr() are all called from the agent thread only (RcAgent's
// contract). apply_op() publishes the new AppliedOp into shared_op via an
// atomic store of a fresh shared_ptr — the hot thread picks it up with an
// atomic load, so there is no lock and no torn read.
struct RealActuator : mabur::Actuator {
  mabur::RadioTx* tx = nullptr;
  mabur::FrameSink* sink = nullptr;
  std::atomic<std::shared_ptr<const mabur::AppliedOp>>* shared_op = nullptr;
  // Set after every accepted set_fps; the hot thread consumes it to
  // re-anchor FramePipeline's vanish period (note_rate_change).
  std::atomic<bool>* rate_change = nullptr;
  IRtlRadio* dev = nullptr;  // nullptr in dry-run
  bool dry_run = false;
  // Per-rung A-MPDU (ampdu_policy.h): the aggregation mode follows the
  // op's MCS. Programmed here, on the agent thread, only when the derived
  // mode differs from what the chip holds — SetAmpduMode is one 0x455
  // register write plus the per-frame descriptor state, live on this
  // family, and the chip's post-InitWrite state is "singles" (enabled
  // false), which is what `last_ampdu` starts as. Nothing at bring-up
  // programs the mode any more; the first apply_op (BOOT -> MAX_RANGE,
  // rung 0) decides it. Agent thread only, like everything else here.
  AmpduCfg ampdu;
  devourer::AmpduMode last_ampdu;
  bool ampdu_logged = false;  // the first op logs its state even without a write
  // False once calibration has programmed the mode itself (no aggregation
  // for power/verify frames, the production mode per capacity point): the
  // chip then holds something last_ampdu does not describe, so the next op
  // writes its mode unconditionally. Cleared by on_cal_end().
  bool ampdu_known = true;

  void apply_ampdu_for(const AppliedOp& op);
  // Null in dry-run and in run_dry_run's own RealActuator (calibration is
  // real-mode only), set to run_real_mode's cal_active once it exists. Gates
  // the set_ladder() call below -- see that call site's comment for why a
  // second writer during a sweep is a measurement bug, not a memory-safety
  // one.
  std::atomic<bool>* cal_active = nullptr;
  // The per-width TX-power table (null in dry-run): apply_op makes the
  // op's width's plan live.
  LivePowerTable* power_table = nullptr;
  // For the IGI floor watchdog (null in dry-run): the width the video layers
  // air at (both layers share it), and a count of retunes, each of which makes
  // the window it lands in unjudged.
  std::atomic<uint8_t>* video_bw = nullptr;
  std::atomic<uint32_t>* retune_seq = nullptr;

  // Agent thread, on the falling edge of cal_active, before the op is
  // re-applied: calibration programmed the A-MPDU mode behind last_ampdu's
  // back.
  void on_cal_end() { ampdu_known = false; }

  ControlRadiotap control_rt;  // agent thread only, like send_control
  uint16_t control_seq = 0;

  // Last values commanded to the encoder — read by the telemetry collector
  // (agent thread only; RcAgent's contract calls these setters from the
  // agent thread exclusively, same thread the collector runs on, so plain
  // ints are safe with no lock).
  int last_bitrate_kbps = 0;
  int last_roi_qp = 0;
  // Lifetime count of encoder verbs the venc core refused. Replaces the
  // deleted waybeam_failures on the 1 Hz stats line: without it a drone
  // whose encoder silently rejects every set_bitrate looks identical from
  // the ground to one tracking the ladder perfectly. Same agent-thread-only
  // access as the two above, so a plain counter is safe.
  uint64_t venc_verb_failures = 0;

  void apply_op(const AppliedOp& op) override;

  void send_control(const std::vector<uint8_t>& body) override;

  // No retry loop lives here. A failed bitrate/ROI verb is reported UP (the
  // bool return): RcAgent declines to latch it and re-issues the same value
  // on its next policy tick, which is the only place that knows what the
  // current operating point should be. Retrying inside the actuator would
  // block the agent thread on a sick MI layer instead. A failed IDR is not
  // reported — nothing latches on it, and the next chain break or LINKED
  // re-entry raises another.
  //
  // last_bitrate_kbps/last_roi_qp record the last value ATTEMPTED, not the
  // last one accepted, because they exist to answer "what did the ladder
  // ask for" on the telemetry row; venc_verb_failures next to them is what
  // says whether the encoder is actually keeping up with those asks.
  //
  // On host builds (no MABUR_HAVE_VENC) both verbs report success without
  // doing anything: there is no encoder to diverge from, and reporting
  // failure would make RcAgent retry forever. Tests drive MockActuator.
  bool set_bitrate_kbps(int k) override;

  bool set_roi_qp(int q) override;

  bool set_fps(int f) override;

  void request_idr() override;

  // RcAgent calls this on an RCF hop order ("hop") and an unconfirmed move's return to
  // where it came from ("move_unconfirmed"), from the agent thread only (same contract as
  // apply_op/send_control above). Null in dry-run (dev == nullptr): there is
  // no device and no tx_gate to take, so that path is a pure stderr echo.
  std::shared_mutex* tx_gate = nullptr;
  // Writer-priority flag for tx_gate — raised around the exclusive take so no
  // new shared-taker enters while this thread waits. See await_retune_gate's
  // comment for why a reader-preferring rwlock cannot be left to starve this
  // writer: it runs on the agent thread.
  std::atomic<bool>* retune_waiting = nullptr;
  uint8_t cur = 0;  // set to start_ch where the actuator is configured
  // A retune that arrived during a calibration sweep and has not been
  // performed yet (see retune() below). Agent-thread-only, like every other
  // member here.
  std::optional<uint8_t> deferred_ch;
  const char* deferred_reason = "";

  // Threading: FastRetune is a control-plane call, and devourer's IRtlRadio.h
  // threading contract forbids one concurrent with ANY other device call —
  // not just a bulk-OUT. A calibration sweep runs the three TX-power knobs
  // (DevicePowerCtl, in tx_writer.cpp) from the TX writer thread for up to 180 s, and
  // nothing stops a retune being requested in that time: a hop order in an
  // RCF that was already queued, or
  // move_unconfirmed's return when the GS goes radio-silent for the sweep
  // (move_confirm_ms is 2 s). Two rules keep that safe without ever blocking
  // the agent thread on a sweep:
  //   * the three power calls take tx_gate SHARED (they are device calls,
  //     not senders, but the gate is what serialises them against this one);
  //   * a retune requested while cal_active simply does not happen — it is
  //     latched into deferred_ch and replayed by apply_deferred_retune() on
  //     the agent thread's own cal falling edge, next to the ladder
  //     re-apply. `cur` deliberately stays on the radio's REAL channel
  //     while deferred, so the replayed retune still logs the true from->to
  //     and a same-channel deferral cannot be mistaken for a completed move.
  // RcAgent's move-confirm machinery already handles "the retune did not
  // take" (it hears nothing on the new channel and returns to the channel it
  // came from), so a deferral degrades to that path rather than to a wedged
  // link.
  // Not host-testable: RealActuator lives in maburd/ and needs a real
  // IRtlRadio, so this comment is the specification.
  void retune(uint8_t ch, const char* reason) override;

  // Replays the retune retune() deferred, if any. Called from the agent
  // thread on the falling edge of cal_active (see agent_thread.cpp's agent loop),
  // the same edge that re-applies the ladder.
  void apply_deferred_retune() {
    if (!deferred_ch.has_value()) return;
    const uint8_t ch = *deferred_ch;
    const char* reason = deferred_reason;
    deferred_ch.reset();
    deferred_reason = "";
    if (!dev) return;
    retune_now_(ch, reason);
  }

  void retune_now_(uint8_t ch, const char* reason);

  mabur::VtxRecorder* recorder = nullptr;  // null until the recorder exists
  bool set_record(bool on) override;

  void set_sensor_offset(int mfps) override;

  // Auto channel set: RcAgent calls this once per confirmed move, from the
  // agent thread. A failed write is one stderr line, never fatal -- the drone
  // still parks on the right channel this session, it just won't remember it
  // across a restart.
  void remember_channel(uint8_t ch) override;
};

// Applies a freshly-published AppliedOp (detected via shared_ptr identity,
// not generation — see the callers' pump-loop comments) to the
// hot-thread-owned UepEncoder: the commanded overhead PAIR applied directly
// per layer (RC_VERSION 5 — no ladder/scale translation) plus
// per-layer shed. Called from the hot thread only. The 2-slot AppliedOp's
// indices [0]/[1] map 1:1 onto the 2-stream UepEncoder's sids.
inline void apply_op_to_uep(const AppliedOp& op, UepEncoder& uep) {
  uep.set_layer_overhead(0, op.fec_ov_base);
  uep.set_layer_overhead(1, op.fec_ov_enh);
  for (int i = 0; i < 2; ++i) uep.set_shed(i, op.shed[static_cast<size_t>(i)]);
}

// Re-prices the air clock from a freshly-published AppliedOp: base/enh bodies
// at the ladder's per-layer PHY rate, the probe body at the probe profile's
// rate (0 = probe off -> not booked). Called wherever apply_op_to_uep is, so
// the clock drops to the new rate the instant a demote RCF lands -- while the
// encoder is still producing at the old rung's bitrate, which is exactly when
// the backlog grows. Rates are DELIVERED (air_rate.h: nominal × per-MCS
// efficiency), so the probe body -- one rung up, its own MCS -- is priced at
// its own capacity.
inline void apply_op_to_clock(const AppliedOp& op, const AirClockCfg& c, AirClock& clock) {
  const double probe_mbps = op.probe_profile != rc::kNoProbeProfile
                                ? delivered_mbps(op.probe, c) : 0.0;
  clock.set_rates(delivered_mbps(op.ladder[0], c), delivered_mbps(op.ladder[1], c),
                  probe_mbps, static_cast<uint32_t>(c.body_us));
}

}  // namespace mabur::maburd
