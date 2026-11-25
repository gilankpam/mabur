// Real mode: the TX writer thread body (real_mode.h).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "cal_link.h"
#include "cal_sweep.h"
#include "mav_component.h"
#include "mabur/cal_mav.h"
#include "real_mode.h"
#include "SignalStop.h"

#include <libusb.h>

#if defined(MABUR_DRY_RUN_ONLY)
// Not used; real mode is always compiled in, guarded at runtime by --dry-run
// so the same binary runs (in dry-run) on a machine with no dongle attached.
#endif

namespace mabur::maburd {

using namespace mabur;

namespace {

// Calibration's power actuator: CalSweep::PowerCtl driven by the live
// devourer device -- the flat TXAGC override, the rate-diff table and the
// anchor read-back, under CalSweep's state machine.
//
// Threading: all three are device control-plane calls made from the TX
// writer thread, and devourer's IRtlRadio.h contract forbids any of them
// concurrent with a channel set. RealActuator::retune takes tx_gate
// exclusive around FastRetune, so each call here takes it SHARED -- a cal
// session lasts up to 180 s, long enough for a hop or
// move_unconfirmed retune to be requested mid-sweep, so that is the normal
// case, not a corner. retune's side of this defers the move entirely while
// cal_active, and the deferred replay fires on the agent thread's falling
// edge -- by which time this thread may still be inside the post-session
// power restore. So the gate covers BOTH ends of the session: these three
// sweep-time calls, DeviceCapacityCtl's and restore_operating_power()'s
// writes (LivePowerTable) all take it shared, and the replayed FastRetune (which takes it exclusive)
// therefore cannot overlap either. Held only for the single call, never
// across pump().
struct DevicePowerCtl : CalSweep::PowerCtl {
  IRtlRadio* dev;
  std::shared_mutex* gate = nullptr;
  std::atomic<bool>* gate_waiting = nullptr;
  bool set_index_override(int idx) override {
    // SetTxPowerIndexOverride is void on every family (IRtlRadio.h) —
    // there is no failure it could report back.
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    dev->SetTxPowerIndexOverride(idx);
    return true;
  }
  bool zero_rate_diffs() override {
    // A table of zeros is a no-op at the anchor rate (MCS7) and flattens
    // every other rate onto it (devourer/src/TxPower.h) — exactly "every
    // rate measures against a common base" (cal_sweep.h constraint 2).
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    return dev->SetTxPowerRateDiffs(devourer::TxRateDiffsQdb{});
  }
  int read_anchor_idx() override {
    // mcs7_index with diffs zeroed is the chip's own reference level for
    // the anchor rate -- the anchor the sweep programs each relative
    // cell against (cal_sweep.h). GetTxPowerState().valid stays false on
    // chips this knob doesn't support; -1 means "unreadable" and the
    // caller refuses the session rather than sweeping against a made-up
    // anchor.
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    const auto st = dev->GetTxPowerState();
    return (st.valid && st.mcs7_index >= 0) ? st.mcs7_index : -1;
  }
};

// Calibration's capacity actuator: CalSweep::CapacityCtl on the live
// device and RadioTx, driven by pump() on the TX writer thread only.
//
// set_point programs a point the way production flies it: the ladder at
// (mcs, bw) with the link's LDPC and STBC on, the A-MPDU mode
// ampdu_mode_for(ampdu, mcs, bw), and no flat TXAGC override. A live
// override flattens the per-rate table to zero diffs (devourer
// TxPower.h), so clearing it is what puts the width's plan -- programmed
// when the capacity sweep was accepted -- on the air for the point.
//
// blast saturates the air for the dwell through RadioTx::send_bodies, the
// path video takes, so with radio.tx_threads > 1 every UsbTxPool sender
// stays busy; batches of three (one URB), each frame with a running stamp
// seq. blocked_pct is the share of the dwell the writer spent waiting on
// the device. With the pool that is the pool's own space-wait counter: a
// submitter blocks only while every sender is inside a device call and
// the queue behind them is full. Without one (tx_threads 1) it is the time
// inside send_bodies, which is then the synchronous device call itself.
// RadioTx exposes no device queue depth, and timing the calls against a
// running median would read a saturated point -- every call blocked for
// the same time -- as unblocked.
struct DeviceCapacityCtl : CalSweep::CapacityCtl {
  RadioTx* tx = nullptr;
  IRtlRadio* dev = nullptr;
  mabur::UsbTxPool* pool = nullptr;  // null when radio.tx_threads == 1
  std::shared_mutex* gate = nullptr;
  std::atomic<bool>* gate_waiting = nullptr;
  AmpduCfg ampdu;
  bool ldpc = true;
  bool set_point(uint8_t mcs, uint8_t bw) override {
    const rc::LayerTxSpec spec{rc::PhyMode::HT, mcs, bw, false, ldpc, true};
    tx->set_ladder({spec, spec}, std::nullopt);
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    const bool ok = dev->SetAmpduMode(ampdu_mode_for(ampdu, mcs, bw));
    dev->SetTxPowerIndexOverride(-1);
    return ok;
  }
  BlastResult blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) override {
    // Stream 1, the enhancement layer: the bulk of video. set_point put
    // the same spec on both layers.
    std::vector<UepBody> bodies(3);
    for (auto& b : bodies) {
      b.stream_id = 1;
      b.body.assign(frame, frame + len);
    }
    uint32_t seq = 0;
    uint64_t offered = 0, call_us = 0;
    const uint64_t wait0 = pool ? pool->space_wait_us() : 0;
    const uint64_t t0 = now_steady_us();
    const uint64_t t_end = t0 + static_cast<uint64_t>(dwell_ms) * 1000;
    uint64_t now = t0;
    while (now < t_end && !g_devourer_should_stop) {
      if (len >= cal::kCalHeaderLen) {
        for (auto& b : bodies) {  // cal_wire.h's 24-bit seq: hi, lo, mid
          b.body[9] = static_cast<uint8_t>((seq >> 16) & 0xFF);
          b.body[10] = static_cast<uint8_t>(seq & 0xFF);
          b.body[11] = static_cast<uint8_t>((seq >> 8) & 0xFF);
          seq = (seq + 1) & 0xFFFFFF;
        }
      }
      const uint64_t c0 = now;
      offered += tx->send_bodies(bodies);
      now = now_steady_us();
      call_us += now - c0;
    }
    const uint64_t elapsed = now - t0;
    const uint64_t blocked = pool ? pool->space_wait_us() - wait0 : call_us;
    BlastResult r;
    r.offered = static_cast<uint32_t>(std::min<uint64_t>(offered, UINT32_MAX));
    r.blocked_pct =
        elapsed ? static_cast<uint8_t>(std::min<uint64_t>(100, blocked * 100 / elapsed)) : 0;
    return r;
  }
};

}  // namespace

// TX writer thread: sole caller of tx.send_bodies (RadioTx's
// single-thread contract). Batches up to 3 bodies per call — devourer's
// Jaguar3 send_packets packs them into one bulk-OUT URB (HalMAC parses at
// most 3 descriptors per transfer), amortizing the per-URB tax that
// capped the old inline path at ~2500 fps. It is also, by that same
// contract, cal_sweep.pump()'s sole legal caller (cal_sweep.h) — the
// calibration messages are dispatched here too (cal_queue, fed by the MAV
// thread), so on_cmd/on_apply/on_abort/pump never run off this thread.
void tx_writer_loop(RealMode& m) {
  name_thread("mbr-txw");
  std::vector<UepBody> batch;
  // Calibration flies radio.ldpc as the link flies it now. A RADIO_LDPC
  // set (refused while a run is open) may have moved it since boot, so the
  // engine is rebuilt with the live flag whenever it changes between runs.
  // Capacity frames are the size of a video body at the configured
  // geometry: the enhancement layer's, the bulk of video.
  auto make_cal_cfg = [&](bool ldpc) {
    CalSweepCfg c;
    c.ldpc = ldpc;
    c.capacity_frame_len =
        13 + static_cast<size_t>(m.cfg.fec.blocks_per_body[1]) *
                 (16 + static_cast<size_t>(m.cfg.fec.symbol_size[1]));
    return c;
  };
  bool cal_ldpc = m.live_ldpc.load(std::memory_order_acquire);
  CalSweep cal_sweep(make_cal_cfg(cal_ldpc));
  DevicePowerCtl pwr;
  pwr.dev = m.rtl_device;
  pwr.gate = &m.tx_gate;
  pwr.gate_waiting = &m.retune_waiting;
  DeviceCapacityCtl cap;
  cap.tx = &m.tx;
  cap.dev = m.rtl_device;
  cap.pool = m.dev_sink.pool;
  cap.gate = &m.tx_gate;
  cap.gate_waiting = &m.retune_waiting;
  cap.ampdu = m.cfg.ampdu;
  cap.ldpc = cal_ldpc;
  // One drone->GS calibration message (ack, done, applied) as its own
  // tagged T_MAV_UP frame on the control path: the shared uplink counter
  // and dot11 sequence, the mutex-guarded dev_sink.send(). Sent whatever
  // the session state -- every sweep outlasts the link failsafe and the GS
  // is listening for exactly these -- and outside the MAV thread's pacing
  // and its sweep suppression: an ack or APPLIED per command, and each
  // SweepDone take_done() yields (it repeats until the GS moves on).
  ControlRadiotap cal_radiotap = m.make_control_radiotap();
  auto send_cal = [&](const cal::CalMsg& msg) {
    auto ctx = m.agent.uplink_ctx(/*cal_reply=*/true);
    if (!ctx) {
      m.mav_up_nopair.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    rc::Mav mv;
    mv.type = rc::T_MAV_UP;
    mv.counter = m.mav_up_counter.fetch_add(1, std::memory_order_relaxed);
    mv.payload = cal::encode(msg, MavComponent::sys_id(), MavComponent::comp_id(),
                            mav::kChanDroneRx);
    ctx->seq32 = mv.counter;
    const auto mav = rc::pack_mav(mv, m.cfg.link.key, *ctx);
    const std::vector<uint8_t>& rt = cal_radiotap.get();
    std::vector<uint8_t> frame;
    frame.reserve(rt.size() + kDot11HeaderLen + mav.size());
    frame.insert(frame.end(), rt.begin(), rt.end());
    auto hdr = build_dot11_header(m.telem_dot11_seq.fetch_add(1, std::memory_order_relaxed) & 0xFFF);
    frame.insert(frame.end(), hdr.begin(), hdr.end());
    frame.insert(frame.end(), mav.begin(), mav.end());
    m.dev_sink.send(frame.data(), frame.size());
    m.mav_tx_frames.fetch_add(1, std::memory_order_relaxed);
    m.mav_tx_packets.fetch_add(1, std::memory_order_relaxed);
  };
  // Power and verify frames fly alone, one per PPDU, so each stamped
  // frame's delivery is its own (AmpduMode{} is aggregation off). Under
  // the power table's lock (see LivePowerTable::unless_cal) and tx_gate
  // like every device call on this thread; the agent puts the rung's mode
  // back on the cal_active falling edge (RealActuator::on_cal_end).
  auto cal_no_agg = [&]() {
    m.power_table.with_lock([&] {
      await_retune_gate(&m.retune_waiting);
      std::shared_lock<std::shared_mutex> sg(m.tx_gate);
      if (!m.rtl_device->SetAmpduMode(devourer::AmpduMode{}))
        std::fprintf(stderr, "maburd cal: SetAmpduMode(off) failed\n");
    });
  };
  // Re-reads the config from disk -- not the bring-up `cfg` -- so one path
  // covers "the run applied new walls" (the file holds them) and "the run
  // never applied" (the file is unchanged, so this reproduces bring-up).
  auto reload_config = [&]() {
    try {
      return load_config(m.cfg_path, nullptr);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "maburd cal: config reload failed (%s); using the bring-up config\n",
                   e.what());
      return m.cfg;
    }
  };
  // The run's end, however it ended (done, abort, a deadline, a refusal):
  // close_session() only parks the flat override at the anchor and never
  // restores the diffs a power sweep zeroed, so left alone the drone would
  // fly ONE flat index across every rate until restart -- after an aborted
  // run, every rate below the anchor above its measured wall. This puts
  // the current rung's width's plan (or the chip's own table under
  // power_mode "none") back and only then clears the override, in one
  // tx_gate hold: while the override is live it masks the diffs, so the
  // right table is on the air the moment the override goes, with no window
  // of "no override, zeroed diffs". The config read stays outside the lock
  // (it can block on disk).
  auto restore_operating_power = [&]() {
    const Config live_cfg = reload_config();
    const auto op = m.shared_op.load();
    const uint8_t bw = op ? op->ladder[1].bw : m.cfg.radio.width;
    if (!m.power_table.replace(m.plans_for(live_cfg.radio, m.live_wall_margin.load()), bw,
                             /*zero_offset=*/true, /*clear_override=*/true))
      std::fprintf(stderr, "maburd cal: SetTxPowerRateDiffs failed restoring after the run\n");
  };
  // An ApplyCmd drained from the engine: validate/patch/publish this
  // width's walls and/or efficiency (cal_apply.h), answer APPLIED, and for
  // walls arm the verify sweep the GS expects but never commands. The new
  // plans are only recorded here, not programmed: verify parks each rate
  // with the flat override over zeroed diffs, a following capacity sweep
  // programs its width's plan when it is accepted, and the run's restore
  // programs the rung's. The live margin is the one apply_calibration
  // validates with and the one the parks subtract -- the GS derives the
  // same parks from the same walls and margin.
  CalApplyLedger apply_ledger;
  auto apply_and_arm_verify = [&](const cal::ApplyCmd& a) {
    const double margin_db = m.live_wall_margin.load();
    const CalWrite w = cal_write_for(a, cal_sweep.stamp_flags());
    std::string err, note;
    ApplyResult ar;
    {
      // Held across the write and the applier's notice, so its next apply
      // re-reads the calibrated file rather than racing this write.
      auto lk = lock_config_writes();
      ar = apply_calibration(m.cfg_path, w, margin_db, apply_ledger.want_backup(a.nonce), &err,
                             &note);
      if (ar == ApplyResult::Ok) m.applier.note_file_rewritten_externally();
    }
    cal::Applied out;
    out.nonce = a.nonce;
    out.seq = a.seq;
    out.bw = a.bw;
    out.ok = ar == ApplyResult::Ok;
    if (out.ok) {
      // An ok apply that kept something the GS did not send says so.
      out.reason = note.substr(0, 49);
      m.power_table.set_plans(m.plans_for(reload_config().radio, margin_db));
      std::fprintf(stderr, "maburd cal: applied %u MHz%s%s%s%s\n", static_cast<unsigned>(a.bw),
                   a.has_walls ? " walls" : "", a.has_eff ? " efficiency" : "",
                   note.empty() ? "" : " -- ", note.c_str());
    } else {
      out.reason = loader_reason(err).substr(0, 49);
      std::fprintf(stderr, "maburd cal: apply %u MHz refused: %s\n", static_cast<unsigned>(a.bw),
                   err.c_str());
    }
    apply_ledger.record(out);
    send_cal(out);
    if (out.ok && a.has_walls) {
      cal_no_agg();
      cal_sweep.arm_verify(verify_parks(a, margin_db), a.bw, now_steady_ms());
    }
  };
  bool cal_was_active = false;  // edge-detect: session start -> drain txq once
  CalRestoreOwed restore_owed;  // see cal_link.h
  // dq_queue gauge: TRUE push→pop queue
  // wait from the pre-push pushed_us stamp, per body and for the AU-first
  // body alone (the one whose q_ms the GS latches as dq). Thread-owned,
  // reported every 5 s.
  uint64_t qw_n = 0, qw_sum_us = 0, qw_max_us = 0;
  uint64_t qf_n = 0, qf_sum_us = 0, qf_max_us = 0;
  // tx_send gauge (burst-drain follow-up): wall time of each
  // tx.send_bodies call and its body count, to split the measured
  // ~0.385 ms/body burst pace into USB round-trip vs airtime. A blocking
  // ~1.1 ms per 3-body batch here = the URB round-trip IS the pace.
  uint64_t sb_calls = 0, sb_bodies = 0, sb_sum_us = 0, sb_max_us = 0;
  // SWEEPs accepted only under the calibration run pair (kOkLatched) that
  // named a nonce other than the open run's: dropped, since a run opens
  // only from a SWEEP verified under the published session.
  uint64_t cal_latch_drops = 0;
  uint32_t last_qw_report_ms = static_cast<uint32_t>(now_steady_ms());
  while (!g_devourer_should_stop) {
    if (!cal_sweep.active()) {
      const bool ldpc = m.live_ldpc.load(std::memory_order_acquire);
      if (ldpc != cal_ldpc) {
        cal_ldpc = ldpc;
        cal_sweep = CalSweep(make_cal_cfg(ldpc));
        cap.ldpc = ldpc;
      }
    }
    // Calibration messages (cal_queue, fed by the MAV thread): dispatched
    // here, not on the agent thread, so on_cmd/on_apply/on_abort share
    // pump()'s thread (cal_sweep.h constraint 1).
    CalInbound in;
    while (m.cal_queue.pop(&in)) {
      const uint64_t cal_now = now_steady_ms();
      if (const auto* c = std::get_if<cal::SweepCmd>(&in.msg)) {
        if (in.latched && !(cal_sweep.active() && c->nonce == cal_sweep.run_nonce())) {
          ++cal_latch_drops;
          continue;
        }
        const bool starting = cal_sweep.starts_sweep(*c);
        if (starting) {
          // cal_active up first, under the power table's lock (see
          // LivePowerTable), and the restore owed from here on; then
          // aggregation off for every kind but capacity, whose points
          // program the production mode themselves.
          m.power_table.begin_cal();
          restore_owed.note_device_touched();
          if (c->kind != cal::Kind::Capacity) cal_no_agg();
        }
        cal_sweep.on_cmd(*c, cal_now, pwr);
        const auto ack = cal_sweep.take_ack();
        // An accepted capacity sweep flies this width's production power
        // table: programmed only after on_cmd() accepted it, since a new
        // run zeroes the diffs for its anchor read first. Never for a
        // refused command or a repeat of the running sweep.
        if (starting && c->kind == cal::Kind::Capacity && ack &&
            ack->result == cal::AckResult::Accepted)
          m.power_table.program_for_cal(c->bw);
        // The ack goes out HERE -- before this thread pumps the sweep the
        // command touched, whose first cell waits 200 ms for it -- on every
        // accepted command, an exact repeat included: the GS resends its
        // SWEEP every 200 ms into a lossy uplink, and one lost ack must not
        // cost the sweep.
        if (ack) send_cal(*ack);
        // A command refused because the chip could not report its anchor,
        // or would not zero its rate diffs for a clean read, was refused
        // after on_cmd() had already touched the table and no run is
        // open: restore_owed puts it back below.
        if (cal_sweep.take_refused())
          std::fprintf(stderr,
                       "maburd cal: sweep refused (TXAGC anchor unreadable or rate diffs "
                       "not zeroed); restoring operating power\n");
      } else if (const auto* ap = std::get_if<cal::ApplyCmd>(&in.msg)) {
        // The GS resends APPLY every 200 ms until it hears APPLIED: a
        // (nonce, seq) already applied is answered again, never re-applied.
        if (auto hit = apply_ledger.cached(ap->nonce, ap->seq))
          send_cal(*hit);
        else
          cal_sweep.on_apply(*ap, cal_now);
      } else if (const auto* ab = std::get_if<cal::Abort>(&in.msg)) {
        cal_sweep.on_abort(*ab, cal_now, pwr);
      }
      // SweepAck, SweepDone and Applied travel drone -> GS only.
    }

    if (auto a = cal_sweep.take_pending_apply()) apply_and_arm_verify(*a);

    m.cal_active.store(cal_sweep.active(), std::memory_order_relaxed);
    m.cal_sweeping.store(cal_sweep.state() == CalSweep::State::Sweeping,
                       std::memory_order_relaxed);

    if (cal_sweep.active()) {
      // Video quiesce: the hot thread has already stopped pushing (its own
      // cal_active guard, above), but bodies queued before this exact
      // moment must not trickle out mixed with sweep frames — drain once,
      // on the session's very first iteration here.
      if (!cal_was_active) m.txq.drain();
      cal_was_active = true;
      cal_sweep.pump(now_steady_ms(), m.tx, pwr, cap);
      if (auto d = cal_sweep.take_done()) send_cal(*d);
      // The one blocking wait this loop has is txq.pop_batch(.., 5) at
      // the bottom, which a calibration session never reaches -- and
      // pump() returns immediately whenever the current cell is still
      // settling or the next frame is not due yet. Without this sleep
      // the writer thread spins one of the SigmaStar's two cores at
      // 100% for the length of a run, on a SoC with a documented
      // thermal incident (flight 21, 81 C). 200 us is a fifth of the
      // sweep's 1 ms inter-frame gap, and CalSweep's pacing is
      // deadline-based (cal_sweep.cpp), so a late wake does not
      // accumulate into the sweep's duration.
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;  // never fall through to the video drain below
    }
    // The run is over (or never opened: refused, or opened and closed in
    // this same drain): restore TX power for real before anything below
    // can transmit again. Owed from the sweep's first device write, not
    // keyed on cal_was_active, which a run living within one drain never
    // sets.
    if (restore_owed.take_due(cal_sweep.active())) restore_operating_power();
    if (cal_was_active) {
      // Falling edge: drain TxQueue a SECOND time here too -- the hot
      // thread's own cal_active guard (its loop, above) can race this exact
      // transition and push one more AU's bodies into txq between this
      // thread clearing cal_active and noticing it here; left alone those
      // bodies would sit queued for the length of the NEXT session (or
      // forever, if there isn't one) and go out stale with a session-length
      // q_ms stamp whenever they finally flush.
      m.txq.drain();
    }
    cal_was_active = false;

    batch.clear();
    if (m.txq.pop_batch(batch, 3, 5) == 0) continue;
    // Patch each body's SBI q_ms with its TxQueue wait (push→pop), and fold
    // the batch's worst case into the window-max gauge (latency
    // accounting). enqueued_ms == 0 means the producer never stamped it
    // (shouldn't happen on the real path, but dry-run/test paths may still
    // push bare bodies) -> leave q_ms at the packer's zero placeholder
    // rather than reporting a bogus wait.
    const uint32_t pop_ms = static_cast<uint32_t>(now_steady_ms());
    const uint64_t pop_us = now_steady_us();
    for (auto& b : batch) {
      uint32_t w = (b.enqueued_ms && pop_ms > b.enqueued_ms)
                       ? pop_ms - b.enqueued_ms : 0;
      if (w > 65535) w = 65535;
      mabur::sbi_set_q_ms(b.body.data(), b.body.size(),
                          static_cast<uint16_t>(w));
      uint32_t prev = m.txq_wait_max_ms.load(std::memory_order_relaxed);
      while (w > prev && !m.txq_wait_max_ms.compare_exchange_weak(prev, w)) {}
      if (b.pushed_us && pop_us > b.pushed_us) {
        const uint64_t tw = pop_us - b.pushed_us;
        ++qw_n; qw_sum_us += tw;
        if (tw > qw_max_us) qw_max_us = tw;
        if (b.au_first) {
          ++qf_n; qf_sum_us += tw;
          if (tw > qf_max_us) qf_max_us = tw;
        }
      }
    }
    if (pop_ms - last_qw_report_ms >= 5000) {
      last_qw_report_ms = pop_ms;
      if (qw_n > 0) {
        std::fprintf(stderr,
            "maburd dq_queue: bodies=%llu wait_us mean=%llu max=%llu "
            "au_first n=%llu mean=%llu max=%llu\n",
            (unsigned long long)qw_n,
            (unsigned long long)(qw_sum_us / qw_n),
            (unsigned long long)qw_max_us,
            (unsigned long long)qf_n,
            (unsigned long long)(qf_n ? qf_sum_us / qf_n : 0),
            (unsigned long long)qf_max_us);
      }
      qw_n = qw_sum_us = qw_max_us = 0;
      qf_n = qf_sum_us = qf_max_us = 0;
      if (sb_calls > 0) {
        std::fprintf(stderr,
            "maburd tx_send: calls=%llu bodies=%llu us/call mean=%llu "
            "max=%llu us/body=%llu cal_latch_drop=%llu\n",
            (unsigned long long)sb_calls,
            (unsigned long long)sb_bodies,
            (unsigned long long)(sb_sum_us / sb_calls),
            (unsigned long long)sb_max_us,
            (unsigned long long)(sb_bodies ? sb_sum_us / sb_bodies : 0),
            (unsigned long long)cal_latch_drops);
      }
      sb_calls = sb_bodies = sb_sum_us = sb_max_us = 0;
    }
    const uint64_t sb_t0 = now_steady_us();
    m.tx.send_bodies(batch);
    const uint64_t sb_dt = now_steady_us() - sb_t0;
    ++sb_calls;
    sb_bodies += batch.size();
    sb_sum_us += sb_dt;
    if (sb_dt > sb_max_us) sb_max_us = sb_dt;
  }
}
}  // namespace mabur::maburd
