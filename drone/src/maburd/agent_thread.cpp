// Real mode: the agent thread body (real_mode.h).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <thread>
#include <vector>

#include "trial_proof.h"
#include "peak_rate.h"
#include "tick_gate.h"
#include "loop_watchdog.h"
#include "stats_line.h"
#include "real_mode.h"
#include "SignalStop.h"

#include <libusb.h>

namespace mabur::maburd {

using namespace mabur;

// Agent thread: drains the RC queue every cfg.link.rc_drain_ms, ticks
// RcAgent on cfg.link.tick_ms, runs the watchdog, and handles SIGUSR1
// stats dumps.
void agent_loop(RealMode& m) {
  name_thread("mbr-agent");
  uint64_t start = now_steady_ms();
  LoopWatchdog wd(start);
  uint64_t last_stats_ms = start;
  // The MAV counters as of the last stats line printed with them.
  uint64_t mav_last[8] = {};
  // A trial binary's linked/proven markers for the update loop, sampled
  // once a second.
  TrialProof trial_proof;
  uint64_t last_proof_ms = start;

  // T_TELEM: sent at ~1 Hz on this same periodic path, on the mutex-guarded
  // dev_sink.send() the MSP thread also uses. The tlm_seq/dot11 seq
  // counters are members of RealMode (real_mode.h) — deliberately
  // NOT the video path's tx.seq() or RealActuator's DISC_ACK control_seq,
  // so a telemetry-send bug can never perturb either. The dot11 seq is
  // shared with the MAV thread and the TX writer thread's calibration
  // replies -- see that declaration's comment.
  uint64_t last_telem_ms = start;
  ControlRadiotap agent_radiotap = m.make_control_radiotap();
  mabur::CpuBusySampler cpu_busy;  // /proc/stat delta per telemetry tick

  mabur::TickGate tick_gate(now_steady_ms(), m.cfg.link.tick_ms);
  // Peak 100 ms encoder rate for the stats line (peak_rate.h): fed every
  // wake, which is the finest cadence anything here runs at; normalised
  // by frame count at the configured sensor fps, not wall time.
  mabur::PeakRate enc_peak(100, m.cfg.venc.core.fps);
  // Calibration ladder gate, agent side: RealActuator::apply_op's
  // set_ladder() is gated off while cal_active (see that call site), so the
  // ladder sits at whatever CalSweep's TX-writer-thread sweep last parked
  // it at until this re-applies the last-known-correct op -- once, right on
  // the falling edge, on THIS thread (apply_op's documented contract), not
  // from the TX writer thread that noticed cal_active clear.
  CalEndEdge cal_end_edge;
  // TxStats::failed as of the last housekeeping tick, for send_telem.
  uint64_t telem_usb_fail = 0;
  // One T_TELEM frame: the 1 Hz periodic one, and the link-pairing
  // "promote" frame sent right after an RCF promoted a session, before
  // tick() runs its deferred retune.
  auto send_telem = [&](uint64_t now) {
    TelemInputs ti;
    ti.state = static_cast<int>(m.agent.state());
    ti.failsafe_shed = m.agent.failsafe_shed();
    ti.congestion_shed = m.agent.congestion_shed();
    ti.low_power = m.agent.low_power();
    ti.auth_reject = m.agent.take_auth_reject();
    ti.rec_status = m.vtx_rec.status_byte();
    ti.chip_caps = rc::CAP_FRAME_WIRE | rc::CAP_TELEMETRY | rc::CAP_CALIBRATE | rc::CAP_MAVLINK;
    ti.width = static_cast<uint8_t>(m.cfg.radio.width);
    // have_feedback() false means no RCF has EVER been accepted (still
    // BOOT/RENDEZVOUS) — 0 would read as maximally fresh, the opposite of
    // the truth. Pass a value make_telem's saturate<uint16_t> clamps to
    // 65535 ("never"), matching the wire field's documented sentinel.
    ti.rcf_age_ms = m.agent.have_feedback()
                        ? (now - m.agent.last_feedback_ms())
                        : static_cast<uint64_t>(UINT16_MAX) + 1;
    // link-rtt: which RCF that age references. Invalid (flags bit3
    // clear) outside LINKED (failsafe rebase, unconfirmed-move fallback),
    // where the age is fresh but no RCF backs it — the GS must not match
    // a stale seq against it.
    if (const auto fseq = m.agent.last_feedback_seq()) {
      ti.rcf_seq_echo = *fseq;
      ti.rcf_seq_echo_valid = true;
    }
    ti.rcf_rx = m.agent.rcf_accepted();
    ti.cmd_kbps = m.actuator.last_bitrate_kbps;
    ti.txq_drops = m.txq.dropped();
    ti.txq_wait_max_ms = m.txq_wait_max_ms.exchange(0, std::memory_order_relaxed);
    ti.usb_fail = telem_usb_fail;
    // fec-nack, per period: bumped by the RX thread's T_NACK handler,
    // drained here.
    ti.nack_rx = m.nack_rx.exchange(0, std::memory_order_relaxed);
    ti.retx_syms = m.retx_syms.exchange(0, std::memory_order_relaxed);
    ti.retx_refused = m.retx_refused.exchange(0, std::memory_order_relaxed);
    // RX-side channel view for this period (cca-on): the RX callback's
    // frame split, drained per Telem. No register read here -- see
    // rx_own_frames' declaration for why.
    ti.rx_own = m.rx_own_frames.exchange(0, std::memory_order_relaxed);
    ti.rx_foreign = m.rx_foreign_frames.exchange(0, std::memory_order_relaxed);
    ti.rx_crcfail = m.rx_crcfail_frames.exchange(0, std::memory_order_relaxed);
    ti.uplink = m.uplink_track.snap();
    ti.soc_temp_c = read_soc_temp_c();
    if (ti.soc_temp_c == -128)  // SigmaStar: no thermal_zone
      ti.soc_temp_c = read_soc_temp_c_sigmastar();
    ti.cpu_pct = cpu_busy.sample();
#ifdef MABUR_HAVE_VENC
    // link-rtt t3: pts-domain clock at telem build. Stays 0 (the
    // wire's "unavailable" sentinel) on host builds and when
    // MI_SYS_GetCurPts is unresolved.
    ti.pts_at_build_us = venc_cur_pts_us();
#endif

    const rc::Telem telem_struct = make_telem(
        m.telem_wire_seq.fetch_add(1, std::memory_order_relaxed), ti);
    auto telem = rc::pack_telem(telem_struct);

    const std::vector<uint8_t>& telem_radiotap = agent_radiotap.get();
    const uint16_t dot11_seq =
        m.telem_dot11_seq.fetch_add(1, std::memory_order_relaxed) & 0xFFF;
    const auto frame = build_control_frame(telem_radiotap, dot11_seq,
                                           telem.data(), telem.size());
    m.dev_sink.send(frame.data(), frame.size());
  };
  while (!g_devourer_should_stop) {
    uint64_t now = now_steady_ms();
    enc_peak.sample(now, m.enc_bytes_total.load(std::memory_order_relaxed),
                    m.enc_frames_total.load(std::memory_order_relaxed));
    if (m.ota_trial && now - last_proof_ms >= 1000) {
      last_proof_ms = now;
      switch (trial_proof.tick(now, m.agent.has_published_session(),
                               m.enc_frames_total.load(std::memory_order_relaxed))) {
        case TrialProof::kLinked: touch_update_file(m.ota_dir, "linked"); break;
        case TrialProof::kProven: touch_update_file(m.ota_dir, "proven"); break;
        case TrialProof::kNone: break;
      }
    }

    // The epoch is read before cal_active: see CalEndEdge.
    const uint32_t cal_epoch_now = m.cal_epoch.load(std::memory_order_acquire);
    const bool cal_now_active = m.cal_active.load(std::memory_order_acquire);
    if (cal_end_edge.poll(cal_epoch_now, cal_now_active)) {
      // Video resumes on this same cal_active transition (hot thread's
      // own guard), so this cannot wait for the next RCF -- re-apply the
      // agent's own last-known-good op immediately -- ladder, A-MPDU mode
      // (calibration programmed the chip's mode itself) and power table.
      m.actuator.on_cal_end();
      m.actuator.apply_op(m.agent.current());
      // Deferred retune replay: a retune requested during the sweep was
      // latched, not performed (RealActuator::retune). The sweep's device
      // calls are done as of this edge, so replay it here, on the agent
      // thread, next to the ladder re-apply.
      m.actuator.apply_deferred_retune();
    }

    // Every wake (rc_drain_ms): drain and apply queued RCFs. This is the
    // whole point of the split — op actuation does not wait for the
    // housekeeping tick (a tick-bound drain measured U(0, tick_ms=100),
    // close_ms median ~110 ms).
    std::vector<uint8_t> rc_body;
    while (m.rc_queue.pop(rc_body)) {
      m.agent.on_rc_frame(rc_body.data(), rc_body.size(), now);
    }
    // Promote Telem: one frame on the CURRENT channel telling the GS we
    // hold its session, before tick() runs the deferred retune. The GS
    // follows on a LINKED Telem.
    if (m.agent.take_session_promoted()) send_telem(now);

    if (tick_gate.due(now)) {
      // The power table follows the rung's width on every op (apply_op);
      // this catches the ops that landed before the device was up, or
      // while a parameter set's reprogram raced a width change.
      if (!cal_now_active && m.agent.current().generation != 0)
        m.power_table.follow(m.agent.current().ladder[1].bw);
      devourer::ThermalStatus thermal = m.rtl_device->GetThermalStatus();
      devourer::TxStats txstats = m.rtl_device->GetTxStats();
      RadioHealth health;
      health.thermal_delta = thermal.valid ? thermal.delta : 0;
      health.tx_drops = txstats.failed;
      telem_usb_fail = txstats.failed;
      health.txq_depth = m.txq.depth();
      health.txq_cap = kTxQueueCap;
      m.agent.tick(now, health);
      if (m.agent.take_link_established())
        m.link_up_discont.store(true, std::memory_order_relaxed);

      // Watchdog: see loop_watchdog.h for what counts as a stall.
      uint64_t hb = m.hot_beat.load(std::memory_order_relaxed);
      uint64_t rb = m.rx_beat.load(std::memory_order_relaxed);
      switch (wd.check(now, hb, rb, m.agent.state() == RcAgent::State::LINKED,
                       m.cal_active.load(std::memory_order_relaxed))) {
        case LoopWatchdog::Stall::Hot:
          std::fprintf(stderr, "watchdog: hot thread stalled (no beat for >%llums)\n",
                       static_cast<unsigned long long>(wd.stale_ms()));
          std::abort();
        case LoopWatchdog::Stall::Rx:
          std::fprintf(stderr,
                       "watchdog: rx loop stalled while LINKED (no beat for >%llums)\n",
                       static_cast<unsigned long long>(wd.stale_ms()));
          std::abort();
        case LoopWatchdog::Stall::None: break;
      }

      bool want_stats = g_sigusr1_flag.exchange(false);
      if (want_stats || now - last_stats_ms >= 1000) {
        last_stats_ms = now;
        unsigned snr_mhz = 0;
        int snr_off = 0;
#ifdef MABUR_HAVE_VENC
        {
          VencStats vs{};
          venc_get_stats(&vs);
          snr_mhz = vs.sensor_mhz;
          snr_off = vs.sensor_off_mfps;
        }
#endif
        // MAV fields only when a counter moved since they were last shown.
        const uint64_t mav_now[8] = {
            m.mav_rx_frames.load(std::memory_order_relaxed),
            m.mav_rx_packets.load(std::memory_order_relaxed),
            m.mav_tx_frames.load(std::memory_order_relaxed),
            m.mav_tx_packets.load(std::memory_order_relaxed),
            m.mav_drop.load(std::memory_order_relaxed),
            m.mav_reject.load(std::memory_order_relaxed),
            m.agent.cal_late(),
            m.agent.update_dropped()};
        const std::string mav_part = mav_stats_suffix(mav_now, mav_last);
        const ConfigApplierStats apply_stats = m.applier.stats();
        std::fprintf(stderr,
                     "stats: state=%d hot_beat=%llu rx_beat=%llu seq=%u sent=%llu drops=%llu "
                     "txq=%zu txq_drop=%llu igi_floor=0x%02x "
                     "thermal_delta=%d tx_failed=%llu venc_verb_fail=%llu "
                     "enc_pk100=%uk lp=%d armed=%d snr_mhz=%u snr_off=%d "
                     "mav_apply=%llu/%llu mav_rebuild=%llu%s\n",
                     static_cast<int>(m.agent.state()), static_cast<unsigned long long>(hb),
                     static_cast<unsigned long long>(rb), m.tx.seq(),
                     static_cast<unsigned long long>(m.tx.sent()),
                     static_cast<unsigned long long>(m.tx.drops()),
                     m.txq.depth(), static_cast<unsigned long long>(m.txq.dropped()),
                     static_cast<unsigned>(m.igi_floor_now.load(std::memory_order_relaxed)),
                     health.thermal_delta,
                     static_cast<unsigned long long>(txstats.failed),
                     static_cast<unsigned long long>(m.actuator.venc_verb_failures),
                     enc_peak.take_peak_kbps(),
                     static_cast<int>(m.agent.low_power()), static_cast<int>(m.agent.armed_latched()),
                     snr_mhz, snr_off,
                     static_cast<unsigned long long>(apply_stats.ok),
                     static_cast<unsigned long long>(apply_stats.failed),
                     static_cast<unsigned long long>(apply_stats.rebuilds), mav_part.c_str());
      }

      // Telemetry suppression: non-sweep PPDUs in the airtime a calibration
      // phase is characterizing buy nothing -- drone TX does not itself
      // blank the GS receivers, so this is an airtime courtesy, not a
      // correctness requirement. Gated on cal_sweeping specifically
      // (State::Sweeping), not cal_active (the whole session): telemetry
      // goes out in the between-sweep gaps this narrower flag leaves
      // open, and skipping last_telem_ms's update
      // while suppressed means telemetry resumes on the very next tick once
      // a phase ends, not up to 1 s later.
      if (!m.cal_sweeping.load(std::memory_order_relaxed) &&
          now - last_telem_ms >= 1000) {
        last_telem_ms = now;
        send_telem(now);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(m.cfg.link.rc_drain_ms));
  }
}
}  // namespace mabur::maburd
