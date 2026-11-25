#include "real_actuator.h"

#include <chrono>
#include <cstdio>
#include <thread>

#include "mabur/channel_file.h"
#ifdef MABUR_HAVE_VENC
#include "venc_core.h"  // ARM only: drone/venc is not compiled on host builds
#endif

namespace mabur::maburd {

using namespace mabur;

bool VencRecordChannel::start() {
#ifdef MABUR_HAVE_VENC
  return venc_record_start() == 0;
#else
  return false;
#endif
}

void VencRecordChannel::stop() {
#ifdef MABUR_HAVE_VENC
  venc_record_stop();
#endif
}

void VencRecordChannel::request_idr() {
#ifdef MABUR_HAVE_VENC
  venc_record_request_idr();
#endif
}

void RealActuator::apply_ampdu_for(const AppliedOp& op) {
  if (!dev) return;
  const uint8_t mcs = op.ladder[1].mcs;
  const uint8_t bw = op.ladder[1].bw;
  const int min_mcs = bw == 40 ? ampdu.min_mcs_40 : ampdu.min_mcs_20;
  const devourer::AmpduMode want = ampdu_mode_for(ampdu, mcs, bw);
  if (ampdu_known && ampdu_mode_same(want, last_ampdu)) {
    if (!ampdu_logged) {
      ampdu_logged = true;
      std::fprintf(stderr, "maburd radio: A-MPDU %s at mcs%d/%d (chip default, min_mcs_%d %d, max_num %d)\n",
                   want.enabled ? "ON" : "OFF", mcs, bw, bw, min_mcs, ampdu.max_num);
    }
    return;
  }
  if (!dev->SetAmpduMode(want)) {
    std::fprintf(stderr, "warning: SetAmpduMode failed at mcs%d/%d -- chip keeps %s\n",
                 mcs, bw, last_ampdu.enabled ? "A-MPDU" : "singles");
    return;
  }
  last_ampdu = want;
  ampdu_known = true;
  ampdu_logged = true;
  std::fprintf(stderr, "maburd radio: A-MPDU %s at mcs%d/%d (min_mcs_%d %d, max_num %d)\n",
               want.enabled ? "ON" : "OFF", mcs, bw, bw, min_mcs, ampdu.max_num);
}

void RealActuator::apply_op(const AppliedOp& op) {
  // Calibration owns the radio's ladder for the session's duration:
  // CalSweep::pump (TX writer thread) calls tx->set_ladder() once
  // per cell so every sweep frame transmits at the rate it is STAMPED with
  // -- a concurrent write from here (the agent thread, on every RCF and on
  // the FAILSAFE entry the GS's deliberate ~41 s radio silence triggers
  // about 3 s into any phase in the shipped config) is memory-safe
  // (RadioTx::set_ladder is one atomic shared_ptr swap) but not
  // measurement-safe: whichever writer's call lands last decides what rate
  // actually goes out, and the GS attributes delivery to the cell CalSweep
  // stamped, not the rate the frame was actually sent at. FEC/shed still
  // apply unconditionally below; only the device writes are skipped here. The
  // agent thread's own loop re-applies this exact op once, on the falling
  // edge of cal_active, so the ladder is correct again the instant video
  // resumes rather than waiting for the next RCF. The A-MPDU mode and the
  // TX-power table are the sweep's too (aggregation off for power/verify,
  // the production mode and power per capacity point), and are re-applied
  // on that same edge.
  if (!(cal_active && cal_active->load(std::memory_order_relaxed))) {
    tx->set_ladder(op.ladder, op.probe_profile != rc::kNoProbeProfile
                                  ? std::optional<rc::LayerTxSpec>(op.probe)
                                  : std::nullopt);
    if (video_bw) video_bw->store(op.ladder[0].bw, std::memory_order_relaxed);
    if (power_table) {
      power_table->unless_cal([&] { apply_ampdu_for(op); });
      power_table->follow(op.ladder[1].bw);
    } else {
      apply_ampdu_for(op);
    }
  }
  // Applying an op is a ladder + FEC + shed + aggregation-mode change plus,
  // across a width change, that width's power table -- see the struct
  // comment.
  if (!dev && dry_run) {
    std::fprintf(stderr, "[dry-run] fec_ov_base=%.3f fec_ov_enh=%.3f gen=%llu\n",
                 op.fec_ov_base, op.fec_ov_enh,
                 static_cast<unsigned long long>(op.generation));
  }
  shared_op->store(std::make_shared<const AppliedOp>(op));
}

void RealActuator::send_control(const std::vector<uint8_t>& body) {
  const std::vector<uint8_t>& control_radiotap = control_rt.get();
  const auto frame = build_control_frame(control_radiotap, control_seq,
                                         body.data(), body.size());
  control_seq = static_cast<uint16_t>((control_seq + 1) & 0xFFF);
  sink->send(frame.data(), frame.size());
}

bool RealActuator::set_bitrate_kbps(int k) {
  last_bitrate_kbps = k;
  if (dry_run) {
    std::fprintf(stderr, "[dry-run] set_bitrate_kbps(%d)\n", k);
    return true;
  }
#ifdef MABUR_HAVE_VENC
  if (venc_set_bitrate_kbps(k) != 0) {
    ++venc_verb_failures;
    std::fprintf(stderr, "venc: set_bitrate(%d) FAILED (retry next tick)\n", k);
    return false;
  }
#endif
  return true;
}

bool RealActuator::set_roi_qp(int q) {
  last_roi_qp = q;
  if (dry_run) {
    std::fprintf(stderr, "[dry-run] set_roi_qp(%d)\n", q);
    return true;
  }
#ifdef MABUR_HAVE_VENC
  if (venc_set_roi_qp(q) != 0) {
    ++venc_verb_failures;
    std::fprintf(stderr, "venc: set_roi_qp(%d) FAILED (retry next tick)\n", q);
    return false;
  }
#endif
  return true;
}

bool RealActuator::set_fps(int f) {
  if (dry_run) {
    std::fprintf(stderr, "[dry-run] set_fps(%d)\n", f);
    return true;
  }
#ifdef MABUR_HAVE_VENC
  if (venc_set_fps(f) != 0) {
    ++venc_verb_failures;
    std::fprintf(stderr, "venc: set_fps(%d) FAILED (retry next tick)\n", f);
    return false;
  }
#endif
  if (rate_change) rate_change->store(true, std::memory_order_relaxed);
  return true;
}

void RealActuator::request_idr() {
  if (dry_run) {
    std::fprintf(stderr, "[dry-run] request_idr()\n");
    return;
  }
#ifdef MABUR_HAVE_VENC
  if (venc_request_idr() != 0) {
    ++venc_verb_failures;
    std::fprintf(stderr, "venc: request_idr FAILED\n");
  }
#endif
}

void RealActuator::retune(uint8_t ch, const char* reason) {
  if (!dev) {
    std::fprintf(stderr, "[dry-run] retune(%u, %s)\n", static_cast<unsigned>(ch),
                 reason);
    return;
  }
  if (cal_active && cal_active->load(std::memory_order_relaxed)) {
    deferred_ch = ch;
    deferred_reason = reason;
    std::fprintf(stderr, "maburd: retune %u -> %u (%s) deferred (calibration active)\n",
                 static_cast<unsigned>(cur), static_cast<unsigned>(ch), reason);
    return;
  }
  retune_now_(ch, reason);
}

void RealActuator::retune_now_(uint8_t ch, const char* reason) {
  // Exclusive against every USB sender (DevourerSink/UsbTxPool take the
  // gate shared): FastRetune is a control-plane call and must not overlap
  // a bulk-OUT (devourer threading contract). ~2.4 ms on the 8812EU.
  const uint8_t from = cur;
  bool tx_power_ok = false;
  if (retune_waiting) retune_waiting->store(true, std::memory_order_release);
  {
    std::unique_lock<std::shared_mutex> g(*tx_gate);
    // A frame sent just before a retune (sink->send, synchronous) has
    // RETURNED from send_packet but may still be sitting in the chip's TX
    // FIFO; if FastRetune races it out, it airs on the new channel, where
    // the GS's link card is not listening yet.
    // Holding the gate exclusive already stops any NEW send from
    // starting, but does nothing about a frame the chip already
    // accepted and queued before this lock was taken; this sleep is
    // what gives that frame time to actually leave the antenna on the
    // old channel before FastRetune reprograms it. Once-per-move 5 ms
    // TX stall.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    dev->FastRetune(ch, /*cache_rf=*/true);
    // Critical: FastRetune does NOT re-fold TX power for us at mabur's
    // settings. devourer re-derives it inside FastRetune only on a BAND
    // change and only when a global offset or an index override is live
    // (RtlJaguar3Device.cpp, the `_tx_pwr_offset_steps != 0 ||
    // _tx_pwr_override >= 0` guard); mabur runs offset 0 and override -1
    // by design (power_plan.h: the rate-diff table IS the whole policy),
    // so neither leg fires and the per-rate diff table keeps being added
    // to the BOOT channel's efuse anchor after an auto-select move. The
    // anchors differ per channel -- 39 / 53 / 57 on this unit for
    // ch136 / 149 / 165 -- so that is up to 18 indices, 4.5 dB, of
    // silent error in the overdriven direction on the channel the link
    // just moved to for being quieter.
    //
    // ReApplyTxPower() is apply_tx_power_current(full=true) on Jaguar3:
    // it re-reads the efuse references for the channel the chip is NOW
    // on and rewrites the caller-supplied diffs on top of them. Inside
    // the same exclusive tx_gate as FastRetune, because it is a
    // control-plane register walk and must not overlap a bulk-OUT
    // (devourer threading contract), and because a send landing between
    // the retune and the re-apply would air at the stale anchor.
    //
    // FastRetune returns void (IRtlRadio.h), so there is no success to
    // branch on: re-apply unconditionally. ReApplyTxPower() is the one
    // that reports -- false means the chip is not brought up or a CW
    // tone is active, i.e. the diffs are NOT sitting on this channel's
    // anchor and the log line below is the only trace of it.
    tx_power_ok = dev->ReApplyTxPower();
  }
  if (retune_waiting) retune_waiting->store(false, std::memory_order_release);
  if (retune_seq) retune_seq->fetch_add(1, std::memory_order_relaxed);
  cur = ch;
  std::fprintf(stderr, "maburd: retune %u -> %u (%s)\n", static_cast<unsigned>(from),
               static_cast<unsigned>(ch), reason);
  std::fprintf(stderr,
               "maburd: retune %u -> %u: tx power re-applied (%s)\n",
               static_cast<unsigned>(from), static_cast<unsigned>(ch),
               tx_power_ok ? "ok" : "failed");
}

bool RealActuator::set_record(bool on) {
  if (dry_run) {
    std::fprintf(stderr, "[dry-run] set_record(%d)\n", on ? 1 : 0);
    return true;
  }
  if (recorder) recorder->request(on);   // non-blocking; outcome in rec_status
  return true;
}

void RealActuator::set_sensor_offset(int mfps) {
  if (dry_run) {
    std::fprintf(stderr, "[dry-run] set_sensor_offset(%d)\n", mfps);
    return;
  }
#ifdef MABUR_HAVE_VENC
  (void)venc_set_sensor_offset(mfps);
#endif
}

void RealActuator::remember_channel(uint8_t ch) {
  if (dry_run) { std::fprintf(stderr, "[dry-run] remember_channel(%u)\n", ch); return; }
  if (!mabur::write_channel_file(mabur::kDroneChannelFile, ch))
    std::fprintf(stderr, "maburd: could not write %s (channel %u not remembered)\n",
                 mabur::kDroneChannelFile, static_cast<unsigned>(ch));
}

}  // namespace mabur::maburd
