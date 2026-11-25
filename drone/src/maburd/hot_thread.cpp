// Real mode: the hot (encode) thread body (real_mode.h).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <shared_mutex>
#include <string>
#include <thread>
#include <exception>
#include <variant>
#include <iterator>
#include <vector>

#include <unistd.h>  // _exit() — see the venc on_fault handler

#if defined(__linux__)
#include <dirent.h>       // repin_inherited_threads() walks /proc/self/task
#include <pthread.h>
#include <sched.h>  // cpu_set_t — see the core-placement policy below
#include <sys/stat.h>     // mkdir() — the trial proof files
#include <sys/syscall.h>  // SYS_gettid
#endif

#include "air_clock.h"
#include "build_ident.h"
#include "nack_bucket.h"
#include "air_rate.h"
#include "ampdu_policy.h"
#include "cal_apply.h"
#include "cal_link.h"
#include "cal_sweep.h"
#include "config.h"
#include "config_migrate.h"
#include "config_apply.h"
#include "mabur/config_write.h"
#include "control_tx_mode.h"
#include "debug_http.h"
#include "frame_pipeline.h"
#include "frame_source.h"
#include "live_power_table.h"
#include "mav_component.h"
#include "ota.h"
#include "trial_proof.h"
#include "mabur/channel_file.h"
#include "mabur/cal_mav.h"
#include "mabur/cal_proto.h"
#include "mabur/channel_set.h"
#include "mabur/frame_wire.h"
#include "mabur/ht40.h"
#include "mabur/msp_source.h"
#include "mabur/msp_status.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "mabur/fec_worker.h"
#include "mabur/uep_encoder.h"
#include "msp_serial.h"
#include "power_plan.h"
#include "power_tables.h"
#include "probe_source.h"
#include "radio_tx.h"
#include "rc_agent.h"
#include "mi_ready.h"
#include "telemetry.h"
#include "peak_rate.h"
#include "vtx_recorder.h"
#include "tick_gate.h"
#include "tx_queue.h"
#include "mabur/retx_ring.h"
#include "usb_tx_pool.h"
#include "send_timer.h"
#include "threads.h"
#include "radio_glue.h"
#include "real_actuator.h"
#include "real_mode.h"
#include "igi_floor_watchdog.h"
#ifdef MABUR_HAVE_VENC
#include "venc_core.h"  // ARM only: drone/venc is not compiled on host builds
#endif

#if defined(MABUR_DRY_RUN_ONLY)
// Not used; real mode is always compiled in, guarded at runtime by --dry-run
// so the same binary runs (in dry-run) on a machine with no dongle attached.
#endif

#include "AdapterCaps.h"
#include "AmpduMode.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "RxSense.h"
#include "SignalStop.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "IRtlRadio.h"
#include "WiFiDriver.h"
#include "logger.h"

#include <libusb.h>

namespace mabur::maburd {

using namespace mabur;

// Hot thread: pulls whole frames off the real SHM ring, runs them through
// the UEP pipeline, queues bodies for the TX writer. Owns the UepEncoder
// exclusively; never blocks on USB.
void hot_loop(RealMode& m) {
  name_thread("mbr-hot");
  if (two_core_target()) pin_self_to(kHotCore);
  // Ring name's single authority is the compile-time VENC_RING_NAME
  // (drone/venc/venc_cfg.h "/mabur_f"); there is no frame_ring_name config
  // key. venc_frame_ring_attach() normalises the leading '/' itself, so
  // this is behaviourally identical to the old default "mabur_f".
  FrameSource fsrc(VENC_RING_NAME);
  FramePipeline pipe;
  pipe.set_slice_geometry(
      venc_cfg_ctb64_rows(static_cast<uint16_t>(m.cfg.venc.core.height)),
      venc_cfg_slice_rows(static_cast<uint16_t>(m.cfg.venc.core.height), m.cfg.venc.core.slices));
  std::vector<uint8_t> fbuf(VENC_FRAME_META_SIZE + 512 * 1024);
  uint64_t last_reattach = 0;
  uint64_t last_ring_stats_ms = 0;
  bool vanish_boot_zeroed = false;  // first link-establish zeroes counters

  // Async FEC worker (promoted after hardware acceptance): always on.
  // Declared before the UepEncoder so engine dtors join their jobs first.
  //
  // The explicit pin is load-bearing, not tidiness: this thread is
  // spawned BY the hot thread, so it INHERITS the hot thread's core-1
  // affinity and lands on the producer's core — the one placement that
  // is strictly worse than no policy at all (measured:
  // cpu_us 11.5 ms and flush-join 3.2-3.7 ms, vs 9 ms / 0.4 ms
  // unpinned). Sending it to kRestCore is what makes the policy a win.
  FecWorker fec_worker(two_core_target() ? kRestCore : -1);
  UepEncoder uep(m.cfg.uep_layers(), m.cfg.fec.flush_ms, &fec_worker);
  uep.set_source_tap([&m](uint8_t sid, uint32_t seq, const uint8_t* env, size_t n) {
    if (sid == 0) m.retx.put(seq, env, n);  // base layer only (fec-nack)
  });

  // Probe stream: same FEC geometry as the enh layer
  // (block_payload/bpb), so a probe body is the same wire size as a video
  // body at that rung.
  const auto probe_layer = m.cfg.uep_layers()[1];
  ProbeSource probe_src(probe_layer.blocks_per_body,
                        static_cast<int>(mabur::sw::kSwHeaderLen) + probe_layer.fec.symbol_size,
                        std::random_device{}());

  std::shared_ptr<const AppliedOp> last_applied_op;
  // Debug-HTTP per-layer overhead override transition tracking (fix
  // round 1, review finding): true while the last tick saw both
  // ovr_base_pct/ovr_enh_pct armed. Needed because clearing the override
  // has NO bounded re-assert to fall back on -- run_bitrate_policy's 5 s
  // reassert (kReassertMs) only re-sends the bitrate/roi_qp verbs, never
  // touches the UEP layers' overhead, so without this the encoder would
  // stay pinned at the last override value indefinitely, until the next
  // genuine op change.
  bool ov_override_was_armed = false;

  // dq_split gauge (dq-spike follow-up): per-frame split of the
  // interval the wire's q_ms currently lumps together — venc-ring wait
  // (loop-top → read return) and FEC/SBI-pack CPU (read return → push).
  // Hot-thread-owned, reported on the 5 s ring-stats cadence.
  uint64_t split_n = 0;
  uint64_t split_ring_sum_us = 0, split_ring_max_us = 0;
  uint64_t split_cpu_sum_us = 0, split_cpu_max_us = 0;
  // sink_us: time inside the streaming sink (SBI patch + stamps +
  // txq.push/notify) — the per-body wakeup-chain half of cpu_us; the
  // remainder minus join_wait is the frag/GF-credit/SBI-pack feed itself.
  uint64_t split_sink_sum_us = 0;

  // fec_worker gauge (fec-compute handover): per-layer split of
  // dq_split's cpu_us into wait-on-worker (join spins) vs the hot thread's
  // own work, plus the worker's per-repair build cost and queue depth.
  // take_fec_gauge's sums are cumulative — keep last-window copies and
  // diff here; the maxima reset inside the take.
  SwEncoder::SwFecGauge fec_gauge_prev[UepEncoder::kNumStreams]{};

  while (!g_devourer_should_stop) {
    // Video quiesce: a calibration session owns the radio for its duration
    // (up to CalSweepCfg::hard_cap_ms, 180 s), so this thread stops reading
    // the ring/encoding/pushing bodies for as long as cal_active is set —
    // the TX writer thread has already stopped draining TxQueue in favor of
    // cal_sweep.pump(), so anything pushed here would just pile up behind
    // it. hot_beat still has to advance, though: the watchdog below aborts
    // the whole daemon on "hot thread stalled" if it doesn't see a beat
    // within stale_ms, and that check knows nothing about calibration.
    if (m.cal_active.load(std::memory_order_relaxed)) {
      m.hot_beat.fetch_add(1, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    uint64_t now = now_steady_ms();
    const uint64_t t0_us = now_steady_us();

    // Identity-compare, not generation-compare: see the matching comment
    // in dry_run.cpp's run_dry_run pump_op_change lambda. reapply_with_shed()
    // (congestion) never bumps generation, so gating on generation here
    // would silently drop local shed changes that arrive between two
    // RCF-driven generation bumps — including the congestion-triggered
    // shed this loop is specifically responsible for applying to the
    // UepEncoder (op.shed) independent of the GS.
    auto op = m.shared_op.load();
    if (op && op != last_applied_op) {
      apply_op_to_uep(*op, uep);
      apply_op_to_clock(*op, m.cfg.air_clock, m.air_clock);
      last_applied_op = op;
    }

    // Debug-HTTP per-layer overhead override (bench sweeps, :8301 POST
    // /venc/set?ov_base_pct=N&ov_enh_pct=N): armed (both >= 0) wins over
    // the op pair on the UEP layers every tick. Checked BEFORE this
    // tick's frame is read/encoded (not after, and not gated on a frame
    // having arrived) so the UEP layers already carry whichever value is
    // in effect this tick -- an armed->cleared transition re-applies the
    // op pair exactly once (not per-frame), so the encoder stops flying
    // the stale override in the same tick rather than one tick later.
    if (op) {
      const int ob = m.ov_override.ovr_base_pct.load(std::memory_order_relaxed);
      const int oe = m.ov_override.ovr_enh_pct.load(std::memory_order_relaxed);
      const bool armed = ob >= 0 && oe >= 0;
      if (armed) {
        uep.set_layer_overhead(0, ob / 100.0);
        uep.set_layer_overhead(1, oe / 100.0);
        ov_override_was_armed = true;
      } else if (ov_override_was_armed) {
        apply_op_to_uep(*op, uep);
        ov_override_was_armed = false;
      }
    }

    // One whole Annex-B frame per iteration: a frame is already the atomic
    // thing waybeam's producer publishes, so there's no "drain more" knob
    // here; the ring's own depth (default a handful of slots) is the backlog
    // buffer.
    VencFrameMeta meta{};
    int n = fsrc.read(fbuf.data(), fbuf.size(), 5, &meta);
    const uint64_t t_read_us = now_steady_us();
    if (n <= 0) {
      // Between frames (fec-join-delete): the frame-end flush does not wait
      // for the FEC worker, so the repairs it was still building ship here,
      // on the ring-read timeout, ≤5 ms after the frame. No AU owns these
      // bodies (au_first stays false, enc_us stays the packer's zero
      // placeholder, like poll's idle tail) but their air is real: book it
      // and release the feed_batch group.
      bool any = false;
      uep.collect([&](UepBody&& b) {
        const uint64_t p_us = now_steady_us();
        b.enqueued_ms = static_cast<uint32_t>(p_us / 1000);
        b.pushed_us = p_us;
        const size_t body_bytes = b.body.size();
        const int body_sid = b.stream_id;
        m.txq.push(std::move(b));
        m.air_clock.book(p_us, body_bytes, body_sid);
        any = true;
      });
      if (any) m.txq.flush();
    }
    if (n > 0) {
      // A parameter set rebuilt the pipeline: frames from here on carry
      // the new slice geometry. The rebuild also recreated the ring, so
      // this lands with the re-attach below; marked here as well in case
      // the re-attach was counted before the geometry was published.
      if (m.geometry_changed.exchange(false, std::memory_order_acquire)) {
        pipe.set_slice_geometry(m.live_ctb_rows.load(), m.live_slice_rows.load());
        pipe.mark_discontinuity();
        m.agent.note_chain_break();
      }
      if (fsrc.reattach_count() != last_reattach) {
        last_reattach = fsrc.reattach_count();
        pipe.mark_discontinuity();  // joined a new ring mid-GOP
        // The reattach skips the new ring's backlog, and with it the
        // rebuilt encoder's first IDR: without a fresh one the GS gets P
        // frames it cannot decode until something else asks.
        m.agent.note_chain_break();
      }
      if (m.link_up_discont.exchange(false, std::memory_order_relaxed)) {
        pipe.mark_discontinuity();  // link just came up: pre-link frames died
        // FIRST establish only: drop the boot-window vanish counts
        // (encoder bring-up churn, ~8-9/boot — a flight finding)
        // so telemetry reports in-flight vanishes. A mid-flight
        // re-establish must NOT erase in-flight counts.
        if (!vanish_boot_zeroed) {
          vanish_boot_zeroed = true;
          pipe.reset_vanish_counters();
        }
      }
      // Frame rate changed under us (low-power set_fps): re-anchor the
      // vanish period BEFORE this frame's pts is scored, so a 60 -> 15 fps
      // drop is learned as the new period, not booked as 3 holes/frame.
      if (m.fps_rate_change.exchange(false, std::memory_order_relaxed))
        pipe.note_rate_change();
      // Streaming push (dq-spike follow-up): each body goes to the TxQueue
      // the moment its SBI group seals, so the radio drains this frame's
      // early bodies in parallel with the remaining GF256/SBI packing — an
      // accumulate-then-push shape serialized ~3.4 ms of that CPU in front
      // of an idle radio (au_first queue wait measured ~40 µs).
      // enqueued_ms/pushed_us are per-body actual-push stamps now, so the
      // wire q_ms is the true TxQueue wait, and the enc_us patch rides
      // inside the sink (last_enc_us() latches before the first sink call —
      // see FramePipeline::encode's sink contract; the SBI header sits
      // outside the FEC envelope, so post-pack patching stays CRC-safe).
      // Air-clock admission: read the modelled backlog at this frame's
      // arrival, stamp it on every body of the AU (the decision input, so
      // airdrain.py can compare model vs measured per frame) and close the
      // enh gate while it is at/past shed_ms. 0 = observe only.
      const uint64_t t_arr_us = now_steady_us();
      const uint32_t backlog_us = m.air_clock.backlog_us(t_arr_us);
      const uint16_t air_ms = static_cast<uint16_t>(
          std::min<uint32_t>(backlog_us / 1000u, 65535u));
      pipe.set_enh_gate_closed(
          m.cfg.air_clock.shed_ms > 0 &&
          backlog_us >= static_cast<uint32_t>(m.cfg.air_clock.shed_ms) * 1000u);
      bool first = true;
      uint8_t au_sid = 0;
      pipe.encode(uep, fbuf.data(), static_cast<size_t>(n), meta, now,
                  [&](UepBody&& b) {
                    au_sid = b.stream_id;
                    const uint64_t s_us = now_steady_us();
                    mabur::sbi_set_enc_us(b.body.data(), b.body.size(),
                                          pipe.last_enc_us());
                    mabur::sbi_set_air_ms(b.body.data(), b.body.size(), air_ms);
                    const uint64_t p_us = now_steady_us();
                    b.enqueued_ms = static_cast<uint32_t>(p_us / 1000);
                    b.pushed_us = p_us;
                    b.au_first = first;
                    first = false;
                    const size_t body_bytes = b.body.size();
                    const int body_sid = b.stream_id;
                    m.txq.push(std::move(b));
                    m.air_clock.book(p_us, body_bytes, body_sid);
                    split_sink_sum_us += now_steady_us() - s_us;
                  });
      m.txq.flush();  // release the AU's partial feed_batch group, if any
      // Probe stream (per AU): one body at the tail of every video AU's
      // burst, base and enh -- the probe cadence is the AU rate, not the
      // enh rate. FIFO order through the TxQueue puts it on air after the
      // AU's last body, where the GS RcfSlotter's uplink blast can't reach
      // it. No AU on air (shed) => no probe (probe_follows).
      if (!first && op &&
          probe_follows(au_sid, op->probe_profile != rc::kNoProbeProfile,
                        au_sid < UepEncoder::kNumStreams && op->shed[static_cast<size_t>(au_sid)])) {
        UepBody pb = probe_src.build(op->probe_profile,
                                     static_cast<uint16_t>(pipe.next_frame_id() - 1));
        const uint64_t p_us = now_steady_us();
        pb.enqueued_ms = static_cast<uint32_t>(p_us / 1000);
        pb.pushed_us = p_us;
        m.air_clock.book(p_us, pb.body.size(), AirClock::kProbeSid);
        m.txq.push(std::move(pb));
        m.txq.flush();
      }
      // dq_split accounting: ring wait is loop-top → read return (the
      // interval during which this frame did not yet exist for us), CPU is
      // read return → all bodies pushed (fragmentation + GF256 + SBI pack,
      // now overlapped with the radio drain rather than in front of it).
      if (!first) {
        const uint64_t t_done_us = now_steady_us();
        const uint64_t ring_us = t_read_us - t0_us;
        const uint64_t cpu_us = t_done_us - t_read_us;
        ++split_n;
        split_ring_sum_us += ring_us;
        if (ring_us > split_ring_max_us) split_ring_max_us = ring_us;
        split_cpu_sum_us += cpu_us;
        if (cpu_us > split_cpu_max_us) split_cpu_max_us = cpu_us;
      }
      m.enc_frames_total.fetch_add(1, std::memory_order_relaxed);
      m.enc_bytes_total.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
    }
    // Ring-pressure observability (a future input for a drain-feedback
    // policy): one stderr line every 5 s.
    //
    // Only counters this process's CONSUMER handle can actually move.
    // venc_frame_ring_fill_t snapshots the local handle, and
    // writes/full_drops are incremented solely by the write path — they
    // are structurally 0 here, and the shm header carries
    // write_idx/read_idx but no counters, so the producer handle's copies
    // cannot cross into this one. fill_pct is the ring-wide signal
    // (write_idx - read_idx). Since the venc fold-in the producer is a
    // thread of THIS process, so its drop count is no longer unreachable:
    // it comes from venc_get_stats() (VencStats.full_drops), which reads
    // the encoder's own handle. This line stays consumer-side-only on
    // purpose, so the two provenances never blur.
    // See tests/test_frame_source.cpp
    // (consumer_fill_reports_only_consumer_side_counters).
    if (now - last_ring_stats_ms >= 5000) {
      last_ring_stats_ms = now;
      venc_frame_ring_fill_t f{};
      if (fsrc.fill(&f)) {
        std::fprintf(stderr,
            "maburd frame_ring: fill=%u%% (%u/%u) reads=%llu oversize=%llu "
            "bad_slot=%llu idr_disagree=%llu enhance_disagree=%llu "
            "vanished=%llu/%llu self_idr_refused=%llu slice_mismatch=%llu\n",
            f.fill_pct, f.used_slots, f.slot_count,
            (unsigned long long)f.reads,
            (unsigned long long)f.oversize_drops,
            (unsigned long long)f.bad_slot_drops,
            (unsigned long long)pipe.idr_disagreements(),
            (unsigned long long)pipe.enhance_disagreements(),
            (unsigned long long)pipe.vanished_base(),
            (unsigned long long)pipe.vanished_enhance(),
            (unsigned long long)pipe.self_idr_refused(),
            (unsigned long long)pipe.slice_mismatch());
      }
      // dq_split window report (dq-spike follow-up): the pre-push half of
      // the interval the wire q_ms spans. The post-push half (true queue
      // wait) is the tx thread's dq_queue line on the same cadence.
      if (split_n > 0) {
        std::fprintf(stderr,
            "maburd dq_split: n=%llu ring_us mean=%llu max=%llu "
            "cpu_us mean=%llu max=%llu sink_us mean=%llu\n",
            (unsigned long long)split_n,
            (unsigned long long)(split_ring_sum_us / split_n),
            (unsigned long long)split_ring_max_us,
            (unsigned long long)(split_cpu_sum_us / split_n),
            (unsigned long long)split_cpu_max_us,
            (unsigned long long)(split_sink_sum_us / split_n));
      }
      split_n = 0;
      split_ring_sum_us = split_ring_max_us = 0;
      split_cpu_sum_us = split_cpu_max_us = 0;
      split_sink_sum_us = 0;
      for (int sid = 0; sid < UepEncoder::kNumStreams; ++sid) {
        const auto g = uep.take_fec_gauge(sid);
        const auto& p = fec_gauge_prev[sid];
        const uint64_t jobs = g.jobs - p.jobs;
        const uint64_t build_us = g.build_us - p.build_us;
        const uint64_t joins = g.join_waits - p.join_waits;
        const uint64_t wait_us = g.join_wait_us - p.join_wait_us;
        const uint64_t frame_end = g.frame_end_repairs - p.frame_end_repairs;
        const uint64_t frame_end_us = g.frame_end_us - p.frame_end_us;
        if (jobs > 0 || joins > 0 || frame_end > 0 ||
            g.backlog_drops != p.backlog_drops) {
          std::fprintf(stderr,
              "maburd fec_worker sid=%d: jobs=%llu build_us/job=%llu "
              "inline=%llu joins=%llu join_wait_us mean=%llu max=%llu "
              "qdepth_max=%llu backlog_drops=%llu frame_end=%llu "
              "frame_end_us/rep=%llu\n",
              sid,
              (unsigned long long)jobs,
              (unsigned long long)(jobs ? build_us / jobs : 0),
              (unsigned long long)(g.inline_full - p.inline_full),
              (unsigned long long)joins,
              (unsigned long long)(joins ? wait_us / joins : 0),
              (unsigned long long)g.join_wait_max_us,
              (unsigned long long)g.enq_depth_max,
              (unsigned long long)(g.backlog_drops - p.backlog_drops),
              (unsigned long long)frame_end,
              (unsigned long long)(frame_end ? frame_end_us / frame_end : 0));
        }
        fec_gauge_prev[sid] = g;
      }
    }

    // Idle-tail flush: no single source frame owns these bodies (poll can
    // combine leftovers across several frames' worth of idle time), so
    // they carry no patched enc_us — left at the packer's zero placeholder
    // (0 = unknown on the wire).
    auto polled = uep.poll(now);
    if (!polled.empty()) {
      // Fresh sample, not t0_us: by here the loop top is up to a full
      // read-timeout + encode stale, and these bodies never waited on it.
      const uint64_t t_poll_us = now_steady_us();
      for (auto& b : polled) {
        // Idle-tail bodies carry no per-frame enc_us, but the queue wait is
        // still real once they reach the tx thread — stamp it here too.
        b.enqueued_ms = static_cast<uint32_t>(t_poll_us / 1000);
        b.pushed_us = t_poll_us;
        m.txq.push(std::move(b));
      }
      m.txq.flush();  // idle-tail bodies never wait on a feed_batch group
    }

    m.hot_beat.fetch_add(1, std::memory_order_relaxed);
  }
  m.txq.close();
}
}  // namespace mabur::maburd
