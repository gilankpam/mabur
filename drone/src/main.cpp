// maburd — drone-side daemon: reads whole encoded frames off the in-process
// venc core's frame-shm ring (or, in --dry-run, a fixture file), runs them
// through the UEP/FEC pipeline, and hands radio-bound bodies to the
// adaptive-link-controlled RadioTx. A parallel agent thread runs RcAgent against inbound RC
// frames + periodic radio-health ticks, publishing AppliedOp changes the hot
// path picks up via a lock-free shared_ptr handoff.
//
// Two modes:
//   maburd -c /etc/mabur.toml                     — real mode (devourer USB radio)
//   maburd -c cfg.toml --dry-run --in F --out F [--rc-in F]  — file-driven, no radio
//
// Dry-run is the tested path (see tests/fixtures/frame_stream.bin smoke test);
// real mode must compile and be structurally sound but is not exercised here
// (no bench hardware in this environment).
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
#include <sys/syscall.h>  // SYS_gettid
#endif

#include "air_clock.h"
#include "nack_bucket.h"
#include "air_rate.h"
#include "ampdu_policy.h"
#include "cal_apply.h"
#include "cal_link.h"
#include "cal_sweep.h"
#include "config.h"
#include "config_apply.h"
#include "config_write.h"
#include "control_tx_mode.h"
#include "debug_http.h"
#include "frame_pipeline.h"
#include "frame_source.h"
#include "mav_component.h"
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

namespace {

using namespace mabur;

// A GS at a different RC_VERSION is refused by rc::frame_type(), so its RCFs
// never reach the agent -- and, because the uplink RSSI/SNR EMAs are fed
// inside that same accepted-frame branch, drone.uplink.snr_* goes stale too.
// From the drone's side that is indistinguishable from "no GS is talking",
// and from the GS's side it looks like the stale-caps restart deadlock, which
// sends the operator to `restart maburd` -- which cannot help. So say it out
// loud, but rarely: a mismatched peer transmits continuously and /tmp is
// tmpfs, hence the once-per-5 s gate.
// Not thread-safe; only rx_callback (the RX thread) calls it.
void log_foreign_rc_version(uint8_t peer_ver) {
  using clock = std::chrono::steady_clock;
  static clock::time_point last{};
  const auto now = clock::now();
  if (last.time_since_epoch().count() != 0 &&
      now - last < std::chrono::seconds(5))
    return;
  last = now;
  std::fprintf(stderr,
               "maburd: heard an RC frame at RC_VERSION %u but this build "
               "speaks %u -- ignoring it (rate-limited to 1/5s). The pair is "
               "half-deployed: there is no control link and no video in "
               "either direction. Finish the deploy on BOTH ends; restarting "
               "maburd will not help.\n",
               static_cast<unsigned>(peer_ver),
               static_cast<unsigned>(rc::RC_VERSION));
}

// ---------------------------------------------------------------------------
// Sinks
// ---------------------------------------------------------------------------

// Writes u32_le len | frame records to a file (dry-run --out).
struct FileSink : mabur::FrameSink {
  FILE* f = nullptr;

  bool send(const uint8_t* p, size_t n) override {
    if (!f) return false;
    uint8_t hdr[4] = {
        static_cast<uint8_t>(n & 0xFF),
        static_cast<uint8_t>((n >> 8) & 0xFF),
        static_cast<uint8_t>((n >> 16) & 0xFF),
        static_cast<uint8_t>((n >> 24) & 0xFF),
    };
    if (std::fwrite(hdr, 1, 4, f) != 4) return false;
    if (n > 0 && std::fwrite(p, 1, n, f) != n) return false;
    return true;
  }
};

// u32_le len | body — same record framing as FileSink, for --msp-out.
void write_len_prefixed(FILE* f, const uint8_t* p, size_t n) {
  uint8_t hdr[4] = {static_cast<uint8_t>(n & 0xFF),
                    static_cast<uint8_t>((n >> 8) & 0xFF),
                    static_cast<uint8_t>((n >> 16) & 0xFF),
                    static_cast<uint8_t>((n >> 24) & 0xFF)};
  std::fwrite(hdr, 1, 4, f);
  if (n > 0) std::fwrite(p, 1, n, f);
}

MspSourceCfg to_msp_source_cfg(const MspCfg& m) {
  MspSourceCfg c;
  c.update_rate_hz = m.update_rate_hz;
  c.symbol_size = m.symbol_size;
  c.window = m.window;
  c.overhead = m.overhead;
  return c;
}

// Writer-priority handshake for tx_gate. glibc's pthread_rwlock is
// READER-PREFERRING by default: with video bodies, control frames and the
// calibration writer all taking the gate shared at a few kHz, a thread
// blocked in pthread_rwlock_wrlock can be overtaken indefinitely by newly
// arriving readers. The one writer is RealActuator::retune, which runs on the
// AGENT thread -- a starved writer there stalls the whole tick (RCF drain,
// telemetry, the ladder), so the retune must not be allowed to queue behind
// an unbounded reader stream. Every shared-taker spins on this flag first, so
// once retune raises it no NEW reader enters and the writer gets in after at
// most the readers already inside. The spin is yield-only and the window is
// the ~7 ms of a single FastRetune, once per channel move.
inline void await_retune_gate(const std::atomic<bool>* waiting) {
  if (!waiting) return;
  while (waiting->load(std::memory_order_acquire)) std::this_thread::yield();
}

// Wraps IRtlRadio::send_packet with a mutex — shared between the hot
// thread (video bodies) and the agent thread (send_control / DISC_ACK).
struct DevourerSink : mabur::FrameSink {
  IRtlRadio* dev = nullptr;
  std::mutex m;
  // Opened true only after InitWrite() finishes device bring-up (power-on,
  // firmware download, TX-path enable). Until then every send is dropped:
  // pushing frames into the chip's bulk-OUT FIFO *during* bring-up fills it
  // with packets the not-yet-booted MAC cannot transmit, which then starves
  // devourer's own reserved-page firmware download (its bulk-OUT to the same
  // endpoint times out) — the FW never boots and TX is bricked for the whole
  // session. Bench-confirmed on the SSC338Q: devourer's `doctor` brings the
  // same dongle up HEALTHY in isolation, while maburd's concurrent hot/agent
  // sends made DLFW fail after ~3 frames.
  std::atomic<bool>* ready = nullptr;

  // Parallel USB sender pool (radio.tx_threads > 1): send_many submits
  // frames here and returns immediately; N pool threads each block in
  // their own sync bulk transfer, keeping ~N URBs in flight (the chip
  // flow-controls sync URB acceptance — one blocking sender caps air at
  // ~26 Mbps regardless of MCS). Null = direct synchronous path.
  mabur::UsbTxPool* pool = nullptr;

  // Shared/exclusive gate against RealActuator::retune's FastRetune
  // (devourer threading contract: a control-plane call must not overlap a
  // bulk-OUT from any sender thread). Every USB sender here takes it
  // shared; retune takes it exclusive. Null in dry-run, where there is no
  // device to retune and no gate to take.
  std::shared_mutex* gate = nullptr;
  // Writer-priority flag paired with `gate` — see await_retune_gate above.
  std::atomic<bool>* gate_waiting = nullptr;

  bool send(const uint8_t* p, size_t n) override {
    if (ready && !ready->load(std::memory_order_acquire)) return false;
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    std::lock_guard<std::mutex> l(m);
    return dev->send_packet(p, n);
  }

  // Batch path: devourer's send_packets packs consecutive frames into
  // shared bulk-OUT URBs when tx.usb_agg_max > 0 (one transfer per batch
  // instead of one per frame). Same ready-gate as send(). With a pool,
  // "accepted" means queued to the senders (frames are copied; real air
  // failures surface via GetTxStats + the pool's own counters).
  size_t send_many(const View* frames, size_t n) override {
    if (ready && !ready->load(std::memory_order_acquire)) return 0;
    if (pool) {
      // Grouped enqueue: the whole batch lands under one pool lock with one
      // worker wakeup per 3 frames, so a feed_batch group becomes full
      // 3-descriptor URBs instead of splitting 1+1+1 across idle workers.
      size_t ok = 0;
      mabur::UsbTxPool::Frame g[16];
      while (ok < n) {
        const size_t k = std::min(n - ok, sizeof g / sizeof g[0]);
        for (size_t i = 0; i < k; ++i)
          g[i] = {frames[ok + i].data, frames[ok + i].len};
        const size_t accepted = pool->submit_many(g, k);
        ok += accepted;
        if (accepted < k) break;  // stopped
      }
      return ok;
    }
    std::vector<TxPacketView> v(n);
    for (size_t i = 0; i < n; ++i) v[i] = {frames[i].data, frames[i].len};
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    std::lock_guard<std::mutex> l(m);
    return dev->send_packets(v.data(), n);
  }
};

// ---------------------------------------------------------------------------
// RealActuator — bridges RcAgent to the radio/UEP/encoder world.
// ---------------------------------------------------------------------------

constexpr uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr size_t kDot11HeaderLen = 24;

std::vector<uint8_t> build_dot11_header(uint16_t seq) {
  std::vector<uint8_t> h(kDot11HeaderLen, 0);
  h[0] = 0x40;
  h[1] = 0x00;
  h[2] = 0x00;
  h[3] = 0x00;
  std::memset(h.data() + 4, 0xff, 6);
  std::memcpy(h.data() + 10, kCanonicalSa, 6);
  std::memcpy(h.data() + 16, kCanonicalSa, 6);
  uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  h[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  h[23] = static_cast<uint8_t>((seq_ctl >> 8) & 0xff);
  return h;
}

// The recorder's encoder side: drone/venc/venc_record.c on the drone, a
// channel that is never there on host builds (the recorder then reports
// Disabled on a start, which a host build can never be asked for anyway).
struct VencRecordChannel : mabur::RecordChannel {
  bool start() override {
#ifdef MABUR_HAVE_VENC
    return venc_record_start() == 0;
#else
    return false;
#endif
  }
  void stop() override {
#ifdef MABUR_HAVE_VENC
    venc_record_stop();
#endif
  }
  void request_idr() override {
#ifdef MABUR_HAVE_VENC
    venc_record_request_idr();
#endif
  }
};

// The radiotap of a control-modulation frame (control_tx_mode: DISC_ACK,
// Telem, MSP, MAVLink) for the live LDPC flag. A parameter set can flip
// that flag at run time: the setter stores the flag, then bumps the
// generation, and each sender rebuilds its header the first time it sees a
// generation other than the one it built at. One instance per sending
// thread, so no thread ever reads a vector another one rebuilds. Null
// pointers (dry run) mean LDPC on, built once.
struct ControlRadiotap {
  const std::atomic<bool>* ldpc = nullptr;
  const std::atomic<uint32_t>* gen = nullptr;

  const std::vector<uint8_t>& get() {
    const uint32_t g = gen ? gen->load(std::memory_order_acquire) : 0;
    if (rt_.empty() || g != built_gen_) {
      const bool on = ldpc ? ldpc->load(std::memory_order_acquire) : true;
      rt_ = devourer::build_stream_radiotap(control_tx_mode(on));
      built_gen_ = g;
    }
    return rt_;
  }

 private:
  std::vector<uint8_t> rt_;
  uint32_t built_gen_ = 0;
};

// The chip's per-width TX-power table: which width's wall-equalized
// rate-diff plan (power_tables.h) is live. Shared by every thread that
// programs it -- bring-up, the agent thread on a rung width change
// (RealActuator::apply_op), the config-apply thread on a RADIO_WALL_MARGN
// set and the TX writer thread for calibration. `mu_` serialises the choice
// of plan with its device write, so two writers can never leave the chip on
// one width's diffs while bw_ names the other. Every device call takes
// tx_gate shared (it must not overlap a retune's FastRetune), always inside
// mu_, never the other way round.
//
// Calibration owns the table while cal_active: follow() is a no-op then,
// checked under mu_, and begin_cal() raises cal_active under the same lock
// before the TX writer's first device write of a sweep. So a follow() that
// read cal_active clear has finished its write before the sweep zeroes the
// diffs, and none starts after.
//
// No plans (power_mode "none") means the chip's own efuse table: never
// written, except to put it back after calibration zeroed it.
struct LivePowerTable {
  IRtlRadio* dev = nullptr;
  std::shared_mutex* gate = nullptr;
  std::atomic<bool>* gate_waiting = nullptr;
  std::atomic<bool>* ready = nullptr;       // device_ready: no write before InitWrite is done
  std::atomic<bool>* cal_active = nullptr;
  // Bumped by every begin_cal(), after cal_active: the agent thread treats a
  // new epoch seen with cal_active clear as a calibration falling edge, even
  // one too brief for it to have seen cal_active up.
  std::atomic<uint32_t>* cal_epoch = nullptr;

  // TX writer, before a sweep's first device write: see the struct comment.
  // The chip's diffs stop being the live plan's until a follow()/replace().
  void begin_cal() {
    std::lock_guard<std::mutex> l(mu_);
    cal_active->store(true, std::memory_order_release);
    cal_epoch->fetch_add(1, std::memory_order_release);
    dirty_ = true;
  }

  // Agent thread: makes `bw`'s plan the live one unless it already is.
  void follow(uint8_t bw) {
    std::lock_guard<std::mutex> l(mu_);
    if (cal_active->load(std::memory_order_relaxed)) return;
    if (!ready->load(std::memory_order_acquire)) return;
    if (!dirty_ && bw_ == bw) return;
    (void)write_locked(bw, Write{});
  }

  // New plans, `bw`'s programmed at once (callers pass the rung's width as
  // they last saw it; a width change racing this is put right by the
  // agent's next follow(), which sees bw_ disagree). zero_offset also zeroes
  // the global offset beside a plan; clear_override then clears the flat
  // TXAGC override, in the same gate hold. False when SetTxPowerRateDiffs
  // refused the plan.
  bool replace(WidthPlans p, uint8_t bw, bool zero_offset, bool clear_override) {
    std::lock_guard<std::mutex> l(mu_);
    plans_ = std::move(p);
    Write w;
    w.zero_offset = zero_offset;
    w.clear_override = clear_override;
    return write_locked(bw, w);
  }

  // Calibration's apply: the plans a following capacity sweep and the run's
  // restore program, not programmed now -- the verify sweep that follows a
  // power apply flies the zeroed diffs under the parked override.
  void set_plans(WidthPlans p) {
    std::lock_guard<std::mutex> l(mu_);
    plans_ = std::move(p);
  }

  // Capacity sweep (TX writer, cal_active): `bw`'s plan, or the chip's own
  // table with no plan -- what production flies at that width.
  bool program_for_cal(uint8_t bw) {
    std::lock_guard<std::mutex> l(mu_);
    Write w;
    w.for_cal = true;
    return write_locked(bw, w);
  }

 private:
  struct Write {
    bool zero_offset = false;
    bool clear_override = false;
    bool for_cal = false;  // calibration keeps owning the table afterwards
  };

  bool write_locked(uint8_t bw, const Write& w) {
    const PowerPlan* p = plan_for(plans_, bw);
    bool ok = true;
    {
      await_retune_gate(gate_waiting);
      std::shared_lock<std::shared_mutex> sg;
      if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
      if (p) {
        devourer::TxRateDiffsQdb diffs;
        diffs.cck = p->cck;
        diffs.legacy = p->legacy;
        for (int i = 0; i < 8; ++i) diffs.mcs[i] = p->mcs[i];
        ok = dev->SetTxPowerRateDiffs(diffs);
        // devourer documents the offset as sticky across retunes: zeroed
        // explicitly rather than trusting whatever a prior process or
        // bench tool left in the chip.
        if (w.zero_offset) dev->SetTxPowerOffsetQdb(0);
      } else if (dirty_ || w.for_cal) {
        // No plan: back to the untrimmed efuse table calibration zeroed.
        ok = dev->SetTxPowerRateDiffs(std::nullopt);
      }
      if (w.clear_override) dev->SetTxPowerIndexOverride(-1);
    }
    if (p) {
      std::fprintf(stderr, "maburd radio: power table %d MHz live (mcs5 diff %d)%s%s\n", bw,
                   p->mcs[5], w.for_cal ? " for calibration" : "",
                   ok ? "" : " -- SetTxPowerRateDiffs FAILED, chip keeps its previous table");
    } else if (dirty_ || w.for_cal) {
      std::fprintf(stderr, "maburd radio: power table: chip default (no walls)%s\n",
                   ok ? "" : " -- SetTxPowerRateDiffs FAILED");
    }
    // A refused write is not retried: a chip without a rate-diff table
    // refuses every time, and the log line above says so once. A
    // calibration write leaves the table dirty and bw_ on the rung's width,
    // for the run's restore and the agent's follow() to put back.
    if (!w.for_cal) {
      bw_ = bw;
      dirty_ = false;
    }
    return ok;
  }

  std::mutex mu_;
  WidthPlans plans_;  // empty when power_mode != "offset"
  uint8_t bw_ = 0;    // the rung width whose plan was last programmed, 0 = none yet
  bool dirty_ = false;  // the chip's diffs are not plan_for(plans_, bw_)
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

  void apply_ampdu_for(const AppliedOp& op) {
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
  // Null in dry-run and in run_dry_run's own RealActuator (calibration is
  // real-mode only), set to run_real_mode's cal_active once it exists. Gates
  // the set_ladder() call below -- see that call site's comment for why a
  // second writer during a sweep is a measurement bug, not a memory-safety
  // one.
  std::atomic<bool>* cal_active = nullptr;
  // The per-width TX-power table (null in dry-run): apply_op makes the
  // op's width's plan live.
  LivePowerTable* power_table = nullptr;

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

  void apply_op(const AppliedOp& op) override {
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
      apply_ampdu_for(op);
      if (power_table) power_table->follow(op.ladder[1].bw);
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

  void send_control(const std::vector<uint8_t>& body) override {
    const std::vector<uint8_t>& control_radiotap = control_rt.get();
    std::vector<uint8_t> frame;
    frame.reserve(control_radiotap.size() + kDot11HeaderLen + body.size());
    frame.insert(frame.end(), control_radiotap.begin(), control_radiotap.end());
    auto hdr = build_dot11_header(control_seq);
    control_seq = static_cast<uint16_t>((control_seq + 1) & 0xFFF);
    frame.insert(frame.end(), hdr.begin(), hdr.end());
    frame.insert(frame.end(), body.begin(), body.end());
    sink->send(frame.data(), frame.size());
  }

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
  bool set_bitrate_kbps(int k) override {
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

  bool set_roi_qp(int q) override {
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

  bool set_fps(int f) override {
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

  void request_idr() override {
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

  // RcAgent calls this on a promoted session's agreed-channel move
  // ("disc"), an RCF hop order ("hop") and an unconfirmed move's return to
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
  // (DevicePowerCtl, below) from the TX writer thread for up to 180 s, and
  // nothing stops a retune being requested in that time: a hop order or a
  // promoted session's DISC move in an RCF that was already queued, or
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
  // Not host-testable: RealActuator lives in main.cpp and needs a real
  // IRtlRadio, so this comment is the specification.
  void retune(uint8_t ch, const char* reason) override {
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

  // Replays the retune retune() deferred, if any. Called from the agent
  // thread on the falling edge of cal_active (see main.cpp's agent loop),
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

  void retune_now_(uint8_t ch, const char* reason) {
    // Exclusive against every USB sender (DevourerSink/UsbTxPool take the
    // gate shared): FastRetune is a control-plane call and must not overlap
    // a bulk-OUT (devourer threading contract). ~2.4 ms on the 8812EU.
    const uint8_t from = cur;
    bool tx_power_ok = false;
    if (retune_waiting) retune_waiting->store(true, std::memory_order_release);
    {
      std::unique_lock<std::shared_mutex> g(*tx_gate);
      // The promote Telem that precedes a "disc" retune (main sends it on
      // RcAgent::take_session_promoted(), before tick() executes the
      // deferred move; sink->send, synchronous) has RETURNED from
      // send_packet but may still be sitting in the chip's TX FIFO -- the
      // GS commits the move on hearing that LINKED Telem arrive on the OLD
      // channel (link pairing); if FastRetune races it
      // out from under the Telem and it actually airs on the new channel
      // instead, the GS never hears it there and every move falls back to
      // the slower kMoveAfterRcfs path.
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
    cur = ch;
    std::fprintf(stderr, "maburd: retune %u -> %u (%s)\n", static_cast<unsigned>(from),
                 static_cast<unsigned>(ch), reason);
    std::fprintf(stderr,
                 "maburd: retune %u -> %u: tx power re-applied (%s)\n",
                 static_cast<unsigned>(from), static_cast<unsigned>(ch),
                 tx_power_ok ? "ok" : "failed");
  }

  mabur::VtxRecorder* recorder = nullptr;  // null until the recorder exists
  bool set_record(bool on) override {
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] set_record(%d)\n", on ? 1 : 0);
      return true;
    }
    if (recorder) recorder->request(on);   // non-blocking; outcome in rec_status
    return true;
  }

  void set_sensor_offset(int mfps) override {
    if (dry_run) {
      std::fprintf(stderr, "[dry-run] set_sensor_offset(%d)\n", mfps);
      return;
    }
#ifdef MABUR_HAVE_VENC
    (void)venc_set_sensor_offset(mfps);
#endif
  }

  // Auto channel set: RcAgent calls this once per confirmed move, from the
  // agent thread. A failed write is one stderr line, never fatal -- the drone
  // still parks on the right channel this session, it just won't remember it
  // across a restart.
  void remember_channel(uint8_t ch) override {
    if (dry_run) { std::fprintf(stderr, "[dry-run] remember_channel(%u)\n", ch); return; }
    if (!mabur::write_channel_file(mabur::kDroneChannelFile, ch))
      std::fprintf(stderr, "maburd: could not write %s (channel %u not remembered)\n",
                   mabur::kDroneChannelFile, static_cast<unsigned>(ch));
  }
};

uint64_t now_steady_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Names show in /proc/<pid>/task/<tid>/comm (16-char cap) — without them
// every maburd thread reads "maburd" next to the vendor SDK's unnamed
// threads, and the fec-compute forensics once spent an hour pinning
// the wrong tid. Names also let an operator taskset/renice by name.
void name_thread(const char* n) {
#if defined(__linux__)
  pthread_setname_np(pthread_self(), n);
#else
  (void)n;
#endif
}

// Core-placement policy (findings §19). The SSC338Q has two
// A7s and the frame producer is the pacer for the whole link: dq_split
// cpu_us is ~9 ms/frame unpinned but ~6.3 ms with the producer alone on a
// core, because every txq.push notify lets the TX writer preempt it
// mid-frame and the FEC worker migrates onto it (sink_us 1.9 -> 0.15 ms,
// GS arrival span 8.6 -> 7.9 ms, air spacing 382 -> 346 us).
//
// Applied as: process affinity -> core 0 BEFORE any thread exists (radio,
// venc SDK, USB pool, FEC worker all inherit it), then the hot thread
// moves itself to core 1. Cost: the FEC worker shares core 0 and slows to
// ~120 us/repair, so the producer's flush-join rises ~1 ms — a net win at
// today's geometry, NOT a free one. Revisit if per-frame repair count grows.
//
// Guarded on exactly-2 online CPUs: that is the deployment target, and it
// keeps a dev host (where this same binary builds and runs) from cramming
// every thread onto CPU 0.
constexpr int kHotCore = 1;
constexpr int kRestCore = 0;

// Moves every OTHER thread of this process whose comm equals `comm` onto
// `cpu`. A thread's children inherit its affinity AND its name, so a thread
// that pins itself somewhere unusual before calling into a library that
// spawns workers (devourer's InitWrite starts a periodic ticker) leaves
// those workers on that core with its name on them -- which is exactly how
// they are found again here. Returns the number moved.
int repin_inherited_threads(const char* comm, int cpu) {
#if defined(__linux__)
  const long self = syscall(SYS_gettid);
  int moved = 0;
  DIR* d = opendir("/proc/self/task");
  if (!d) return 0;
  while (dirent* e = readdir(d)) {
    if (e->d_name[0] == '.') continue;
    const long tid = std::atol(e->d_name);
    if (tid == self) continue;
    char path[64], name[32] = {0};
    std::snprintf(path, sizeof path, "/proc/self/task/%ld/comm", tid);
    FILE* f = std::fopen(path, "r");
    if (!f) continue;
    if (!std::fgets(name, sizeof name, f)) name[0] = '\0';
    std::fclose(f);
    name[strcspn(name, "\n")] = '\0';
    if (std::strcmp(name, comm) != 0) continue;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (sched_setaffinity(static_cast<pid_t>(tid), sizeof set, &set) == 0) ++moved;
  }
  closedir(d);
  return moved;
#else
  (void)comm; (void)cpu;
  return 0;
#endif
}
bool two_core_target() {
#if defined(__linux__)
  return sysconf(_SC_NPROCESSORS_ONLN) == 2;
#else
  return false;
#endif
}

void pin_self_to(int cpu) {
#if defined(__linux__)
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof set, &set) != 0)
    std::fprintf(stderr, "maburd: affinity pin to cpu%d failed\n", cpu);
#else
  (void)cpu;
#endif
}

// µs sibling for the dq_split gauge: the intervals it separates (venc-ring
// wait 0–5 ms, FEC/SBI CPU, queue wait) are each of the same order as
// now_steady_ms()'s 1 ms quantum, so a ms clock cannot split them.
uint64_t now_steady_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Applies a freshly-published AppliedOp (detected via shared_ptr identity,
// not generation — see the callers' pump-loop comments) to the
// hot-thread-owned UepEncoder: the commanded overhead PAIR applied directly
// per layer (RC_VERSION 5 — no ladder/scale translation) plus
// per-layer shed. Called from the hot thread only. The 2-slot AppliedOp's
// indices [0]/[1] map 1:1 onto the 2-stream UepEncoder's sids.
void apply_op_to_uep(const AppliedOp& op, UepEncoder& uep) {
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
void apply_op_to_clock(const AppliedOp& op, const AirClockCfg& c, AirClock& clock) {
  const double probe_mbps = op.probe_profile != rc::kNoProbeProfile
                                ? delivered_mbps(op.probe, c) : 0.0;
  clock.set_rates(delivered_mbps(op.ladder[0], c), delivered_mbps(op.ladder[1], c),
                  probe_mbps, static_cast<uint32_t>(c.body_us));
}

// ---------------------------------------------------------------------------
// Dry-run mode
// ---------------------------------------------------------------------------

struct RcInRecord {
  // Wire field (u32-LE in the --rc-in file): deliver this frame once that many
  // video frames have been consumed.
  uint32_t after_frame_index;
  std::vector<uint8_t> body;
};

// One record of a --dry-run frame file: the whole-frame records waybeam's
// frame-shm ring publishes, serialized as
//   u32-LE record length | VencFrameMeta (8 B) | Annex-B frame
// The buffer keeps the meta room up front exactly as FrameSource::read fills
// it, so FramePipeline can stamp the FrameHdr over it in place.
struct DryRunFrame {
  VencFrameMeta meta{};
  std::vector<uint8_t> buf;  // VENC_FRAME_META_SIZE + payload
  size_t payload_len() const { return buf.size() - VENC_FRAME_META_SIZE; }
};

std::vector<DryRunFrame> read_frame_file(const std::string& path) {
  std::vector<DryRunFrame> out;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return out;
  while (true) {
    uint8_t lenb[4];
    if (std::fread(lenb, 1, 4, f) != 4) break;
    const uint32_t len = static_cast<uint32_t>(lenb[0]) |
                         (static_cast<uint32_t>(lenb[1]) << 8) |
                         (static_cast<uint32_t>(lenb[2]) << 16) |
                         (static_cast<uint32_t>(lenb[3]) << 24);
    if (len < VENC_FRAME_META_SIZE) break;
    DryRunFrame fr;
    fr.buf.resize(len);
    if (std::fread(fr.buf.data(), 1, len, f) != len) break;
    std::memcpy(&fr.meta, fr.buf.data(), VENC_FRAME_META_SIZE);
    out.push_back(std::move(fr));
  }
  std::fclose(f);
  return out;
}

std::vector<uint8_t> read_whole_file(const std::string& path) {
  std::vector<uint8_t> v;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return v;
  uint8_t buf[4096];
  size_t r;
  while ((r = std::fread(buf, 1, sizeof buf, f)) > 0) v.insert(v.end(), buf, buf + r);
  std::fclose(f);
  return v;
}

std::vector<RcInRecord> read_rc_in(const std::string& path) {
  std::vector<RcInRecord> recs;
  if (path.empty()) return recs;
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) {
    std::fprintf(stderr, "warning: cannot open --rc-in '%s'\n", path.c_str());
    return recs;
  }
  while (true) {
    uint8_t hdr[8];
    if (std::fread(hdr, 1, 8, f) != 8) break;
    uint32_t after = static_cast<uint32_t>(hdr[0]) | (static_cast<uint32_t>(hdr[1]) << 8) |
                     (static_cast<uint32_t>(hdr[2]) << 16) | (static_cast<uint32_t>(hdr[3]) << 24);
    uint32_t len = static_cast<uint32_t>(hdr[4]) | (static_cast<uint32_t>(hdr[5]) << 8) |
                   (static_cast<uint32_t>(hdr[6]) << 16) | (static_cast<uint32_t>(hdr[7]) << 24);
    std::vector<uint8_t> body(len);
    if (len > 0 && std::fread(body.data(), 1, len, f) != len) break;
    recs.push_back(RcInRecord{after, std::move(body)});
  }
  std::fclose(f);
  return recs;
}

int run_dry_run(const Config& cfg, uint8_t start_ch, const std::string& in_path,
                const std::string& out_path, const std::string& rc_in_path,
                const std::string& msp_in_path, const std::string& msp_out_path) {
  FileSink file_sink;
  file_sink.f = std::fopen(out_path.c_str(), "wb");
  if (!file_sink.f) {
    std::fprintf(stderr, "error: cannot open --out '%s'\n", out_path.c_str());
    return 1;
  }

  RadioTx tx(file_sink);

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  RealActuator actuator;
  actuator.tx = &tx;
  actuator.sink = &file_sink;
  actuator.shared_op = &shared_op;
  actuator.dev = nullptr;
  actuator.dry_run = true;

  RcAgent agent(cfg, actuator, nullptr, start_ch);
  agent.install_session_for_replay(1, 1);  // --rc-in frames are tagged under (1,1) by tests/integration/mabur_rc.py
  // Debug endpoint is startable here too (no MABUR_HAVE_VENC on a host
  // build, so every route just answers "disabled") -- keeps host/dry-run
  // and real mode on one code path instead of special-casing it out.
  debug_http_start(cfg.venc.debug_port, cfg.venc.core.snapshot_quality);
  // Deterministic replay output: the async FEC worker is never attached in
  // dry-run mode (repair emission order would depend on thread timing).
  UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms);

  // Probe stream: same FEC geometry as the enh layer
  // (block_payload/bpb), so a probe body is the same wire size as a video
  // body at that rung.
  const auto probe_layer = cfg.uep_layers()[1];
  ProbeSource probe_src(probe_layer.blocks_per_body,
                        static_cast<int>(mabur::sw::kSwHeaderLen) + probe_layer.fec.symbol_size,
                        std::random_device{}());

  auto frames = read_frame_file(in_path);
  FramePipeline pipe;
  pipe.set_slice_geometry(
      venc_cfg_ctb64_rows(static_cast<uint16_t>(cfg.venc.core.height)),
      venc_cfg_slice_rows(static_cast<uint16_t>(cfg.venc.core.height), cfg.venc.core.slices));
  auto rc_recs = read_rc_in(rc_in_path);
  size_t rc_idx = 0;

  std::shared_ptr<const AppliedOp> last_applied_op;

  uint64_t sent_bodies = 0;
  uint64_t consumed_frames = 0;

  // First tick: BOOT -> RENDEZVOUS, applies MAX_RANGE op.
  agent.tick(now_steady_ms(), RadioHealth{});

  // Detects "a new AppliedOp was published" by shared_ptr IDENTITY, not by
  // generation: reapply_with_shed() (congestion) publishes a fresh AppliedOp
  // via apply_op() WITHOUT bumping generation (by design — generation
  // tracks new operating points, not shed adjustments to the current one),
  // but every apply_op() call, including reapplies, always stores a
  // brand-new shared_ptr<const AppliedOp>. Comparing against generation
  // alone would silently miss local congestion shed changes whenever no new
  // RCF/DISC/failsafe op happened in between — the exact bug this
  // identity-compare fixes.
  auto pump_op_change = [&]() {
    auto op = shared_op.load();
    if (op && op != last_applied_op) {
      apply_op_to_uep(*op, uep);
      // Dry-run deliberately does not call apply_op_to_clock: the air
      // clock is not priced here, so replayed bodies carry air_ms 0.
      last_applied_op = op;
    }
  };
  pump_op_change();

  // Drain semantics: deliver every RC record whose after_frame_index is
  // <= consumed_frames, in file order, regardless of whether that exact
  // count was ever hit as a distinct step. A strict `==` check (the
  // previous implementation) blocks forever on a record whose index is 0
  // (never equal to consumed_frames, which is incremented starting from
  // 1), duplicated, out-of-order, or >= frames.size() — and because rc_idx
  // only ever advances past a match, one stuck record wedges every
  // subsequent record too. Draining on `<=` delivers exact matches at the
  // same point as before (e.g. after_frame_index=3 still lands right
  // after the 3rd frame is consumed) while guaranteeing every record is
  // eventually delivered — before the loop for index 0, inline as the
  // frame count catches up, and at EOF for anything left over.
  auto drain_rc_records = [&](uint64_t now, bool drain_all = false) {
    uint64_t gate = drain_all ? UINT64_MAX : consumed_frames;
    while (rc_idx < rc_recs.size() && rc_recs[rc_idx].after_frame_index <= gate) {
      agent.on_rc_frame(rc_recs[rc_idx].body.data(), rc_recs[rc_idx].body.size(), now);
      std::fprintf(stderr, "[dry-run] rc-in delivered record %zu (after_frame_index=%u) at consumed_frames=%llu\n",
                  rc_idx, rc_recs[rc_idx].after_frame_index,
                  static_cast<unsigned long long>(consumed_frames));
      ++rc_idx;
    }
  };

  // Deliver any records due before the first frame (after_frame_index=0).
  drain_rc_records(now_steady_ms());

  for (size_t i = 0; i < frames.size(); ++i) {
    uint64_t now = now_steady_ms();

    pump_op_change();

    // Same ingest step as the real hot thread (classify, IDR protect-up,
    // FrameHdr stamp), so replayed bytes are the bytes the drone would send.
    auto bodies = pipe.encode(uep, frames[i].buf.data(), frames[i].payload_len(),
                              frames[i].meta, now);
    for (auto& b : bodies) {
      tx.send_body(b.stream_id, b.body.data(), b.body.size());
      ++sent_bodies;
    }

    // Probe stream (per AU): one body at the tail of every video AU's burst,
    // base or enh, sent right after the AU's own bodies so it lands at the
    // tail of the burst on the wire too. No AU on air (shed) => no probe
    // (probe_follows).
    if (!bodies.empty()) {
      const int au_sid = bodies.back().stream_id;
      auto op_now = shared_op.load();
      if (op_now && probe_follows(au_sid, op_now->probe_profile != rc::kNoProbeProfile,
                                  au_sid < UepEncoder::kNumStreams && op_now->shed[static_cast<size_t>(au_sid)])) {
        UepBody pb = probe_src.build(op_now->probe_profile,
                                     static_cast<uint16_t>(pipe.next_frame_id() - 1));
        tx.send_body(pb.stream_id, pb.body.data(), pb.body.size());
        ++sent_bodies;
      }
    }

    auto polled = uep.poll(now);
    for (auto& b : polled) {
      tx.send_body(b.stream_id, b.body.data(), b.body.size());
      ++sent_bodies;
    }

    ++consumed_frames;

    // Deliver any RC records due at or before this frame index, then tick
    // the agent (simulated radio health: empty/no drops).
    drain_rc_records(now);
    agent.tick(now, RadioHealth{});
    pump_op_change();
  }

  // EOF: deliver any records left over (duplicate/out-of-order/>=
  // frames.size() indices), flush every layer, send whatever falls out, then
  // stop.
  uint64_t now = now_steady_ms();
  drain_rc_records(now, true);
  auto flushed = uep.flush_all();
  for (auto& b : flushed) {
    tx.send_body(b.stream_id, b.body.data(), b.body.size());
    ++sent_bodies;
  }

  std::fclose(file_sink.f);

  if (!msp_in_path.empty() && !msp_out_path.empty()) {
    FILE* mf = std::fopen(msp_out_path.c_str(), "wb");
    if (mf) {
      MspSource msp(to_msp_source_cfg(cfg.msp),
                    [&](const uint8_t* body, size_t n){ write_len_prefixed(mf, body, n); });
      auto bytes = read_whole_file(msp_in_path);
      // Advance the clock past the rate-gate period per feed chunk so every
      // captured screen forwards (gate correctness is unit-tested separately).
      uint64_t clk = 0;
      for (size_t off = 0; off < bytes.size(); off += 64, clk += 10000)
        msp.on_serial_bytes(bytes.data() + off, std::min<size_t>(64, bytes.size() - off), clk);
      std::fclose(mf);
      std::fprintf(stderr, "[dry-run] msp: snapshots_sent=%llu\n",
                   static_cast<unsigned long long>(msp.snapshots_sent()));
    }
  }

  std::fprintf(stderr,
              "maburd dry-run stats: frames_in=%zu bodies_out=%llu seq=%u sent=%llu drops=%llu "
              "agent_state=%d rc_records=%zu/%zu idr_disagree=%llu enhance_disagree=%llu\n",
              frames.size(), static_cast<unsigned long long>(sent_bodies), tx.seq(),
              static_cast<unsigned long long>(tx.sent()), static_cast<unsigned long long>(tx.drops()),
              static_cast<int>(agent.state()), rc_idx, rc_recs.size(),
              static_cast<unsigned long long>(pipe.idr_disagreements()),
              static_cast<unsigned long long>(pipe.enhance_disagreements()));
  return 0;
}

// ---------------------------------------------------------------------------
// Real mode
// ---------------------------------------------------------------------------

std::atomic<bool> g_sigusr1_flag{false};

void handle_sigusr1(int) { g_sigusr1_flag.store(true); }

// SIGINT/SIGTERM shutdown: devourer's IRtlRadio::Init() runs a blocking RX
// loop that only observes the library-global g_devourer_should_stop (see
// ../devourer/src/SignalStop.h and how examples/rx, examples/tx, and
// examples/doctor all call install_devourer_signal_handlers() instead of
// installing their own SIGINT/SIGTERM handlers). A locally-scoped flag set by
// a handler main.cpp installs itself is never consulted by Init()'s loop, so
// Ctrl-C/SIGTERM would never unblock it. The hot/agent thread loops below
// read g_devourer_should_stop directly too, so all three (Init()'s RX loop,
// hot_thread, agent_thread) stop on the same signal.

// Small mutex-guarded deque standing in for an SPSC queue — RC control
// traffic arrives at ~10 Hz, far below anything a mutex can't absorb.
struct RcQueue {
  std::mutex m;
  std::deque<std::vector<uint8_t>> q;

  void push(const uint8_t* p, size_t n) {
    std::lock_guard<std::mutex> l(m);
    q.emplace_back(p, p + n);
  }
  bool pop(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> l(m);
    if (q.empty()) return false;
    out = std::move(q.front());
    q.pop_front();
    return true;
  }
};

// Calibration messages from the MAV thread (which verifies each T_MAV and
// decodes its cal packets) to the TX writer thread (CalSweep's owner).
// `latched` marks a frame check_mav accepted under the calibration run pair
// rather than the published session (RcAgent::MavCheck::kOkLatched). The
// GS resends at most every 200 ms and the writer drains every pass, so the
// cap only bounds a stalled writer; the oldest message goes first.
struct CalInbound {
  cal::CalMsg msg;
  bool latched = false;
};
struct CalMsgQueue {
  static constexpr size_t kCap = 64;
  std::mutex m;
  std::deque<CalInbound> q;

  void push(CalInbound in) {
    std::lock_guard<std::mutex> l(m);
    if (q.size() >= kCap) q.pop_front();
    q.push_back(std::move(in));
  }
  bool pop(CalInbound* out) {
    std::lock_guard<std::mutex> l(m);
    if (q.empty()) return false;
    *out = std::move(q.front());
    q.pop_front();
    return true;
  }
};

uint16_t open_usb_and_get_pid(uint16_t vid, uint16_t configured_pid,
                              libusb_context* ctx, libusb_device_handle** out_handle) {
  std::vector<uint16_t> pids;
  if (configured_pid != 0) {
    pids.push_back(configured_pid);
  } else {
    // Scan order: 8812EU, then the Jaguar3 8822C dies (RTL8812CU c812,
    // RTL8822CU c82c -- the EMAX Wyvern Link board), then 8812AU. Which of
    // these the binary can actually drive is devourer's build-time chip
    // selection; an unbuilt chip fails at CreateRadio, not here.
    pids = {0xa81a, 0xc812, 0xc82c, 0x881a, 0x8812};
  }
  for (uint16_t pid : pids) {
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (h) {
      *out_handle = h;
      return pid;
    }
  }
  *out_handle = nullptr;
  return 0;
}

int run_real_mode(const Config& cfg, uint8_t start_ch, const std::string& cfg_path) {
  // Before libusb_init and the venc bring-up, so every thread either
  // library spawns inherits core 0; the hot thread claims core 1 itself.
  if (two_core_target()) {
    pin_self_to(kRestCore);
    std::fprintf(stderr, "maburd: core policy = producer on cpu%d, rest on cpu%d\n",
                 kHotCore, kRestCore);
  }
  std::signal(SIGUSR1, handle_sigusr1);
  // Installs the SIGINT/SIGTERM handlers that set g_devourer_should_stop —
  // the flag IRtlRadio::Init()'s blocking RX loop actually watches.
  install_devourer_signal_handlers();

  // Cold boot: the MI kernel modules are loaded by maburd itself (see the
  // wait before venc_core_start). Kicked off HERE, on a thread, so the
  // ~0.7 s insmod chain runs under the USB port reset below -- an idle wait
  // -- rather than after it. Warm restarts find the modules live and skip.
  // The same thread then dlopens the MI libraries (venc_core_preload, ~0.4 s
  // of squashfs page-in cold) -- sequenced after the insmods because both are
  // NOR-bound and would only contend, and off the main thread so the radio's
  // USB reset + InitWrite (USB-bound waits) run alongside. Warm restarts:
  // modules live, libraries page-cached -- a few ms.
  std::thread module_loader_thread;
  struct JoinOnExit {  // every early `return 1` below must not terminate() on a live thread
    std::thread& t;
    ~JoinOnExit() { if (t.joinable()) t.join(); }
  } module_loader_join{module_loader_thread};
  {
    // The OpenIPC script that insmods the MI stack and the sensor driver --
    // a fixed property of the image maburd is built for, not a knob (it was
    // briefly a config key; nothing ever needed to change it).
    static constexpr const char* kModuleLoader = "/usr/bin/load_sigmastar -i";
    const bool run_loader = !mabur::mi_modules_live("/sys/module");
    if (run_loader)
      std::fprintf(stderr, "MI modules not live: running `%s`\n", kModuleLoader);
    module_loader_thread = std::thread([run_loader]() {
      if (run_loader) {
        const int lrc = std::system(kModuleLoader);
        if (lrc != 0) std::fprintf(stderr, "warning: module loader exited %d\n", lrc);
      }
#ifdef MABUR_HAVE_VENC
      venc_core_preload();
#endif
    });
  }

  auto logger = std::make_shared<Logger>();
  // devourer has two independent output channels and this daemon wants both
  // quiet. set_level() gates only the human diagnostics (logger->info/warn/…);
  // the JSON event stream is gated solely by EventSink::enabled(), which
  // defaults to stdout + enabled + flush-per-line. S96mabur redirects stdout
  // into the RAM-backed /tmp/mabur.log, so jaguar3's per-URB "tx.agg" event
  // (~600/s at video rate) wrote ~1.5 MB/min and filled the drone's 45 MB
  // /tmp in ~30 min — after which every log write failed silently. Nothing is
  // lost by muting the stream: our stats line already carries tx_failed=.
  logger->set_level(Logger::Level::Warn);
  // MABUR_DEVOURER_EVENTS=1 keeps the stream on for a bench run from /tmp
  // (e.g. devourer's per-stage `init.timing` events, which are how the radio
  // bring-up is attributed -- docs/findings/2026-09-07-boot-time.md). Never
  // set it under the S00mabur wrapper: see the /tmp-filling note above.
  if (const char* ev = std::getenv("MABUR_DEVOURER_EVENTS"); ev && ev[0] == '1') {
    // A private dup of stdout: the venc's sdk_quiet brackets dup2() /dev/null
    // over fds 1 and 2 process-wide, and the radio bring-up (and its timing
    // events) runs concurrently on its own thread.
    if (FILE* f = fdopen(dup(fileno(stdout)), "w")) logger->events().configure(f);
  } else {
    logger->events().disable();
  }

  libusb_context* usb_ctx = nullptr;
  int rc = libusb_init(&usb_ctx);
  if (rc < 0) {
    std::fprintf(stderr, "error: libusb_init failed (%d)\n", rc);
    return 1;
  }

  libusb_device_handle* handle = nullptr;
  uint16_t pid = open_usb_and_get_pid(cfg.radio.usb_vid, cfg.radio.usb_pid, usb_ctx, &handle);
  if (!handle) {
    std::fprintf(stderr, "error: no radio found under VID 0x%04x\n", cfg.radio.usb_vid);
    libusb_exit(usb_ctx);
    return 1;
  }
  std::fprintf(stderr, "opened device %04x:%04x\n", cfg.radio.usb_vid, pid);

  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  rc = devourer::claim_interface_then_reset(handle, 0, logger, /*do_reset=*/true, usb_lock);
  if (rc != 0) {
    std::fprintf(stderr, "error: claim_interface_then_reset failed (%d)\n", rc);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 1;
  }

  // Jaguar3 TX+RX on one claimed handle: enable_with_tx makes InitWrite keep
  // the RX filters open so a later StartRxLoop can run concurrently with TX
  // (mirrors devourer's doctor/streamtx examples). Retrofitting RX after a
  // plain InitWrite is unreliable on this chip.
  devourer::DeviceConfig dev_cfg;
  dev_cfg.rx.enable_with_tx = true;
  // USB TX aggregation: pack up to 3 frames (the HalMAC per-transfer
  // descriptor limit) into one bulk-OUT URB via send_packets — amortizes
  // the per-URB tax that capped inline per-frame injection at ~2500 fps.
  dev_cfg.tx.usb_agg_max = 3;
  // MAC carrier sense ON (the chip default; devourer leaves 0x520[14]/[15]
  // alone and lets phydm track the EDCCA thresholds). The argument for OFF is
  // that the downlink owns its channel and CSMA only stutters it (devourer
  // measured injection deferring 41-45% to a co-channel 802.11 transmitter,
  // tests/dis_cca_tx_onair.sh). Two things overturn that: with both ends
  // blind, the GS's own uplink send kills ~1.5 drone PPDUs/s on BOTH ground
  // cards (2-3 % of the enh layer repaired from parity, +2 ms fec p99), which
  // carrier sense on both ends removes outright -- 0 repairs, RCF delivery 90
  // -> 97 % -- and every recorded session on the DVR showed a 0/s
  // foreign-preamble channel, so the deferral being avoided was never there
  // (docs/tx-rx-timing.md §3, docs/findings/2026-09-23-cca-on.md). The
  // altitude view the ground cards cannot see rides Telem
  // rx_foreign/rx_crcfail; if a flight shows the drone deferring to
  // neighbours the hop verdict ignores (foreign_pps < 50), that is the number
  // to act on. GS-only carrier sense is NOT a fallback: it defers to the
  // burst edge where a blind drone restarts
  // (docs/findings/2026-08-14-rcf-uplink-loss.md §6).
  dev_cfg.tuning.disable_cca = false;
  // devourer's wide-TX IGI floor: off on an 8812EU image, devourer's default
  // on an 8812CU one. The floor raises the DIG lower bound for the rest of
  // the session once the first >= 40 MHz frame goes out, because on a silent
  // channel the receiver can park where the secondary 20 MHz never reads
  // idle and the MAC holds that frame forever. The shipped ladder airs every
  // rung at 40 MHz, so a floor is on from the first video frame and costs
  // the RCF uplink idle-channel sensitivity for the whole flight (devourer
  // estimates ~1 dB per IGI step; its default 0x35 is ~20 steps over the
  // 0x1e-0x22 window). The 8812EU flew the 40 MHz rungs with no floor and
  // never stalled; the 8822C is the die the stall was found on (one CU unit
  // only clears at 0x35), so it keeps the default. The choice is per image,
  // not per card: each drone build compiles exactly one Jaguar3 die
  // (tools/build-arm.sh and the openipc-builder mabur package), and a binary
  // refuses the other die at CreateRadio. An 8812EU that does stall loses the
  // downlink at its first 40 MHz frame with the chip's TX FIFO full; give it
  // a floor then (devourer measured one EU unit that clears at 0x2c, not
  // 0x20).
#if defined(DEVOURER_HAVE_JAGUAR3_8822E) && !defined(DEVOURER_HAVE_JAGUAR3_8822C)
  dev_cfg.tuning.wide_tx_igi_floor = 0;
#endif
  // Pinned here now that devourer defaults both the other way. A multi-packet
  // bulk-OUT that times out mid-transfer wedges the 8822EU's TX endpoint for
  // the rest of the session, so data sends never cancel one (the cost -- a
  // send can block while the chip NAKs -- is devourer's DeviceConfig doc).
  // Zerocopy RX went deaf intermittently on an xhci host; the heap path has
  // not.
  dev_cfg.tx.no_cancel_multipkt = true;
  dev_cfg.usb.rx_zerocopy = false;

  WiFiDriver wifi_driver{logger};
  // mabur drives Realtek-only controls (TX-power tables, FastRetune, the
  // energy reads), so the radio must be an IRtlRadio; anything else is
  // refused here like an unsupported chip.
  std::unique_ptr<IRtlRadio> rtl_device;
  if (auto radio = wifi_driver.CreateRadio(handle, usb_ctx, usb_lock, dev_cfg);
      radio && dynamic_cast<IRtlRadio*>(radio.get()))
    rtl_device.reset(static_cast<IRtlRadio*>(radio.release()));
  if (!rtl_device) {
    std::fprintf(stderr, "error: CreateRadio failed (unsupported or non-Realtek chip, or already in use)\n");
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 1;
  }


  // Radio bring-up on its own thread, started HERE so it overlaps the venc
  // bring-up below instead of following it. Measured
  // (docs/findings/2026-09-07-boot-time.md, "maburd's own startup, stamped"):
  // InitWrite is 1.73 s of power-on + firmware download + TX enable, all of
  // it USB-bound; run strictly after venc_core_start (~1.5 s on a cold boot),
  // the encoder would produce frames into the void for the whole of it. The
  // two touch disjoint hardware (USB radio vs the SigmaStar MI stack), and
  // nothing transmits until device_ready opens below, which still happens
  // after this thread is joined and the TX power / A-MPDU writes have run on
  // the main thread in their old order.
  //
  // InitWrite throws on a USB/firmware failure. Today that escapes main()
  // and the wrapper respawns; capturing it here and rethrowing after the
  // join keeps exactly that behaviour instead of turning it into a
  // std::terminate inside a thread.
  std::exception_ptr radio_init_error;
  std::thread radio_init_thread([&]() {
    name_thread("mbr-radio-init");
    // Off core 0 for the duration of the bring-up. This thread inherits the
    // main thread's kRestCore pin, and on a cold boot that core also hosts
    // the venc bring-up (main thread) and, from ~1 s in, the encoder thread
    // at SCHED_FIFO 50 -- InitWrite measured 2.12 s cold against 1.33 s on
    // a warm restart, where the venc side sleeps in the kernel and it has
    // the core to itself. kHotCore is idle until video flows (the hot
    // thread only spins on an empty ring), so the radio borrows it and
    // exits before there is anything to contend with.
    if (two_core_target()) pin_self_to(kHotCore);
    try {
      const uint8_t ch = start_ch;
      rtl_device->InitWrite(cfg.radio.width == 40
                                ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                                : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
    } catch (...) {
      radio_init_error = std::current_exception();
    }
    // Undo the borrow for anything InitWrite left behind: devourer spawns a
    // periodic thread inside it, and that thread inherited this thread's
    // core-1 pin (and its name, which is how it is found). The policy is
    // "every library thread on kRestCore"; put it back there.
    if (two_core_target()) repin_inherited_threads("mbr-radio-init", kRestCore);
  });
  // Joins the bring-up thread; must run on every path out of this function
  // that releases the USB handle, or InitWrite keeps driving a device that
  // has been closed under it.
  auto join_radio_init = [&]() {
    if (radio_init_thread.joinable()) radio_init_thread.join();
  };

  // Gate for DevourerSink: stays false until InitWrite() completes bring-up.
  std::atomic<bool> device_ready{false};

  // Exclusive against RealActuator::retune's FastRetune (devourer threading
  // contract: control-plane calls must not overlap a bulk-OUT from any
  // sender thread). Every USB sender in this function -- DevourerSink::send/
  // send_many's direct branch, and the UsbTxPool send lambda below -- takes
  // this shared; retune takes it exclusive. Declared here, ahead of every
  // sender that references it.
  std::shared_mutex tx_gate;
  // Writer-priority flag for tx_gate: raised by
  // RealActuator::retune while it waits for the exclusive lock, spun on by
  // every shared-taker before it enters. glibc's rwlock is reader-preferring
  // and the writer runs on the agent thread — see await_retune_gate.
  std::atomic<bool> retune_waiting{false};

  DevourerSink dev_sink;
  dev_sink.dev = rtl_device.get();
  dev_sink.ready = &device_ready;
  dev_sink.gate = &tx_gate;
  dev_sink.gate_waiting = &retune_waiting;

  // Capacity 6 frames per sender: enough to keep every sender's next ≤3-
  // frame URB staged while it blocks in the current one, small enough that
  // backlog still lands in TxQueue (whose drop-oldest policy is the
  // FEC-recoverable erasure path).
  mabur::UsbTxPool tx_pool(
      [dev = rtl_device.get(), &tx_gate, &retune_waiting](
          const std::vector<std::vector<uint8_t>>& b) {
        std::vector<TxPacketView> v(b.size());
        for (size_t i = 0; i < b.size(); ++i) v[i] = {b[i].data(), b[i].size()};
        await_retune_gate(&retune_waiting);
        std::shared_lock<std::shared_mutex> sg(tx_gate);
        return dev->send_packets(v.data(), v.size());
      },
      cfg.radio.tx_threads,
      static_cast<size_t>(cfg.radio.tx_threads) * 6);
  if (cfg.radio.tx_threads > 1) dev_sink.pool = &tx_pool;

  RadioTx tx(dev_sink);

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  // Debug-HTTP per-layer overhead override, shared by the HTTP thread (the
  // writer), the hot thread (which applies it to the UEP layers) and the
  // agent thread (whose bitrate target must describe what is actually
  // flying) — see OvOverride's doc comment in rc_agent.h. Must outlive both
  // hot_thread and agent_thread, so it lives in this outer scope beside
  // shared_op, not inside either thread's lambda.
  OvOverride ov_override;

  // Agent thread -> hot thread: the encoder's frame rate changed (low-power
  // set_fps), so the vanish tracker must re-learn its period rather than
  // book the new deltas as holes. Same shape as link_up_discont below.
  std::atomic<bool> fps_rate_change{false};

  // The LDPC flag every control-modulation sender builds its radiotap from
  // (ControlRadiotap). A parameter set stores the flag, then bumps the
  // generation.
  std::atomic<bool> live_ldpc{cfg.radio.ldpc};
  std::atomic<uint32_t> radiotap_gen{0};

  RealActuator actuator;
  actuator.tx = &tx;
  actuator.sink = &dev_sink;
  actuator.shared_op = &shared_op;
  actuator.rate_change = &fps_rate_change;
  actuator.dev = rtl_device.get();
  actuator.dry_run = false;
  actuator.ampdu = cfg.ampdu;
  actuator.tx_gate = &tx_gate;
  actuator.retune_waiting = &retune_waiting;
  actuator.cur = start_ch;
  actuator.control_rt.ldpc = &live_ldpc;
  actuator.control_rt.gen = &radiotap_gen;
  // Encoder starts at the "normal" ROI QP (RcAgent only calls set_roi_qp on
  // a low<->normal transition — see run_bitrate_policy's roi_low_ default),
  // so the telemetry collector needs this seeded to reflect what's actually
  // commanded before the first transition ever happens.
  actuator.last_roi_qp = cfg.encoder.roi_qp_normal;

  // VTX onboard recorder. Constructed before the agent (which reaches it
  // through actuator.recorder) and before venc boots (venc_record_configure
  // hands it the drain sink).
  VencRecordChannel rec_channel;
  mabur::VtxRecorder vtx_rec(cfg.record, rec_channel, cfg.venc.core.width,
                             cfg.venc.core.height);
  actuator.recorder = &vtx_rec;

  RcAgent agent(cfg, actuator, &ov_override, start_ch);

#ifdef MABUR_HAVE_VENC
  // Boot the encoder BEFORE the radio and before any thread starts: it
  // creates the frame-shm ring the hot thread reads, and RcAgent's very
  // first tick (BOOT -> MAX_RANGE) commands a bitrate through the verbs
  // above, which no-op until the core is up. `agent` is constructed on the
  // line above and outlives every venc thread (venc_core_stop() joins them
  // before this scope ends), so handing its address to the callbacks is
  // safe without a file-scope indirection.
  VencCallbacks vcb{};
  vcb.on_chain_break = [](void* u) {
    static_cast<RcAgent*>(u)->note_chain_break();  // atomic set; acted on at tick
  };
  // Fault policy: log and _exit(3) for the wrapper to respawn. Deliberately
  // NOT venc_core_stop() — this runs ON the encoder thread and stop() joins
  // that same thread (venc_core.c's contract), so calling it here would
  // self-deadlock the very failure it is meant to escape.
  //
  // _exit(), not std::exit(): this fires on the ENCODER thread of a live
  // multi-threaded process whose other threads (agent, hot TX, debug HTTP,
  // MSP) keep running through the teardown. std::exit() runs atexit
  // handlers and static destructors on that thread while the rest of the
  // process still touches the same objects — a hang there (a destructor
  // blocking on a lock another thread holds, or on the same MI call that
  // just faulted) leaves a wedged, video-less maburd that the wrapper never
  // gets to respawn, which is the exact outcome this policy exists to
  // avoid. stderr is unbuffered, and the explicit fflush covers the case
  // where something upstream has set a buffer on it, so skipping _exit()'s
  // omitted flush-at-exit costs no diagnostics.
  vcb.on_fault = [](void*, const char* what) {
    std::fprintf(stderr, "venc FATAL: %s — exiting for wrapper respawn\n", what);
    std::fflush(stderr);
    _exit(3);
  };
  vcb.user = &agent;
  // Cold boot: init starts maburd BEFORE the MI modules exist (S00mabur is
  // the first rcS entry; there is no S38vendor any more) and the loader
  // thread spawned at the top of this function is inserting them under the
  // USB port reset. Join it and wait for /sys/module to say live before the
  // venc touches MI. Warm restarts find the modules live and skip the
  // loader; the wait then costs one sysfs scan. On a loader failure the
  // wait times out and the MI init below reports it exactly as it always
  // did -- and the wrapper's respawn retries the loader, which S38vendor
  // never did.
  {
    if (module_loader_thread.joinable()) module_loader_thread.join();
    const auto mi = mabur::wait_for_mi_modules("/sys/module", 15000);
    if (!mi.ready) {
      std::fprintf(stderr, "warning: MI modules not live after %d ms, starting venc anyway\n",
                   mi.waited_ms);
    } else if (mi.waited_ms > 0) {
      std::fprintf(stderr, "waited %d ms for the MI modules\n", mi.waited_ms);
    }
  }
  // The record channel's configuration from [record]: here at boot, and
  // again by the config applier before every pipeline rebuild, since the
  // record channel is part of the pipeline it rebuilds.
  auto record_configure = [&vtx_rec](const Config& c) {
    VencRecordConfig rcfg{};
    rcfg.enabled = c.record.enable ? 1 : 0;
    rcfg.bitrate_kbps = static_cast<uint32_t>(c.record.bitrate_kbps);
    rcfg.fps = static_cast<uint32_t>(c.record.fps);
    rcfg.width = static_cast<uint32_t>(c.record.width);
    rcfg.height = static_cast<uint32_t>(c.record.height);
    venc_record_configure(
        &rcfg,
        [](void* u, const uint8_t* au, size_t n, uint32_t pts, int key, int prefixed) {
          static_cast<mabur::VtxRecorder*>(u)->on_au(au, n, pts, key != 0, prefixed != 0);
        },
        &vtx_rec);
  };
  record_configure(cfg);
  if (venc_core_start(&cfg.venc.core, &vcb) != 0) {
    // Boot failure, not a transient: the wrapper's 2 s respawn is the retry.
    // Release the USB device on the way out (same shape as the
    // CreateRadio failure path above) — the radio is not up yet, so
    // there is nothing else to unwind.
    std::fprintf(stderr, "venc_core_start failed — exiting\n");
    join_radio_init();
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 3;
  }
#endif
  vtx_rec.start_thread();
  // After venc_core_start: RcAgent's first tick (below) already commands a
  // bitrate through the verbs, so the ring/stats the debug endpoint reads
  // are live from here on. localhost-only, always on -- bind failure logs
  // and disables itself, never fatal (see debug_http.h).
  debug_http_start(cfg.venc.debug_port, cfg.venc.core.snapshot_quality,
                   &ov_override);

  RcQueue rc_queue;
  // T_MAV lands here (RX callback, below), consumed by the MAV thread: a
  // parameter set rewrites the config file, I/O that belongs on neither the
  // RX nor the agent thread.
  RcQueue mav_queue;
  // The calibration messages the MAV thread pulls out of verified cal-only
  // T_MAV frames, for the TX writer thread: CalSweep's on_cmd/on_apply/
  // on_abort must run on pump()'s thread (cal_sweep.h constraint 1).
  CalMsgQueue cal_queue;
  // The drone's T_MAV uplink counter, shared by the MAV thread's own uplink
  // and the TX writer thread's calibration replies: both send T_MAV frames
  // to the same GS duplicate filter, which drops a repeat of the last
  // counter (the second card's copy of the same frame).
  std::atomic<uint32_t> mav_up_counter{0};
  // MAV thread -> agent thread's stats line, cumulative. drop = frames that
  // failed parse_mav plus packets the component dropped (not for us, bad
  // MAVLink); reject = frames that parsed but failed the tag or the counter.
  std::atomic<uint64_t> mav_rx_frames{0}, mav_rx_packets{0}, mav_tx_frames{0},
      mav_tx_packets{0}, mav_drop{0}, mav_reject{0};
  std::atomic<uint64_t> rx_beat{0};
  // RX callback -> agent thread, per telemetry period (exchange(0) at each
  // Telem build): frames that were ours (CRC-clean RC from the GS), CRC-clean
  // frames that were not (anything else this monitor-mode receiver decoded
  // on the channel) and CRC-failed frames (preamble heard, payload not
  // decodable). With carrier sense ON, foreign + crcfail is what the
  // transmitter deferred to. Software counts on the RX path on purpose: the
  // chip's CCA/FA registers would need a control-plane read under the TX
  // gate (like a retune), and that stalled the USB TX pool once a second
  // -- 795 TxQueue drops in 20 min, +5 ms air p99 (bench).
  std::atomic<uint64_t> rx_own_frames{0}, rx_foreign_frames{0}, rx_crcfail_frames{0};
  std::atomic<uint64_t> hot_beat{0};
  // Calibration session state the hot/agent threads need to read without
  // taking on CalSweep's own "single owner thread" contract (cal_sweep.h):
  // CalSweep itself lives on and is mutated only by the TX writer thread,
  // which mirrors these two booleans out every time it pumps. cal_active
  // spans the whole session (any phase, or a between-phase gap) and gates
  // the hot thread's video quiesce; cal_sweeping is narrower (true only
  // during CalSweep::State::Sweeping) and gates the agent thread's
  // telemetry and MAV-uplink suppression -- the calibration replies (TX
  // writer thread) are sent regardless, so this only keeps the periodic
  // traffic out of the air a sweep is measuring. cal_active is also what
  // check_mav honours the calibration run pair under.
  std::atomic<bool> cal_active{false};
  std::atomic<bool> cal_sweeping{false};
  std::atomic<uint32_t> cal_epoch{0};  // LivePowerTable::begin_cal's sweep-start count
  // The Telem wire tlm_seq (agent thread; gs/src/drone_restart.h treats a
  // big backward step as "the drone restarted") and the 802.11 sequence
  // every control-path frame carries -- Telem, the MAV thread's uplink and
  // the TX writer thread's calibration replies -- so the control path's
  // dot11 sequence stays one stream whichever thread sent the last frame.
  std::atomic<uint16_t> telem_wire_seq{0};
  std::atomic<uint16_t> telem_dot11_seq{0};
  // Each Telem/MAVLink sender (agent thread, TX writer thread's calibration
  // replies, MAV thread) builds its own radiotap from this.
  auto make_control_radiotap = [&live_ldpc, &radiotap_gen] {
    ControlRadiotap rt;
    rt.ldpc = &live_ldpc;
    rt.gen = &radiotap_gen;
    return rt;
  };
  // The chip's TXAGC reference for the BOOT channel (mcs7_index read back
  // with no custom rate-diff table live, i.e. before SetTxPowerRateDiffs is
  // first applied). Drone-internal only: it caps reference + diff at 127
  // (power_plan.h) and never reaches config or the wire. 0 = not yet read
  // by bring-up (below); make_power_plan treats anchor_idx <= 0 as no cap.
  //
  // Deliberately NOT re-read on a channel move, and it does not need to be.
  // The `127 - anchor` cap it feeds is a BOOT-CHANNEL APPROXIMATION of a
  // guard that never binds in practice: measured efuse anchors on this unit
  // are 39-57, so the cap lands at +70..+88, well above kRelMax (+63). It
  // exists for devourer's blank-efuse fallback (75), where it caps at +52
  // and touches only no-dip rows. The per-channel anchor the diffs actually
  // ride on is re-derived by devourer itself on every retune, via the
  // ReApplyTxPower() call in RealActuator::retune_now_() above -- that, not
  // this number, is what keeps the walls valid across an auto-select move.
  //
  // Re-reading it after a retune is not merely unnecessary but WRONG:
  // GetTxPowerState reports the raw reference only while no custom table is
  // live, and by then SetTxPowerRateDiffs has been applied, so a fresh read
  // would return the trimmed value, not the anchor.
  int boot_anchor_idx = 0;

  // The per-width TX-power table (LivePowerTable above): bring-up programs
  // the start width's plan, the agent thread follows the rung's width,
  // RADIO_WALL_MARGN's reprogram and calibration replace the plans.
  LivePowerTable power_table;
  power_table.dev = rtl_device.get();
  power_table.gate = &tx_gate;
  power_table.gate_waiting = &retune_waiting;
  power_table.ready = &device_ready;
  power_table.cal_active = &cal_active;
  power_table.cal_epoch = &cal_epoch;
  // The plans a RadioCfg flies: one per width with a wall table of its own,
  // none at all under power_mode "none" (the bundle keeps reference walls
  // there, inert).
  auto plans_for = [&boot_anchor_idx](const RadioCfg& r, double margin_db) {
    return r.power_mode == "offset" ? make_width_plans(r, margin_db, boot_anchor_idx)
                                    : WidthPlans{};
  };

  // Uplink RSSI/SNR EMAs, fed from rx_callback (RX thread) on CRC-clean RC
  // frames, read by the agent thread's 1 Hz telemetry collector. Thread-safe
  // per UplinkTrack's own mutex.
  UplinkTrack uplink_track;

  // The calibration ladder gate lives on the agent thread (RealActuator::
  // apply_op) -- wire it up now that cal_active exists.
  actuator.cal_active = &cal_active;
  actuator.power_table = &power_table;

  // Cumulative encoder output: written by the hot thread, read by the agent
  // thread's peak-rate sampler for the stats line. Telemetry does not carry
  // them (RC_VERSION 13).
  std::atomic<uint64_t> enc_frames_total{0};
  std::atomic<uint64_t> enc_bytes_total{0};
  // TxQueue wait window max (latency accounting): tx thread publishes the
  // largest push→pop delay it saw since the last 1 Hz telemetry read; the
  // agent thread's collector exchanges it back to 0 so each tick reports its
  // own window, not a running max.
  std::atomic<uint32_t> txq_wait_max_ms{0};
  // Agent thread -> hot thread: link came up from BOOT/RENDEZVOUS, so every
  // frame encoded so far died before the air — re-mark the discontinuity
  // window so the GS gets the re-base signal on frames that can actually
  // land (docs/findings/2026-07-25-gs-frame-stall-after-drone-restart.md).
  std::atomic<bool> link_up_discont{false};

  // RX callback: pulls RC frames (rc::frame_type >= 0) off the air and
  // queues them for the agent thread. Runs on the main thread (inside
  // rtl_device->Init's blocking RX loop).
  // fec-nack: retransmit ring of base-layer source envelopes (hot thread
  // writes via the UepEncoder source tap, RX thread reads) and the T_NACK
  // handler, bound once txq exists (it is declared after this callback).
  // Sized by [nack].ring_ms of base-layer history at
  // encoder.bitrate_max_kbps; env_len is the sealed base envelope (sw header
  // + base symbol_size). Counters are per Telem period (exchanged at the 1 Hz
  // build); nack_bad / retx_miss are drone-local only.
  const size_t retx_env_len = static_cast<size_t>(mabur::sw::kSwHeaderLen) + static_cast<size_t>(cfg.fec.symbol_size[0]);
  mabur::RetxRing retx(mabur::RetxRing::slots_for(cfg.encoder.bitrate_max_kbps, cfg.nack.ring_ms, cfg.fec.symbol_size[0]), retx_env_len);
  std::fprintf(stderr, "maburd: nack retx ring %zu slots x %zu B = %zu B (ring_ms %d at %d kbps)\n",
               retx.slots(), retx_env_len, retx.slots() * retx_env_len, cfg.nack.ring_ms,
               cfg.encoder.bitrate_max_kbps);
  std::function<void(const uint8_t*, size_t)> nack_hook;
  std::atomic<uint64_t> nack_rx{0}, nack_bad{0}, retx_syms{0}, retx_refused{0}, retx_miss{0};
  auto rx_callback = [&](const Packet& pkt) {
    rx_beat.fetch_add(1, std::memory_order_relaxed);
    if (pkt.RxAtrib.crc_err) rx_crcfail_frames.fetch_add(1, std::memory_order_relaxed);
    if (pkt.Data.size() < kDot11HeaderLen + 4) return;
    const uint8_t* body = pkt.Data.data() + kDot11HeaderLen;
    size_t body_len = pkt.Data.size() - kDot11HeaderLen;
    const int rc_type = rc::frame_type(body, body_len);
    if (rc_type >= 0) {
      // Calibration rides T_MAV (the mabur dialect): the MAV thread verifies
      // it and hands its cal packets to the TX writer thread.
      if (rc_type == rc::T_NACK) {
        if (nack_hook) nack_hook(body, body_len);  // RX thread (fec-nack)
      } else if (rc_type == rc::T_MAV) {
        mav_queue.push(body, body_len);
      } else {
        rc_queue.push(body, body_len);
      }
      // Uplink EMAs feed off CRC-clean RC frames only — a corrupt frame's
      // attrib (rssi/snr) is not a trustworthy sample.
      if (!pkt.RxAtrib.crc_err) {
        uplink_track.on_rc_frame(pkt.RxAtrib.rssi, pkt.RxAtrib.snr);
        rx_own_frames.fetch_add(1, std::memory_order_relaxed);
      }
    } else if (!pkt.RxAtrib.crc_err) {
      // Same crc gate as the EMAs, and for the same class of reason:
      // RC_MAGIC is two bytes, so ~1 in 65536 corrupt bodies matches it by
      // chance and must not print a version-mismatch scare, nor count as a
      // neighbour. Everything CRC-clean that is not ours is a foreign
      // 802.11 frame on our channel (Telem rx_foreign); a foreign RC
      // version is logged on top. The frame is still dropped as before.
      rx_foreign_frames.fetch_add(1, std::memory_order_relaxed);
      if (rc::is_foreign_rc_version(body, body_len))
        log_foreign_rc_version(body[2]);
    }
  };

  // The MSP thread: reads the FC's serial link, sends its DisplayPort stream
  // to the GS and polls the arm state for low-power mode. A restartable
  // unit: boot starts it iff msp.enable, and a parameter set starts or stops
  // it (start_msp/stop_msp, called from one thread at a time: boot, then the
  // config applier, then teardown after the applier has shut down). `m` is
  // the [msp] section it runs with; `stop` ends it within one serial read
  // (MspSerial::read returns within its 100 ms VTIME).
  std::thread msp_thread;
  std::atomic<bool> msp_stop{false};
  auto run_msp = [&](std::atomic<bool>& stop, const MspCfg m) {
      name_thread("mbr-msp");
      // Robust control modulation, same tier as DISC_ACK; MSP is a third
      // producer on the mutex-guarded dev_sink.send() path (never the pool).
      ControlRadiotap msp_radiotap = make_control_radiotap();
      uint16_t seq = 0;
      std::random_device rd;
      MspSource src(to_msp_source_cfg(m),
        [&](const uint8_t* body, size_t n) {
          // Quiesce for the calibration session's duration -- the same
          // "non-sweep PPDUs in the characterized airtime" telemetry
          // suppression exists to avoid. Reading/decoding serial bytes
          // continues regardless (src still gets on_serial_bytes() below),
          // only the TRANSMIT is skipped, so the UART side never desyncs or
          // overflows waiting out a session.
          if (cal_active.load(std::memory_order_relaxed)) return;
          const std::vector<uint8_t>& radiotap = msp_radiotap.get();
          std::vector<uint8_t> frame;
          frame.reserve(radiotap.size() + kDot11HeaderLen + n);
          frame.insert(frame.end(), radiotap.begin(), radiotap.end());
          auto hdr = build_dot11_header(seq);
          seq = static_cast<uint16_t>((seq + 1) & 0xFFF);
          frame.insert(frame.end(), hdr.begin(), hdr.end());
          frame.insert(frame.end(), body, body + n);
          dev_sink.send(frame.data(), frame.size());
        },
        rd());  // random initial_seq (SwEncoder restart-safety contract)
      std::fprintf(stderr,
          "maburd msp: enabled symbol_size=%d window=%d block_payload=%d update_rate_hz=%.2g serial=%s baud=%d\n",
          m.symbol_size, m.window,
          m.symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen),
          m.update_rate_hz, m.serial.c_str(), m.baud);

      // Low-power mode: ask the FC for its arm state.
      // MSP_STATUS is request/reply -- the FC pushes only DisplayPort on
      // its own -- so poll at kArmPollMs and hand every decoded reply to
      // RcAgent. Kept polling after the arm latch: 6 B out + ~33 B in per
      // 500 ms is free and the live flag stays observable.
      //
      // COUPLED to low_power.stale_ms, which is operator-configurable while
      // this is not: a report is fresh for stale_ms and arrives at best once
      // per kArmPollMs, so stale_ms <= 500 makes the mode flap once per poll
      // (fresh on the reply tick, stale before the next one). Deliberately
      // NOT validated -- the flap direction is toward FULL power, i.e. the
      // safe side, so a too-small stale_ms costs bitrate, never safety.
      // Keep stale_ms comfortably above this (the bundle ships 2000), and if
      // this poll period ever changes, revisit that margin.
      constexpr uint64_t kArmPollMs = 500;
      std::vector<uint8_t> status_req;
      mabur::msp_append_status_request(status_req);
      uint64_t last_poll_ms = 0;
      // Installed whether or not low power is on: low_power.enable is a live
      // knob, and the poll below is what gates the replies.
      src.set_message_hook([&](const mabur::MspMessage& msg) {
        if (auto armed = mabur::msp_status_armed(msg))
          agent.note_arm_state(*armed, now_steady_ms());
      });

      MspSerial serial;
      uint8_t buf[512];
      while (!stop.load(std::memory_order_relaxed) && !g_devourer_should_stop) {
        if (!serial.is_open()) {
          if (!serial.open(m.serial, m.baud)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
          }
          std::fprintf(stderr, "maburd msp: reading %s @ %d\n",
                       m.serial.c_str(), m.baud);
        }
        int n = serial.read(buf, sizeof buf);
        if (n > 0) src.on_serial_bytes(buf, static_cast<size_t>(n), now_steady_ms());
        else if (n < 0) serial.close();  // error -> reconnect

        if (agent.low_power_enabled() && serial.is_open()) {
          uint64_t t = now_steady_ms();
          if (t - last_poll_ms >= kArmPollMs) {
            last_poll_ms = t;
            if (serial.write(status_req.data(), status_req.size()) < 0) serial.close();
          }
        }
      }
      std::fprintf(stderr, "maburd msp: stopped\n");
  };
  auto start_msp = [&](const MspCfg& m) {
    if (msp_thread.joinable()) return;
    msp_stop.store(false);
    msp_thread = std::thread([&run_msp, &msp_stop, m] { run_msp(msp_stop, m); });
  };
  auto stop_msp = [&] {
    if (!msp_thread.joinable()) return;
    msp_stop.store(true);
    msp_thread.join();
    msp_thread = std::thread();
  };
  if (cfg.msp.enable) start_msp(cfg.msp);

  // State a MAVLink parameter set changes in the running process, read by
  // the threads that use it: the wall margin (calibration's reprogram and
  // its post-session restore), and the H.265 slice geometry the hot thread
  // stamps on every frame (re-read when geometry_changed is raised).
  std::atomic<double> live_wall_margin{cfg.radio.wall_margin_db};
  std::atomic<uint16_t> live_ctb_rows{
      venc_cfg_ctb64_rows(static_cast<uint16_t>(cfg.venc.core.height))};
  std::atomic<uint8_t> live_slice_rows{
      venc_cfg_slice_rows(static_cast<uint16_t>(cfg.venc.core.height), cfg.venc.core.slices)};
  std::atomic<bool> geometry_changed{false};

  // Applies a MAVLink parameter set to this process (config_apply.h), on its
  // own thread. Every hook runs on that thread. Without an encoder (host
  // build) the pipeline hooks refuse, so a rebuild set walks the failure
  // ladder to its end.
  ConfigApplyHooks apply_hooks;
  apply_hooks.set_knobs = [&agent](const Config& c) {
    agent.set_live_knobs(c.encoder.bitrate_max_kbps, c.encoder.airtime_budget,
                         c.low_power.enable, static_cast<int>(c.venc.core.fps));
  };
  apply_hooks.power_plan = [&](const Config& c, std::string* err) {
    // The applier refuses a set during calibration already; the sweep owns
    // the TX-power table, so this never reprograms under one either.
    if (cal_active.load()) { *err = "calibration in progress"; return false; }
    if (!rtl_device || !device_ready.load(std::memory_order_acquire)) {
      *err = "radio not up";
      return false;
    }
    if (c.radio.power_mode != "offset") { *err = "no offset power plan"; return false; }
    // Both widths' plans move with the margin; the rung's width's is
    // written now (under tx_gate, inside the table's lock), the other's when
    // the rung next crosses to it.
    const auto op = shared_op.load();
    if (!power_table.replace(plans_for(c.radio, c.radio.wall_margin_db),
                             op ? op->ladder[1].bw : c.radio.width,
                             /*zero_offset=*/false, /*clear_override=*/false)) {
      *err = "SetTxPowerRateDiffs failed";
      return false;
    }
    live_wall_margin.store(c.radio.wall_margin_db);
    return true;
  };
  apply_hooks.ldpc = [&](bool ldpc) {
    live_ldpc.store(ldpc, std::memory_order_release);
    radiotap_gen.fetch_add(1, std::memory_order_release);
    agent.note_ldpc_changed(ldpc);
  };
  apply_hooks.msp = [&](bool enable, const Config& c) {
    if (enable) start_msp(c.msp);
    else stop_msp();
  };
  apply_hooks.recorder_stop = [&vtx_rec] {
    // Stopped before the pipeline that feeds it goes away, so the file is
    // closed on a whole fragment and the worker is not inside the record
    // channel when the rebuild tears it down. settled(), not state(): an
    // armed recorder already reads Off before its first file. The worker
    // acts within one service() wait (200 ms); the wait is bounded at 1 s.
    vtx_rec.request(false);
    for (int i = 0; i < 100 && !vtx_rec.settled(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!vtx_rec.settled())
      std::fprintf(stderr, "maburd: config apply: recorder not stopped after 1 s\n");
    else if (vtx_rec.state() != mabur::RecState::Off)
      std::fprintf(stderr, "maburd: config apply: recorder stopped with an error\n");
  };
  apply_hooks.record_configure = [&](const Config& c) {
#ifdef MABUR_HAVE_VENC
    record_configure(c);
#endif
    // The recorder follows the rebuilt pipeline: its record channel exists
    // only if [record] enables it, and its files carry the link's size.
    vtx_rec.set_config(c.record);
    vtx_rec.set_link_size(c.venc.core.width, c.venc.core.height);
  };
  apply_hooks.pipeline_restart = [](const Config& c) {
#ifdef MABUR_HAVE_VENC
    return venc_core_restart(&c.venc.core) == 0;
#else
    (void)c;
    return false;
#endif
  };
  apply_hooks.pipeline_cold_restart = [&](const Config& c) {
#ifdef MABUR_HAVE_VENC
    venc_core_stop();
    return venc_core_start(&c.venc.core, &vcb) == 0;
#else
    (void)c;
    return false;
#endif
  };
  apply_hooks.after_rebuild = [&](const Config& c) {
    const uint16_t h = static_cast<uint16_t>(c.venc.core.height);
    live_ctb_rows.store(venc_cfg_ctb64_rows(h));
    live_slice_rows.store(venc_cfg_slice_rows(h, c.venc.core.slices));
    geometry_changed.store(true, std::memory_order_release);
    fps_rate_change.store(true, std::memory_order_relaxed);
    // The agent reasserts the fps verb from its live knob right after a
    // rebuild: the knob must already hold the rebuilt pipeline's rate.
    agent.set_live_knobs(c.encoder.bitrate_max_kbps, c.encoder.airtime_budget,
                         c.low_power.enable, static_cast<int>(c.venc.core.fps));
    agent.note_pipeline_rebuilt();
  };
  apply_hooks.request_exit = [] {
    // The pipeline is gone and neither restart brought it back. Same policy
    // as the encoder's on_fault: _exit, not exit, because destructors must
    // not run on a live multi-threaded process; the wrapper respawns maburd
    // from the unchanged file.
    std::fprintf(stderr, "maburd: config apply: pipeline lost — exiting for wrapper respawn\n");
    std::fflush(stderr);
    _exit(3);
  };
  apply_hooks.calibration_active = [&cal_active] { return cal_active.load(); };
  ConfigApplier applier(cfg_path, cfg, std::move(apply_hooks), [](const std::string& line) {
    std::fprintf(stderr, "maburd config: %s\n", line.c_str());
  });

  // MAVLink parameter component. Verifies each T_MAV off mav_queue
  // (RcAgent::check_mav), feeds the payload to the component -- or, for a
  // frame of calibration packets only, decodes them onto cal_queue for the
  // TX writer thread -- and sends its
  // queued replies back as untagged T_MAV frames on the Telem path (control
  // radiotap, the shared telem dot11 seq, the mutex-guarded dev_sink.send()),
  // at most one frame per kMavUplinkMinMs so the tunnel never crowds Telem
  // or the OSD stream. Uplink and the 1 Hz HEARTBEAT need a published
  // session (nobody is listening without one), and both follow Telem's
  // suppression while a calibration phase sweeps: replies wait queued. RcQueue has no
  // condvar, so this polls at 5 ms, like cal_queue's consumer.
  std::thread mav_thread([&]() {
    name_thread("mbr-mav");
    ApplyPort port;
    port.submit = [&applier](size_t row, const mavparam::Value& want, std::string* why) {
#ifndef MABUR_HAVE_VENC
      // No encoder in this build: every rung of the rebuild ladder would
      // fail and end in the exit rung, so a rebuild row is refused here.
      if (apply_path_for(row) == ConfigApplyPath::Rebuild) {
        if (why) *why = "no encoder in this build";
        return false;
      }
#endif
      return applier.submit(row, want, why);
    };
    port.take_result = [&applier](ConfigApplyResult* out) { return applier.take_result(out); };
    port.busy = [&applier] { return applier.busy(); };
    MavComponent comp(cfg_path, std::move(port), [](const std::string& line) {
      std::fprintf(stderr, "%s\n", line.c_str());
    });
    ControlRadiotap mav_radiotap = make_control_radiotap();
    std::fprintf(stderr, "maburd mav: component sys %u comp %u, config %s\n",
                 static_cast<unsigned>(MavComponent::sys_id()),
                 static_cast<unsigned>(MavComponent::comp_id()), cfg_path.c_str());
    uint64_t last_hb = 0, last_up = 0, bad_parse = 0, rejected = 0;
    std::vector<uint8_t> body;
    // A cal-only frame's packets go to the TX writer, never the component;
    // a frame mixing cal and other packets goes whole to the component,
    // which ignores the cal ids.
    auto push_cal = [&](const rc::Mav& m, bool latched) {
      for (auto& msg : cal::decode_all(m.payload.data(), m.payload.size()))
        cal_queue.push(CalInbound{std::move(msg), latched});
    };
    while (!g_devourer_should_stop) {
      while (mav_queue.pop(body)) {
        rc::Mav m;
        // While a calibration run is open, a frame tagged under the run's
        // pair is still accepted (kOkLatched) after the drone has dropped
        // the session -- every sweep outlasts the link failsafe. check_mav
        // admits only cal-only frames on that path.
        switch (agent.check_mav(body.data(), body.size(), &m,
                                cal_active.load(std::memory_order_relaxed))) {
          case RcAgent::MavCheck::kOk:
            mav_rx_frames.fetch_add(1, std::memory_order_relaxed);
            if (cal::all_packets_are_cal(m.payload.data(), m.payload.size()))
              push_cal(m, /*latched=*/false);
            else
              comp.on_payload(m.payload.data(), m.payload.size());
            break;
          case RcAgent::MavCheck::kOkLatched:
            mav_rx_frames.fetch_add(1, std::memory_order_relaxed);
            push_cal(m, /*latched=*/true);
            break;
          case RcAgent::MavCheck::kMalformed: ++bad_parse; break;
          case RcAgent::MavCheck::kRejected: ++rejected; break;
        }
      }
      // Every pass, linked or not: the applier stays busy (refusing the
      // next set) until its result is taken.
      comp.poll_results();
      const uint64_t now = now_steady_ms();
      const bool linked = agent.has_published_session();
      if (linked && !cal_sweeping.load(std::memory_order_relaxed) && now - last_hb >= 1000) {
        comp.queue_heartbeat();
        last_hb = now;
      }
      if (linked && !cal_sweeping.load(std::memory_order_relaxed) &&
          comp.outbound_packets() > 0 && now - last_up >= kMavUplinkMinMs) {
        rc::Mav m;
        m.counter = mav_up_counter.fetch_add(1, std::memory_order_relaxed);
        const int packets = comp.take_outbound(&m.payload);
        const auto mav = rc::pack_mav(m, /*tagged=*/false);
        const std::vector<uint8_t>& telem_radiotap = mav_radiotap.get();
        std::vector<uint8_t> frame;
        frame.reserve(telem_radiotap.size() + kDot11HeaderLen + mav.size());
        frame.insert(frame.end(), telem_radiotap.begin(), telem_radiotap.end());
        auto hdr = build_dot11_header(telem_dot11_seq.fetch_add(1, std::memory_order_relaxed) & 0xFFF);
        frame.insert(frame.end(), hdr.begin(), hdr.end());
        frame.insert(frame.end(), mav.begin(), mav.end());
        dev_sink.send(frame.data(), frame.size());
        mav_tx_frames.fetch_add(1, std::memory_order_relaxed);
        mav_tx_packets.fetch_add(static_cast<uint64_t>(packets), std::memory_order_relaxed);
        last_up = now;
      }
      const MavComponentStats& cs = comp.stats();
      mav_rx_packets.store(cs.rx_packets, std::memory_order_relaxed);
      mav_drop.store(bad_parse + cs.drop_not_for_us + cs.drop_bad_packet + cs.drop_queue_full,
                     std::memory_order_relaxed);
      mav_reject.store(rejected, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  // TX body queue between the encode (hot) thread and the USB writer
  // thread: a bulk-OUT stall backs up HERE (bounded, drop-oldest =
  // FEC-recoverable erasures) instead of in the waybeam SHM ring, whose
  // overflow makes the RTP packetizer abort NALs mid-chain — sender-side
  // slice truncation no FEC can repair (the PixelPilot glitch root cause).
  // ~256 bodies ≈ 150 ms at 1700 bodies/s.
  constexpr size_t kTxQueueCap = 256;  // also the congestion shed's txq_cap
  TxQueue txq(kTxQueueCap);
  // Air clock. Lives out here, not in the hot thread, since fec-nack: the hot
  // thread prices it (set_rates via apply_op_to_clock), books every
  // video/probe body and reads the backlog; the RX thread's T_NACK handler
  // below books every retransmit body. AirClock's own mutex guards it; both
  // sides lock per BODY, never per symbol.
  AirClock air_clock;
  // Software NACK answer (fec-nack), RX thread. Verified request -> ring
  // lookups -> fresh SBI bodies marked retx -> TxQueue head, under a token
  // bucket of nack.air_pct of the base layer's delivered capacity,
  // nack.burst_ms of that capacity deep (flight 0026: a fixed 64-symbol depth
  // was 4 ms of air at rung 5 and 30 ms at rung 0, and refused half of every
  // cascade's requests). A repeat-flagged request is answered twice (and
  // draws two tokens per symbol). The bucket, packer and scratch envelope are
  // RX-thread-only; ring, txq, air_clock and agent are thread-safe. The
  // UepBody carries stream_id 0, so RadioTx::build_frame and the air clock
  // see plain base-layer bodies; the retx mark lives only in the SBI stream
  // byte, which the GS routes as base and decodes as a retransmit.
  TokenBucket retx_bucket(64.0);  // re-sized from the op at every refill
  mabur::SbiPacker retx_packer(static_cast<int>(retx_env_len), cfg.fec.blocks_per_body[0],
                               static_cast<uint8_t>(0 | mabur::kSbiRetxMark));
  std::vector<uint8_t> retx_env(retx_env_len);
  std::vector<std::vector<uint8_t>> retx_out;
  nack_hook = [&](const uint8_t* body, size_t len) {
    // A malformed frame (radio corruption, sid != 0) counts nack_bad only;
    // check_nack raises auth_reject itself for a tag/counter failure.
    rc::Nack nk;
    if (agent.check_nack(body, len, &nk) != RcAgent::NackCheck::kOk) {
      nack_bad.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const rc::Nack* n = &nk;
    nack_rx.fetch_add(1, std::memory_order_relaxed);
    const auto op = shared_op.load();
    const double sym_per_s =
        op ? delivered_mbps(op->ladder[0], cfg.air_clock) * 1e6 / 8.0 / static_cast<double>(retx_env_len)
           : 0.0;
    if (op) retx_bucket.set_depth(TokenBucket::depth_for(sym_per_s, cfg.nack.burst_ms));
    retx_bucket.refill(now_steady_us(), sym_per_s * cfg.nack.air_pct / 100.0);
    const bool twice = (n->flags & rc::kNackFlagRepeat) != 0;
    const double cost = twice ? 2.0 : 1.0;
    retx_out.clear();
    for (uint8_t i = 0; i < n->n; ++i) {
      for (int b = 0; b < 32; ++b) {
        if (!(n->e[i].bitmap & (1u << b))) continue;
        const uint32_t seq = n->e[i].first_seq + static_cast<uint32_t>(b);
        if (!retx.get(seq, retx_env.data())) { retx_miss.fetch_add(1, std::memory_order_relaxed); continue; }
        if (!retx_bucket.take(cost)) { retx_refused.fetch_add(1, std::memory_order_relaxed); continue; }
        retx_syms.fetch_add(1, std::memory_order_relaxed);
        auto out = retx_packer.add_one(retx_env.data(), retx_env.size());
        if (!out.empty()) retx_out.push_back(std::move(out));
      }
    }
    auto tail = retx_packer.flush_one();
    if (!tail.empty()) retx_out.push_back(std::move(tail));
    // push_front stacks, so push each pass back-to-front: the bodies then
    // leave in request order (pass 1 then pass 2) ahead of queued video.
    const uint32_t now_ms = static_cast<uint32_t>(now_steady_ms());
    for (int rep = 0; rep < (twice ? 2 : 1); ++rep)
      for (auto it = retx_out.rbegin(); it != retx_out.rend(); ++it) {
        const uint64_t p_us = now_steady_us();
        const size_t bytes = it->size();
        txq.push_front(UepBody{0, *it, now_ms, p_us, false});
        air_clock.book(p_us, bytes, 0);
      }
  };
  // fec.feed_batch: group the TX writer's wakeups so bodies leave in
  // URB-filling batches (see TxQueue::set_batch); the hot thread flushes at
  // every AU end so a tail group never waits on the next frame.
  if (cfg.fec.feed_batch > 1)
    txq.set_batch(static_cast<size_t>(cfg.fec.feed_batch));

  // Hot thread: pulls whole frames off the real SHM ring, runs them through
  // the UEP pipeline, queues bodies for the TX writer. Owns the UepEncoder
  // exclusively; never blocks on USB.
  std::thread hot_thread([&]() {
    name_thread("mbr-hot");
    if (two_core_target()) pin_self_to(kHotCore);
    // Ring name's single authority is the compile-time VENC_RING_NAME
    // (drone/venc/venc_cfg.h "/mabur_f"); there is no frame_ring_name config
    // key. venc_frame_ring_attach() normalises the leading '/' itself, so
    // this is behaviourally identical to the old default "mabur_f".
    FrameSource fsrc(VENC_RING_NAME);
    FramePipeline pipe;
    pipe.set_slice_geometry(
        venc_cfg_ctb64_rows(static_cast<uint16_t>(cfg.venc.core.height)),
        venc_cfg_slice_rows(static_cast<uint16_t>(cfg.venc.core.height), cfg.venc.core.slices));
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
    UepEncoder uep(cfg.uep_layers(), cfg.fec.flush_ms, &fec_worker);
    uep.set_source_tap([&retx](uint8_t sid, uint32_t seq, const uint8_t* env, size_t n) {
      if (sid == 0) retx.put(seq, env, n);  // base layer only (fec-nack)
    });

    // Probe stream: same FEC geometry as the enh layer
    // (block_payload/bpb), so a probe body is the same wire size as a video
    // body at that rung.
    const auto probe_layer = cfg.uep_layers()[1];
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
      if (cal_active.load(std::memory_order_relaxed)) {
        hot_beat.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      uint64_t now = now_steady_ms();
      const uint64_t t0_us = now_steady_us();

      // Identity-compare, not generation-compare: see the matching comment
      // in run_dry_run's pump_op_change lambda above. reapply_with_shed()
      // (congestion) never bumps generation, so gating on generation here
      // would silently drop local shed changes that arrive between two
      // RCF-driven generation bumps — including the congestion-triggered
      // shed this loop is specifically responsible for applying to the
      // UepEncoder (op.shed) independent of the GS.
      auto op = shared_op.load();
      if (op && op != last_applied_op) {
        apply_op_to_uep(*op, uep);
        apply_op_to_clock(*op, cfg.air_clock, air_clock);
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
        const int ob = ov_override.ovr_base_pct.load(std::memory_order_relaxed);
        const int oe = ov_override.ovr_enh_pct.load(std::memory_order_relaxed);
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
          txq.push(std::move(b));
          air_clock.book(p_us, body_bytes, body_sid);
          any = true;
        });
        if (any) txq.flush();
      }
      if (n > 0) {
        // A parameter set rebuilt the pipeline: frames from here on carry
        // the new slice geometry. The rebuild also recreated the ring, so
        // this lands with the re-attach below; marked here as well in case
        // the re-attach was counted before the geometry was published.
        if (geometry_changed.exchange(false, std::memory_order_acquire)) {
          pipe.set_slice_geometry(live_ctb_rows.load(), live_slice_rows.load());
          pipe.mark_discontinuity();
        }
        if (fsrc.reattach_count() != last_reattach) {
          last_reattach = fsrc.reattach_count();
          pipe.mark_discontinuity();  // joined a new ring mid-GOP
        }
        if (link_up_discont.exchange(false, std::memory_order_relaxed)) {
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
        if (fps_rate_change.exchange(false, std::memory_order_relaxed))
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
        const uint32_t backlog_us = air_clock.backlog_us(t_arr_us);
        const uint16_t air_ms = static_cast<uint16_t>(
            std::min<uint32_t>(backlog_us / 1000u, 65535u));
        pipe.set_enh_gate_closed(
            cfg.air_clock.shed_ms > 0 &&
            backlog_us >= static_cast<uint32_t>(cfg.air_clock.shed_ms) * 1000u);
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
                      txq.push(std::move(b));
                      air_clock.book(p_us, body_bytes, body_sid);
                      split_sink_sum_us += now_steady_us() - s_us;
                    });
        txq.flush();  // release the AU's partial feed_batch group, if any
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
          air_clock.book(p_us, pb.body.size(), AirClock::kProbeSid);
          txq.push(std::move(pb));
          txq.flush();
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
        enc_frames_total.fetch_add(1, std::memory_order_relaxed);
        enc_bytes_total.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
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
          if (jobs > 0 || joins > 0 || g.backlog_drops != p.backlog_drops) {
            std::fprintf(stderr,
                "maburd fec_worker sid=%d: jobs=%llu build_us/job=%llu "
                "inline=%llu joins=%llu join_wait_us mean=%llu max=%llu "
                "qdepth_max=%llu backlog_drops=%llu\n",
                sid,
                (unsigned long long)jobs,
                (unsigned long long)(jobs ? build_us / jobs : 0),
                (unsigned long long)(g.inline_full - p.inline_full),
                (unsigned long long)joins,
                (unsigned long long)(joins ? wait_us / joins : 0),
                (unsigned long long)g.join_wait_max_us,
                (unsigned long long)g.enq_depth_max,
                (unsigned long long)(g.backlog_drops - p.backlog_drops));
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
          txq.push(std::move(b));
        }
        txq.flush();  // idle-tail bodies never wait on a feed_batch group
      }

      hot_beat.fetch_add(1, std::memory_order_relaxed);
    }
    txq.close();
  });

  // Calibration's power actuator: CalSweep::PowerCtl driven by the live
  // devourer device -- the flat TXAGC override, the rate-diff table and the
  // anchor read-back, under CalSweep's state machine.
  //
  // Threading: all three are device control-plane calls made from the TX
  // writer thread, and devourer's IRtlRadio.h contract forbids any of them
  // concurrent with a channel set. RealActuator::retune takes tx_gate
  // exclusive around FastRetune, so each call here takes it SHARED -- a cal
  // session lasts up to 180 s, long enough for a hop, DISC or
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

  // TX writer thread: sole caller of tx.send_bodies (RadioTx's
  // single-thread contract). Batches up to 3 bodies per call — devourer's
  // Jaguar3 send_packets packs them into one bulk-OUT URB (HalMAC parses at
  // most 3 descriptors per transfer), amortizing the per-URB tax that
  // capped the old inline path at ~2500 fps. It is also, by that same
  // contract, cal_sweep.pump()'s sole legal caller (cal_sweep.h) — the
  // calibration messages are dispatched here too (cal_queue, fed by the MAV
  // thread), so on_cmd/on_apply/on_abort/pump never run off this thread.
  std::thread tx_thread([&]() {
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
          13 + static_cast<size_t>(cfg.fec.blocks_per_body[1]) *
                   (16 + static_cast<size_t>(cfg.fec.symbol_size[1]));
      return c;
    };
    bool cal_ldpc = live_ldpc.load(std::memory_order_acquire);
    CalSweep cal_sweep(make_cal_cfg(cal_ldpc));
    DevicePowerCtl pwr;
    pwr.dev = rtl_device.get();
    pwr.gate = &tx_gate;
    pwr.gate_waiting = &retune_waiting;
    DeviceCapacityCtl cap;
    cap.tx = &tx;
    cap.dev = rtl_device.get();
    cap.pool = dev_sink.pool;
    cap.gate = &tx_gate;
    cap.gate_waiting = &retune_waiting;
    cap.ampdu = cfg.ampdu;
    cap.ldpc = cal_ldpc;
    // One drone->GS calibration message (ack, done, applied) as its own
    // untagged T_MAV frame on the control path: the shared uplink counter
    // and dot11 sequence, the mutex-guarded dev_sink.send(). Sent whatever
    // the session state -- every sweep outlasts the link failsafe and the GS
    // is listening for exactly these -- and outside the MAV thread's pacing
    // and its sweep suppression: one per command, sweep or apply.
    ControlRadiotap cal_radiotap = make_control_radiotap();
    auto send_cal = [&](const cal::CalMsg& msg) {
      rc::Mav m;
      m.counter = mav_up_counter.fetch_add(1, std::memory_order_relaxed);
      m.payload = cal::encode(msg, MavComponent::sys_id(), MavComponent::comp_id(),
                              mav::kChanDroneRx);
      const auto mav = rc::pack_mav(m, /*tagged=*/false);
      const std::vector<uint8_t>& rt = cal_radiotap.get();
      std::vector<uint8_t> frame;
      frame.reserve(rt.size() + kDot11HeaderLen + mav.size());
      frame.insert(frame.end(), rt.begin(), rt.end());
      auto hdr = build_dot11_header(telem_dot11_seq.fetch_add(1, std::memory_order_relaxed) & 0xFFF);
      frame.insert(frame.end(), hdr.begin(), hdr.end());
      frame.insert(frame.end(), mav.begin(), mav.end());
      dev_sink.send(frame.data(), frame.size());
      mav_tx_frames.fetch_add(1, std::memory_order_relaxed);
      mav_tx_packets.fetch_add(1, std::memory_order_relaxed);
    };
    // Power and verify frames fly alone, one per PPDU, so each stamped
    // frame's delivery is its own (AmpduMode{} is aggregation off). Under
    // tx_gate like every device call on this thread; the agent puts the
    // rung's mode back on the cal_active falling edge
    // (RealActuator::on_cal_end).
    auto cal_no_agg = [&]() {
      await_retune_gate(&retune_waiting);
      std::shared_lock<std::shared_mutex> sg(tx_gate);
      if (!rtl_device->SetAmpduMode(devourer::AmpduMode{}))
        std::fprintf(stderr, "maburd cal: SetAmpduMode(off) failed\n");
    };
    // Re-reads the config from disk -- not the bring-up `cfg` -- so one path
    // covers "the run applied new walls" (the file holds them) and "the run
    // never applied" (the file is unchanged, so this reproduces bring-up).
    auto reload_config = [&]() {
      try {
        return load_config(cfg_path, nullptr);
      } catch (const std::exception& e) {
        std::fprintf(stderr, "maburd cal: config reload failed (%s); using the bring-up config\n",
                     e.what());
        return cfg;
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
      const auto op = shared_op.load();
      const uint8_t bw = op ? op->ladder[1].bw : cfg.radio.width;
      if (!power_table.replace(plans_for(live_cfg.radio, live_wall_margin.load()), bw,
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
      const double margin_db = live_wall_margin.load();
      const CalWrite w = cal_write_for(a, cal_sweep.stamp_flags());
      std::string err;
      ApplyResult ar;
      {
        // Held across the write and the applier's notice, so its next apply
        // re-reads the calibrated file rather than racing this write.
        auto lk = lock_config_writes();
        ar = apply_calibration(cfg_path, w, margin_db, apply_ledger.want_backup(a.nonce), &err);
        if (ar == ApplyResult::Ok) applier.note_file_rewritten_externally();
      }
      cal::Applied out;
      out.nonce = a.nonce;
      out.bw = a.bw;
      out.ok = ar == ApplyResult::Ok;
      if (out.ok) {
        power_table.set_plans(plans_for(reload_config().radio, margin_db));
        std::fprintf(stderr, "maburd cal: applied %u MHz%s%s\n", static_cast<unsigned>(a.bw),
                     a.has_walls ? " walls" : "", a.has_eff ? " efficiency" : "");
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
    // dq_queue gauge (dq-spike follow-up): TRUE push→pop queue
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
        const bool ldpc = live_ldpc.load(std::memory_order_acquire);
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
      while (cal_queue.pop(&in)) {
        const uint64_t cal_now = now_steady_ms();
        if (const auto* c = std::get_if<cal::SweepCmd>(&in.msg)) {
          if (in.latched && !(cal_sweep.active() && c->nonce == cal_sweep.run_nonce())) {
            ++cal_latch_drops;
            continue;
          }
          if (cal_sweep.starts_sweep(*c)) {
            // cal_active up first, under the power table's lock (see
            // LivePowerTable), then what the sweep needs from the device:
            // a capacity sweep this width's production power table, every
            // other kind aggregation off.
            power_table.begin_cal();
            if (c->kind == cal::Kind::Capacity)
              power_table.program_for_cal(c->bw);
            else
              cal_no_agg();
          }
          cal_sweep.on_cmd(*c, cal_now, pwr);
          // The ack goes out HERE -- before this thread pumps the sweep the
          // command touched, whose first cell waits 200 ms for it -- on every
          // accepted command, an exact repeat included: the GS resends its
          // SWEEP every 200 ms into a lossy uplink, and one lost ack must not
          // cost the sweep.
          if (auto ack = cal_sweep.take_ack()) send_cal(*ack);
          // A command refused because the chip could not report its anchor
          // was refused after on_cmd() had already zeroed the diffs, and no
          // run opened, so no falling edge will restore them: do it now.
          if (cal_sweep.take_refused()) {
            std::fprintf(stderr,
                         "maburd cal: sweep refused (TXAGC anchor unreadable); restoring "
                         "operating power\n");
            restore_operating_power();
          }
        } else if (const auto* ap = std::get_if<cal::ApplyCmd>(&in.msg)) {
          // The GS resends APPLY every 200 ms until it hears APPLIED: a
          // (nonce, bw) already applied is answered again, never re-applied.
          if (auto hit = apply_ledger.cached(ap->nonce, ap->bw))
            send_cal(*hit);
          else
            cal_sweep.on_apply(*ap, cal_now);
        } else if (const auto* ab = std::get_if<cal::Abort>(&in.msg)) {
          cal_sweep.on_abort(*ab, cal_now, pwr);
        }
        // SweepAck, SweepDone and Applied travel drone -> GS only.
      }

      if (auto a = cal_sweep.take_pending_apply()) apply_and_arm_verify(*a);

      cal_active.store(cal_sweep.active(), std::memory_order_relaxed);
      cal_sweeping.store(cal_sweep.state() == CalSweep::State::Sweeping,
                         std::memory_order_relaxed);

      if (cal_sweep.active()) {
        // Video quiesce: the hot thread has already stopped pushing (its own
        // cal_active guard, above), but bodies queued before this exact
        // moment must not trickle out mixed with sweep frames — drain once,
        // on the session's very first iteration here.
        if (!cal_was_active) txq.drain();
        cal_was_active = true;
        cal_sweep.pump(now_steady_ms(), tx, pwr, cap);
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
      if (cal_was_active) {
        // Falling edge: restore TX power for real before anything below can
        // transmit again. Drain TxQueue a SECOND time here too -- the hot
        // thread's own cal_active guard (its loop, above) can race this exact
        // transition and push one more AU's bodies into txq between this
        // thread clearing cal_active and noticing it here; left alone those
        // bodies would sit queued for the length of the NEXT session (or
        // forever, if there isn't one) and go out stale with a session-length
        // q_ms stamp whenever they finally flush.
        restore_operating_power();
        txq.drain();
      }
      cal_was_active = false;

      batch.clear();
      if (txq.pop_batch(batch, 3, 5) == 0) continue;
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
        uint32_t prev = txq_wait_max_ms.load(std::memory_order_relaxed);
        while (w > prev && !txq_wait_max_ms.compare_exchange_weak(prev, w)) {}
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
      tx.send_bodies(batch);
      const uint64_t sb_dt = now_steady_us() - sb_t0;
      ++sb_calls;
      sb_bodies += batch.size();
      sb_sum_us += sb_dt;
      if (sb_dt > sb_max_us) sb_max_us = sb_dt;
    }
  });

  // Agent thread: drains the RC queue every cfg.link.rc_drain_ms, ticks
  // RcAgent on cfg.link.tick_ms, runs the watchdog, and handles SIGUSR1
  // stats dumps.
  std::thread agent_thread([&]() {
    name_thread("mbr-agent");
    const uint64_t grace_ms = 10000;
    const uint64_t stale_ms = 3000;
    uint64_t start = now_steady_ms();

    uint64_t last_hot_beat = 0, last_rx_beat = 0;
    uint64_t last_hot_change_ms = start, last_rx_change_ms = start;
    uint64_t last_stats_ms = start;
    // The MAV counters as of the last stats line printed with them.
    uint64_t mav_last[6] = {};

    // T_TELEM: sent at ~1 Hz on this same periodic path, on the mutex-guarded
    // dev_sink.send() the MSP thread also uses. The tlm_seq/dot11 seq
    // counters are declared at run_real_mode scope (not here) — deliberately
    // NOT the video path's tx.seq() or RealActuator's DISC_ACK control_seq,
    // so a telemetry-send bug can never perturb either. The dot11 seq is
    // shared with the MAV thread and the TX writer thread's calibration
    // replies -- see that declaration's comment.
    uint64_t last_telem_ms = start;
    ControlRadiotap agent_radiotap = make_control_radiotap();
    mabur::CpuBusySampler cpu_busy;  // /proc/stat delta per telemetry tick

    mabur::TickGate tick_gate(now_steady_ms(), cfg.link.tick_ms);
    // Peak 100 ms encoder rate for the stats line (peak_rate.h): fed every
    // wake, which is the finest cadence anything here runs at; normalised
    // by frame count at the configured sensor fps, not wall time.
    mabur::PeakRate enc_peak(100, cfg.venc.core.fps);
    // Calibration ladder gate, agent side: RealActuator::apply_op's
    // set_ladder() is gated off while cal_active (see that call site), so the
    // ladder sits at whatever CalSweep's TX-writer-thread sweep last parked
    // it at until this re-applies the last-known-correct op -- once, right on
    // the falling edge, on THIS thread (apply_op's documented contract), not
    // from the TX writer thread that noticed cal_active clear.
    bool cal_was_active_for_ladder = false;
    uint32_t cal_epoch_seen = 0;
    // TxStats::failed as of the last housekeeping tick, for send_telem.
    uint64_t telem_usb_fail = 0;
    // One T_TELEM frame: the 1 Hz periodic one, and the link-pairing
    // "promote" frame sent right after an RCF promoted a session, before
    // tick() runs its deferred retune.
    auto send_telem = [&](uint64_t now) {
      TelemInputs ti;
      ti.state = static_cast<int>(agent.state());
      ti.failsafe_shed = agent.failsafe_shed();
      ti.congestion_shed = agent.congestion_shed();
      ti.low_power = agent.low_power();
      ti.auth_reject = agent.take_auth_reject();
      ti.rec_status = vtx_rec.status_byte();
      // have_feedback() false means no RCF has EVER been accepted (still
      // BOOT/RENDEZVOUS) — 0 would read as maximally fresh, the opposite of
      // the truth. Pass a value make_telem's saturate<uint16_t> clamps to
      // 65535 ("never"), matching the wire field's documented sentinel.
      ti.rcf_age_ms = agent.have_feedback()
                          ? (now - agent.last_feedback_ms())
                          : static_cast<uint64_t>(UINT16_MAX) + 1;
      // link-rtt: which RCF that age references. Invalid (flags bit3
      // clear) outside LINKED (failsafe rebase, unconfirmed-move fallback),
      // where the age is fresh but no RCF backs it — the GS must not match
      // a stale seq against it.
      if (const auto fseq = agent.last_feedback_seq()) {
        ti.rcf_seq_echo = *fseq;
        ti.rcf_seq_echo_valid = true;
      }
      ti.rcf_rx = agent.rcf_accepted();
      ti.cmd_kbps = actuator.last_bitrate_kbps;
      ti.txq_drops = txq.dropped();
      ti.txq_wait_max_ms = txq_wait_max_ms.exchange(0, std::memory_order_relaxed);
      ti.usb_fail = telem_usb_fail;
      // fec-nack, per period: bumped by the RX thread's T_NACK handler,
      // drained here.
      ti.nack_rx = nack_rx.exchange(0, std::memory_order_relaxed);
      ti.retx_syms = retx_syms.exchange(0, std::memory_order_relaxed);
      ti.retx_refused = retx_refused.exchange(0, std::memory_order_relaxed);
      // RX-side channel view for this period (cca-on): the RX callback's
      // frame split, drained per Telem. No register read here -- see
      // rx_own_frames' declaration for why.
      ti.rx_own = rx_own_frames.exchange(0, std::memory_order_relaxed);
      ti.rx_foreign = rx_foreign_frames.exchange(0, std::memory_order_relaxed);
      ti.rx_crcfail = rx_crcfail_frames.exchange(0, std::memory_order_relaxed);
      ti.uplink = uplink_track.snap();
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
          telem_wire_seq.fetch_add(1, std::memory_order_relaxed), ti);
      auto telem = rc::pack_telem(telem_struct);

      const std::vector<uint8_t>& telem_radiotap = agent_radiotap.get();
      std::vector<uint8_t> frame;
      frame.reserve(telem_radiotap.size() + kDot11HeaderLen + telem.size());
      frame.insert(frame.end(), telem_radiotap.begin(), telem_radiotap.end());
      const uint16_t dot11_seq =
          telem_dot11_seq.fetch_add(1, std::memory_order_relaxed) & 0xFFF;
      auto hdr = build_dot11_header(dot11_seq);
      frame.insert(frame.end(), hdr.begin(), hdr.end());
      frame.insert(frame.end(), telem.begin(), telem.end());
      dev_sink.send(frame.data(), frame.size());
    };
    while (!g_devourer_should_stop) {
      uint64_t now = now_steady_ms();
      enc_peak.sample(now, enc_bytes_total.load(std::memory_order_relaxed),
                      enc_frames_total.load(std::memory_order_relaxed));

      // A sweep start raises cal_epoch after cal_active (LivePowerTable::
      // begin_cal), so reading the epoch first means a new epoch is seen
      // with cal_active already up -- unless calibration has since let go,
      // which is the edge this handles. The epoch catches the up-and-down
      // too short for two polls to straddle (a sweep refused, or a command
      // the engine ignored): the device writes made for it still need
      // undoing.
      const uint32_t cal_epoch_now = cal_epoch.load(std::memory_order_acquire);
      const bool cal_now_active = cal_active.load(std::memory_order_acquire);
      if (!cal_now_active &&
          (cal_was_active_for_ladder || cal_epoch_now != cal_epoch_seen)) {
        // Video resumes on this same cal_active transition (hot thread's
        // own guard), so this cannot wait for the next RCF -- re-apply the
        // agent's own last-known-good op immediately -- ladder, A-MPDU mode
        // (calibration programmed the chip's mode itself) and power table.
        actuator.on_cal_end();
        actuator.apply_op(agent.current());
        // Deferred retune replay: a retune requested during the sweep was
        // latched, not performed (RealActuator::retune). The sweep's device
        // calls are done as of this edge, so replay it here, on the agent
        // thread, next to the ladder re-apply.
        actuator.apply_deferred_retune();
      }
      cal_was_active_for_ladder = cal_now_active;
      if (!cal_now_active) cal_epoch_seen = cal_epoch_now;

      // Every wake (rc_drain_ms): drain and apply queued RCFs. This is the
      // whole point of the split — op actuation does not wait for the
      // housekeeping tick (a tick-bound drain measured U(0, tick_ms=100),
      // close_ms median ~110 ms).
      std::vector<uint8_t> rc_body;
      while (rc_queue.pop(rc_body)) {
        agent.on_rc_frame(rc_body.data(), rc_body.size(), now);
      }
      // Promote Telem: one frame on the CURRENT channel telling the GS we
      // hold its session, before tick() runs the deferred retune. The GS
      // follows on a LINKED Telem.
      if (agent.take_session_promoted()) send_telem(now);

      if (tick_gate.due(now)) {
        // The power table follows the rung's width on every op (apply_op);
        // this catches the ops that landed before the device was up, or
        // while a parameter set's reprogram raced a width change.
        if (!cal_now_active && agent.current().generation != 0)
          power_table.follow(agent.current().ladder[1].bw);
        devourer::ThermalStatus thermal = rtl_device->GetThermalStatus();
        devourer::TxStats txstats = rtl_device->GetTxStats();
        RadioHealth health;
        health.thermal_delta = thermal.valid ? thermal.delta : 0;
        health.tx_drops = txstats.failed;
        telem_usb_fail = txstats.failed;
        health.txq_depth = txq.depth();
        health.txq_cap = kTxQueueCap;
        agent.tick(now, health);
        if (agent.take_link_established())
          link_up_discont.store(true, std::memory_order_relaxed);

        // Watchdog: after an initial grace period, a heartbeat going stale for
        // > stale_ms means the corresponding loop is wedged — EXCEPT rx_beat,
        // which only bumps when a frame is actually received off the air.
        // Silence on the RX side is the expected steady state whenever the
        // agent isn't LINKED (RENDEZVOUS: waiting for a DISC beacon; FAILSAFE:
        // GS has gone quiet, which is exactly the scenario failsafe exists
        // for) — gating rx-stale detection on agent.state() == LINKED tells
        // "the receive path is stuck" apart from "the ground station turned
        // off," which used to abort()/respawn-loop maburd forever on a merely
        // quiet channel. The hot-thread check stays unconditional: hot_beat
        // bumps every ring-read iteration regardless of RF activity, so its
        // staleness always means the pipeline thread itself is wedged. This
        // check runs on the agent thread, which already owns `agent`
        // (RcAgent::tick() above is called from here), so reading
        // agent.state() here is the same-thread access it already is
        // elsewhere in this loop — no cross-thread synchronization needed.
        uint64_t hb = hot_beat.load(std::memory_order_relaxed);
        uint64_t rb = rx_beat.load(std::memory_order_relaxed);
        if (hb != last_hot_beat) {
          last_hot_beat = hb;
          last_hot_change_ms = now;
        }
        if (rb != last_rx_beat) {
          last_rx_beat = rb;
          last_rx_change_ms = now;
        }
        // Calibration radio silence: the GS transmits nothing while a sweep
        // airs, so
        // rx_beat legitimately stalls for the session's duration -- pin the
        // "last seen" clock to now instead of gating the check below on
        // cal_active, so the instant the session ends the watchdog gets a
        // full fresh stale_ms window rather than reading however long the
        // session already ran as instant staleness.
        if (cal_active.load(std::memory_order_relaxed)) last_rx_change_ms = now;
        if (now - start > grace_ms) {
          if (now - last_hot_change_ms > stale_ms) {
            std::fprintf(stderr, "watchdog: hot thread stalled (no beat for >%llums)\n",
                         static_cast<unsigned long long>(stale_ms));
            std::abort();
          }
          if (agent.state() == RcAgent::State::LINKED && now - last_rx_change_ms > stale_ms) {
            std::fprintf(stderr,
                         "watchdog: rx loop stalled while LINKED (no beat for >%llums)\n",
                         static_cast<unsigned long long>(stale_ms));
            std::abort();
          }
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
          char mav_part[128] = "";
          const uint64_t mav_now[6] = {
              mav_rx_frames.load(std::memory_order_relaxed),
              mav_rx_packets.load(std::memory_order_relaxed),
              mav_tx_frames.load(std::memory_order_relaxed),
              mav_tx_packets.load(std::memory_order_relaxed),
              mav_drop.load(std::memory_order_relaxed),
              mav_reject.load(std::memory_order_relaxed)};
          if (!std::equal(std::begin(mav_now), std::end(mav_now), std::begin(mav_last))) {
            std::copy(std::begin(mav_now), std::end(mav_now), std::begin(mav_last));
            std::snprintf(mav_part, sizeof mav_part,
                          " mav rx=%llu/%llu tx=%llu/%llu drop=%llu reject=%llu",
                          static_cast<unsigned long long>(mav_now[0]),
                          static_cast<unsigned long long>(mav_now[1]),
                          static_cast<unsigned long long>(mav_now[2]),
                          static_cast<unsigned long long>(mav_now[3]),
                          static_cast<unsigned long long>(mav_now[4]),
                          static_cast<unsigned long long>(mav_now[5]));
          }
          const ConfigApplierStats apply_stats = applier.stats();
          std::fprintf(stderr,
                       "stats: state=%d hot_beat=%llu rx_beat=%llu seq=%u sent=%llu drops=%llu "
                       "txq=%zu txq_drop=%llu "
                       "thermal_delta=%d tx_failed=%llu venc_verb_fail=%llu "
                       "enc_pk100=%uk lp=%d armed=%d snr_mhz=%u snr_off=%d "
                       "mav_apply=%llu/%llu mav_rebuild=%llu%s\n",
                       static_cast<int>(agent.state()), static_cast<unsigned long long>(hb),
                       static_cast<unsigned long long>(rb), tx.seq(),
                       static_cast<unsigned long long>(tx.sent()),
                       static_cast<unsigned long long>(tx.drops()),
                       txq.depth(), static_cast<unsigned long long>(txq.dropped()),
                       health.thermal_delta,
                       static_cast<unsigned long long>(txstats.failed),
                       static_cast<unsigned long long>(actuator.venc_verb_failures),
                       enc_peak.take_peak_kbps(),
                       static_cast<int>(agent.low_power()), static_cast<int>(agent.armed_latched()),
                       snr_mhz, snr_off,
                       static_cast<unsigned long long>(apply_stats.ok),
                       static_cast<unsigned long long>(apply_stats.failed),
                       static_cast<unsigned long long>(apply_stats.rebuilds), mav_part);
        }

        // Telemetry suppression: non-sweep PPDUs in the airtime a calibration
        // phase is characterizing buy nothing -- drone TX does not itself
        // blank the GS receivers, so this is an airtime courtesy, not a
        // correctness requirement. Gated on cal_sweeping specifically
        // (State::Sweeping), not cal_active (the whole session): the ack
        // Telem (TX writer thread) is sent in the between-phase gap this
        // narrower flag leaves open, and skipping last_telem_ms's update
        // while suppressed means telemetry resumes on the very next tick once
        // a phase ends, not up to 1 s later.
        if (!cal_sweeping.load(std::memory_order_relaxed) &&
            now - last_telem_ms >= 1000) {
          last_telem_ms = now;
          send_telem(now);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(cfg.link.rc_drain_ms));
    }
  });


  // radio.width is honoured by the InitWrite above: 40 tunes the standard
  // pair via ht40_offset; FastRetune keeps width and offset across every
  // later hop.

  // TX bring-up must be complete before anything transmits. InitWrite (on
  // radio_init_thread, started right after CreateRadio above) runs the
  // full power-on + firmware download + TX-path enable and returns only
  // once the chip is ready; StartRxLoop then runs the (blocking) RX worker
  // with TX+RX concurrent on the same handle. Opening device_ready between
  // the two is what keeps the hot/agent threads from clogging the bulk-OUT
  // FIFO mid-DLFW — see DevourerSink::ready.
  join_radio_init();
  if (radio_init_error) std::rethrow_exception(radio_init_error);

  // Adapter caps + RX-sensor availability record, once at bring-up. Static
  // per chip identity (AdapterCaps.h), so one read on the main thread here
  // -- before StartRxLoop, so there is no sender/hot-thread contention to
  // worry about -- is all this ever needs. GetRxEnergy(with_nhm=true) is
  // the with-NHM variant (~2 ms) since this is a one-shot startup read, not
  // a sampling-cadence call.
  {
    const devourer::AdapterCaps ac = rtl_device->GetAdapterCaps();
    const RxEnergy e = rtl_device->GetRxEnergy(/*with_nhm=*/true);
    std::fprintf(stderr,
                 "maburd radio caps: %s %s %ux%u bw=%x tune5g=%u-%u fast_retune=%d "
                 "sensors fa=%d igi=%d nhm=%d floor=%d\n",
                 ac.chip_name ? ac.chip_name : "?", devourer::generation_name(ac.generation),
                 ac.tx_chains, ac.rx_chains, ac.bw_mask,
                 ac.tune_5g.valid ? ac.tune_5g.min_mhz : 0, ac.tune_5g.valid ? ac.tune_5g.max_mhz : 0,
                 ac.fastretune_ok ? 1 : 0, e.valid_fa ? 1 : 0, e.valid_igi ? 1 : 0,
                 e.valid_nhm ? 1 : 0, e.valid_noise_floor ? 1 : 0);
  }

  // Bring-up record of the MAC carrier-sense state requested via
  // dev_cfg.tuning.disable_cca above. devourer logs its own carrier-sense line
  // at info, and the production cross-build compiles info out
  // (DEVOURER_LOG_MAX_LEVEL=WARN), so without this the deployed daemon leaves
  // no trace of which way it was built. Records what maburd REQUESTED of
  // devourer, not a register readback.
  std::fprintf(stderr,
               "maburd radio: MAC carrier sense (CCA+EDCCA) requested %s\n",
               dev_cfg.tuning.disable_cca
                   ? "OFF -- TX will not defer to co-channel traffic"
                   : "ON -- TX defers to any decodable 802.11 preamble (chip default)");

  // Always start clean: a flat TXAGC index override is sticky in the chip
  // across a process restart, and the ONLY place that ever sets one is a
  // calibration sweep (cal_sweep.cpp/CalSweep::PowerCtl -- pump parks it
  // per cell, close_session parks it at the anchor). A maburd
  // killed mid-session (operator interrupt, wrapper respawn, watchdog)
  // never reaches the TX writer thread's falling-edge restore
  // (restore_operating_power(), below) and hands off to the next process
  // with the override still live -- reverting it here, unconditionally,
  // before the power plan is applied, means a fresh process never depends
  // on how the previous one died. -1 reverts to the calibrated table
  // (devourer/src/TxPower.h); a bring-up with no history at all just
  // clears a knob that was already clear.
  rtl_device->SetTxPowerIndexOverride(-1);

  // The chip's TXAGC reference for the boot channel, read with no custom
  // table live (GetTxPowerState reports the reference itself then). Used
  // ONLY to cap reference + diff at 127 (power_plan.h); it never reaches
  // config or the wire. 0 = unknown, no cap. This is the ONE read: see the
  // declaration of boot_anchor_idx for why a channel move re-applies TX
  // power (devourer re-derives the new channel's anchor) instead of
  // re-reading here.
  {
    const auto st = rtl_device->GetTxPowerState();
    if (st.valid && st.mcs7_index >= 0) boot_anchor_idx = st.mcs7_index;
  }

  // power_mode == "offset": program the start width's wall-equalized
  // per-rate diff table and zero the global offset; from here the table
  // follows the rung's width (RealActuator::apply_op), and nothing else
  // moves power. "none" never touches it. SetTxPowerRateDiffs returns false
  // on chips without a rate-diff table (both Jaguar3 dies, 8822E and 8822C,
  // have one) -- warn and continue rather than aborting bring-up, so
  // "offset" configured on an unsupported chip degrades to the chip's
  // default power instead of failing to fly.
  for (const auto& note : missing_wall_notes(cfg.radio))
    std::fprintf(stderr, "maburd %s\n", note.c_str());
  for (const auto& warn : ldpc_mismatch_warnings(cfg.radio))
    std::fprintf(stderr, "maburd warning: %s\n", warn.c_str());
  if (!power_table.replace(plans_for(cfg.radio, cfg.radio.wall_margin_db), cfg.radio.width,
                           /*zero_offset=*/true, /*clear_override=*/false)) {
    std::fprintf(stderr,
                 "warning: SetTxPowerRateDiffs failed; power_mode=offset "
                 "falls back to the chip's default TX power\n");
  }

  // A-MPDU TX aggregation is NOT programmed here: the mode follows the ladder
  // rung (RealActuator::apply_ampdu_for, drone/src/ampdu_policy.h) and the
  // agent thread programs it on every op change. The chip leaves InitWrite in
  // the singles state, which is the right state for the boot MAX_RANGE op
  // (rung 0) under the shipped ampdu.min_mcs_20 4, so there is no window in
  // which frames fly the wrong mode. Frames are QoS-Data whether or not
  // aggregation is on (radio_tx.cpp), so the wire never depends on this.

  device_ready.store(true, std::memory_order_release);
  std::fprintf(stderr, "maburd entering RX loop on channel %d\n", start_ch);
  rtl_device->StartRxLoop(rx_callback);

  // Init() returns once g_devourer_should_stop is set (SIGINT/SIGTERM) or the
  // device errors out internally. Ensure the flag is set on the error-out
  // path too, so hot_thread/agent_thread (which key off the same flag) are
  // guaranteed to exit and these joins complete.
  g_devourer_should_stop = true;
  // First: an apply in flight finishes here, and none starts after, so no
  // hook (a pipeline restart, start_msp) runs against the teardown below.
  applier.shutdown();
  if (hot_thread.joinable()) hot_thread.join();
  if (tx_thread.joinable()) tx_thread.join();
  if (agent_thread.joinable()) agent_thread.join();
  stop_msp();
  if (mav_thread.joinable()) mav_thread.join();
  tx_pool.stop();  // drain + join senders before device teardown

  vtx_rec.shutdown();   // close the file before the record channel is torn down
#ifdef MABUR_HAVE_VENC
  // After the thread joins above: the hot thread reads the frame ring that
  // stop() tears down.
  venc_core_stop();
#endif

  rtl_device->Stop();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(usb_ctx);
  return 0;
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

void print_usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s -c <config.toml> [--dry-run --in <file> --out <file> [--rc-in <file>]]\n",
               argv0);
}

}  // namespace

int main(int argc, char** argv) {
  std::string cfg_path;
  bool dry_run = false;
  std::string in_path, out_path, rc_in_path;
  std::string msp_in_path, msp_out_path;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) {
      cfg_path = argv[++i];
    } else if (a == "--dry-run") {
      dry_run = true;
    } else if (a == "--in" && i + 1 < argc) {
      in_path = argv[++i];
    } else if (a == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else if (a == "--rc-in" && i + 1 < argc) {
      rc_in_path = argv[++i];
    } else if (a == "--msp-in" && i + 1 < argc) {
      msp_in_path = argv[++i];
    } else if (a == "--msp-out" && i + 1 < argc) {
      msp_out_path = argv[++i];
    } else if (a == "-h" || a == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      print_usage(argv[0]);
      return 1;
    }
  }

  if (cfg_path.empty()) {
    print_usage(argv[0]);
    return 1;
  }

  Config cfg;
  std::vector<std::string> defaulted;
  try {
    cfg = load_config(cfg_path, &defaulted);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  if (!defaulted.empty()) {
    std::fprintf(stderr, "config: %zu key(s) defaulted:\n", defaulted.size());
    for (const std::string& d : defaulted)
      std::fprintf(stderr, "  %s\n", d.c_str());
  }

  // Auto channel set: park on the remembered member, else the first. A
  // missing/garbage/non-member file is silently the first member -- nothing
  // to configure, nothing to repair.
  const auto remembered_ch = mabur::read_channel_file(mabur::kDroneChannelFile);
  const bool use_remembered =
      remembered_ch && mabur::channel_set_member(cfg.radio.channels, *remembered_ch);
  const uint8_t start_ch = use_remembered ? *remembered_ch : cfg.radio.channels.front();
  {
    std::string set_str;
    for (size_t i = 0; i < cfg.radio.channels.size(); ++i) {
      if (i) set_str += ",";
      set_str += std::to_string(cfg.radio.channels[i]);
    }
    std::fprintf(stderr, "maburd: channel set [%s], parking on %u%s\n", set_str.c_str(),
                 static_cast<unsigned>(start_ch), use_remembered ? " (remembered)" : "");
  }

  std::fprintf(stderr,
               "fec: symbol_size=[%d,%d] bpb=[%d,%d] window=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.blocks_per_body[0], cfg.fec.blocks_per_body[1],
               cfg.fec.window);

  std::fprintf(stderr, "link: key %s (%s)\n",
               mabur::key_fingerprint(cfg.link.key).c_str(), cfg.link.key_source.c_str());
  if (cfg.link.key_is_default)
    std::fprintf(stderr, "link: DEFAULT key in use -- any default install can control this drone; "
                         "see docs/deploy.md 'Pairing'\n");

  if (dry_run) {
    if (in_path.empty() || out_path.empty()) {
      std::fprintf(stderr, "error: --dry-run requires --in and --out\n");
      print_usage(argv[0]);
      return 1;
    }
    return run_dry_run(cfg, start_ch, in_path, out_path, rc_in_path, msp_in_path, msp_out_path);
  }

  return run_real_mode(cfg, start_ch, cfg_path);
}
