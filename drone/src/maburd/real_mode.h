#pragma once
// Real mode: the devourer USB radio, the in-process venc core and the
// threads between them. RealMode holds the state those threads share.
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "air_clock.h"
#include "config.h"
#include "config_apply.h"
#include "igi_floor_watchdog.h"
#include "live_power_table.h"
#include "power_tables.h"
#include "radio_glue.h"
#include "radio_tx.h"
#include "rc_agent.h"
#include "real_actuator.h"
#include "send_timer.h"
#include "telemetry.h"
#include "tx_queue.h"
#include "usb_tx_pool.h"
#include "vtx_recorder.h"
#include "mabur/cal_proto.h"
#include "mabur/retx_ring.h"
#include "mabur/sw_wire.h"
#ifdef MABUR_HAVE_VENC
#include "venc_core.h"  // ARM only: drone/venc is not compiled on host builds
#endif

#include "IRtlRadio.h"

namespace mabur::maburd {

using namespace mabur;

// Set by SIGUSR1; the agent thread prints a stats line on its next tick.
extern std::atomic<bool> g_sigusr1_flag;

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

// TX body queue between the encode (hot) thread and the USB writer
// thread: a bulk-OUT stall backs up HERE (bounded, drop-oldest =
// FEC-recoverable erasures) instead of in the waybeam SHM ring, whose
// overflow makes the RTP packetizer abort NALs mid-chain — sender-side
// slice truncation no FEC can repair (the PixelPilot glitch root cause).
// ~256 bodies ≈ 150 ms at 1700 bodies/s.
constexpr size_t kTxQueueCap = 256;  // also the congestion shed's txq_cap

// The state run_real_mode's threads share. Members are declared in the order
// run_real_mode's bring-up needs; construction is allocation and wiring only,
// so building it before the venc boot costs nothing. Built once, after the
// radio bring-up thread starts; threads hold references into it, so it is
// neither copyable nor movable.
struct RealMode {
  RealMode(const Config& cfg, uint8_t start_ch, const std::string& cfg_path, IRtlRadio* dev,
           const IgiFloorParams& igi);
  RealMode(const RealMode&) = delete;
  RealMode& operator=(const RealMode&) = delete;

  const Config& cfg;
  const uint8_t start_ch;
  const std::string& cfg_path;
  // Set by run_real_mode right after construction, before any thread starts.
  uint64_t process_start_ms = 0;
  bool ota_trial = false;
  std::string ota_dir;
  const IgiFloorParams igi_params;
  // Owned by run_real_mode's unique_ptr, which outlives this struct.
  IRtlRadio* const rtl_device;

  // Gate for DevourerSink: stays false until InitWrite() completes bring-up.
  std::atomic<bool> device_ready{false};

  // Exclusive against RealActuator::retune's FastRetune (devourer threading
  // contract: control-plane calls must not overlap a bulk-OUT from any
  // sender thread). Every USB sender -- DevourerSink::send/
  // send_many's direct branch, and the UsbTxPool send lambda -- takes
  // this shared; retune takes it exclusive. Declared ahead of every
  // sender that references it.
  std::shared_mutex tx_gate;
  // Writer-priority flag for tx_gate: raised by
  // RealActuator::retune while it waits for the exclusive lock, spun on by
  // every shared-taker before it enters. glibc's rwlock is reader-preferring
  // and the writer runs on the agent thread — see await_retune_gate.
  std::atomic<bool> retune_waiting{false};

  // What the IGI floor watchdog thread reads, all lock-free: the longest
  // video send, the video width, the retune count, and (for the stats line)
  // the floor it last set.
  mabur::SendTimer send_timer;
  std::atomic<uint8_t> video_bw{20};
  std::atomic<uint32_t> retune_seq{0};
  std::atomic<uint8_t> igi_floor_now{igi_params.base};

  DevourerSink dev_sink;

  // Capacity 6 frames per sender: enough to keep every sender's next ≤3-
  // frame URB staged while it blocks in the current one, small enough that
  // backlog still lands in TxQueue (whose drop-oldest policy is the
  // FEC-recoverable erasure path).
  mabur::UsbTxPool tx_pool;

  RadioTx tx;

  std::atomic<std::shared_ptr<const AppliedOp>> shared_op{nullptr};

  // Debug-HTTP per-layer overhead override, shared by the HTTP thread (the
  // writer), the hot thread (which applies it to the UEP layers) and the
  // agent thread (whose bitrate target must describe what is actually
  // flying) — see OvOverride's doc comment in rc_agent.h. Must outlive both
  // hot_thread and agent_thread, so it is a RealMode member beside
  // shared_op, not a local of either thread function.
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

  // VTX onboard recorder. Constructed before the agent (which reaches it
  // through actuator.recorder) and before venc boots (venc_record_configure
  // hands it the drain sink).
  VencRecordChannel rec_channel;
  mabur::VtxRecorder vtx_rec;

  RcAgent agent;

#ifdef MABUR_HAVE_VENC
  // The encoder's callbacks: filled in at boot, reused by the config
  // applier's pipeline_cold_restart.
  VencCallbacks vcb{};
#endif

  RcQueue rc_queue;
  // T_MAV lands here (RX callback, rx_path.cpp), consumed by the MAV thread: a
  // parameter set rewrites the config file, I/O that belongs on neither the
  // RX nor the agent thread.
  RcQueue mav_queue;
  // The radio's chip name for the update lane's MABUR_IDENT: read on the
  // main thread at bring-up, after the MAV thread has started. radio_chip is
  // written once, before the release store; the MAV thread reads it only
  // after an acquire load sees true.
  std::string radio_chip;
  std::atomic<bool> radio_chip_ready{false};
  // The calibration messages the MAV thread pulls out of verified cal-only
  // T_MAV frames, for the TX writer thread: CalSweep's on_cmd/on_apply/
  // on_abort must run on pump()'s thread (cal_sweep.h constraint 1).
  CalMsgQueue cal_queue;
  // The drone's T_MAV_UP counter, shared by the MAV thread's own uplink and
  // the TX writer thread's calibration replies: the GS takes each counter
  // once inside a 64-wide anti-replay window per pair, so the two threads
  // must never reuse one, and may air theirs out of order.
  std::atomic<uint32_t> mav_up_counter{0};
  // MAV thread -> agent thread's stats line, cumulative. drop = frames that
  // failed parse_mav, frames under the update pair carrying anything but the
  // update lane, plus packets the component dropped (not for us, bad
  // MAVLink); reject = frames that parsed but failed the tag or the counter.
  std::atomic<uint64_t> mav_rx_frames{0}, mav_rx_packets{0}, mav_tx_frames{0},
      mav_tx_packets{0}, mav_drop{0}, mav_reject{0};
  // Calibration replies not sent for want of a pair to tag under; folded into
  // mav_drop's count.
  std::atomic<uint64_t> mav_up_nopair{0};
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
  // The measured TX walls per (channel, width) (wall_cache.h): loaded at
  // bring-up, written by a wall measurement on the TX writer thread, read
  // by every plans_for() caller.
  WallCache wall_cache;
  std::mutex wall_cache_mu;

  // The TX-power table (live_power_table.h): bring-up programs the start
  // channel and width's plan, the agent thread follows the rung's width and
  // every retune, RADIO_WALL_MARGN's and RADIO_LDPC's reprogram and
  // calibration replace the plans.
  RadioPowerTableDevice power_dev;
  LivePowerTable power_table;

  // Uplink RSSI/SNR EMAs, fed from rx_callback (RX thread) on CRC-clean RC
  // frames, read by the agent thread's 1 Hz telemetry collector. Thread-safe
  // per UplinkTrack's own mutex.
  UplinkTrack uplink_track;

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

  // fec-nack: retransmit ring of base-layer source envelopes (hot thread
  // writes via the UepEncoder source tap, RX thread reads) and the T_NACK
  // handler, bound by install_nack_hook (rx_path.cpp) once txq exists.
  // Sized by [nack].ring_ms of base-layer history at
  // encoder.bitrate_max_kbps; env_len is the sealed base envelope (sw header
  // + base symbol_size). Counters are per Telem period (exchanged at the 1 Hz
  // build); nack_bad / retx_miss are drone-local only.
  const size_t retx_env_len = static_cast<size_t>(mabur::sw::kSwHeaderLen) + static_cast<size_t>(cfg.fec.symbol_size[0]);
  mabur::RetxRing retx;
  std::function<void(const uint8_t*, size_t)> nack_hook;
  std::atomic<uint64_t> nack_rx{0}, nack_bad{0}, retx_syms{0}, retx_refused{0}, retx_miss{0};

  // The MSP thread: reads the FC's serial link, sends its DisplayPort stream
  // to the GS and polls the arm state for low-power mode. A restartable
  // unit: boot starts it iff msp.enable, and a parameter set starts or stops
  // it (start_msp/stop_msp, called from one thread at a time: boot, then the
  // config applier, then teardown after the applier has shut down). `m` is
  // the [msp] section it runs with; `stop` ends it within one serial read
  // (MspSerial::read returns within its 100 ms VTIME).
  std::thread msp_thread;
  std::atomic<bool> msp_stop{false};

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
  ConfigApplier applier;

  TxQueue txq;

  // Air clock. Lives out here, not in the hot thread, since fec-nack: the hot
  // thread prices it (set_rates via apply_op_to_clock), books every
  // video/probe body and reads the backlog; the RX thread's T_NACK handler
  // books every retransmit body. AirClock's own mutex guards it; both
  // sides lock per BODY, never per symbol.
  AirClock air_clock;

  // Each Telem/MAVLink sender (agent thread, TX writer thread's calibration
  // replies, MAV thread) builds its own radiotap from this.
  ControlRadiotap make_control_radiotap();
  // The plans a RadioCfg flies: one per wall cache entry, derived under the
  // live LDPC flag, none at all under power_mode "none". Takes
  // wall_cache_mu.
  ChannelPlans plans_for(const RadioCfg& r, double margin_db);
#ifdef MABUR_HAVE_VENC
  // The record channel's configuration from [record]: here at boot, and
  // again by the config applier before every pipeline rebuild, since the
  // record channel is part of the pipeline it rebuilds.
  void record_configure(const Config& c);
#endif
  // The MSP thread's body (see msp_thread), and its start and stop.
  void run_msp(std::atomic<bool>& stop, const MspCfg m);
  void start_msp(const MspCfg& m);
  void stop_msp();
  // The config applier's hooks; every one runs on the applier's thread.
  ConfigApplyHooks make_apply_hooks();
  // The sink and actuator wiring, run from agent's initializer so RcAgent is
  // built on a wired actuator, as it always was.
  RealActuator& wire_sink_and_actuator();
};

// One function per thread body (and the RX callback), each in its own file.
void rx_on_packet(RealMode& m, const Packet& pkt);
void install_nack_hook(RealMode& m);  // assigns m.nack_hook; the hook owns its RX-only state
void hot_loop(RealMode& m);
void tx_writer_loop(RealMode& m);
void agent_loop(RealMode& m);
void mav_loop(RealMode& m);
void igi_wd_loop(RealMode& m);

int run_real_mode(const Config& cfg, uint8_t start_ch, const std::string& cfg_path);

}  // namespace mabur::maburd
