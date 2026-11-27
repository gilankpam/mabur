// Real mode: bring-up, the threads, the RX loop and teardown (real_mode.h).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "cal_link.h"
#include "debug_http.h"
#include "ota.h"
#include "mi_ready.h"
#include "real_mode.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"

#include <libusb.h>

namespace mabur::maburd {

using namespace mabur;

std::atomic<bool> g_sigusr1_flag{false};

namespace {

void handle_sigusr1(int) { g_sigusr1_flag.store(true); }

}  // namespace

// SIGINT/SIGTERM shutdown: devourer's IRtlRadio::Init() runs a blocking RX
// loop that only observes the library-global g_devourer_should_stop (see
// ../devourer/src/SignalStop.h and how examples/rx, examples/tx, and
// examples/doctor all call install_devourer_signal_handlers() instead of
// installing their own SIGINT/SIGTERM handlers). A locally-scoped flag set by
// a handler maburd installs itself is never consulted by Init()'s loop, so
// Ctrl-C/SIGTERM would never unblock it. The loops in hot_thread.cpp and agent_thread.cpp
// read g_devourer_should_stop directly too, so all three (Init()'s RX loop,
// hot_thread, agent_thread) stop on the same signal.

RealMode::RealMode(const Config& cfg, uint8_t start_ch, const std::string& cfg_path,
                   IRtlRadio* dev, const IgiFloorParams& igi)
    : cfg(cfg),
      start_ch(start_ch),
      cfg_path(cfg_path),
      igi_params(igi),
      rtl_device(dev),
      // Capacity 6 frames per sender: enough to keep every sender's next ≤3-
      // frame URB staged while it blocks in the current one, small enough that
      // backlog still lands in TxQueue (whose drop-oldest policy is the
      // FEC-recoverable erasure path).
      tx_pool(
        [dev = rtl_device, this](
            const std::vector<std::vector<uint8_t>>& b) {
          std::vector<TxPacketView> v(b.size());
          for (size_t i = 0; i < b.size(); ++i) v[i] = {b[i].data(), b[i].size()};
          await_retune_gate(&retune_waiting);
          std::shared_lock<std::shared_mutex> sg(tx_gate);
          // A frame queued before a wall measurement began (a capacity
          // blast's tail) must not air inside its cross-path session.
          if (radio_quiet.load(std::memory_order_acquire)) return size_t{0};
          const int slot = send_timer.begin(now_steady_us());
          const size_t sent = dev->send_packets(v.data(), v.size());
          send_timer.end(slot, now_steady_us());
          return sent;
        },
        cfg.radio.tx_threads,
        static_cast<size_t>(cfg.radio.tx_threads) * 6),
      tx(dev_sink),
      // VTX onboard recorder. Constructed before the agent (which reaches it
      // through actuator.recorder) and before venc boots (venc_record_configure
      // hands it the drain sink).
      vtx_rec(cfg.record, rec_channel, cfg.venc.core.width, cfg.venc.core.height),
      agent(cfg, wire_sink_and_actuator(), &ov_override, start_ch),
      retx(mabur::RetxRing::slots_for(cfg.encoder.bitrate_max_kbps, cfg.nack.ring_ms,
                                      cfg.fec.symbol_size[0]),
           retx_env_len),
      applier(cfg_path, cfg, make_apply_hooks(), [](const std::string& line) {
        std::fprintf(stderr, "maburd config: %s\n", line.c_str());
      }),
      txq(kTxQueueCap),
      air_clock(kTxQueueCap) {
  power_dev.dev = rtl_device;
  power_dev.gate = &tx_gate;
  power_dev.gate_waiting = &retune_waiting;
  power_table.dev = &power_dev;
  power_table.ready = &device_ready;
  power_table.cal_active = &cal_active;
  power_table.cal_epoch = &cal_epoch;
  // The calibration ladder gate lives on the agent thread (RealActuator::
  // apply_op) -- wire it up now that cal_active exists.
  actuator.cal_active = &cal_active;
  actuator.power_table = &power_table;
  // fec.feed_batch: group the TX writer's wakeups so bodies leave in
  // URB-filling batches (see TxQueue::set_batch); the hot thread flushes at
  // every AU end so a tail group never waits on the next frame.
  if (cfg.fec.feed_batch > 1)
    txq.set_batch(static_cast<size_t>(cfg.fec.feed_batch));
}

RealActuator& RealMode::wire_sink_and_actuator() {
  dev_sink.dev = rtl_device;
  dev_sink.ready = &device_ready;
  dev_sink.gate = &tx_gate;
  dev_sink.gate_waiting = &retune_waiting;
  dev_sink.quiet = &radio_quiet;
  dev_sink.timer = &send_timer;
  if (cfg.radio.tx_threads > 1) dev_sink.pool = &tx_pool;
  actuator.tx = &tx;
  actuator.sink = &dev_sink;
  actuator.shared_op = &shared_op;
  actuator.rate_change = &fps_rate_change;
  actuator.dev = rtl_device;
  actuator.dry_run = false;
  actuator.ampdu = cfg.ampdu;
  actuator.tx_gate = &tx_gate;
  actuator.retune_waiting = &retune_waiting;
  actuator.video_bw = &video_bw;
  actuator.retune_seq = &retune_seq;
  actuator.cur = start_ch;
  actuator.control_rt.ldpc = &live_ldpc;
  actuator.control_rt.gen = &radiotap_gen;
  // Encoder starts at the "normal" ROI QP (RcAgent only calls set_roi_qp on
  // a low<->normal transition — see run_bitrate_policy's roi_low_ default),
  // so the telemetry collector needs this seeded to reflect what's actually
  // commanded before the first transition ever happens.
  actuator.last_roi_qp = cfg.encoder.roi_qp_normal;
  actuator.recorder = &vtx_rec;
  return actuator;
}

ControlRadiotap RealMode::make_control_radiotap() {
  ControlRadiotap rt;
  rt.ldpc = &live_ldpc;
  rt.gen = &radiotap_gen;
  return rt;
}

ChannelPlans RealMode::plans_for(const RadioCfg& r, double margin_db) {
  std::lock_guard<std::mutex> l(wall_cache_mu);
  return make_channel_plans(wall_cache, r, margin_db, live_ldpc.load(std::memory_order_acquire));
}

#ifdef MABUR_HAVE_VENC
void RealMode::record_configure(const Config& c) {
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
}
#endif

ConfigApplyHooks RealMode::make_apply_hooks() {
  ConfigApplyHooks apply_hooks;
  apply_hooks.set_knobs = [this](const Config& c) {
    agent.set_live_knobs(c.encoder.bitrate_max_kbps, c.encoder.airtime_budget,
                         c.low_power.enable, static_cast<int>(c.venc.core.fps));
  };
  apply_hooks.power_plan = [this](const Config& c, std::string* err) {
    // The applier refuses a set during calibration already; the sweep owns
    // the TX-power table, so this never reprograms under one either.
    if (cal_active.load()) { *err = "calibration in progress"; return false; }
    if (!rtl_device || !device_ready.load(std::memory_order_acquire)) {
      *err = "radio not up";
      return false;
    }
    if (c.radio.power_mode != "offset") { *err = "no offset power plan"; return false; }
    // Every plan moves with the margin; the link channel's for the rung's
    // width is written now (under tx_gate, inside the table's lock), the
    // others when the rung or a retune next reaches them.
    const auto op = shared_op.load();
    bool refused_cal = false;
    if (!power_table.replace_unless_cal(plans_for(c.radio, c.radio.wall_margin_db),
                                        op ? op->ladder[1].bw : c.radio.width, &refused_cal)) {
      *err = refused_cal ? "calibration in progress" : "SetTxPowerRateDiffs failed";
      return false;
    }
    live_wall_margin.store(c.radio.wall_margin_db);
    return true;
  };
  apply_hooks.ldpc = [this](bool ldpc) {
    live_ldpc.store(ldpc, std::memory_order_release);
    radiotap_gen.fetch_add(1, std::memory_order_release);
    agent.note_ldpc_changed(ldpc);
    // The walls are derived from the stored curves under the LDPC flag (an
    // LDPC-off link decodes 4 dB worse), so every plan is rebuilt now. A
    // run that owns the table refuses it; its end rebuilds them anyway.
    if (!rtl_device || !device_ready.load(std::memory_order_acquire)) return;
    const auto op = shared_op.load();
    bool refused_cal = false;
    if (!power_table.replace_unless_cal(plans_for(cfg.radio, live_wall_margin.load()),
                                        op ? op->ladder[1].bw : cfg.radio.width, &refused_cal) &&
        !refused_cal)
      std::fprintf(stderr, "maburd radio: SetTxPowerRateDiffs failed after the ldpc change\n");
  };
  apply_hooks.msp = [this](bool enable, const Config& c) {
    if (enable) start_msp(c.msp);
    else stop_msp();
  };
  apply_hooks.recorder_stop = [this] {
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
  apply_hooks.record_configure = [this](const Config& c) {
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
  apply_hooks.pipeline_cold_restart = [this](const Config& c) {
#ifdef MABUR_HAVE_VENC
    venc_core_stop();
    return venc_core_start(&c.venc.core, &vcb) == 0;
#else
    (void)c;
    return false;
#endif
  };
  apply_hooks.after_rebuild = [this](const Config& c) {
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
  apply_hooks.calibration_active = [this] { return cal_active.load(); };
  return apply_hooks;
}

int run_real_mode(const Config& cfg, uint8_t start_ch, const std::string& cfg_path) {
  // The update trial's window counts from here.
  const uint64_t process_start_ms = now_steady_ms();
  // Set by the update loop when this process is a trial of a new binary.
  const char* trial_env = std::getenv("MABUR_TRIAL");
  const bool ota_trial = trial_env && std::strcmp(trial_env, "1") == 0;
  const std::string ota_dir = OtaEnv{}.dir;
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
  // MAC carrier sense follows radio.cca. On (the default, the chip default
  // too: devourer leaves 0x520[14]/[15] alone and lets phydm track the EDCCA
  // thresholds), the transmitter defers to any decodable 802.11 preamble.
  // The argument for off is that the downlink owns its channel and CSMA only
  // stutters it (devourer measured injection deferring 41-45% to a
  // co-channel 802.11 transmitter, tests/dis_cca_tx_onair.sh). The argument
  // for on: with both ends blind, the GS's own uplink send kills ~1.5 drone
  // PPDUs/s on BOTH ground cards (2-3 % of the enh layer repaired from
  // parity, +2 ms fec p99), which carrier sense on both ends removes
  // outright -- 0 repairs, RCF delivery 90 -> 97 % -- and every recorded
  // session on the DVR showed a 0/s foreign-preamble channel, so the
  // deferral being avoided was never there (docs/tx-rx-timing.md §3,
  // docs/findings/2026-09-23-cca-on.md). The altitude view the ground cards
  // cannot see rides Telem rx_foreign/rx_crcfail; both of the hop verdict's
  // routes to interfered (sustained NHM occupancy, and a lower NHM floor
  // confirmed by loss) read the ground cards' airtime and never react to
  // this counter, so if a flight shows the drone deferring to neighbours,
  // this is the only place it shows up. With the key off and the GS's
  // carrier sense on, the GS defers to the drone's bursts but the drone no
  // longer defers to a GS frame already on air: the GS's RcfSlotter is then
  // the only thing keeping uplink sends out of the bursts.
  dev_cfg.tuning.disable_cca = !cfg.radio.cca;
  // devourer's wide-TX IGI floor: a low start, raised only when sends stall.
  // While the radio is tuned 40 MHz wide (from the session's first such
  // frame) the floor is the DIG lower bound, because on a quiet channel the
  // receiver can park where the secondary 20 MHz never reads idle and the
  // MAC holds the frame for good. Every IGI step above the DIG window costs
  // the RCF uplink idle-channel sensitivity (devourer estimates ~1 dB a step),
  // so each image starts low -- 0x22 on the 8812EU, 0x26 on the 8812CU, the
  // die the stall was found on -- and the IGI floor watchdog (started before
  // the RX loop) climbs toward the 0x35 cap only while 40 MHz video sends
  // stall, keeping the first level that clears them. The shipped ladder airs
  // every rung at 20 MHz, so the floor only matters on a 40 MHz rung. The
  // start is per image, not per card: each drone build compiles exactly one
  // Jaguar3 die (tools/build-arm.sh and firmware/package/mabur), and a binary
  // refuses the other die at CreateRadio. MABUR_IGI_FLOOR_BASE and
  // MABUR_IGI_FLOOR_CAP override the start and the cap for a bench run.
#if defined(DEVOURER_HAVE_JAGUAR3_8822E) && !defined(DEVOURER_HAVE_JAGUAR3_8822C)
  constexpr uint8_t kIgiFloorImageBase = 0x22;   // 8812EU image
#else
  constexpr uint8_t kIgiFloorImageBase = 0x26;   // 8812CU image
#endif
  const char* igi_base_env = std::getenv("MABUR_IGI_FLOOR_BASE");
  const char* igi_cap_env = std::getenv("MABUR_IGI_FLOOR_CAP");
  const mabur::IgiFloorParams igi_params =
      mabur::igi_floor_params(kIgiFloorImageBase, igi_base_env, igi_cap_env);
  dev_cfg.tuning.wide_tx_igi_floor = igi_params.base;
  std::fprintf(stderr, "maburd radio: IGI floor base 0x%02x cap 0x%02x%s\n", igi_params.base,
               igi_params.cap, igi_base_env || igi_cap_env ? " (env)" : "");
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

  // The threads' shared state (real_mode.h). Its constructor wires the sink,
  // the actuator and the agent as this function always has.
  RealMode m(cfg, start_ch, cfg_path, rtl_device.get(), igi_params);
  m.process_start_ms = process_start_ms;
  m.ota_trial = ota_trial;
  m.ota_dir = ota_dir;

#ifdef MABUR_HAVE_VENC
  // Boot the encoder BEFORE the radio and before any thread starts: it
  // creates the frame-shm ring the hot thread reads, and RcAgent's very
  // first tick (BOOT -> MAX_RANGE) commands a bitrate through the venc
  // verbs, which no-op until the core is up. `m.agent` is constructed with
  // RealMode and outlives every venc thread (venc_core_stop() joins them
  // before this scope ends), so handing its address to the callbacks is
  // safe without a file-scope indirection.
  m.vcb.on_chain_break = [](void* u) {
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
  m.vcb.on_fault = [](void*, const char* what) {
    std::fprintf(stderr, "venc FATAL: %s — exiting for wrapper respawn\n", what);
    std::fflush(stderr);
    _exit(3);
  };
  m.vcb.user = &m.agent;
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
  m.record_configure(cfg);
  if (venc_core_start(&cfg.venc.core, &m.vcb) != 0) {
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
  m.vtx_rec.start_thread();
  // After venc_core_start: RcAgent's first tick (agent_thread.cpp) already commands a
  // bitrate through the verbs, so the ring/stats the debug endpoint reads
  // are live from here on. localhost-only, always on -- bind failure logs
  // and disables itself, never fatal (see debug_http.h).
  debug_http_start(cfg.venc.debug_port, cfg.venc.core.snapshot_quality,
                   &m.ov_override);

  std::fprintf(stderr, "maburd: nack retx ring %zu slots x %zu B = %zu B (ring_ms %d at %d kbps)\n",
               m.retx.slots(), m.retx_env_len, m.retx.slots() * m.retx_env_len, cfg.nack.ring_ms,
               cfg.encoder.bitrate_max_kbps);

  auto rx_callback = [&m](const Packet& pkt) { rx_on_packet(m, pkt); };

  if (cfg.msp.enable) m.start_msp(cfg.msp);

  std::thread mav_thread([&m] { mav_loop(m); });

  install_nack_hook(m);

  std::thread hot_thread([&m] { hot_loop(m); });

  std::thread tx_thread([&m] { tx_writer_loop(m); });

  std::thread agent_thread([&m] { agent_loop(m); });

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
    m.radio_chip = ac.chip_name ? ac.chip_name : "";
    m.radio_chip_ready.store(true, std::memory_order_release);
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
  // (restore_operating_power(), in tx_writer.cpp) and hands off to the next process
  // with the override still live -- reverting it here, unconditionally,
  // before the power plan is applied, means a fresh process never depends
  // on how the previous one died. -1 reverts to the calibrated table
  // (devourer/src/TxPower.h); a bring-up with no history at all just
  // clears a knob that was already clear.
  rtl_device->SetTxPowerIndexOverride(-1);

  // power_mode == "offset": program the start channel and width's
  // wall-equalized per-rate diff table from the wall cache and zero the
  // global offset; from here the table follows the rung's width
  // (RealActuator::apply_op) and every retune (RealActuator::retune_now_),
  // and nothing else moves power. "none", or nothing measured on the start
  // channel, flies the chip's own table. SetTxPowerRateDiffs returns false
  // on chips without a rate-diff table (both Jaguar3 dies, 8822E and 8822C,
  // have one) -- warn and continue rather than aborting bring-up, so
  // "offset" configured on an unsupported chip degrades to the chip's
  // default power instead of failing to fly. A missing or unreadable cache
  // is "nothing measured", never a bring-up failure.
  {
    std::string note;
    WallCache c = load_wall_cache(kWallCacheFile, &note);
    if (note.empty())
      std::fprintf(stderr, "maburd radio: wall cache %s: %zu measured (channel, width)\n",
                   kWallCacheFile, c.entries.size());
    else
      std::fprintf(stderr, "maburd radio: wall cache %s: %s\n", kWallCacheFile, note.c_str());
    std::lock_guard<std::mutex> l(m.wall_cache_mu);
    m.wall_cache = std::move(c);
  }
  m.power_table.set_channel(m.start_ch);
  if (!m.power_table.replace(m.plans_for(cfg.radio, cfg.radio.wall_margin_db), cfg.radio.width,
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

  m.device_ready.store(true, std::memory_order_release);

  std::thread igi_wd_thread;
  if (igi_params.cap > igi_params.base) igi_wd_thread = std::thread([&m] { igi_wd_loop(m); });

  std::fprintf(stderr, "maburd entering RX loop on channel %d\n", start_ch);
  rtl_device->StartRxLoop(rx_callback);

  // Init() returns once g_devourer_should_stop is set (SIGINT/SIGTERM) or the
  // device errors out internally. Ensure the flag is set on the error-out
  // path too, so hot_thread/agent_thread (which key off the same flag) are
  // guaranteed to exit and these joins complete.
  g_devourer_should_stop = true;
  // First: an apply in flight finishes here, and none starts after, so no
  // hook (a pipeline restart, start_msp) runs against the teardown below.
  m.applier.shutdown();
  if (hot_thread.joinable()) hot_thread.join();
  if (tx_thread.joinable()) tx_thread.join();
  if (agent_thread.joinable()) agent_thread.join();
  if (igi_wd_thread.joinable()) igi_wd_thread.join();
  m.stop_msp();
  if (mav_thread.joinable()) mav_thread.join();
  m.tx_pool.stop();  // drain + join senders before device teardown

  m.vtx_rec.shutdown();   // close the file before the record channel is torn down
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

}  // namespace mabur::maburd
