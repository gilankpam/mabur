// Real mode: the MAV thread body (real_mode.h).
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

// MAVLink parameter component. Verifies each T_MAV off mav_queue
// (RcAgent::check_mav), feeds the payload to the component -- or, for a
// frame of calibration packets only, decodes them onto cal_queue for the
// TX writer thread -- and sends its
// queued replies back as tagged T_MAV_UP frames on the Telem path (control
// radiotap, the shared telem dot11 seq, the mutex-guarded dev_sink.send()),
// at most one frame per kMavUplinkMinMs while a session is published so the
// tunnel never crowds Telem or the OSD stream. Uplink needs a published
// session or an update-only pair (nobody is listening without one); with
// only the update pair there is no video to protect and no pacing. The
// 1 Hz HEARTBEAT needs a published session. Both follow Telem's
// suppression while a calibration sweep airs: replies wait queued.
// Frames go out tagged under the pair of the latest accepted inbound frame
// (the update pair after a kOkUpdate), so an update GS that is not the
// linked one still verifies its replies. Once the update server asks to
// exit, the thread stops maburd (as SIGTERM does) when its replies have
// gone, or after kOtaExitGraceMs regardless. RcQueue has no
// condvar, so this polls at 5 ms, like cal_queue's consumer.
void mav_loop(RealMode& m) {
  name_thread("mbr-mav");
  ApplyPort port;
  port.submit = [&m](size_t row, const mavparam::Value& want, std::string* why) {
#ifndef MABUR_HAVE_VENC
    // No encoder in this build: every rung of the rebuild ladder would
    // fail and end in the exit rung, so a rebuild row is refused here.
    if (apply_path_for(row) == ConfigApplyPath::Rebuild) {
      if (why) *why = "no encoder in this build";
      return false;
    }
#endif
    return m.applier.submit(row, want, why);
  };
  port.take_result = [&m](ConfigApplyResult* out) { return m.applier.take_result(out); };
  port.busy = [&m] { return m.applier.busy(); };
  OtaEnv ota_env;
  ota_env.dir = m.ota_dir;
  ota_env.ident = {build_release(), build_board(), ""};
  ota_env.trial = m.ota_trial;
  ota_env.start_ms = m.process_start_ms;
  OtaServer ota(std::move(ota_env));
  bool chip_set = false;
  MavComponent comp(m.cfg_path, std::move(port), [](const std::string& line) {
    std::fprintf(stderr, "%s\n", line.c_str());
  }, &ota);
  ControlRadiotap mav_radiotap = m.make_control_radiotap();
  std::fprintf(stderr, "maburd mav: component sys %u comp %u, config %s\n",
               static_cast<unsigned>(MavComponent::sys_id()),
               static_cast<unsigned>(MavComponent::comp_id()), m.cfg_path.c_str());
  uint64_t last_hb = 0, last_up = 0, bad_parse = 0, rejected = 0;
  // Whether the latest accepted inbound frame came under the update pair.
  bool reply_to_update = false;
  // A lost reply pair must not keep this process alive past the request.
  constexpr uint64_t kOtaExitGraceMs = 1000;
  std::optional<uint64_t> exit_seen_ms;
  std::vector<uint8_t> body;
  // A cal-only frame's packets go to the TX writer, never the component;
  // a frame mixing cal and other packets goes whole to the component,
  // which ignores the cal ids.
  auto push_cal = [&](const rc::Mav& mv, bool latched) {
    for (auto& msg : cal::decode_all(mv.payload.data(), mv.payload.size()))
      m.cal_queue.push(CalInbound{std::move(msg), latched});
  };
  while (!g_devourer_should_stop) {
    if (!chip_set && m.radio_chip_ready.load(std::memory_order_acquire)) {
      ota.set_chip(m.radio_chip);
      chip_set = true;
    }
    while (m.mav_queue.pop(body)) {
      rc::Mav mv;
      // While a calibration run is open, a frame tagged under the run's
      // pair is still accepted (kOkLatched) after the drone has dropped
      // the session -- every sweep outlasts the link failsafe. check_mav
      // admits only cal-only frames on that path.
      switch (m.agent.check_mav(body.data(), body.size(), &mv,
                              m.cal_active.load(std::memory_order_relaxed))) {
        case RcAgent::MavCheck::kOk:
          m.mav_rx_frames.fetch_add(1, std::memory_order_relaxed);
          reply_to_update = false;
          if (cal::all_packets_are_cal(mv.payload.data(), mv.payload.size()))
            push_cal(mv, /*latched=*/false);
          else
            comp.on_payload(mv.payload.data(), mv.payload.size(), now_steady_ms());
          break;
        case RcAgent::MavCheck::kOkUpdate:
          m.mav_rx_frames.fetch_add(1, std::memory_order_relaxed);
          reply_to_update = true;
          comp.on_payload(mv.payload.data(), mv.payload.size(), now_steady_ms());
          break;
        case RcAgent::MavCheck::kOkLatched:
          m.mav_rx_frames.fetch_add(1, std::memory_order_relaxed);
          push_cal(mv, /*latched=*/true);
          break;
        // The run's own trailing frames under its pair after it closed:
        // dropped, counted by the agent (cal_late=), not a reject.
        case RcAgent::MavCheck::kLateCal: break;
        // Not update-lane traffic under the update pair: counted by the
        // agent (update_dropped=), folded into drop below.
        case RcAgent::MavCheck::kUpdateDropped: break;
        case RcAgent::MavCheck::kMalformed: ++bad_parse; break;
        case RcAgent::MavCheck::kRejected: ++rejected; break;
      }
    }
    // Every pass, linked or not: the applier stays busy (refusing the
    // next set) until its result is taken.
    comp.poll_results();
    const uint64_t now = now_steady_ms();
    const bool linked = m.agent.has_published_session();
    if (linked && !m.cal_sweeping.load(std::memory_order_relaxed) && now - last_hb >= 1000) {
      comp.queue_heartbeat();
      last_hb = now;
    }
    // Replies wait queued without a pair.
    std::optional<rc::TagCtx> ctx = reply_to_update ? m.agent.update_ctx() : std::nullopt;
    if (!ctx) ctx = m.agent.uplink_ctx(false);
    const uint64_t pace_ms = linked ? kMavUplinkMinMs : 0;
    if ((linked || m.agent.update_session_open()) && ctx &&
        !m.cal_sweeping.load(std::memory_order_relaxed) && comp.outbound_packets() > 0 &&
        now - last_up >= pace_ms) {
      rc::Mav mv;
      mv.type = rc::T_MAV_UP;
      mv.counter = m.mav_up_counter.fetch_add(1, std::memory_order_relaxed);
      const int packets = comp.take_outbound(&mv.payload);
      ctx->seq32 = mv.counter;
      const auto mav = rc::pack_mav(mv, m.cfg.link.key, *ctx);
      const std::vector<uint8_t>& telem_radiotap = mav_radiotap.get();
      std::vector<uint8_t> frame;
      frame.reserve(telem_radiotap.size() + kDot11HeaderLen + mav.size());
      frame.insert(frame.end(), telem_radiotap.begin(), telem_radiotap.end());
      auto hdr = build_dot11_header(m.telem_dot11_seq.fetch_add(1, std::memory_order_relaxed) & 0xFFF);
      frame.insert(frame.end(), hdr.begin(), hdr.end());
      frame.insert(frame.end(), mav.begin(), mav.end());
      m.dev_sink.send(frame.data(), frame.size());
      m.mav_tx_frames.fetch_add(1, std::memory_order_relaxed);
      m.mav_tx_packets.fetch_add(static_cast<uint64_t>(packets), std::memory_order_relaxed);
      last_up = now;
    }
    if (comp.exit_requested()) {
      if (!exit_seen_ms) exit_seen_ms = now;
      if (comp.outbound_packets() == 0 || now - *exit_seen_ms >= kOtaExitGraceMs) {
        std::fprintf(stderr, "maburd: update accepted, exiting for the trial\n");
        g_devourer_should_stop = true;
      }
    }
    const MavComponentStats& cs = comp.stats();
    m.mav_rx_packets.store(cs.rx_packets, std::memory_order_relaxed);
    m.mav_drop.store(bad_parse + cs.drop_not_for_us + cs.drop_bad_packet + cs.drop_queue_full +
                       m.mav_up_nopair.load(std::memory_order_relaxed) + m.agent.update_dropped(),
                   std::memory_order_relaxed);
    m.mav_reject.store(rejected, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
}  // namespace mabur::maburd
