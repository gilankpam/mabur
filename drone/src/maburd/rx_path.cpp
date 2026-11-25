// Real mode: the RX callback (real_mode.h).
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

// RX callback: pulls RC frames (rc::frame_type >= 0) off the air and
// queues them for the agent thread. Runs on the main thread (inside
// rtl_device->Init's blocking RX loop).
void rx_on_packet(RealMode& m, const Packet& pkt) {
  m.rx_beat.fetch_add(1, std::memory_order_relaxed);
  if (pkt.RxAtrib.crc_err) m.rx_crcfail_frames.fetch_add(1, std::memory_order_relaxed);
  if (pkt.Data.size() < kDot11HeaderLen + 4) return;
  const uint8_t* body = pkt.Data.data() + kDot11HeaderLen;
  size_t body_len = pkt.Data.size() - kDot11HeaderLen;
  const int rc_type = rc::frame_type(body, body_len);
  // A DISC of another version is still ours to answer (frozen type), and
  // a T_MAV of another version may be the update lane: check_mav decides.
  const int frozen = rc_type >= 0 ? rc_type : rc::frozen_type(body, body_len);
  if (rc_type >= 0 || frozen == rc::T_DISC || frozen == rc::T_MAV) {
    // Calibration rides T_MAV (the mabur dialect): the MAV thread verifies
    // it and hands its cal packets to the TX writer thread.
    if (rc_type == rc::T_NACK) {
      if (m.nack_hook) m.nack_hook(body, body_len);  // RX thread (fec-nack)
    } else if (frozen == rc::T_MAV) {
      m.mav_queue.push(body, body_len);
    } else {
      m.rc_queue.push(body, body_len);
    }
    // Uplink EMAs feed off CRC-clean RC frames only — a corrupt frame's
    // attrib (rssi/snr) is not a trustworthy sample.
    if (!pkt.RxAtrib.crc_err) {
      m.uplink_track.on_rc_frame(pkt.RxAtrib.rssi, pkt.RxAtrib.snr);
      m.rx_own_frames.fetch_add(1, std::memory_order_relaxed);
    }
  } else if (!pkt.RxAtrib.crc_err) {
    // Same crc gate as the EMAs, and for the same class of reason:
    // RC_MAGIC is two bytes, so ~1 in 65536 corrupt bodies matches it by
    // chance and must not print a version-mismatch scare, nor count as a
    // neighbour. Everything CRC-clean that is not ours is a foreign
    // 802.11 frame on our channel (Telem rx_foreign); a foreign RC
    // version is logged on top. The frame is still dropped as before.
    m.rx_foreign_frames.fetch_add(1, std::memory_order_relaxed);
    if (rc::is_foreign_rc_version(body, body_len))
      log_foreign_rc_version(body[2]);
  }
}

namespace {

// RX-thread-only state of the NACK answer, owned by the hook.
struct NackState {
  TokenBucket retx_bucket{64.0};  // re-sized from the op at every refill
  mabur::SbiPacker retx_packer;
  std::vector<uint8_t> retx_env;
  std::vector<std::vector<uint8_t>> retx_out;
  explicit NackState(RealMode& m)
      : retx_packer(static_cast<int>(m.retx_env_len), m.cfg.fec.blocks_per_body[0],
                    static_cast<uint8_t>(0 | mabur::kSbiRetxMark)),
        retx_env(m.retx_env_len) {}
};

}  // namespace

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
void install_nack_hook(RealMode& m) {
  auto st = std::make_shared<NackState>(m);
  m.nack_hook = [&m, st](const uint8_t* body, size_t len) {
    TokenBucket& retx_bucket = st->retx_bucket;
    mabur::SbiPacker& retx_packer = st->retx_packer;
    std::vector<uint8_t>& retx_env = st->retx_env;
    std::vector<std::vector<uint8_t>>& retx_out = st->retx_out;
    // A malformed frame (radio corruption, sid != 0) counts nack_bad only;
    // check_nack raises auth_reject itself for a tag/counter failure.
    rc::Nack nk;
    if (m.agent.check_nack(body, len, &nk) != RcAgent::NackCheck::kOk) {
      m.nack_bad.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const rc::Nack* n = &nk;
    m.nack_rx.fetch_add(1, std::memory_order_relaxed);
    const auto op = m.shared_op.load();
    const double sym_per_s =
        op ? delivered_mbps(op->ladder[0], m.cfg.air_clock) * 1e6 / 8.0 / static_cast<double>(m.retx_env_len)
           : 0.0;
    if (op) retx_bucket.set_depth(TokenBucket::depth_for(sym_per_s, m.cfg.nack.burst_ms));
    retx_bucket.refill(now_steady_us(), sym_per_s * m.cfg.nack.air_pct / 100.0);
    const bool twice = (n->flags & rc::kNackFlagRepeat) != 0;
    const double cost = twice ? 2.0 : 1.0;
    retx_out.clear();
    for (uint8_t i = 0; i < n->n; ++i) {
      for (int b = 0; b < 32; ++b) {
        if (!(n->e[i].bitmap & (1u << b))) continue;
        const uint32_t seq = n->e[i].first_seq + static_cast<uint32_t>(b);
        if (!m.retx.get(seq, retx_env.data())) { m.retx_miss.fetch_add(1, std::memory_order_relaxed); continue; }
        if (!retx_bucket.take(cost)) { m.retx_refused.fetch_add(1, std::memory_order_relaxed); continue; }
        m.retx_syms.fetch_add(1, std::memory_order_relaxed);
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
        m.txq.push_front(UepBody{0, *it, now_ms, p_us, false});
        m.air_clock.book(p_us, bytes, 0);
      }
  };
}
}  // namespace mabur::maburd
