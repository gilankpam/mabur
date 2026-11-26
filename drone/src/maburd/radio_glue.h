#pragma once
// Radio-side glue for maburd: the devourer FrameSink, 802.11 header builder,
// control radiotap and the gated power-table device.
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <thread>
#include <vector>

#include <libusb.h>

#include "config.h"
#include "control_frame.h"
#include "control_tx_mode.h"
#include "frame_pipeline.h"
#include "live_power_table.h"
#include "radio_tx.h"
#include "send_timer.h"
#include "threads.h"
#include "usb_tx_pool.h"
#include "mabur/msp_source.h"

#include "IRtlRadio.h"
#include "RadiotapBuilder.h"
#include "TxMode.h"

namespace mabur::maburd {

using namespace mabur;

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
  // Times the direct-path video sends for the IGI floor watchdog (null in
  // dry-run). Control sends are not timed.
  mabur::SendTimer* timer = nullptr;

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
    const int slot = timer ? timer->begin(now_steady_us()) : -1;
    const size_t sent = dev->send_packets(v.data(), n);
    if (timer) timer->end(slot, now_steady_us());
    return sent;
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

// LivePowerTable's device (live_power_table.h): the radio, every call made
// inside the retune gate -- await_retune_gate, then tx_gate shared.
struct RadioPowerTableDevice : PowerTableDevice {
  IRtlRadio* dev = nullptr;
  std::shared_mutex* gate = nullptr;
  std::atomic<bool>* gate_waiting = nullptr;

  void gated(const std::function<void()>& f) override {
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg;
    if (gate) sg = std::shared_lock<std::shared_mutex>(*gate);
    f();
  }
  bool set_rate_diffs(const std::optional<devourer::TxRateDiffsQdb>& diffs) override {
    return dev->SetTxPowerRateDiffs(diffs);
  }
  void set_offset_qdb(int qdb) override { dev->SetTxPowerOffsetQdb(qdb); }
  void set_index_override(int idx) override { dev->SetTxPowerIndexOverride(idx); }
};

uint16_t open_usb_and_get_pid(uint16_t vid, uint16_t configured_pid,
                              libusb_context* ctx, libusb_device_handle** out_handle);

// A GS at a different RC_VERSION is refused by rc::frame_type(), so its RCFs
// never reach the agent -- and, because the uplink RSSI/SNR EMAs are fed
// inside that same accepted-frame branch, drone.uplink.snr_* goes stale too.
// From the drone's side that is indistinguishable from "no GS is talking",
// and from the GS's side it looks like the stale-caps restart deadlock, which
// sends the operator to `restart maburd` -- which cannot help. So say it out
// loud, but rarely: a mismatched peer transmits continuously and /tmp is
// tmpfs, hence the once-per-5 s gate.
// Not thread-safe; only rx_callback (the RX thread) calls it.
void log_foreign_rc_version(uint8_t peer_ver);

MspSourceCfg to_msp_source_cfg(const MspCfg& m);

}  // namespace mabur::maburd
