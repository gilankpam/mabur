// Real mode: the wall self-measurement on the radio (wall_ctl.h).

#include "wall_ctl.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>

#include "mabur/channel_set.h"
#include "real_mode.h"
#include "threads.h"
#include "wall_cache.h"
#include "wall_curve.h"

namespace mabur::maburd {

namespace {

const char* result_name(cal::WallResult r) {
  static const char* const kNames[] = {"Ok",           "Unparked", "TooFewPoints",
                                       "FloorTooHigh", "NotSaturated", "Deadline",
                                       "DeviceError",  "Aborted",  "Disturbed"};
  const auto i = static_cast<size_t>(r);
  return i < sizeof kNames / sizeof kNames[0] ? kNames[i] : "?";
}

// One exclusive hold of the retune gate, taken the way RealActuator's
// retune takes it: retune_waiting up first so no new shared-taker enters
// (glibc's rwlock prefers readers), and 5 ms before the first radio call so a
// frame the chip already accepted leaves on the state it was sent for.
// Callers hold the power table's lock first, as every table writer does.
template <class F>
void exclusive(RealMode& m, F&& f) {
  m.retune_waiting.store(true, std::memory_order_release);
  {
    std::unique_lock<std::shared_mutex> g(m.tx_gate);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    f();
  }
  m.retune_waiting.store(false, std::memory_order_release);
}

}  // namespace

bool DeviceCaptureSession::begin(uint8_t bw) {
  await_retune_gate(gate_waiting);
  std::shared_lock<std::shared_mutex> sg(*gate);
  const bool ok = dev->CrossPathBegin(bw == 40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20);
  // The EVM maths runs on this thread. Core 0 is saturated during a measurement
  // (the probe train, spawned above and so left on core 0, plus the rest of maburd);
  // the hot core idles while nothing airs. Back in end().
  if (two_core_target()) pin_self_to(kHotCore);
  return ok;
}

bool DeviceCaptureSession::park(bool parked) {
  await_retune_gate(gate_waiting);
  std::shared_lock<std::shared_mutex> sg(*gate);
  return dev->CrossPathPark(parked);
}

bool DeviceCaptureSession::capture(int tx_index, int gain, std::vector<devourer::IqSample>* out) {
  ++captures;
  await_retune_gate(gate_waiting);
  std::shared_lock<std::shared_mutex> sg(*gate);
  return dev->CrossPathCapture(tx_index, gain, *out);
}

void DeviceCaptureSession::end() {
  if (two_core_target()) pin_self_to(kRestCore);
  bool ok;
  {
    await_retune_gate(gate_waiting);
    std::shared_lock<std::shared_mutex> sg(*gate);
    ok = dev->CrossPathEnd();
  }
  if (!ok) {
    end_ok = false;
    std::fprintf(stderr,
                 "maburd cal: ERROR: CrossPathEnd failed to restore the radio (TX may be wedged); "
                 "this measurement is not trusted\n");
  }
}

DeviceWallCtl::DeviceWallCtl(RealMode& m) : m_(m) {
  sess_.dev = m.rtl_device;
  sess_.gate = &m.tx_gate;
  sess_.gate_waiting = &m.retune_waiting;
}

cal::AckResult DeviceWallCtl::check(uint8_t channel, uint8_t bw) {
  if (channel == 0) channel = m_.power_table.channel();  // 0 = the link's channel
  if (!m_.rtl_device->CrossPathSupported()) return cal::AckResult::Unsupported;
  // The cross-path capture is validated at 5 GHz only.
  if (channel <= 14 || !channel_set_member(m_.cfg.radio.channels, channel))
    return cal::AckResult::BadChannel;
  if (bw != 20 && bw != 40) return cal::AckResult::BadWidth;
  if (bw == 40 && m_.cfg.radio.width == 20) return cal::AckResult::BadWidth;
  return cal::AckResult::Accepted;
}

void DeviceWallCtl::start(uint8_t channel, uint8_t bw, uint64_t now_ms) {
  // 0 = the link's channel; the report and the cache carry the resolved one.
  if (channel == 0) channel = m_.power_table.channel();
  ch_ = channel;
  bw_ = bw;
  t0_ms_ = now_ms;
  anchor_ = -1;
  sess_.captures = 0;
  sess_.end_ok = true;
  sweep_.reset();
  open_ = true;
  // The session does not tune: at 40 MHz FastRetune keeps the link's
  // ht40_offset, and every member of radio.channels shares one at width 40
  // (channel_set_issue), so the measured channel lands on the standard pair
  // the link flies there.
  narrowed_ = bw == 20 && m_.cfg.radio.width == 40;
  IRtlRadio* dev = m_.rtl_device;
  m_.power_table.with_lock([&] {
    exclusive(m_, [&] {
      retuned_ = channel != m_.power_table.channel();
      // From here to the restore nothing else may send (the session moves
      // the TX queue boundaries): every DevourerSink send is dropped.
      m_.radio_quiet.store(true, std::memory_order_release);
      if (retuned_) dev->FastRetune(channel, /*cache_rf=*/false);
      if (narrowed_) dev->FastSetBandwidth(CHANNEL_WIDTH_20);
      // A flat override left parked by an earlier run reads back as the
      // anchor in place of the chip's reference; the session saves and
      // restores the override it finds, so it finds none.
      dev->SetTxPowerIndexOverride(-1);
      const bool zeroed = dev->SetTxPowerRateDiffs(devourer::TxRateDiffsQdb{});
      // FastRetune does not re-fold TX power onto the new channel's efuse
      // reference; this does.
      const bool applied = dev->ReApplyTxPower();
      const auto st = dev->GetTxPowerState();
      if (zeroed && applied && st.valid && st.mcs7_index >= 0) anchor_ = st.mcs7_index;
    });
  });
  if (anchor_ < 0) {
    std::fprintf(stderr,
                 "maburd cal: wall ch=%u bw=%u: rate diffs not zeroed or anchor unreadable\n",
                 static_cast<unsigned>(channel), static_cast<unsigned>(bw));
    return;
  }
  sweep_.emplace(sess_, bw, anchor_, now_ms);
}

bool DeviceWallCtl::step(uint64_t now_ms) { return sweep_ && sweep_->step(now_ms); }

void DeviceWallCtl::restore() {
  if (!open_) return;
  open_ = false;
  if (sweep_ && !sweep_->finished()) sweep_->cancel();  // ends the capture session
  IRtlRadio* dev = m_.rtl_device;
  m_.power_table.with_lock([&] {
    exclusive(m_, [&] {
      // Width first: widening on the measured channel keeps its offset, the
      // set's shared one, so the hop back lands on the link's pair.
      if (narrowed_) dev->FastSetBandwidth(CHANNEL_WIDTH_40);
      if (retuned_) dev->FastRetune(m_.power_table.channel(), /*cache_rf=*/false);
      if (!dev->ReApplyTxPower())
        std::fprintf(stderr, "maburd cal: wall restore: ReApplyTxPower failed\n");
      m_.radio_quiet.store(false, std::memory_order_release);
    });
  });
  if (!sess_.end_ok) untrusted_ = true;
}

void DeviceWallCtl::log_result(cal::WallResult r, const std::array<int8_t, 8>& w,
                               int16_t floor_x10, bool ldpc) const {
  const float rise = sweep_ ? sweep_->park_rise_db() : 0.0f;
  std::fprintf(stderr,
               "maburd cal: wall ch=%u bw=%u result=%s park_rise=%.1f floor=%.1f anchor=%d "
               "walls=%d,%d,%d,%d,%d,%d,%d,%d captures=%d time_ms=%llu%s%s\n",
               static_cast<unsigned>(ch_), static_cast<unsigned>(bw_), result_name(r), rise,
               floor_x10 / 10.0, anchor_, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
               sess_.captures, static_cast<unsigned long long>(now_steady_ms() - t0_ms_),
               ldpc ? "" : " (LDPC off: thresholds lowered 4 dB)",
               sweep_ && sweep_->curve().weak_park ? " park=weak (thresholds lowered 2.5 dB)" : "");
}

cal::SweepDone DeviceWallCtl::finish() {
  restore();
  cal::WallResult r = sweep_ ? sweep_->result() : cal::WallResult::DeviceError;
  if (!sess_.end_ok) r = cal::WallResult::DeviceError;  // a failed restore is not trusted

  cal::SweepDone d;
  d.channel = ch_;
  d.bw = bw_;
  d.anchor = static_cast<uint8_t>(anchor_ < 0 ? 0 : anchor_);
  const bool ldpc = m_.live_ldpc.load(std::memory_order_acquire);
  if (sweep_) {
    if (const auto f = curve_floor_db(sweep_->curve()))
      d.floor_evm_x10 = static_cast<int16_t>(std::lround(*f * 10));
  }
  if (r == cal::WallResult::Ok) {
    const WallCurve& curve = sweep_->curve();
    const auto walls = walls_from_curve(curve, ldpc);
    for (size_t i = 0; i < walls.size(); ++i) d.walls[i] = static_cast<int8_t>(walls[i]);
    WallCache snapshot;
    {
      std::lock_guard<std::mutex> l(m_.wall_cache_mu);
      m_.wall_cache.entries[{ch_, bw_}] = curve;
      snapshot = m_.wall_cache;
    }
    // A failed write leaves the entry live until restart; the result stays Ok.
    if (!save_wall_cache(kWallCacheFile, snapshot))
      std::fprintf(stderr, "maburd cal: wall cache write to %s failed\n", kWallCacheFile);
    // plans_for takes wall_cache_mu itself, so it runs after the lock above
    // is released, and before the table's own lock.
    const Config cfg = reload_config ? reload_config() : m_.cfg;
    m_.power_table.set_plans(m_.plans_for(cfg.radio, m_.live_wall_margin.load()));
  }
  d.wall_result = r;
  log_result(r, d.walls, d.floor_evm_x10, ldpc);
  return d;
}

void DeviceWallCtl::cancel() {
  if (!open_) return;
  restore();
  log_result(cal::WallResult::Aborted, {}, 0, m_.live_ldpc.load(std::memory_order_acquire));
}

}  // namespace mabur::maburd
