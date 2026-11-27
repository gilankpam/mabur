#pragma once
// The chip's TX-power table: which (channel, width)'s wall-equalized
// rate-diff plan (power_tables.h) is live. Shared by every thread that
// programs it -- bring-up, the agent thread on a rung width change
// (RealActuator::apply_op) and on a retune (RealActuator::retune_now_), the
// config-apply thread on a RADIO_WALL_MARGN or RADIO_LDPC set and the TX
// writer thread for calibration. `mu_` serialises the choice of plan with
// its device write, so two writers can never leave the chip on one plan's
// diffs while ch_/bw_ name another. Lock order is always mu_, then the
// retune gate: every device write runs inside PowerTableDevice::gated (it
// must not overlap a retune's FastRetune), always inside mu_; a retune
// takes mu_ first (with_lock) and the gate exclusive inside it, and
// programs the new channel's plan through retuned_in_gate, which never
// takes the gate again.
//
// Calibration owns the table while cal_active: follow() is a no-op then,
// checked under mu_, and begin_cal() raises cal_active under the same lock
// before the TX writer's first device write of a sweep. So a follow() that
// read cal_active clear has finished its write before the sweep zeroes the
// diffs, and none starts after.
//
// No plan for the live (channel, width) -- power_mode "none", or nothing
// measured there -- means the chip's own efuse table: written only to put
// it back after a plan or calibration changed it.

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <set>
#include <utility>

#include "TxPower.h"  // devourer::TxRateDiffsQdb
#include "power_tables.h"

namespace mabur {

// The device calls LivePowerTable makes: maburd's radio behind the retune
// gate, or a test's fake.
struct PowerTableDevice {
  virtual ~PowerTableDevice() = default;
  // Runs `f` with the retune gate held shared, so the device calls inside
  // it never overlap a retune's FastRetune.
  virtual void gated(const std::function<void()>& f) = 0;
  virtual bool set_rate_diffs(const std::optional<devourer::TxRateDiffsQdb>& diffs) = 0;
  virtual void set_offset_qdb(int qdb) = 0;
  virtual void set_index_override(int idx) = 0;
};

struct LivePowerTable {
  PowerTableDevice* dev = nullptr;
  std::atomic<bool>* ready = nullptr;  // device_ready: no write before InitWrite is done
  std::atomic<bool>* cal_active = nullptr;
  // Bumped by every begin_cal(), after cal_active: the agent thread treats a
  // new epoch seen with cal_active clear as a calibration falling edge, even
  // one too brief for it to have seen cal_active up (CalEndEdge).
  std::atomic<uint32_t>* cal_epoch = nullptr;

  // TX writer, before a sweep's first device write: see the file comment.
  // The chip's diffs stop being the live plan's until a follow()/replace().
  void begin_cal();

  // Agent thread: makes the link channel's `bw` plan the live one unless it
  // already is.
  void follow(uint8_t bw);

  // Bring-up, before the first replace(): the channel the radio starts on.
  void set_channel(uint8_t ch);
  // The link's channel: the one the last retune went to. Lock-free, so a
  // caller holding the table's lock or the retune gate may read it.
  uint8_t channel() const { return ch_.load(std::memory_order_acquire); }
  // The agent's retune, inside with_lock() while it holds the retune gate
  // exclusive, after ReApplyTxPower: records the channel and programs its
  // plan for the current width directly, never through
  // PowerTableDevice::gated (the caller already holds the gate). Only
  // records while calibration owns the table, whose end programs the
  // recorded channel, and before the first replace(), which does.
  void retuned_in_gate(uint8_t ch);

  // New plans, the link channel's `bw` plan programmed at once (callers
  // pass the rung's width as they last saw it; a width change racing this
  // is put right by the agent's next follow(), which sees bw_ disagree). zero_offset also zeroes
  // the global offset beside a plan; clear_override then clears the flat
  // TXAGC override, in the same gate hold. False when the device refused
  // the plan.
  bool replace(ChannelPlans p, uint8_t bw, bool zero_offset, bool clear_override);

  // RADIO_WALL_MARGN's and RADIO_LDPC's reprogram: replace(), refused under mu_ while
  // calibration owns the table (*refused_cal set), so a run starting
  // between the caller's own cal_active check and here is never written
  // over. False on that refusal or when the device refused the plan.
  bool replace_unless_cal(ChannelPlans p, uint8_t bw, bool* refused_cal);

  // The A-MPDU mode shares this lock: the agent's production write runs
  // only while calibration does not own the radio, checked under mu_
  // (unless_cal), and calibration's own writes run under it (with_lock),
  // so a stale production write can never land after begin_cal() and the
  // sweep's aggregation-off write that follows it.
  template <class F>
  void unless_cal(F&& f) {
    std::lock_guard<std::mutex> l(mu_);
    if (cal_active->load(std::memory_order_relaxed)) return;
    f();
  }
  template <class F>
  void with_lock(F&& f) {
    std::lock_guard<std::mutex> l(mu_);
    f();
  }

  // A calibration run's new plans (a wall measurement stored, an apply):
  // what a following capacity sweep and the run's restore program, not
  // programmed now -- calibration owns the table.
  void set_plans(ChannelPlans p);

  // Capacity sweep (TX writer, cal_active): the link channel's `bw` plan,
  // or the chip's own table with no plan -- what production flies there.
  bool program_for_cal(uint8_t bw);

 private:
  struct Write {
    bool zero_offset = false;
    bool clear_override = false;
    bool for_cal = false;  // calibration keeps owning the table afterwards
  };

  // `gated` false: the caller already holds the retune gate.
  bool write_locked(uint8_t bw, const Write& w, bool gated = true);

  std::mutex mu_;
  ChannelPlans plans_;            // empty when power_mode != "offset"
  std::atomic<uint8_t> ch_{0};    // the link's channel; written under mu_
  uint8_t bw_ = 0;                // the rung width last programmed, 0 = nothing yet
  bool dirty_ = false;            // calibration changed the chip's diffs
  bool holds_plan_ = false;       // the chip's diffs are a plan, not its own table
  std::set<std::pair<uint8_t, uint8_t>> own_logged_;  // (ch, bw) "chip's own table" logged
};

// The agent thread's calibration falling edge: the moment it must put back
// the ladder, the A-MPDU mode and the power table a sweep took over. Fed
// once per loop with cal_epoch read BEFORE cal_active (begin_cal raises
// cal_active first, so a new epoch is seen with cal_active already up --
// unless calibration has since let go, which is the edge). The epoch
// catches an up-and-down too short for two polls to straddle (a sweep
// refused, or a command the engine ignored): the device writes made for it
// still need undoing.
class CalEndEdge {
 public:
  // True once per falling edge.
  bool poll(uint32_t epoch_now, bool active_now) {
    const bool ended = !active_now && (was_active_ || epoch_now != epoch_seen_);
    was_active_ = active_now;
    if (!active_now) epoch_seen_ = epoch_now;
    return ended;
  }

 private:
  bool was_active_ = false;
  uint32_t epoch_seen_ = 0;
};

}  // namespace mabur
