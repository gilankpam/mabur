#pragma once
// The chip's per-width TX-power table: which width's wall-equalized
// rate-diff plan (power_tables.h) is live. Shared by every thread that
// programs it -- bring-up, the agent thread on a rung width change
// (RealActuator::apply_op), the config-apply thread on a RADIO_WALL_MARGN
// set and the TX writer thread for calibration. `mu_` serialises the choice
// of plan with its device write, so two writers can never leave the chip on
// one width's diffs while bw_ names the other. Every device write runs
// inside PowerTableDevice::gated (it must not overlap a retune's
// FastRetune), always inside mu_, never the other way round.
//
// Calibration owns the table while cal_active: follow() is a no-op then,
// checked under mu_, and begin_cal() raises cal_active under the same lock
// before the TX writer's first device write of a sweep. So a follow() that
// read cal_active clear has finished its write before the sweep zeroes the
// diffs, and none starts after.
//
// No plans (power_mode "none") means the chip's own efuse table: never
// written, except to put it back after calibration zeroed it.

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

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

  // Agent thread: makes `bw`'s plan the live one unless it already is.
  void follow(uint8_t bw);

  // New plans, `bw`'s programmed at once (callers pass the rung's width as
  // they last saw it; a width change racing this is put right by the
  // agent's next follow(), which sees bw_ disagree). zero_offset also zeroes
  // the global offset beside a plan; clear_override then clears the flat
  // TXAGC override, in the same gate hold. False when the device refused
  // the plan.
  bool replace(WidthPlans p, uint8_t bw, bool zero_offset, bool clear_override);

  // RADIO_WALL_MARGN's reprogram: replace(), refused under mu_ while
  // calibration owns the table (*refused_cal set), so a run starting
  // between the caller's own cal_active check and here is never written
  // over. False on that refusal or when the device refused the plan.
  bool replace_unless_cal(WidthPlans p, uint8_t bw, bool* refused_cal);

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

  // Calibration's apply: the plans a following capacity sweep and the run's
  // restore program, not programmed now -- the verify sweep that follows a
  // power apply flies the zeroed diffs under the parked override.
  void set_plans(WidthPlans p);

  // Capacity sweep (TX writer, cal_active): `bw`'s plan, or the chip's own
  // table with no plan -- what production flies at that width.
  bool program_for_cal(uint8_t bw);

 private:
  struct Write {
    bool zero_offset = false;
    bool clear_override = false;
    bool for_cal = false;  // calibration keeps owning the table afterwards
  };

  bool write_locked(uint8_t bw, const Write& w);

  std::mutex mu_;
  WidthPlans plans_;    // empty when power_mode != "offset"
  uint8_t bw_ = 0;      // the rung width whose plan was last programmed, 0 = none yet
  bool dirty_ = false;  // the chip's diffs are not plan_for(plans_, bw_)
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
