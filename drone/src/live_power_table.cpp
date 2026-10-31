#include "live_power_table.h"

#include <cstdio>
#include <utility>

namespace mabur {

void LivePowerTable::begin_cal() {
  std::lock_guard<std::mutex> l(mu_);
  cal_active->store(true, std::memory_order_release);
  cal_epoch->fetch_add(1, std::memory_order_release);
  dirty_ = true;
}

void LivePowerTable::follow(uint8_t bw) {
  std::lock_guard<std::mutex> l(mu_);
  if (cal_active->load(std::memory_order_relaxed)) return;
  if (!ready->load(std::memory_order_acquire)) return;
  if (!dirty_ && bw_ == bw) return;
  (void)write_locked(bw, Write{});
}

bool LivePowerTable::replace(WidthPlans p, uint8_t bw, bool zero_offset, bool clear_override) {
  std::lock_guard<std::mutex> l(mu_);
  plans_ = std::move(p);
  Write w;
  w.zero_offset = zero_offset;
  w.clear_override = clear_override;
  return write_locked(bw, w);
}

bool LivePowerTable::replace_unless_cal(WidthPlans p, uint8_t bw, bool* refused_cal) {
  std::lock_guard<std::mutex> l(mu_);
  *refused_cal = cal_active->load(std::memory_order_relaxed);
  if (*refused_cal) return false;
  plans_ = std::move(p);
  return write_locked(bw, Write{});
}

void LivePowerTable::set_plans(WidthPlans p) {
  std::lock_guard<std::mutex> l(mu_);
  plans_ = std::move(p);
}

bool LivePowerTable::program_for_cal(uint8_t bw) {
  std::lock_guard<std::mutex> l(mu_);
  Write w;
  w.for_cal = true;
  return write_locked(bw, w);
}

bool LivePowerTable::write_locked(uint8_t bw, const Write& w) {
  const PowerPlan* p = plan_for(plans_, bw);
  bool ok = true;
  dev->gated([&] {
    if (p) {
      devourer::TxRateDiffsQdb diffs;
      diffs.cck = p->cck;
      diffs.legacy = p->legacy;
      for (int i = 0; i < 8; ++i) diffs.mcs[i] = p->mcs[i];
      ok = dev->set_rate_diffs(diffs);
      // devourer documents the offset as sticky across retunes: zeroed
      // explicitly rather than trusting whatever a prior process or
      // bench tool left in the chip.
      if (w.zero_offset) dev->set_offset_qdb(0);
    } else if (dirty_ || w.for_cal) {
      // No plan: back to the untrimmed efuse table calibration zeroed.
      ok = dev->set_rate_diffs(std::nullopt);
    }
    if (w.clear_override) dev->set_index_override(-1);
  });
  if (p) {
    // plan_for falls back to the other width's table when bw has none.
    const bool own = (bw == 40 ? plans_.p40 : plans_.p20).has_value();
    std::fprintf(stderr, "maburd radio: power table %d MHz live (mcs5 diff %d)%s%s%s\n", bw,
                 p->mcs[5], own ? "" : (bw == 40 ? " (20 MHz table)" : " (40 MHz table)"),
                 w.for_cal ? " for calibration" : "",
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

}  // namespace mabur
