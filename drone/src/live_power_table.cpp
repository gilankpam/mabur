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

void LivePowerTable::set_channel(uint8_t ch) {
  std::lock_guard<std::mutex> l(mu_);
  ch_.store(ch, std::memory_order_release);
}

void LivePowerTable::retuned_in_gate(uint8_t ch) {
  // mu_ is the caller's (with_lock).
  ch_.store(ch, std::memory_order_release);
  if (bw_ == 0 || cal_active->load(std::memory_order_relaxed)) return;  // nothing programmed yet
  (void)write_locked(bw_, Write{}, /*gated=*/false);
}

bool LivePowerTable::replace(ChannelPlans p, uint8_t bw, bool zero_offset, bool clear_override) {
  std::lock_guard<std::mutex> l(mu_);
  plans_ = std::move(p);
  Write w;
  w.zero_offset = zero_offset;
  w.clear_override = clear_override;
  return write_locked(bw, w);
}

bool LivePowerTable::replace_unless_cal(ChannelPlans p, uint8_t bw, bool* refused_cal) {
  std::lock_guard<std::mutex> l(mu_);
  *refused_cal = cal_active->load(std::memory_order_relaxed);
  if (*refused_cal) return false;
  plans_ = std::move(p);
  return write_locked(bw, Write{});
}

void LivePowerTable::set_plans(ChannelPlans p) {
  std::lock_guard<std::mutex> l(mu_);
  plans_ = std::move(p);
}

bool LivePowerTable::program_for_cal(uint8_t bw) {
  std::lock_guard<std::mutex> l(mu_);
  Write w;
  w.for_cal = true;
  return write_locked(bw, w);
}

bool LivePowerTable::write_locked(uint8_t bw, const Write& w, bool gated) {
  const uint8_t ch = ch_.load(std::memory_order_relaxed);
  const PlanPick pick = plan_for(plans_, ch, bw);
  const PowerPlan* p = pick.plan;
  // No plan: back to the untrimmed efuse table, but only over a plan or
  // what calibration left -- a chip already on its own table is not
  // rewritten on every hop between unmeasured channels.
  const bool write_own = !p && (holds_plan_ || dirty_ || w.for_cal);
  bool ok = true;
  auto calls = [&] {
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
    } else if (write_own) {
      ok = dev->set_rate_diffs(std::nullopt);
    }
    if (w.clear_override) dev->set_index_override(-1);
  };
  if (p || write_own || w.clear_override) {
    if (gated)
      dev->gated(calls);
    else
      calls();
  }
  if (p || write_own) holds_plan_ = p != nullptr;
  const char* cal = w.for_cal ? " for calibration" : "";
  if (p) {
    std::fprintf(stderr, "maburd radio: power table ch %u %u MHz: %s (mcs5 diff %d)%s%s\n",
                 static_cast<unsigned>(ch), static_cast<unsigned>(bw),
                 pick.source == PlanSource::Own ? "measured plan"
                 : bw == 40                     ? "the 20 MHz plan"
                                                : "the 40 MHz plan",
                 p->mcs[5], cal,
                 ok ? "" : " -- SetTxPowerRateDiffs FAILED, chip keeps its previous table");
  } else if (own_logged_.insert({ch, bw}).second || !ok) {
    std::fprintf(stderr, "maburd radio: power table ch %u %u MHz: chip's own table (no plan)%s%s\n",
                 static_cast<unsigned>(ch), static_cast<unsigned>(bw), cal,
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
