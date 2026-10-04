#include "channel_scout.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mabur/channel_set.h"
#include "mabur/ht40.h"
#include "nhm_busy.h"
#include "pair_pick.h"

namespace maburgs {

namespace {
// The channels the scheduler dwells on, in dwell-set order (also the
// ranker's row order: the K log line and ranking() expose it). Pinned
// (search only): the members. Measuring at 40: both halves of every
// member's pair. Measuring at 20: the members.
std::vector<uint8_t> dwell_channels(const ScoutCfg& c) {
  if (c.measure && c.link_width_mhz == 40) return scan_half_set(c.channels);
  return c.channels;
}

devourer::chanmig::ScanPlanConfig plan_for(const ScoutCfg& c) {
  devourer::chanmig::ScanPlanConfig p;
  for (uint8_t ch : dwell_channels(c)) {
    bool dup = false;
    for (const auto& d : p.candidates) dup = dup || d.primary == ch;
    if (dup) continue;
    devourer::chanmig::ChannelDef d;
    d.band = ch >= 36 ? 5 : 2;
    d.primary = ch;
    d.width = CHANNEL_WIDTH_20;
    p.candidates.push_back(d);
  }
  p.dwell_ms = c.dwell_ms;
  p.settle_ms = c.settle_ms;
  // One flat cadence: every bin is equally due, so next() is plan-order
  // round-robin and rounds_complete() counts full passes.
  p.backup_revisit_ms = 0;
  p.bg_revisit_ms = 0;
  p.fail_retry_ms = 0;
  return p;
}
}  // namespace

ChannelScout::ChannelScout(ScoutCfg cfg, ScoutRadio& radio, NowFn now_ms, SleepFn sleep_ms)
    : cfg_(std::move(cfg)),
      radio_(radio),
      now_(std::move(now_ms)),
      sleep_(std::move(sleep_ms)),
      sched_(plan_for(cfg_)),
      ranker_(dwell_channels(cfg_), cfg_.min_rounds, cfg_.pick_margin, cfg_.blocked_pct),
      proposal_(cfg_.channels.empty() ? 0 : cfg_.channels.front()),
      op_(cfg_.channels.empty() ? 0 : cfg_.channels.front()) {
  prelude_done_.store(!(cfg_.one_card && cfg_.measure), std::memory_order_release);
  prelude_ack_.store(!(cfg_.one_card && cfg_.measure), std::memory_order_release);
}

void ChannelScout::ack_prelude(uint8_t op) {
  set_op(op);
  prelude_ack_.store(true, std::memory_order_release);
}

void ChannelScout::set_op(uint8_t ch) {
  op_.store(ch, std::memory_order_release);
  publish_();   // the margin is against op
}

bool ChannelScout::has_work_() const {
  return search_.load(std::memory_order_acquire) || measuring_();
}

bool ChannelScout::measuring_() const {
  return cfg_.measure && !frozen_.load(std::memory_order_acquire);
}

bool ChannelScout::member_(uint8_t ch) const {
  return mabur::channel_set_member(cfg_.channels, ch);
}

// One card: at the prelude deadline the pick may run on a single visit per
// half until min_rounds full passes exist.
int ChannelScout::eff_min_rounds_() const {
  return (cfg_.one_card && prelude_done_.load(std::memory_order_acquire) &&
          rounds_.load(std::memory_order_acquire) < static_cast<uint64_t>(cfg_.min_rounds))
             ? 1
             : cfg_.min_rounds;
}

void ChannelScout::park_() {
  const uint8_t op = op_.load(std::memory_order_acquire);
  if (cfg_.link_width_mhz == 40) {
    // Join the link at 40 (docs/bw40.md §3). A failure (dead or unready
    // card) is not retried here: the core loop's width resync
    // (width_resync.h) sets the card's width once it is ready again.
    if (!radio_.retune_width(op, 40))
      std::fprintf(stderr, "maburgs channel: scout card width switch to 40 MHz on ch %u "
                           "failed; the core loop retries once the card is ready\n",
                   static_cast<unsigned>(op));
  } else {
    radio_.retune(op);
  }
  beaconing_.store(false, std::memory_order_release);
  quiet_.store(false, std::memory_order_release);
  working_.store(false, std::memory_order_release);
  need_width_ = true;
}

bool ChannelScout::tune_(uint8_t ch) {
  const bool ok = need_width_ ? radio_.retune_width(ch, 20) : radio_.retune(ch);
  if (ok) need_width_ = false;
  return ok;
}

void ChannelScout::run() {
  while (!stop_.load(std::memory_order_acquire)) {
    if (!run_once()) sleep_(cfg_.beacon_period_ms);
  }
  if (working_.load(std::memory_order_acquire)) park_();
  done_.store(true, std::memory_order_release);
}

// Final review I1: while the search is off (linked, or inside
// search_after_ms after a loss) op's own pair carries the drone's video.
// At width 40 a 20 MHz observe cannot decode it (counted busy), and NHM
// counts its airtime either way, so a linked visit would score op against
// the link itself and the boot hop would leave a clean op almost every
// time. Those halves are skipped instead -- no retune, no observe -- and op
// keeps only its pre-link visits (mature() then does not wait for it, and
// the core freezes in place when op_ranked() is false).
bool ChannelScout::is_op_half_(uint8_t ch) const {
  const uint8_t op = op_.load(std::memory_order_acquire);
  if (ch == op) return true;
  return cfg_.measure && cfg_.link_width_mhz == 40 && ch == mabur::ht40_pair_other(op);
}

bool ChannelScout::skip_op_half_(uint8_t ch) const {
  return !search_.load(std::memory_order_acquire) && measuring_() && is_op_half_(ch);
}

// One scheduler dwell. burst_ok: a DISC burst is allowed on this dwell
// (still only when searching and on a member). A skipped op half is
// completed in the scheduler without touching the card (so the round-robin
// and rounds() advance) and the next bin is taken; false when every bin is
// one of op's.
bool ChannelScout::step_dwell_(bool burst_ok, bool observe) {
  const size_t bins = dwell_channels(cfg_).size();
  for (size_t i = 0; i <= bins; ++i) {
    auto p = sched_.next(now_());
    if (!p.valid) return true;
    if (skip_op_half_(p.bin_ch)) {
      // Stamped one dwell ahead: a skipped bin completed at `now` ties with
      // the dwell that just finished and, by plan order, could win the tie
      // forever (width 20, one other member).
      sched_.complete(p, now_() + std::max(cfg_.dwell_ms, 1), true);
      rounds_.store(sched_.rounds_complete(), std::memory_order_release);
      continue;
    }
    const bool burst = burst_ok && search_.load(std::memory_order_acquire) && member_(p.bin_ch);
    const bool ok = dwell(p.bin_ch, p.round, burst, observe);
    sched_.complete(p, now_(), ok);
    rounds_.store(sched_.rounds_complete(), std::memory_order_release);
    return true;
  }
  return false;
}

bool ChannelScout::run_once() {
  if (t0_ < 0) t0_ = now_();
  if (!has_work_()) {
    if (working_.load(std::memory_order_acquire)) park_();
    return false;
  }
  working_.store(true, std::memory_order_release);
  if (cfg_.one_card) {
    if (!prelude_done_.load(std::memory_order_acquire)) {
      if (measuring_() && now_() - t0_ < cfg_.one_card_ms)
        return step_dwell_(/*burst_ok=*/false, /*observe=*/true);   // silent prelude
      prelude_done_.store(true, std::memory_order_release);          // never re-arms
      publish_();
      return true;   // the core reads the ranking, commits, then ack_prelude()s
    }
    // Spec §5: the prelude ranking commits BEFORE the first DISC. Until the
    // core has committed the pick (ack_prelude(), which sets op), no op
    // window runs: a window on the old op would carry DISCs the drone may
    // ack there, and the ack_override would discard the pick. Not while
    // frozen: then there is no pick to wait for.
    if (measuring_() && !prelude_ack_.load(std::memory_order_acquire)) return false;
    // Op window: the core may beacon on op through this (only) card, then a
    // gap so the last DISC's ack lands before leaving.
    if (!tune_(op_.load(std::memory_order_acquire))) return true;
    beaconing_.store(true, std::memory_order_release);
    sleep_(cfg_.op_window_ms);
    beaconing_.store(false, std::memory_order_release);
    sleep_(cfg_.beacon_period_ms);
    if (!has_work_()) return true;
  }
  return step_dwell_(/*burst_ok=*/true, measuring_());
}

bool ChannelScout::dwell(uint8_t ch, uint64_t round, bool burst, bool observe) {
  ScoutDwell d;
  auto& s = d.survey;
  s.seq = seq_++;
  s.def.band = ch >= 36 ? 5 : 2;
  s.def.primary = ch;
  s.def.width = CHANNEL_WIDTH_20;
  s.round = round;
  s.t_start_ms = now_();
  s.settle_ms = cfg_.settle_ms;
  if (!tune_(ch)) {
    s.flags |= devourer::chanmig::kFlagRetuneFailed;
    s.t_end_ms = now_();
    std::lock_guard<std::mutex> lk(mu_);
    dwells_.push_back(d);
    return false;
  }
  s.retune_us = (now_() - s.t_start_ms) * 1000;
  sleep_(cfg_.settle_ms);
  if (burst) {
    // Search: DISC on this member, then a gap so the last DISC's ack lands
    // before the (quiet) observe.
    beaconing_.store(true, std::memory_order_release);
    sleep_(cfg_.search_ms);
    beaconing_.store(false, std::memory_order_release);
    sleep_(cfg_.beacon_period_ms);
  }
  if (!observe) {
    s.t_end_ms = now_();
    std::lock_guard<std::mutex> lk(mu_);
    dwells_.push_back(d);
    return true;
  }
  // Unlinked: the TX card holds its DISC for the observe. Linked: the TX
  // card carries video and its leak is subtracted (rs.leak) instead.
  if (search_.load(std::memory_order_acquire)) quiet_.store(true, std::memory_order_release);
  // Discard barrier: zero the delta counters and let the USB pipe drain.
  (void)radio_.read_energy(false);
  const uint16_t nhm_period = nhm_period_4us(cfg_.dwell_ms);   // clamps at ~262 ms
  const bool nhm_armed = radio_.arm_nhm_busy(nhm_period);
  const ScoutFrames f0 = radio_.frames();
  const uint64_t tx0 = tx_frames_.load(std::memory_order_acquire);
  const int64_t t0 = now_();
  sleep_(cfg_.dwell_ms);
  const NhmBusyRead nb = nhm_armed ? radio_.read_nhm_busy() : NhmBusyRead{};
  const std::optional<double> busy =
      (nb.valid && nb.period == nhm_period) ? nhm_busy_pct(nb, cfg_.busy_dbm) : std::nullopt;
  d.busy_valid = busy.has_value();
  d.busy_pct = busy.value_or(0.0);
  const ScoutEnergy e = radio_.read_energy(true);
  const ScoutFrames f1 = radio_.frames();
  const uint64_t tx1 = tx_frames_.load(std::memory_order_acquire);
  quiet_.store(false, std::memory_order_release);
  s.observe_ms = now_() - t0;
  s.t_end_ms = now_();
  s.valid_fa = e.fa_valid;
  s.fa_ofdm = e.fa_ofdm;
  s.cca_ofdm = e.cca_ofdm;
  s.valid_igi = e.igi_valid;
  s.igi = e.igi;
  s.valid_nhm = e.nhm_valid;
  if (!e.nhm_valid) s.flags |= devourer::chanmig::kFlagNhmMissing;
  if (!e.fa_valid) s.flags |= devourer::chanmig::kFlagReadFailed;
  s.dvr_frames = static_cast<uint32_t>(f1.own - f0.own);
  s.frames = s.dvr_frames + static_cast<uint32_t>(f1.foreign - f0.foreign);
  d.floor_valid = e.floor_valid;
  d.floor_dbm = e.floor_dbm;

  RankSample rs;
  rs.ch = ch;
  rs.cca = e.cca_ofdm;
  rs.fa = e.fa_ofdm;
  rs.own = s.dvr_frames;
  rs.foreign = s.frames - s.dvr_frames;
  rs.floor_valid = e.floor_valid;
  rs.floor_dbm = e.floor_dbm;
  rs.busy_valid = d.busy_valid;
  rs.busy_pct = d.busy_pct;
  const uint64_t dtx = tx1 >= tx0 ? tx1 - tx0 : 0;   // a counter that went back is no leak
  rs.leak = static_cast<uint32_t>(std::lround(cfg_.leak_per_frame * static_cast<double>(dtx)));
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (e.fa_valid) ranker_.add(rs);
    dwells_.push_back(d);
  }
  publish_();
  return e.fa_valid;
}

void ChannelScout::publish_() {
  if (frozen_.load(std::memory_order_acquire)) return;
  // Lock BEFORE reading op: a concurrent set_op(B) that publishes against B
  // must not be overwritten by a proposal computed against a stale op.
  std::lock_guard<std::mutex> lk(mu_);
  const uint8_t op = op_.load(std::memory_order_acquire);
  const int mr = eff_min_rounds_();
  const uint8_t p = cfg_.link_width_mhz == 40
                        ? pair_proposal(ranker_.all(), op, cfg_.channels, mr, cfg_.pick_margin,
                                        cfg_.blocked_pct)
                        : ranker_.proposal(op, mr);
  proposal_.store(p, std::memory_order_release);
}

bool ChannelScout::op_ranked_locked_(int mr) const {
  const uint8_t op = op_.load(std::memory_order_acquire);
  const auto all = ranker_.all();
  if (cfg_.link_width_mhz == 40) return any_pair_ranked(all, {op}, mr);
  for (const RankEntry& e : all)
    if (e.ch == op) return e.visits >= static_cast<uint32_t>(mr);
  return false;
}

bool ChannelScout::op_ranked() const {
  std::lock_guard<std::mutex> lk(mu_);
  return op_ranked_locked_(eff_min_rounds_());
}

// Every member but op ranked (I1: a linked scout never visits op's pair, so
// waiting for it would never end). While searching op is visited like any
// other channel, so it is waited for too -- otherwise maturity could land a
// dwell before op's last visit and an unlinked commit would compare against
// an op that simply had not been reached yet.
bool ChannelScout::mature() const {
  std::lock_guard<std::mutex> lk(mu_);
  const int mr = eff_min_rounds_();
  const uint8_t op = op_.load(std::memory_order_acquire);
  const auto all = ranker_.all();
  if (all.empty()) return false;
  if (cfg_.link_width_mhz == 40) {
    std::vector<uint8_t> others;
    for (uint8_t c : cfg_.channels)
      if (c != op) others.push_back(c);
    if (!others.empty() && !all_pairs_ranked(all, others, mr)) return false;
  } else {
    for (const RankEntry& e : all)
      if (e.ch != op && e.visits < static_cast<uint32_t>(mr)) return false;
  }
  return !search_.load(std::memory_order_acquire) || op_ranked_locked_(mr);
}

std::vector<uint8_t> ChannelScout::pick_ranking() const {
  std::lock_guard<std::mutex> lk(mu_);
  const int mr = eff_min_rounds_();
  if (cfg_.link_width_mhz == 40)
    return pair_ranking(ranker_.all(), cfg_.channels, mr, cfg_.blocked_pct);
  std::vector<uint8_t> out;
  for (const RankEntry& e : ranker_.ranked(mr)) out.push_back(e.ch);
  return out;
}

std::vector<RankEntry> ChannelScout::ranking() const {
  std::lock_guard<std::mutex> lk(mu_);
  return ranker_.all();
}

std::vector<ScoutDwell> ChannelScout::take_dwells() {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<ScoutDwell> out;
  out.swap(dwells_);
  return out;
}

}  // namespace maburgs
