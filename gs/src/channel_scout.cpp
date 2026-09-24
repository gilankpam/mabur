#include "channel_scout.h"

#include <cstdio>

#include "pair_pick.h"

namespace maburgs {

namespace {
// Channels the scan dwells on (before the one-card home exclusion).
std::vector<uint8_t> dwell_channels(const ScoutCfg& c) {
  if (c.link_width_mhz == 40) return scan_half_set(c.home, c.candidates);
  std::vector<uint8_t> out{c.home};
  for (uint8_t ch : c.candidates) out.push_back(ch);
  return out;
}

devourer::chanmig::ScanPlanConfig plan_for(const ScoutCfg& c) {
  devourer::chanmig::ScanPlanConfig p;
  auto add = [&p](uint8_t ch) {
    for (const auto& d : p.candidates) if (d.primary == ch) return;
    devourer::chanmig::ChannelDef d;
    d.band = ch >= 36 ? 5 : 2;
    d.primary = ch;
    d.width = CHANNEL_WIDTH_20;
    p.candidates.push_back(d);
  };
  for (uint8_t ch : dwell_channels(c))
    if (!(c.one_card && ch == c.home)) add(ch);   // one card: home is the window, not a dwell
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
      ranker_(cfg_.home, dwell_channels(cfg_), cfg_.min_rounds, cfg_.home_margin),
      proposal_(cfg_.home) {}

void ChannelScout::freeze(uint8_t target) {
  target_.store(target, std::memory_order_release);
  proposal_.store(target, std::memory_order_release);
  frozen_.store(true, std::memory_order_release);
}

void ChannelScout::run() {
  while (run_once()) {}
  const uint8_t target = target_.load(std::memory_order_acquire);
  if (cfg_.link_width_mhz == 40) {
    // Join the link at 40 (docs/bw40.md §3). A failure (dead or unready
    // card) is not retried here: the core loop's width resync
    // (width_resync.h) sets the card's width once it is ready again.
    if (!radio_.retune_width(target, 40))
      std::fprintf(stderr, "maburgs channel: scout card width switch to 40 MHz on ch %u "
                           "failed; the core loop retries once the card is ready\n",
                   static_cast<unsigned>(target));
  } else {
    radio_.retune(target);
  }
  at_home_.store(false, std::memory_order_release);
  quiet_.store(false, std::memory_order_release);
  done_.store(true, std::memory_order_release);
}

bool ChannelScout::run_once() {
  if (frozen()) return false;
  const uint64_t round = sched_.rounds_complete();
  if (cfg_.one_card) {
    if (!dwell(cfg_.home, Kind::HomeOneCard, round)) return !frozen();
    if (frozen()) return false;
  } else if (beacon_due_) {
    // Beacon window: the core may send DISC on the home card. Then a quiet
    // gap so the last DISC's ack has landed before the silent dwells.
    quiet_.store(false, std::memory_order_release);
    sleep_(cfg_.home_window_ms);
    quiet_.store(true, std::memory_order_release);
    sleep_(cfg_.beacon_period_ms);
    beacon_due_ = false;
    if (frozen()) return false;
  }
  auto p = sched_.next(now_());
  if (!p.valid) return !frozen();
  const bool ok = dwell(p.bin_ch, Kind::Silent, p.round);
  sched_.complete(p, now_(), ok);
  const uint64_t now_rounds = sched_.rounds_complete();
  if (now_rounds != round) beacon_due_ = true;
  rounds_.store(now_rounds, std::memory_order_release);
  return !frozen();
}

bool ChannelScout::dwell(uint8_t ch, Kind kind, uint64_t round) {
  ScoutDwell d;
  auto& s = d.survey;
  s.seq = seq_++;
  s.def.band = ch >= 36 ? 5 : 2;
  s.def.primary = ch;
  s.def.width = CHANNEL_WIDTH_20;
  s.round = round;
  s.t_start_ms = now_();
  s.settle_ms = cfg_.settle_ms;
  if (!radio_.retune(ch)) {
    s.flags |= devourer::chanmig::kFlagRetuneFailed;
    s.t_end_ms = now_();
    std::lock_guard<std::mutex> lk(mu_);
    dwells_.push_back(d);
    return false;
  }
  s.retune_us = (now_() - s.t_start_ms) * 1000;
  sleep_(cfg_.settle_ms);
  if (kind == Kind::HomeOneCard) {
    // Beacon phase: the core may send DISC on this card. Then a quiet gap so
    // the last DISC's ack has landed, and the measurement below is silent.
    at_home_.store(true, std::memory_order_release);
    sleep_(cfg_.home_window_ms);
    at_home_.store(false, std::memory_order_release);
    sleep_(cfg_.beacon_period_ms);
  }
  // Discard barrier: zero the delta counters and let the USB pipe drain.
  (void)radio_.read_energy(false);
  const ScoutFrames f0 = radio_.frames();
  const int64_t t0 = now_();
  sleep_(cfg_.dwell_ms);
  const ScoutEnergy e = radio_.read_energy(true);
  const ScoutFrames f1 = radio_.frames();
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
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (e.fa_valid) ranker_.add(rs);
    dwells_.push_back(d);
  }
  publish_();
  return e.fa_valid;
}

void ChannelScout::publish_() {
  if (frozen()) return;
  std::lock_guard<std::mutex> lk(mu_);
  const uint8_t p = cfg_.link_width_mhz == 40
                        ? pair_proposal(ranker_.all(), cfg_.home, cfg_.candidates,
                                        cfg_.min_rounds, cfg_.home_margin)
                        : ranker_.proposal();
  proposal_.store(p, std::memory_order_release);
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
