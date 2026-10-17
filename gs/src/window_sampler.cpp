#include "window_sampler.h"

#include <algorithm>
#include <utility>

#include "hop_burst_gate.h"
#include "nhm_busy.h"

namespace maburgs {

WindowSampler::WindowSampler(std::vector<LinkCard*> cards, WindowSamplerCfg cfg, WindowGates gates)
    : cards_(std::move(cards)), cfg_(cfg), gates_(gates), card_mu_(cards_.size()), trackers_(cards_.size()) {}

WindowSampler::~WindowSampler() { stop(); }

bool WindowSampler::active() const {
  return gates_.in_session.load(std::memory_order_relaxed) &&
         !gates_.cal_running.load(std::memory_order_relaxed);
}

WindowRecord WindowSampler::sample_once(uint64_t now_ms) {
  if (reset_req_.exchange(false, std::memory_order_acq_rel))
    for (auto& t : trackers_) t.invalidate();
  WindowRecord rec;
  rec.t_ms = now_ms;
  rec.epoch = epoch_.load(std::memory_order_acquire);
  rec.cards.resize(cards_.size());
  const uint8_t op = gates_.op.load(std::memory_order_relaxed);
  for (size_t i = 0; i < cards_.size(); ++i) {
    LinkCard& fe = *cards_[i];
    CardSample& s = rec.cards[i];
    // The card's whole pass under its device-use lock: the card lifecycle
    // (stop/reopen), the core's dwell burst and the scout dwell take the
    // same lock, so none of them overlaps these reads.
    std::lock_guard<std::mutex> card_lk(card_mu_[i]);
    // Always: a relay's survey window rebases on every read, usable or not.
    s.survey = fe.read_survey_window();
    const bool blocked =
        gates_.card_blocked[i].load(std::memory_order_acquire) ||
        (gates_.dwell_busy.load(std::memory_order_acquire) &&
         gates_.dwell_card.load(std::memory_order_acquire) == static_cast<int>(i));
    if (!verdict_card_usable(fe.ready(), blocked, fe.channel(), op)) {
      trackers_[i].invalidate();
      continue;
    }
    const NhmBusyRead nb = fe.read_nhm_busy();
    const uint32_t dgen = gates_.dwell_gen[i].load(std::memory_order_acquire);
    const uint32_t tgen = gates_.tune_gen[i].load(std::memory_order_acquire);
    if (trackers_[i].usable(nb, fe.channel(), dgen, tgen))
      s.nhm_busy_pct = nhm_busy_pct(nb, cfg_.busy_dbm);
    s.energy = fe.read_energy_scout();
    s.frames = fe.frames();
    s.channel = fe.channel();
    if (fe.arm_nhm_busy(cfg_.nhm_period_4us))
      trackers_[i].armed(s.channel, cfg_.nhm_period_4us, dgen, tgen);
    else
      trackers_[i].invalidate();
    s.valid = true;
  }
  return rec;
}

std::unique_lock<std::mutex> WindowSampler::hold_card(size_t i) {
  return std::unique_lock<std::mutex>(card_mu_.at(i));
}

void WindowSampler::reset(uint64_t now_ms) {
  reset_req_.store(true, std::memory_order_release);
  last_ms_ = now_ms;
  std::lock_guard<std::mutex> lk(mu_);
  box_.reset();
  epoch_.fetch_add(1, std::memory_order_release);
}

void WindowSampler::step(uint64_t now_ms) {
  if (!active()) return;
  if (now_ms - last_ms_ < static_cast<uint64_t>(cfg_.window_ms)) return;
  last_ms_ = now_ms;
  publish(sample_once(now_ms));
}

void WindowSampler::step_now(uint64_t now_ms) {
  last_ms_ = now_ms;
  publish(sample_once(now_ms));
}

void WindowSampler::start(NowMsFn now_ms, SleepFn sleep_ms) {
  if (run_.exchange(true, std::memory_order_acq_rel)) return;
  th_ = std::thread([this, now_ms = std::move(now_ms), sleep_ms = std::move(sleep_ms)] {
    while (run_.load(std::memory_order_acquire)) {
      sleep_ms(cfg_.window_ms);
      if (!run_.load(std::memory_order_acquire)) break;
      if (active()) publish(sample_once(now_ms()));
    }
  });
}

void WindowSampler::stop() {
  run_.store(false, std::memory_order_release);
  if (th_.joinable()) th_.join();
}

void WindowSampler::publish(WindowRecord r) {
  std::lock_guard<std::mutex> lk(mu_);
  // A record sampled before the last reset() belongs to a pass that was
  // mid-read across the reset edge; it must not land in a mailbox the
  // core has already been told is clear, and it is not a dropped sample
  // either -- it was never going to be a live window.
  if (r.epoch != epoch_.load(std::memory_order_acquire)) return;
  if (box_) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    // FA/CCA are deltas since the previous read, so the replaced record's
    // share would be lost with it; fold it into the newer record (a card
    // valid in both, on the same channel) so the consumed delta covers the
    // whole gap, as the cumulative frame counters already do.
    const size_t n = std::min(box_->cards.size(), r.cards.size());
    for (size_t i = 0; i < n; ++i) {
      const CardSample& old = box_->cards[i];
      CardSample& cur = r.cards[i];
      if (!old.valid || !cur.valid || old.channel != cur.channel) continue;
      cur.energy.fa_ofdm += old.energy.fa_ofdm;
      cur.energy.cca_ofdm += old.energy.cca_ofdm;
    }
  }
  box_ = std::move(r);
}

std::optional<WindowRecord> WindowSampler::take() {
  std::lock_guard<std::mutex> lk(mu_);
  std::optional<WindowRecord> r = std::move(box_);
  box_.reset();
  return r;
}

}  // namespace maburgs
