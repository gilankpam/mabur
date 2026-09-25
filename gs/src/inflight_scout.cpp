#include "inflight_scout.h"

#include <algorithm>

#include "mabur/ht40.h"
#include "nhm_busy.h"

namespace maburgs {

InflightScout::InflightScout(InflightScoutCfg cfg, ScoutRadio& radio, NowUsFn now_us, SleepFn sleep_ms)
    : cfg_(std::move(cfg)), radio_(&radio), now_us_(std::move(now_us)), sleep_ms_(std::move(sleep_ms)) {
  set_ = cfg_.candidates;
  if (cfg_.home != 0 && std::find(set_.begin(), set_.end(), cfg_.home) == set_.end()) set_.push_back(cfg_.home);
}

void InflightScout::set_radio(ScoutRadio& radio) { radio_ = &radio; }

bool InflightScout::dwell(uint8_t ch, uint8_t back, ScoutDwell& d, HopVisit& visit) {
  auto& s = d.survey;
  s.seq = seq_++;
  s.def.band = ch >= 36 ? 5 : 2;
  s.def.primary = ch;
  // FastRetune keeps the card's width and offset: a 40 MHz dwell scores the
  // candidate's PRIMARY only (secondary-blind, docs/bw40.md §3).
  s.def.width = cfg_.width_mhz == 40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20;
  s.def.offset = cfg_.width_mhz == 40 ? mabur::ht40_offset(ch) : 0;
  d.in_session = true;

  const int64_t t0 = now_us_();
  s.t_start_ms = t0 / 1000;
  if (!radio_->retune(ch)) {
    s.flags |= devourer::chanmig::kFlagRetuneFailed;
    // Never leave the card stranded on a candidate: it is the link's only
    // fallback diversity while the video card stays put. This return
    // retune's own result isn't separately actionable here -- the flag
    // already stands, and the caller must not trust this record's card
    // position either way.
    (void)radio_->retune(back);
    s.t_end_ms = now_us_() / 1000;
    return false;
  }
  const int64_t t1 = now_us_();

  // Discard barrier: the scout read's delta is since the PREVIOUS read of
  // either kind, so this throwaway call zeroes it before the real
  // observation window starts (see ScoutRadio::read_energy_scout and
  // devourer's GetRxEnergyScout contract).
  (void)radio_->read_energy_scout();
  const uint16_t nhm_period = nhm_period_4us(cfg_.observe_ms);
  const bool nhm_armed = radio_->arm_nhm_busy(nhm_period);
  const ScoutFrames f0 = radio_->frames();
  const int64_t t2 = now_us_();

  sleep_ms_(cfg_.observe_ms);
  const int64_t t3 = now_us_();

  const NhmBusyRead nb = nhm_armed ? radio_->read_nhm_busy() : NhmBusyRead{};
  const std::optional<double> busy =
      (nb.valid && nb.period == nhm_period) ? nhm_busy_pct(nb, cfg_.busy_dbm) : std::nullopt;
  d.busy_valid = busy.has_value();
  d.busy_pct = busy.value_or(0.0);

  const ScoutEnergy e = radio_->read_energy_scout();
  const ScoutFrames f1 = radio_->frames();
  const int64_t t4 = now_us_();

  const bool back_ok = radio_->retune(back);
  const int64_t t5 = now_us_();

  s.retune_us = t1 - t0;
  s.settle_ms = 0;   // no settle phase -- this dwell is the ~10 ms budget itself
  s.observe_ms = (t3 - t2) / 1000;
  s.t_end_ms = t5 / 1000;
  s.valid_fa = e.fa_valid;
  s.fa_ofdm = e.fa_ofdm;
  s.cca_ofdm = e.cca_ofdm;
  s.valid_igi = e.igi_valid;
  s.igi = e.igi;
  s.valid_nhm = e.nhm_valid;
  if (!e.nhm_valid) s.flags |= devourer::chanmig::kFlagNhmMissing;
  if (!e.fa_valid) s.flags |= devourer::chanmig::kFlagReadFailed;
  const uint32_t dvr_frames = static_cast<uint32_t>(f1.own - f0.own);
  const uint32_t foreign_delta = static_cast<uint32_t>(f1.foreign - f0.foreign);
  s.dvr_frames = dvr_frames;
  s.frames = dvr_frames + foreign_delta;

  d.floor_valid = e.floor_valid;
  d.floor_dbm = e.floor_dbm;
  d.to_us = t1 - t0;
  d.read_us = t4 - t3;
  d.back_us = t5 - t4;

  if (!back_ok) {
    // The observation itself is good, but the card may still be parked on
    // `ch` -- the return retune is what proves it left, and it just told
    // us it didn't. Live video is on the OTHER card, so this is exactly
    // the stranding the module exists to prevent: surface it via the flag
    // (it reaches the D line) and refuse to book a HopVisit gathered
    // around a card whose position is unknown. Recovery is Task 11's
    // mechanical retune loop, not this module's job.
    s.flags |= devourer::chanmig::kFlagRetuneFailed;
    return false;
  }

  visit.ch = ch;
  visit.t_ms = static_cast<double>(t0) / 1000.0;
  visit.fa = e.fa_ofdm;
  visit.cca = e.cca_ofdm;
  visit.own = dvr_frames;
  visit.foreign = foreign_delta;
  visit.busy_valid = d.busy_valid;
  visit.busy_pct = d.busy_pct;
  return true;
}

std::optional<uint8_t> InflightScout::next_candidate(uint8_t skip) {
  for (size_t i = 0; i < set_.size(); ++i) {
    const uint8_t ch = set_[next_idx_];
    next_idx_ = (next_idx_ + 1) % set_.size();
    if (ch != skip) return ch;
  }
  return std::nullopt;
}

std::vector<HopVisit> InflightScout::burst(uint8_t back, std::vector<ScoutDwell>& records) {
  std::vector<HopVisit> visits;
  visits.reserve(set_.size());
  for (uint8_t ch : set_) {
    if (ch == back) continue;
    ScoutDwell d;
    HopVisit v;
    if (dwell(ch, back, d, v)) visits.push_back(v);
    records.push_back(d);
  }
  return visits;
}

}  // namespace maburgs
