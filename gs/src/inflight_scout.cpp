#include "inflight_scout.h"

namespace maburgs {

InflightScout::InflightScout(InflightScoutCfg cfg, ScoutRadio& radio, NowUsFn now_us, SleepFn sleep_ms)
    : cfg_(std::move(cfg)), radio_(&radio), now_us_(std::move(now_us)), sleep_ms_(std::move(sleep_ms)) {}

void InflightScout::set_radio(ScoutRadio& radio) { radio_ = &radio; }

bool InflightScout::dwell(uint8_t ch, uint8_t back, ScoutDwell& d, HopVisit& visit) {
  auto& s = d.survey;
  s.seq = seq_++;
  s.def.band = ch >= 36 ? 5 : 2;
  s.def.primary = ch;
  s.def.width = CHANNEL_WIDTH_20;
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
  const ScoutFrames f0 = radio_->frames();
  const int64_t t2 = now_us_();

  sleep_ms_(cfg_.observe_ms);
  const int64_t t3 = now_us_();

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
  return true;
}

uint8_t InflightScout::next_candidate() {
  const uint8_t ch = cfg_.candidates[next_idx_];
  next_idx_ = (next_idx_ + 1) % cfg_.candidates.size();
  return ch;
}

std::vector<HopVisit> InflightScout::burst(uint8_t back, std::vector<ScoutDwell>& records) {
  std::vector<HopVisit> visits;
  visits.reserve(cfg_.candidates.size());
  for (uint8_t ch : cfg_.candidates) {
    ScoutDwell d;
    HopVisit v;
    if (dwell(ch, back, d, v)) visits.push_back(v);
    records.push_back(d);
  }
  return visits;
}

}  // namespace maburgs
