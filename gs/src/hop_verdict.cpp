#include "hop_verdict.h"

#include <algorithm>
#include <numeric>

namespace maburgs {

const char* to_string(Verdict v) {
  switch (v) {
    case Verdict::Healthy: return "healthy";
    case Verdict::Fade: return "fade";
    case Verdict::Interfered: return "interfered";
    case Verdict::Unknown: return "unknown";
  }
  return "unknown";
}

double HopVerdict::median(std::deque<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return (n % 2) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0;
}

HopVerdict::HopVerdict(HopCfg cfg, int n_cards)
    : cfg_(cfg), n_cards_(n_cards), rssi_hist_(n_cards) {}

void HopVerdict::reset() {
  frozen_ = false;
  ref_rec_ = 0;
  ref_rung_ = -1;
  ref_rssi_.clear();
  healthy_streak_ = 0;
  seen_interfered_ = false;
  recent_interfered_.clear();
  // The persistence window is cleared with the snapshot: leaving it
  // latched would hand the controller a trigger built from windows
  // measured on the channel we have just left.
  //
  // Histories (rssi_hist_, rec_hist_) survive a reset -- they are the
  // trailing baseline, not per-hop state. ref_rssi_/ref_rec_/ref_rung_ are
  // the frozen snapshot taken AT a hop's onset and must not leak across a
  // hop boundary.
}

int HopVerdict::ref_rung() const { return ref_rung_; }

VerdictOut HopVerdict::window(double now_ms, const std::vector<VerdictCardIn>& cards,
                              const VerdictLinkIn& link, int rung) {
  const double w_s = cfg_.window_ms / 1000.0;
  const size_t hist_n = static_cast<size_t>(std::max(1, 5000 / cfg_.window_ms));
  // best card = highest RSSI among valid cards
  int best = -1;
  for (size_t i = 0; i < cards.size(); ++i)
    if (cards[i].valid && (best < 0 || cards[i].rssi_dbm > cards[best].rssi_dbm)) best = (int)i;
  VerdictOut o;
  // The measurement span, stamped before any early return so even a
  // Verdict::Unknown "no valid card" window carries an honest one.
  o.t_start_ms = have_prev_ ? prev_ms_ : now_ms;
  o.t_ms = now_ms;
  prev_ms_ = now_ms;
  have_prev_ = true;
  if (best < 0) { o.v = Verdict::Unknown; o.ref_frozen = frozen_; return o; }
  const auto& hv = cfg_.verdict;
  // ---- terms
  const double rec_ref = frozen_ ? ref_rec_
                                  : (rec_hist_.empty() ? 0.0
                                         : std::accumulate(rec_hist_.begin(), rec_hist_.end(), 0.0) / rec_hist_.size());
  // recovered_min: a floor under the recovered term. On a clean channel
  // the trailing mean sits near 0.2/window, so 1-5 recovered symbols read
  // "3x the reference" with 0 % loss (bench 2026-09-26, session 0232: 97 %
  // of recovered-only impaired windows had <= 8). starved: no own frame on
  // any valid card -- no frames, no loss, but not healthy either.
  const bool impaired = link.pre_fec_loss * 100.0 > hv.loss_pct ||
                        (rec_ref > 0 && link.recovered > hv.recovered_x * rec_ref &&
                         link.recovered >= static_cast<uint32_t>(std::max(hv.recovered_min, 0))) ||
                        link.starved;
  const bool weak = cards[best].rssi_dbm < hv.weak_rssi_dbm && cards[best].snr_db < hv.weak_snr_db;
  // Guard: a frozen reference that was never established for this card
  // (out-of-range or invalid-with-no-history at freeze time, see below)
  // reads back as 0 -- a real RSSI is never exactly 0 dBm, so treat 0 as
  // "no reference yet" and fall back to the card's own current reading
  // (d_rssi = 0, fading can't fire) instead of fabricating a fade.
  const double frozen_ref = (frozen_ && best < (int)ref_rssi_.size()) ? ref_rssi_[best] : 0.0;
  const double rssi_ref = frozen_
      ? (frozen_ref != 0.0 ? frozen_ref : cards[best].rssi_dbm)
      : (rssi_hist_[best].empty() ? cards[best].rssi_dbm : median(rssi_hist_[best]));
  const bool fading = cards[best].rssi_dbm < rssi_ref - hv.fading_drop_db;
  bool contended = false, raised = false, blocked = false;
  // blocked = the MINIMUM foreign-busy reading across cards that have one,
  // not any-card. A weakly-receiving diversity card's NHM busy counts our
  // own frames too, but its own_air_pct is reconstructed only from the
  // frames it actually decoded -- so a card that hears little of our video
  // reads "foreign" close to our own airtime and would false-block alone.
  // The strong card in the same window reads the interferer's real share,
  // so requiring every reading card to agree needs an interferer that all
  // of them can see, not a receive-weak diversity path. No card with a
  // reading -> not blocked (same as today).
  bool have_busy_reading = false;
  double min_foreign_busy_pct = 0.0;
  for (const auto& c : cards) {
    if (!c.valid) continue;
    contended = contended || c.foreign / w_s > hv.foreign_pps;
    raised = raised || c.fa / w_s > hv.fa_pps;
    if (c.busy_valid) {
      const double foreign_pct = std::max(c.nhm_busy_pct - c.own_air_pct, 0.0);
      min_foreign_busy_pct = have_busy_reading ? std::min(min_foreign_busy_pct, foreign_pct)
                                                : foreign_pct;
      have_busy_reading = true;
    }
  }
  blocked = have_busy_reading && min_foreign_busy_pct >= hv.blocked_pct;
  o.evidence = (impaired ? kEvImpaired : 0) | (weak ? kEvWeak : 0) | (fading ? kEvFading : 0) |
               (contended ? kEvContended : 0) | (raised ? kEvRaised : 0) | (blocked ? kEvBlocked : 0) |
               (link.starved ? kEvStarved : 0);
  if (!impaired) o.v = Verdict::Healthy;
  else if (weak) o.v = Verdict::Fade;
  // blocked beats fading: a real fade lowers power and cannot raise NHM
  // busy; analog desense reads as a fade (docs/analog-vtx-findings-2026-09-25.md).
  else if (blocked) o.v = Verdict::Interfered;
  else if ((contended || raised) && !fading) o.v = Verdict::Interfered;
  else o.v = Verdict::Unknown;
  // ---- reference freeze / thaw
  if (impaired && !frozen_) {
    frozen_ = true; ref_rung_ = rung; ref_rec_ = rec_ref;
    if ((int)ref_rssi_.size() != n_cards_) ref_rssi_.assign(n_cards_, 0);
    for (int i = 0; i < n_cards_; ++i) {
      if (!rssi_hist_[i].empty()) {
        ref_rssi_[i] = median(rssi_hist_[i]);
      } else if (i < (int)cards.size() && cards[i].valid) {
        ref_rssi_[i] = cards[i].rssi_dbm;
      }
      // else: card i is beyond the cards vector, or mid-dwell (invalid)
      // with no trailing history yet -- leave its reference untouched
      // rather than fabricate one from a skipped card's meaningless
      // rssi_dbm == 0.
    }
  }
  // The rung-store blank's arming edge: the first `interfered` window of
  // this frozen episode (hop_blank.h). Re-armed only by a thaw, below or
  // in reset().
  if (o.v == Verdict::Interfered && !seen_interfered_) {
    seen_interfered_ = true;
    o.first_interfered = true;
  }
  healthy_streak_ = (o.v == Verdict::Healthy) ? healthy_streak_ + 1 : 0;
  if (frozen_ && healthy_streak_ >= 3) { frozen_ = false; ref_rung_ = -1; seen_interfered_ = false; }
  if (!frozen_) {   // only unfrozen windows feed the trailing references
    for (int i = 0; i < n_cards_ && i < (int)cards.size(); ++i)
      if (cards[i].valid) {
        rssi_hist_[i].push_back(cards[i].rssi_dbm);
        if (rssi_hist_[i].size() > hist_n) rssi_hist_[i].pop_front();
      }
    rec_hist_.push_back(link.recovered);
    if (rec_hist_.size() > hist_n) rec_hist_.pop_front();
  }
  o.ref_rung = ref_rung_; o.ref_rssi_dbm = rssi_ref; o.d_rssi_db = cards[best].rssi_dbm - rssi_ref;
  o.ref_frozen = frozen_;
  // ---- persistence
  recent_interfered_.push_back(o.v == Verdict::Interfered);
  if (recent_interfered_.size() > 3) recent_interfered_.pop_front();
  o.trigger = std::count(recent_interfered_.begin(), recent_interfered_.end(), true) >= cfg_.persist;
  return o;
}

}  // namespace maburgs
