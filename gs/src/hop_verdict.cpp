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

HopVerdict::HopVerdict(HopCfg cfg, BusyCfg busy, int n_cards)
    : cfg_(cfg), busy_(busy), n_cards_(n_cards) {}

void HopVerdict::reset() {
  frozen_ = false;
  ref_rec_ = 0;
  ref_rung_ = -1;
  healthy_streak_ = 0;
  seen_blocked_ = false;
  recent_interfered_.clear();
  blocked_streak_ = 0;
  // The persistence window is cleared with the snapshot: leaving it
  // latched would hand the controller a trigger built from windows
  // measured on the channel we have just left.
  //
  // rec_hist_ survives a reset -- it is the trailing baseline, not
  // per-hop state. ref_rec_/ref_rung_ are the frozen snapshot taken AT a
  // hop's onset and must not leak across a hop boundary.
}

// A new in-session run: the blocked streak is per-run state.
void HopVerdict::new_session() {
  blocked_streak_ = 0;
}

int HopVerdict::ref_rung() const { return ref_rung_; }

VerdictOut HopVerdict::window(double now_ms, const std::vector<VerdictCardIn>& cards,
                              const VerdictLinkIn& link, int rung) {
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
  // ---- the per-card term (blocked), computed before `impaired` so the
  // occupancy trigger below can fold into it on the SAME window that trips
  // it -- the reference freeze right after reads `impaired` once it
  // already carries `occupied`.
  bool blocked = false;
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
    if (c.busy_valid) {
      const double foreign_pct = std::max(c.nhm_busy_pct - c.own_air_pct, 0.0);
      min_foreign_busy_pct = have_busy_reading ? std::min(min_foreign_busy_pct, foreign_pct)
                                                : foreign_pct;
      have_busy_reading = true;
    }
  }
  blocked = have_busy_reading && min_foreign_busy_pct >= busy_.blocked_pct;
  // Sustained occupancy is impaired on its own, no loss or weak signal
  // needed: a window with a busy reading either continues or resets the
  // streak; a window with none (mid-dwell, no usable card) holds it.
  if (have_busy_reading) blocked_streak_ = blocked ? blocked_streak_ + 1 : 0;
  const bool occupied = blocked_streak_ >= hv.occupied_windows;
  // ---- terms
  const double rec_ref = frozen_ ? ref_rec_
                                  : (rec_hist_.empty() ? 0.0
                                         : std::accumulate(rec_hist_.begin(), rec_hist_.end(), 0.0) / rec_hist_.size());
  // recovered_min: a floor under the recovered term. On a clean channel
  // the trailing mean sits near 0.2/window, so 1-5 recovered symbols read
  // "3x the reference" with 0 % loss (bench session 0232: 97 %
  // of recovered-only impaired windows had <= 8).
  const bool loss_impaired = link.pre_fec_loss * 100.0 > hv.loss_pct ||
                             (rec_ref > 0 && link.recovered > hv.recovered_x * rec_ref &&
                              link.recovered >= static_cast<uint32_t>(std::max(hv.recovered_min, 0)));
  // occupied: foreign airtime held for occupied_windows consecutive
  // windows that had a busy reading -- impaired on its own, with or without
  // loss, and the only route to `interfered` below.
  const bool impaired = loss_impaired || occupied;
  const bool weak = cards[best].rssi_dbm < hv.weak_rssi_dbm &&
                    (!cards[best].snr_valid || cards[best].snr_db < hv.weak_snr_db);
  o.evidence = (impaired ? kEvImpaired : 0) | (weak ? kEvWeak : 0) | (blocked ? kEvBlocked : 0) |
               (occupied ? kEvOccupied : 0);
  // Detection is NHM occupancy only. A fade -- range, a tree, a flip, a
  // wall -- can lower our signal but cannot add foreign energy to the air,
  // so sustained foreign airtime is the one signal no fade can fake. Loss
  // and recovered still mark the window impaired (the reference freeze),
  // but neither routes to `interfered` on its own: each also moves when
  // the link merely fades.
  //
  // weak gates first: at the range edge our own frames stop decoding, so
  // own_air_pct (rebuilt from decoded frames) undercounts and the
  // subtraction can read our own video as foreign airtime.
  //
  // `occupied && blocked`, not `occupied` alone: an occupied window without
  // a busy reading (the usual shape right after a hop lands, before the
  // NHM arm is back) stays unknown, so a hop's verify fails only on a
  // blocked reading actually taken on the target.
  if (!impaired) o.v = Verdict::Healthy;
  else if (weak) o.v = Verdict::Fade;
  else if (occupied && blocked) o.v = Verdict::Interfered;
  else o.v = Verdict::Unknown;
  // ---- reference freeze / thaw
  if (impaired && !frozen_) {
    frozen_ = true; ref_rung_ = rung; ref_rec_ = rec_ref;
  }
  // The rung-store blank's arming edge: the first impaired, non-weak,
  // blocked window of this frozen episode (hop_blank.h) -- the jam's lossy
  // lead-in, not the occupied_windows-th window that classifies
  // `interfered`. Re-armed only by a thaw, below or in reset().
  if (impaired && blocked && !weak && !seen_blocked_) {
    seen_blocked_ = true;
    o.first_blocked = true;
  }
  healthy_streak_ = (o.v == Verdict::Healthy) ? healthy_streak_ + 1 : 0;
  if (frozen_ && healthy_streak_ >= 3) { frozen_ = false; ref_rung_ = -1; seen_blocked_ = false; }
  if (!frozen_) {   // only unfrozen windows feed the trailing reference
    rec_hist_.push_back(link.recovered);
    if (rec_hist_.size() > hist_n) rec_hist_.pop_front();
  }
  o.ref_rung = ref_rung_;
  o.ref_frozen = frozen_;
  // ---- persistence
  recent_interfered_.push_back(o.v == Verdict::Interfered);
  if (recent_interfered_.size() > 3) recent_interfered_.pop_front();
  o.trigger = std::count(recent_interfered_.begin(), recent_interfered_.end(), true) >= cfg_.persist;
  return o;
}

}  // namespace maburgs
