#include "hop_verdict.h"

#include <algorithm>
#include <numeric>

#include "hop_blank.h"

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

HopVerdict::HopVerdict(HopCfg cfg, BusyCfg busy) : cfg_(cfg), busy_(busy) {}

void HopVerdict::reset() {
  frozen_ = false;
  ref_rec_ = 0;
  ref_rung_ = -1;
  healthy_streak_ = 0;
  seen_blocked_ = false;
  recent_interfered_.clear();
  blocked_streak_ = 0;
  detect_ring_.clear();
  have_landed_ = false;
  // The persistence window is cleared with the snapshot: leaving it
  // latched would hand the controller a trigger built from windows
  // measured on the channel we have just left.
  //
  // rec_hist_ survives a reset -- it is the trailing baseline, not
  // per-hop state. ref_rec_/ref_rung_ are the frozen snapshot taken AT a
  // hop's onset and must not leak across a hop boundary.
}

// A new in-session run: the blocked streak and the detect ring are per-run
// state.
void HopVerdict::new_session() {
  blocked_streak_ = 0;
  detect_ring_.clear();
  have_landed_ = false;
}

void HopVerdict::landed(double now_ms) {
  detect_ring_.clear();
  landed_ms_ = now_ms;
  have_landed_ = true;
}

bool HopVerdict::ring_reads_detect_(int lossy_needed) const {
  const int n = cfg_.verdict.detect_windows;
  std::vector<double> fg;
  int lossy = 0;
  for (const auto& s : detect_ring_) {
    if (s.reading) fg.push_back(s.foreign);
    if (s.lossy) ++lossy;
  }
  if (lossy < lossy_needed || static_cast<int>(fg.size()) < (n + 1) / 2) return false;
  // The median of the readings; with an even count, the mean of the two
  // middle ones (so at the threshold the two middle readings must average
  // detect_pct).
  const size_t mid = fg.size() / 2;
  std::nth_element(fg.begin(), fg.begin() + mid, fg.end());
  double med = fg[mid];
  if (fg.size() % 2 == 0) med = (med + *std::max_element(fg.begin(), fg.begin() + mid)) / 2.0;
  return med >= cfg_.verdict.detect_pct;
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
  // ---- the detect ring: every window with a usable card takes a slot,
  // reading-less ones too (they count against the ceil(N/2) readings rule
  // and can carry the ring's loss window). A window with no valid card at
  // all returned above and takes none.
  //
  // A window whose span starts inside the landing settle keeps its reading
  // but is never the ring's loss window: the hop's own retune gap is still
  // being repaired then and reads as recovered symbols (loss_impaired), the
  // same debris HopController::verifying_tick ignores for its fail
  // decision. Counted here, it would confirm a target's foreign reading a
  // ring later and fail verify on a channel with no loss of its own.
  const bool settle_debris = have_landed_ && o.t_start_ms < landed_ms_ + kHopSettleBlankMs;
  detect_ring_.push_back({have_busy_reading, min_foreign_busy_pct, loss_impaired && !settle_debris});
  while (detect_ring_.size() > static_cast<size_t>(std::max(hv.detect_windows, 1)))
    detect_ring_.pop_front();
  // detected: a low-floor occupant confirmed by loss -- the median foreign
  // reading over a full ring at or above detect_pct, with detect_impaired
  // lossy windows inside it. `loss_impaired`, not `impaired`: counting
  // `occupied` (or `detected` itself) would make the confirmation circular.
  const bool detected = detect_ring_.size() == static_cast<size_t>(hv.detect_windows) &&
                        ring_reads_detect_(hv.detect_impaired);
  // occupied: foreign airtime held for occupied_windows consecutive
  // windows that had a busy reading -- impaired on its own, with or without
  // loss. detected: impaired on its own too, so a run whose last window is
  // clean still reads impaired (the reference freeze and the persist see
  // it). These are the two routes to `interfered` below.
  const bool impaired = loss_impaired || occupied || detected;
  const bool weak = cards[best].rssi_dbm < hv.weak_rssi_dbm &&
                    (!cards[best].snr_valid || cards[best].snr_db < hv.weak_snr_db);
  o.evidence = (impaired ? kEvImpaired : 0) | (weak ? kEvWeak : 0) | (detected ? kEvDetected : 0) |
               (blocked ? kEvBlocked : 0) | (occupied ? kEvOccupied : 0);
  // Every route to `interfered` reads foreign NHM airtime. A fade -- range,
  // a tree, a flip, a wall -- can lower our signal but cannot add foreign
  // energy to the air, so foreign airtime is the one signal no fade can
  // fake. Two routes: sustained occupancy over blocked_pct (`occupied`,
  // loss or not), and a lower foreign floor (detect_pct) that only counts
  // once loss confirms it (`detected`). Loss and recovered alone still mark
  // the window impaired (the reference freeze) but never route to
  // `interfered`: each also moves when the link merely fades.
  //
  // weak gates first: at the range edge our own frames stop decoding, so
  // own_air_pct (rebuilt from decoded frames) undercounts and the
  // subtraction can read our own video as foreign airtime.
  //
  // Each route also needs this window's own busy reading (`blocked` for
  // occupied, `have_busy_reading` for detected): a window without one (the
  // usual shape right after a hop lands, before the NHM arm is back) stays
  // unknown, so a hop's verify fails only on a reading actually taken on
  // the target.
  if (!impaired) o.v = Verdict::Healthy;
  else if (weak) o.v = Verdict::Fade;
  else if (occupied && blocked) o.v = Verdict::Interfered;
  else if (detected && have_busy_reading) o.v = Verdict::Interfered;
  else o.v = Verdict::Unknown;
  // ---- reference freeze / thaw
  if (impaired && !frozen_) {
    frozen_ = true; ref_rung_ = rung; ref_rec_ = rec_ref;
  }
  // The rung-store blank's arming edge: the first impaired, non-weak window
  // of this frozen episode that is either route's lead-in (hop_blank.h) --
  // blocked, or lossy with its own foreign reading at or above detect_pct
  // -- not the later window that classifies `interfered`; or, when a detect
  // run's loss window read under detect_pct and so was no lead-in, the
  // first `detected` window, so the detect route never reaches an order
  // with the store unprotected. Re-armed only by a thaw, below or in
  // reset().
  const bool lead_in = blocked || detected ||
                       (loss_impaired && have_busy_reading && min_foreign_busy_pct >= hv.detect_pct);
  if (impaired && lead_in && !weak && !seen_blocked_) {
    seen_blocked_ = true;
    o.first_blocked = true;
  }
  healthy_streak_ = (o.v == Verdict::Healthy) ? healthy_streak_ + 1 : 0;
  // The thaw is held while the ring holds a detection in the making: the
  // windows between a detect run's loss window and the ring filling read
  // healthy, and thawing on them would let the detected window re-freeze
  // ref_rung at the rung the ladder sank to meanwhile -- the hop would then
  // restore that rung, not the pre-onset one. In clean air the ring's
  // median sits far under detect_pct and the hold never applies.
  if (frozen_ && healthy_streak_ >= 3 && !ring_holds_detection()) {
    frozen_ = false; ref_rung_ = -1; seen_blocked_ = false;
  }
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
