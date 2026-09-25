#pragma once
#include <cstdint>
#include <deque>
#include <vector>

#include "config.h"
#include "snr_units.h"

namespace maburgs {

enum class Verdict { Healthy, Fade, Interfered, Unknown };
const char* to_string(Verdict v);   // "healthy" "fade" "interfered" "unknown"

enum : uint8_t { kEvImpaired = 1, kEvWeak = 2, kEvFading = 4, kEvContended = 8, kEvRaised = 16,
                 kEvBlocked = 32, kEvStarved = 64 };

// The verdict thresholds (hop.verdict.weak_rssi_dbm / weak_snr_db /
// fading_drop_db) are written in dBm / dB. The aggregator's EMAs are
// devourer raw units -- RSSI on a 0..110 scale where raw - 110 is dBm
// (the sideport's own conversion), SNR in half-dB (snr_units.h) -- so the
// fill site converts HERE, never at the threshold, and scan.log V lines
// and flightreport's per-card medians read in dBm/dB like ctl.log. Found
// on the 2026-09-15 bench: unconverted, `weak` could never trip. A raw
// RSSI of exactly 0 means "no frame heard on this card yet" and stays 0,
// which HopVerdict::window() already treats as its no-reference sentinel.
inline double rssi_raw_to_dbm(double raw) { return raw > 0.0 ? raw - 110.0 : 0.0; }
inline double snr_raw_to_db(double raw) { return raw * kSnrRawToDb; }

struct VerdictCardIn {
  bool valid = false;
  uint32_t foreign = 0, crc_fail = 0, fa = 0, cca = 0;
  double rssi_dbm = 0, snr_db = 0;
  // Busy airtime (spec 2026-09-25-nhm-airtime §6): NHM busy % of the window
  // and our own video's airtime %, valid only when the card's NHM window
  // covered this window on this channel.
  bool busy_valid = false;
  double nhm_busy_pct = 0, own_air_pct = 0;
};

struct VerdictLinkIn {
  double pre_fec_loss = 0;   // 0..1 over the window
  uint32_t recovered = 0;
  // No own frame on ANY valid card this window (main.cpp: the own-frame
  // delta is 0 on every card with a valid reading, and at least one card
  // is valid). No frames means no loss, so without this a drone starved
  // by a jam read `healthy`. Counts as impaired (kEvStarved).
  bool starved = false;
  // Video AUs published this window (main.cpp: the au_seq delta). Against
  // its trailing mean it widens `starved` to an AU-rate collapse -- see
  // HopVerdictCfg::starved_frac.
  uint32_t au_count = 0;
};

struct VerdictOut {
  Verdict v = Verdict::Healthy;
  uint8_t evidence = 0;
  int ref_rung = -1;
  bool trigger = false;   // interfered in >= persist of the last 3 windows
  double ref_rssi_dbm = 0;
  double d_rssi_db = 0;
  // The wall-clock span this verdict's counter deltas were gathered over:
  // t_start_ms = the previous window's now_ms (== now_ms on the first
  // window ever), t_ms = this window's now_ms. Carried because a cached
  // VerdictOut outlives the window that produced it -- main.cpp recomputes
  // one only every hop.window_ms but feeds HopController every ~10 ms
  // control tick, so the CONSUMER has to be able to tell a fresh verdict
  // from one measured before a hop landed (HopController::verifying_tick,
  // C1). Never compare it against anything but the caller's own clock:
  // both are the same now_ms the caller passes window().
  double t_start_ms = 0;
  double t_ms = 0;
  // The frozen-reference episode is open (the first impaired window has
  // been seen and the references have not thawed yet).
  bool ref_frozen = false;
  // True on the FIRST `interfered` window of a frozen-reference episode,
  // and never again until the references thaw (3 healthy windows, or
  // HopVerdict::reset() after a hop's verify window ends). This is the
  // edge spec section 4 scopes the rung-store blank to -- see
  // gs/src/hop_blank.h. It is deliberately NOT `ref_frozen`, which is
  // keyed on `impaired`: Fade and Unknown are impaired too, and section 4
  // ends "fade/unknown: unchanged ladder behaviour". One edge per episode
  // is also what bounds the blank -- a jam that alternates interfered and
  // healthy windows stays inside ONE frozen episode and re-arms nothing.
  bool first_interfered = false;
};

// Per-window classifier: fade / interfered / unknown / healthy (spec
// 2026-09-14-inflight-channel-hop). Pure: the caller passes the clock, no
// I/O, no threads, no hardware.
class HopVerdict {
 public:
  HopVerdict(HopCfg cfg, int n_cards);
  // One window. cards[i].valid=false = skipped (mid-dwell / dead); rung = ladder rung now.
  VerdictOut window(double now_ms, const std::vector<VerdictCardIn>& cards,
                    const VerdictLinkIn& link, int rung);
  // After a hop's verify window ends (spec section 2's second thaw rule).
  // Called from main.cpp on HopAction::VerifyPass -- see hop_controller.h.
  void reset();
  int ref_rung() const;               // -1 while healthy

 private:
  HopCfg cfg_;
  int n_cards_;
  // trailing references (5 s = 5000 / window_ms samples), per card RSSI median, link recovered mean
  std::vector<std::deque<double>> rssi_hist_;
  std::deque<double> rec_hist_;
  std::deque<double> au_hist_;     // per-window AU count, same trailing rule as rec_hist_
  bool frozen_ = false;
  double prev_ms_ = 0;
  bool have_prev_ = false;
  bool seen_interfered_ = false;   // an `interfered` window in the current frozen episode
  std::vector<double> ref_rssi_;
  double ref_rec_ = 0;
  double ref_au_ = 0;
  int ref_rung_ = -1;
  int healthy_streak_ = 0;
  std::deque<bool> recent_interfered_;   // last 3
  static double median(std::deque<double> v);
};

}  // namespace maburgs
