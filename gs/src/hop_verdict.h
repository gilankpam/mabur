#pragma once
#include <cstdint>
#include <deque>
#include <vector>

#include "config.h"
#include "snr_units.h"

namespace maburgs {

enum class Verdict { Healthy, Fade, Interfered, Unknown };
const char* to_string(Verdict v);   // "healthy" "fade" "interfered" "unknown"

// 0x04/0x08/0x10 are unassigned (the retired fading/contended/raised
// evidence-only bits); 0x40 is unassigned too (the retired starved bit,
// which drove `impaired` rather than being evidence-only).
enum : uint8_t { kEvImpaired = 1, kEvWeak = 2,
                 kEvBlocked = 32, kEvOccupied = 128 };

// The verdict thresholds (hop.verdict.weak_rssi_dbm / weak_snr_db) are
// written in dBm / dB. The aggregator's EMAs are
// devourer raw units -- RSSI on a 0..110 scale where raw - 110 is dBm
// (the sideport's own conversion), SNR in half-dB (snr_units.h) -- so the
// fill site converts HERE, never at the threshold, and scan.log V lines
// and flightreport's per-card medians read in dBm/dB like ctl.log. Found
// on the bench: unconverted, `weak` could never trip. A raw
// RSSI of exactly 0 means "no frame heard on this card yet" and stays 0,
// which HopVerdict::window() already treats as its no-reference sentinel.
inline double rssi_raw_to_dbm(double raw) { return raw > 0.0 ? raw - 110.0 : 0.0; }
inline double snr_raw_to_db(double raw) { return raw * kSnrRawToDb; }

struct VerdictCardIn {
  bool valid = false;
  uint32_t foreign = 0, crc_fail = 0, fa = 0, cca = 0;
  double rssi_dbm = 0, snr_db = 0;
  // snr_db is a real per-frame measurement (CardCaps::snr_ok). False on the
  // CPE510 relay, whose SNR is RSSI above a calibrated floor: the weak test
  // then reads RSSI alone instead of demanding both.
  bool snr_valid = true;
  // Busy airtime: NHM busy % of the window
  // and our own video's airtime %, valid only when the card's NHM window
  // covered this window on this channel.
  bool busy_valid = false;
  double nhm_busy_pct = 0, own_air_pct = 0;
};

struct VerdictLinkIn {
  double pre_fec_loss = 0;   // 0..1 over the window
  uint32_t recovered = 0;
};

struct VerdictOut {
  Verdict v = Verdict::Healthy;
  uint8_t evidence = 0;
  int ref_rung = -1;
  bool trigger = false;   // interfered in >= persist of the last 3 windows
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
  // True on the FIRST impaired, non-weak, `blocked` window of a
  // frozen-reference episode, and never again until the references thaw
  // (3 healthy windows, or HopVerdict::reset() after a hop's verify window
  // ends). This is the edge the rung-store blank is scoped to -- see
  // gs/src/hop_blank.h. A jam is acted on only occupied_windows windows in,
  // but its lossy lead-in windows are already blocked: arming here keeps
  // their demotes out of the per-rung store. It is deliberately NOT
  // `ref_frozen`, which is keyed on `impaired` alone: a fade (weak) and an
  // unknown window with no blocked reading leave the ladder's behaviour
  // unchanged. One edge per episode is also what bounds the blank -- a jam
  // that alternates blocked and healthy windows stays inside ONE frozen
  // episode and re-arms nothing.
  bool first_blocked = false;
};

// Per-window classifier: fade / interfered / unknown / healthy. Pure: the
// caller passes the clock, no
// I/O, no threads, no hardware.
class HopVerdict {
 public:
  HopVerdict(HopCfg cfg, BusyCfg busy, int n_cards);
  // One window. cards[i].valid=false = skipped (mid-dwell / dead); rung = ladder rung now.
  VerdictOut window(double now_ms, const std::vector<VerdictCardIn>& cards,
                    const VerdictLinkIn& link, int rung);
  // After a hop's verify window ends (the second thaw rule).
  // Called from ChannelCore::apply_hop_action_ on HopAction::VerifyPass -- see hop_controller.h.
  void reset();
  // A new in-session verdict run is starting (ChannelCore: hop_active's
  // rising edge, where the recovered counter baseline is re-primed).
  // Clears only the blocked streak: a target already occupied before this
  // session must fail verify on its first window, not get
  // occupied_windows of free pass left over from the previous session.
  // RSSI/recovered histories are untouched; reset() is still the per-hop
  // thaw.
  void new_session();
  int ref_rung() const;               // -1 while healthy
  // A live ladder swap inside a frozen episode: the frozen rung index is
  // re-pointed at the same operating point in the new ladder (the caller
  // translates it). No-op while healthy.
  void remap_ref_rung(int rung) { if (ref_rung_ >= 0) ref_rung_ = rung; }

 private:
  HopCfg cfg_;
  BusyCfg busy_;
  int n_cards_;
  // trailing reference (5 s = 5000 / window_ms samples): link recovered mean.
  std::deque<double> rec_hist_;
  bool frozen_ = false;
  double prev_ms_ = 0;
  bool have_prev_ = false;
  bool seen_blocked_ = false;   // a first_blocked edge in the current frozen episode
  double ref_rec_ = 0;
  int ref_rung_ = -1;
  int healthy_streak_ = 0;
  // Consecutive `blocked` windows with a usable busy reading. A window
  // with no busy reading (mid-dwell, no usable card) holds this rather
  // than resetting or advancing it -- see occupied's fold into `impaired`
  // in window(). Survives a hop (reset()/new_session() clear it, nothing
  // else does): a target that is already occupied must fail verify on
  // its first window, not get occupied_windows of free pass.
  int blocked_streak_ = 0;
  std::deque<bool> recent_interfered_;   // last 3
};

}  // namespace maburgs
