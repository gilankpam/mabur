#pragma once
#include <cstdint>
#include <deque>
#include <vector>

#include "config.h"

namespace maburgs {

enum class Verdict { Healthy, Fade, Interfered, Unknown };
const char* to_string(Verdict v);   // "healthy" "fade" "interfered" "unknown"

enum : uint8_t { kEvImpaired = 1, kEvWeak = 2, kEvFading = 4, kEvContended = 8, kEvRaised = 16 };

struct VerdictCardIn {
  bool valid = false;
  uint32_t foreign = 0, crc_fail = 0, fa = 0, cca = 0;
  double rssi_dbm = 0, snr_db = 0;
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
  // been seen and the references have not thawed yet). Exported so the
  // caller can blank the ladder's rung store "from the first impaired
  // window" as spec section 4 requires -- see gs/src/hop_blank.h -- rather
  // than only from the hop order, which is 300-450 ms of detection windows
  // too late.
  bool ref_frozen = false;
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
  bool frozen_ = false;
  double prev_ms_ = 0;
  bool have_prev_ = false;
  std::vector<double> ref_rssi_;
  double ref_rec_ = 0;
  int ref_rung_ = -1;
  int healthy_streak_ = 0;
  std::deque<bool> recent_interfered_;   // last 3
  static double median(std::deque<double> v);
};

}  // namespace maburgs
