#pragma once
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "config.h"
#include "hop_verdict.h"

namespace maburgs {

enum class HopState { Idle, Ordered, Verifying, Hold };

struct HopTick {
  double now_ms = 0;
  VerdictOut verdict;
  std::optional<uint8_t> best;
  uint32_t best_score = 0;
  bool video_on_target = false;      // lead card (or the only card) received a video AU on hop_ch
  int rcf_sent_since_order = 0;      // one-card: how many RCFs left carrying this epoch
  uint8_t cur_op = 0;
  int n_cards = 2;
  int lead_card = -1;                // non-TX card index, -1 = one card
};

struct HopAction {
  // VerifyPass: the verify window closed clean and the hop stands. The
  // only action with no radio/plan consequence -- it exists so the caller
  // can run spec section 2's second thaw rule, HopVerdict::reset(), at the
  // one instant the spec names ("after a hop's verify window ends"). Like
  // every other kind it is suppressed wholesale when cfg_.enable is false,
  // which is what keeps an observe-only flight's references measuring the
  // channel the link is actually still on.
  enum Kind { None, Order, OneCardRetune, Confirm, Withdraw, Hold, VerifyPass } kind = None;
  uint8_t target = 0;
  uint8_t epoch = 0;
  int restore_rung = -1;
  int lead_card = -1;
};

struct HopEvent {
  double t_ms = 0;
  std::string kind;
  uint8_t epoch = 0;
  uint8_t target = 0;
  uint32_t score = 0;
  double elapsed_ms = 0;
};

// Turns a persistent HopVerdict::trigger into an actual hop order, confirms
// it off lead-card video, verifies it holds, withdraws a hop that never
// confirms, backs off targets that fail verify, and rate-limits the whole
// thing (spec 2026-09-14-inflight-channel-hop §5). Pure: no I/O, no
// threads, no hardware, clock strictly as the caller's now_ms. Does not
// query HopRanker -- the caller has already picked `best` for this tick.
class HopController {
 public:
  HopController(HopCfg cfg, uint8_t home);

  HopAction tick(const HopTick& in);

  uint8_t hop_ch() const;   // what every RCF carries: the standing target (0 until the first order)
  uint8_t epoch() const;
  HopState state() const;
  std::vector<uint8_t> backed_off(double now_ms) const;
  std::vector<HopEvent> take_events();
  uint32_t hops() const;
  // Hold EPISODES entered, not ticks spent holding: idle_tick() runs from
  // Hold as well as Idle, so a held controller with the trigger latched
  // re-enters it at the ~100 Hz control-tick rate. Counting ticks made the
  // sideport number (hop.holds) a meaningless six-digit ramp and pushed an
  // H line into scan.log and a stderr line per tick with it.
  uint32_t holds() const;

 private:
  void idle_tick(const HopTick& in, HopAction& out);
  void ordered_tick(const HopTick& in, HopAction& out);
  void verifying_tick(const HopTick& in, HopAction& out);
  void order(uint8_t target, int restore_rung, int lead_card, uint32_t score, double now,
             const char* event_kind, HopAction& out);
  // A hold is a STATE: enter_hold() logs and counts only the transition
  // into it, leave_hold() logs the matching "hold_end" with how long it
  // lasted. Re-entering while already held sets the action and nothing
  // else.
  void enter_hold(double now, const char* why, uint8_t target, double elapsed_ms, HopAction& out);
  void leave_hold(double now, uint8_t cur_op);
  void withdraw(uint8_t restore_to, double now, HopAction& out);
  void back_off(uint8_t ch, double now);
  bool is_backed_off(uint8_t ch, double now) const;
  void prune_hop_times(double now);
  void log_event(double now, const std::string& kind, uint8_t epoch, uint8_t target, uint32_t score,
                 double elapsed_ms);

  HopCfg cfg_;
  uint8_t home_;
  HopState state_ = HopState::Idle;
  uint8_t hop_ch_ = 0;          // true internal standing target, regardless of cfg_.enable
  uint8_t epoch_ = 0;
  double order_ms_ = 0;
  double verify_start_ = 0;
  double last_confirm_ms_ = -1e18;   // -inf: the first-ever trigger always clears cooldown
  uint32_t hops_ = 0;
  uint32_t holds_ = 0;
  bool one_card_retuned_ = false;
  double hold_start_ms_ = 0;
  std::map<uint8_t, std::pair<double, int>> backoff_;   // ch -> {until_ms, repeat count k}
  std::deque<double> hop_times_;                        // order timestamps, trailing 60 s (rate cap)
  std::vector<HopEvent> events_;
};

}  // namespace maburgs
