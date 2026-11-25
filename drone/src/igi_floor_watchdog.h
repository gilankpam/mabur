// IGI floor watchdog. A Jaguar3 radio flies a low IGI floor for range; when
// its 40 MHz sends stall (a send taking 30 ms or more in two of the last three
// judged one-second windows) the floor climbs in steps until the sends clear.
// The first level that holds two clean windows is kept; a climb that reaches
// the cap and is still slow, or stops being judged, is undone and the next
// episode waits out a hold that doubles each failure. Pure logic: the caller
// supplies the clock and the per-tick longest send, and sets the radio's floor
// to result.level on Raised and Reverted.
#pragma once
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>

namespace mabur {

inline constexpr uint8_t kIgiFloorCap = 0x35;

struct IgiFloorParams {
  uint8_t base = 0, cap = 0;            // cap <= base: watchdog off
  uint32_t window_ms = 1000;
  uint64_t slow_us = 30000;
  uint8_t queue_slow_pct = 40;          // chip TX queue backlog; 0 = queue signal off
  uint32_t settle_ms = 300;
  uint32_t history_ms = 30000;
  uint32_t abandon_ms = 3000;
  uint32_t hold_ms = 30000, hold_max_ms = 300000;
};

// A bench override value: decimal, or hex with 0x; a leading 0 is decimal,
// never octal. Anything else (empty, sign, junk, > 0x7f) is rejected.
inline bool parse_igi_floor(const char* s, uint8_t& out) {
  if (!s || !std::isdigit(static_cast<unsigned char>(s[0]))) return false;
  const int radix = (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ? 16 : 10;
  char* end = nullptr;
  const unsigned long v = std::strtoul(s, &end, radix);
  if (*end || v > 0x7f) return false;
  out = static_cast<uint8_t>(v);
  return true;
}

// image_base = the build's starting floor. base_env / cap_env = the bench
// overrides' raw strings (nullptr when unset); an invalid one is ignored.
// A cap below the base becomes the base, which turns the watchdog off.
inline IgiFloorParams igi_floor_params(uint8_t image_base, const char* base_env,
                                       const char* cap_env) {
  IgiFloorParams p;
  p.base = image_base;
  p.cap = kIgiFloorCap;
  parse_igi_floor(base_env, p.base);
  parse_igi_floor(cap_env, p.cap);
  p.cap = std::max(p.cap, p.base);
  return p;
}

class IgiFloorWatchdog {
 public:
  enum class Event { None, Raised, Kept, Reverted };
  struct Result {
    Event event = Event::None;
    uint8_t from = 0, level = 0;        // level = the floor to have set now
    uint32_t longest_ms = 0;            // Raised: the slow window's longest send
    uint8_t backlog_pct = 0;            // Raised: the slow window's deepest backlog
    uint32_t hold_s = 0;                // Reverted: seconds before a new episode
  };

  IgiFloorWatchdog(const IgiFloorParams& p, uint64_t now_ms)
      : p_(p), level_(p.base), hold_len_(p.hold_ms) { open(now_ms); }

  // One call per tick (about 100 ms). eligible: video is airing at 40 MHz, the
  // radio is up, no calibration is active, and no retune happened since the
  // previous call. longest_send_us: SendTimer::take_longest_us for that interval.
  // queue_backlog_pct: the chip TX queue in use right now (0 when unknown); a
  // low floor backs frames up in the chip while every USB send still returns fast.
  Result step(uint64_t now_ms, bool eligible, uint64_t longest_send_us,
              uint8_t queue_backlog_pct) {
    if (p_.cap <= p_.base) return none();
    // No judged window for abandon_ms (link gone, rung dropped to 20 MHz):
    // the raise was never proven, so it does not stay.
    if (climbing_ && now_ms >= last_act_ + p_.abandon_ms) return fail(now_ms);
    if (now_ms <= win_start_) return none();  // settling after a level change
    win_ok_ = win_ok_ && eligible;
    win_max_ = std::max(win_max_, longest_send_us);
    win_q_ = std::max(win_q_, queue_backlog_pct);
    if (now_ms < win_start_ + p_.window_ms) return none();

    // A window stretched past twice its length (a starved thread, a clock
    // jump) is one ignored window, never several slow ones.
    const bool judged = win_ok_ && now_ms <= win_start_ + 2ull * p_.window_ms;
    const uint64_t longest = win_max_;
    const uint8_t backlog = win_q_;
    const bool slow = longest >= p_.slow_us ||
                      (p_.queue_slow_pct != 0 && backlog >= p_.queue_slow_pct);
    open(now_ms);
    if (!judged) return none();

    if (climbing_) {
      last_act_ = now_ms;
      if (slow) return level_ < p_.cap ? raise(now_ms, longest, backlog) : fail(now_ms);
      if (!clean_) { clean_ = true; return none(); }
      hist_n_ = 0;
      hold_len_ = p_.hold_ms;
      climbing_ = false;
      Result r = none();
      r.event = Event::Kept;
      return r;
    }

    // Watching: the last three judged windows (newest first) of the past
    // history_ms, so two slow ones around a 20 MHz excursion still count.
    int n = std::min(hist_n_, 2);
    while (n > 0 && now_ms > hist_[n - 1].t + p_.history_ms) --n;
    hist_[2] = hist_[1]; hist_[1] = hist_[0];
    hist_[0] = {now_ms, slow};
    hist_n_ = n + 1;
    int slow_n = 0;
    for (int i = 0; i < hist_n_; ++i) slow_n += hist_[i].slow;
    if (slow_n < 2 || now_ms < hold_end_ || level_ >= p_.cap) return none();
    climbing_ = true;
    episode_base_ = level_;
    raises_ = 0;
    return raise(now_ms, longest, backlog);
  }

  uint8_t level() const { return level_; }

 private:
  struct Judged { uint64_t t; bool slow; };

  Result none() const { Result r; r.from = r.level = level_; return r; }

  void open(uint64_t t) { win_start_ = t; win_ok_ = true; win_max_ = 0; win_q_ = 0; }

  void changed(uint64_t now) { last_act_ = now; open(now + p_.settle_ms); }

  Result raise(uint64_t now, uint64_t longest_us, uint8_t backlog) {
    static constexpr uint8_t kSteps[] = {2, 4, 8};  // then straight to the cap
    Result r = none();
    const int up = raises_ < 3 ? kSteps[raises_] : p_.cap;
    level_ = static_cast<uint8_t>(std::min<int>(level_ + up, p_.cap));
    ++raises_;
    clean_ = false;
    changed(now);
    r.event = Event::Raised;
    r.level = level_;
    r.longest_ms = static_cast<uint32_t>(longest_us / 1000);
    r.backlog_pct = backlog;
    return r;
  }

  Result fail(uint64_t now) {
    Result r = none();
    level_ = episode_base_;
    climbing_ = false;
    hist_n_ = 0;
    hold_end_ = now + hold_len_;
    r.event = Event::Reverted;
    r.level = level_;
    r.hold_s = hold_len_ / 1000;
    hold_len_ = std::min(2 * hold_len_, p_.hold_max_ms);
    changed(now);
    return r;
  }

  IgiFloorParams p_;
  uint8_t level_;
  uint64_t win_start_ = 0, win_max_ = 0;
  uint8_t win_q_ = 0;
  bool win_ok_ = true;
  Judged hist_[3] = {};
  int hist_n_ = 0;
  bool climbing_ = false, clean_ = false;
  uint8_t episode_base_ = 0;
  int raises_ = 0;
  uint64_t last_act_ = 0;   // the later of the last level change and judged window
  uint64_t hold_end_ = 0;   // no new episode before this
  uint32_t hold_len_;
};

}  // namespace mabur
