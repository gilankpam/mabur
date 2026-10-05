#include "rc_agent.h"
#include "air_rate.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "mabur/link_key.h"
#include "mabur/rc_proto.h"
#include "mabur/uep_encoder.h"

namespace mabur {

using rc::Disc;
using rc::DiscAck;
using rc::LayerTxSpec;
using rc::PhyMode;
using rc::Rcf;

namespace {

int round_to_100(double v) { return static_cast<int>(std::lround(v / 100.0) * 100); }

}  // namespace

RcAgent::RcAgent(const Config& cfg, Actuator& act, OvOverride* ovr, uint8_t start_ch)
    : cfg_(cfg), act_(act), ovr_(ovr),
      channel_(mabur::channel_set_member(cfg.radio.channels, start_ch) ? start_ch
                                                                        : cfg.radio.channels.front()) {}

uint32_t RcAgent::fresh_vtx_nonce_() {
  static thread_local std::mt19937 rng{std::random_device{}()};
  uint32_t n;
  do { n = rng(); } while (n == 0);
  return n;
}

void RcAgent::clear_sessions_() {
  current_ = Session{};
  pending_ = Session{};
  publish_session_();
}

void RcAgent::publish_session_() {
  published_session_.store(current_.valid
      ? (static_cast<uint64_t>(current_.vrx_nonce) << 32) | current_.vtx_nonce : 0,
      std::memory_order_release);
}

// Freshness first (cheap, and the seq32 candidate is an input to the tag):
// a pair with no accepted RCF yet takes the wire seq as seq32 (the GS starts
// both at 1 on a new vtx_nonce); after that the 16-bit delta extends the
// last accepted seq32. Only a frame that also verifies moves the tracker,
// which the caller does.
bool RcAgent::verify_rcf_(const uint8_t* body, size_t len, const rc::Rcf& r,
                          const Session& s, uint32_t* seq32) const {
  if (!s.valid) return false;
  uint32_t cand;
  if (!s.have_seq) {
    cand = r.seq;
  } else {
    const uint16_t delta = static_cast<uint16_t>(r.seq - static_cast<uint16_t>(s.last_seq32));
    if (delta < 1 || delta > 32767) return false;
    cand = s.last_seq32 + delta;
  }
  if (!rc::verify_control(body, len, cfg_.link.key, rc::TagCtx{s.vrx_nonce, s.vtx_nonce, cand}))
    return false;
  *seq32 = cand;
  return true;
}

bool RcAgent::verify_session_tagged(const uint8_t* body, size_t len, uint32_t seq32,
                                    uint64_t* session) const {
  const uint64_t p = published_session_.load(std::memory_order_acquire);
  if (p == 0) return false;
  if (!rc::verify_control(body, len, cfg_.link.key,
          rc::TagCtx{static_cast<uint32_t>(p >> 32), static_cast<uint32_t>(p & 0xFFFFFFFFu), seq32}))
    return false;
  if (session) *session = p;
  return true;
}

bool RcAgent::accept_nack_counter(uint32_t counter) {
  return accept_nack_counter(counter, published_session_.load(std::memory_order_acquire));
}

// Keyed by the pair's vtx nonce rather than reset from publish_session_():
// a reset store can never be ordered against an RX-thread CAS that verified
// under the OLD pair and lands after it -- that would plant the old pair's
// (possibly large) counter in the new pair's space and refuse the GS's
// restarted counters until they overtake it. With the key in the same word,
// a stale-keyed state just reads as "nothing accepted yet".
bool RcAgent::accept_nack_counter(uint32_t counter, uint64_t session) {
  if (session == 0 || session != published_session_.load(std::memory_order_acquire)) return false;
  const uint64_t key = session & 0xFFFFFFFFu;   // vtx_nonce
  uint64_t cur = nack_last_.load(std::memory_order_relaxed);
  for (;;) {
    const uint32_t last = (cur >> 32) == key ? static_cast<uint32_t>(cur) : 0;
    if (counter <= last) return false;
    if (nack_last_.compare_exchange_weak(cur, (key << 32) | counter, std::memory_order_relaxed))
      return true;
  }
}

bool RcAgent::verify_cal_frame(const uint8_t* body, size_t len, bool sweep_running) {
  auto verifies = [&](uint64_t s) {
    return s != 0 && rc::verify_control(body, len, cfg_.link.key,
        rc::TagCtx{static_cast<uint32_t>(s >> 32), static_cast<uint32_t>(s & 0xFFFFFFFFu), 0});
  };
  // The latch only outlives the published session while a sweep runs: the
  // GS is radio-silent for every phase, so the drone reaches FAILSAFE (and
  // clears current_) inside every run, yet the run's later CAL_CMDs and its
  // CAL_RESULT are still tagged under the pair it was going under.
  if (!sweep_running) cal_latch_session_ = 0;
  const uint64_t p = published_session_.load(std::memory_order_acquire);
  uint64_t used = 0;
  if (verifies(p)) used = p;
  else if (cal_latch_session_ != p && verifies(cal_latch_session_)) used = cal_latch_session_;
  bool ok = used != 0;
  if (ok) {
    cal_latch_session_ = used;
    if (used != cal_ring_session_) {
      // New pair: the seen nonces belonged to the old one, whose tags no
      // longer verify anyway (the latch has moved off it too).
      cal_ring_session_ = used;
      cal_seen_n_ = cal_seen_next_ = 0;
      have_cal_current_ = false;
    }
  }
  if (ok && rc::frame_type(body, len) == rc::T_CAL_CMD) {
    // In-session freshness (spec 2026-10-01 §5): a cal nonce already seen
    // in this session, other than the running one, is a replay.
    if (auto c = rc::parse_cal_cmd(body, len)) {
      if (!have_cal_current_ || c->nonce != cal_current_) {
        const auto seen_end = cal_seen_.begin() + static_cast<std::ptrdiff_t>(cal_seen_n_);
        if (std::find(cal_seen_.begin(), seen_end, c->nonce) != seen_end) {
          ok = false;
        } else {
          cal_seen_[cal_seen_next_] = c->nonce;
          cal_seen_next_ = (cal_seen_next_ + 1) % kCalNonceRing;
          if (cal_seen_n_ < kCalNonceRing) ++cal_seen_n_;
          cal_current_ = c->nonce;
          have_cal_current_ = true;
        }
      }
    }
  }
  if (!ok) auth_reject_.store(true, std::memory_order_relaxed);
  return ok;
}

void RcAgent::install_session_for_replay(uint32_t vrx_nonce, uint32_t vtx_nonce) {
  pending_ = Session{vrx_nonce, vtx_nonce, true, 0, false, channel_};
}

void RcAgent::note_chain_break() {
  chain_break_pending_.store(true, std::memory_order_relaxed);
}

void RcAgent::note_arm_state(bool armed, uint64_t now_ms) {
  arm_report_.store(((now_ms + 1) << 1) | (armed ? 1u : 0u), std::memory_order_relaxed);
}

// Spec 2026-09-20 §2. Runs once per tick, after the BOOT branch. Unpacks
// the latest arm report, latches ARMED forever, recomputes whether the
// low-power operating point is wanted, and on a change runs the policy
// with force in ANY state -- pre-link RENDEZVOUS on the ground is exactly
// when this mode matters. "Fresh" tolerates a report stamped a little
// AFTER this tick's clock: the MSP thread stamps on its own read.
void RcAgent::intake_arm_state_(uint64_t now_ms) {
  const uint64_t r = arm_report_.load(std::memory_order_relaxed);
  if (r != 0) {
    arm_reported_ = true;
    last_armed_ = (r & 1u) != 0;
    last_arm_ms_ = (r >> 1) - 1;
    if (last_armed_) armed_latched_ = true;
  }
  const bool fresh = arm_reported_ &&
                     (now_ms < last_arm_ms_ ||
                      now_ms - last_arm_ms_ <= static_cast<uint64_t>(cfg_.low_power.stale_ms));
  // The mode follows the FC's CURRENT arm state, not a one-shot latch: a
  // DISARMED report re-enters low power however many times the FC has armed
  // before. That is the post-crash case -- aircraft down, FC still
  // answering, link degraded to mcs0 -- where a thin 15 fps stream is more
  // likely to reach the GS than a full-rate one. armed_latched_ survives as
  // observability only (the stats line's armed=), never as policy.
  //
  // What did NOT change is fail open: `fresh` still gates everything, so
  // silence, a dead UART or an FC that lost power all mean FULL power. A
  // crash that kills the FC therefore gives full-rate video, not this mode.
  const bool want = cfg_.low_power.enable && fresh && !last_armed_;
  if (want == low_power_active_) return;
  low_power_active_ = want;
  if (want)
    std::fprintf(stderr, "rc: low_power ENTER fps=%d cap=%d kbps\n",
                 cfg_.low_power.fps, cfg_.low_power.bitrate_kbps);
  else
    // last_armed_, not armed_latched_: the latch never clears, so reading it
    // here would report "armed" for every staleness exit after the first arm.
    std::fprintf(stderr, "rc: low_power EXIT (%s)\n", last_armed_ ? "armed" : "stale");
  run_bitrate_policy(now_ms, /*force=*/true);
}

// One pacer for every IDR producer (spec 2026-08-28 venc-foldin §4). 100 ms
// floor for all of them — an IDR is the most expensive frame on the link and
// two of them back to back buy nothing; chain-break requests also respect a
// 1 s holdoff from the previous chain IDR, because one IDR heals a break and
// a ring that has no consumer needs none at all (upstream measurement,
// waybeam f956a52 venc_frame_ring.h). A refused request is DROPPED, not
// queued: the next real break re-raises it.
bool RcAgent::idr_due(uint64_t now_ms, bool chain) {
  if (have_last_idr_ && now_ms - last_idr_ms_ < 100) return false;
  if (chain && have_last_chain_idr_ && now_ms - last_chain_idr_ms_ < 1000)
    return false;
  last_idr_ms_ = now_ms;
  have_last_idr_ = true;
  if (chain) {
    last_chain_idr_ms_ = now_ms;
    have_last_chain_idr_ = true;
  }
  return true;
}

// Applies the MAX_RANGE operating point (profile_table()[MAX_RANGE_PROFILE]
// via ladder_from). This is a state-transition apply (BOOT's initial op, or
// a LINKED->FAILSAFE entry), so it forces the bitrate policy to the robust
// MCS0 floor bitrate immediately, bypassing the steady-state throttle and
// hysteresis — the spec mandates failsafe = robust MCS + floor bitrate, and
// a degraded radio link must never keep flooding at the last LINKED rate.
void RcAgent::apply_max_range(uint64_t now_ms) {
  auto ladder = rc::ladder_from(PhyMode::HT, 0, 20, cfg_.radio.ldpc);

  // Forced shed while MAX_RANGE is the operating point (BOOT/RENDEZVOUS or a
  // LINKED->FAILSAFE entry) — held sticky in failsafe_shed_ until an
  // RCF/DISC takes the agent back to LINKED (see apply_ladder_op), so a
  // later congestion reapply's recompute of shed[1] (which is otherwise
  // driven by shed_level_ alone) can't silently clobber it.
  failsafe_shed_ = true;

  applied_.ladder = ladder;
  applied_.probe_profile = rc::kNoProbeProfile;
  // 2.0, not 1.0: the pre-Task-1 wire scale doubled every commanded
  // overhead on its way into the budget formula (uep_layer_overhead's ref
  // scale); RC_VERSION 4 made fec_overhead a literal actual-overhead value
  // with no translation left to apply it, so this hardcoded MAX_RANGE
  // constant — the one caller with no wire value to carry the doubling for
  // it — must apply the old ×2 itself to keep the MCS0 floor bitrate
  // unchanged (carried review finding, Task 3 review). Both layers get the
  // same 2.0: MAX_RANGE has no per-stream pair to carry (Task 6), and enh
  // is shed here anyway.
  applied_.fec_ov_base = 2.0;
  applied_.fec_ov_enh = 2.0;
  applied_.shed[0] = false;
  applied_.shed[1] = true;  // enh layer shed in MAX_RANGE, per spec
  ++applied_.generation;
  act_.apply_op(applied_);
  run_bitrate_policy(now_ms, /*force=*/true);
}

// Applies a resolved (DISC row or RCF-decoded) ladder/FEC operating point.
// ov_base/ov_enh are the per-stream pair as-is (Task 6, RC_VERSION 5) — no
// translation, applied directly to the UEP layers by apply_op_to_uep. Does
// NOT run the bitrate policy itself — callers on the RCF path invoke
// run_bitrate_policy() explicitly afterwards, per the spec.
void RcAgent::apply_ladder_op(const std::array<LayerTxSpec, 2>& ladder,
                              double ov_base, double ov_enh,
                              uint8_t probe_profile) {
  // A resolved DISC/RCF op is only ever applied on a path that (re)enters
  // LINKED (see on_rc_frame), so the sticky MAX_RANGE forced-shed from a
  // prior RENDEZVOUS/FAILSAFE no longer applies — clear it here rather than
  // in on_rc_frame so it's cleared atomically with the op that ends it.
  failsafe_shed_ = false;

  applied_.ladder = ladder;
  applied_.fec_ov_base = ov_base;
  applied_.fec_ov_enh = ov_enh;
  applied_.probe_profile = probe_profile;
  if (probe_profile != rc::kNoProbeProfile) {
    // The probe flies the probe rung's own (mcs, bw) — not the current op's
    // width — so a 20->40 promote is actually measured at 40 (controller
    // Task 11b, 2026-09-24, docs/bw40.md). Only the mode is kept from the
    // current op's ladder: the wire profile byte doesn't vary mode
    // independently of mcs/bw in practice, and ladder[1].mode is already
    // validated/known-good, so there's no reason to trust a decoded mode
    // over it.
    PhyMode pm; uint8_t pmcs, pbw;
    rc::decode_profile(probe_profile, pm, pmcs, pbw);
    applied_.probe = rc::ladder_from(ladder[1].mode, pmcs, pbw, cfg_.radio.ldpc)[1];
  }
  applied_.shed[0] = false;
  // shed_level_ still counts 0..3 (congestion semantics untouched — see
  // run_congestion_guard), but with the reserved layer gone there is only
  // one droppable layer left, so any level >= 1 sheds it; there is no
  // second layer for level >= 2 to additionally shed.
  applied_.shed[1] = shed_level_ >= 1;
  ++applied_.generation;
  act_.apply_op(applied_);
}

// Reapplies the current commanded op (ladder/fec unchanged) with the
// current congestion-shed level folded in. Used by the congestion guard,
// which doesn't change the ladder or bump generation on its own —
// generation tracks *new operating points* (BOOT/DISC/RCF/failsafe entry),
// not shed adjustments to the current one, so this function publishes via
// act_.apply_op() WITHOUT touching applied_.generation. Consumers (the
// hot-thread loops in main.cpp) must therefore detect "a new AppliedOp was
// published" by identity-comparing the shared_ptr they last applied against
// the one the atomic handoff currently holds, NOT by checking whether
// generation changed — otherwise a congestion shed would be computed here
// but silently never reach the encoder.
//
// shed[1] ORs together every independent reason the enh layer must be
// shed: failsafe_shed_ (sticky for as long as MAX_RANGE is the operating
// point — see apply_max_range/apply_ladder_op) and the local congestion-
// shed level. Without the OR, this recompute-from-shed_level_-alone would
// clobber a forced failsafe shed the moment a congestion tick runs while in
// FAILSAFE.
void RcAgent::reapply_with_shed() {
  applied_.shed[1] = failsafe_shed_ || (shed_level_ >= 1);
  act_.apply_op(applied_);
}

// Runs whenever a new op is applied. The formula is always recomputed, but
// whether an actual set_bitrate_kbps() call happens depends on `force` and
// on the DIRECTION of the change:
//
//  - a DECREASE vs the last value actually sent always goes out, on the
//    same tick that applied the MCS. The radio capacity has already
//    dropped when the RCF lands; deferring the shed lets the encoder flood
//    a smaller pipe and TxQueue drop-oldest kills whole FEC bodies — a
//    multi-rung demote cascade under a gated shed is the 2026-08-09
//    freeze-crash (docs/shed-lag-findings-2026-08-09.md).
//  - force=false, non-decrease (steady-state LINKED RCFs): the call is
//    throttled to at most once per 1000ms, so repeated RCFs within one
//    second collapse to a single call (scenario: bitrate hysteresis) and a
//    late quality INCREASE stays harmless and lazy. The dedup test is an
//    exact inequality vs the last value actually sent: a target that
//    differs at all is a real operating-point change and must eventually
//    reach the encoder. It was a +-10% magnitude deadband until
//    2026-08-28, which measured as a permanent undershoot — the deadband
//    filtered an ABSOLUTE target with no accumulator, so an error smaller
//    than the band could never grow into it and the last applied value
//    became a fixed point. Prod (airtime_budget 0.60, bitrate_max_kbps
//    10000) hit it at the top of the ladder: rung4 commands 9400, rung5's
//    12480 clamps to 10000, and |10000-9400| = 600 < 940 meant the promote
//    was discarded forever — the link ran mcs5 while the encoder stayed on
//    the mcs4 bitrate, 6% low, on the rung the link occupies almost all
//    the time. Note it takes the CLAMP to create the trap: unclamped, the
//    rung gap is far wider than 10%. Decreases were always exempt, so the
//    error only ever accumulated downward.
//  - force=true (BOOT's initial MAX_RANGE apply, every LINKED->FAILSAFE
//    entry, and RCFs that transition into LINKED from RENDEZVOUS/FAILSAFE):
//    the throttle and dedup gates are bypassed entirely — the new
//    operating point's bitrate (e.g. the MCS0 floor on failsafe entry)
//    takes effect immediately, every time. The value is still clamped and
//    rounded to 100 as usual. The throttle timestamp is updated afterwards
//    either way, so a steady-state RCF arriving shortly after a forced call
//    is throttled normally.
//
// The target is a PURE FUNCTION OF THE OPERATING POINT — rung PHY rates
// and the commanded overhead pair in, kbps out. Nothing measured enters
// it, so a fixed op commands a fixed bitrate for as long as it is held.
//
// It was measurement-anchored between 2026-08-29 and 2026-09-01: AirFeed
// published a live share_base plus per-stream framing excess (emitted air
// bytes over frame bytes, EWMA), and this function blended them in. That
// correction was real — framing excess runs ~5% at large frames and more
// as frames shrink — but it closed a loop the encoder could not afford.
// Lower rate -> smaller frames -> proportionally more padding -> higher
// excess -> lower target, re-evaluated on every RCF at 10-20 Hz, with the
// 100 kbps rounding grid fine enough that the dither kept crossing it.
//
// On Star6E every MI_VENC_SetChnAttr that actually changes the rate emits
// a keyframe (measured 2026-09-01: 15 real changes, 15 IDRs, each on the
// next frame; 16 same-value writes, 0 IDRs), and that keyframe bypasses
// idr_rate_limit entirely. In flight-0000 an IDR cost a median 63.8 kB
// against a 21.4 kB base P frame — 38 ms of airtime at the rung it flew,
// p95 92 ms — and 151 of 226 bitrate writes had no rung change behind
// them. The blend was buying a few percent of airtime accuracy with ~150
// keyframes per 12-minute flight.
//
// So the measurement is gone and the share is a constant. kShareBase is
// the measured median: 561 one-second windows of flight-0000 gave p50
// 0.601, sd 0.044, and fixing it there costs -2.7%/+1.0% of target on the
// worst overhead pair in that flight (1.0/0.5). On an EQUAL pair the term
// cancels exactly, which is most rungs since the UEP flatten. The dropped
// framing excess is not compensated here: airtime_budget is the operator's
// lever if the resulting overshoot ever matters.
void RcAgent::run_bitrate_policy(uint64_t now_ms, bool force) {
  // Fixed base-layer byte share, replacing AirFeed's measured share_base.
  static constexpr double kShareBase = 0.60;

  last_policy_ms_ = now_ms;
  have_last_policy_ = true;
  // DELIVERED rates (air_rate.h): nominal × the measured per-MCS,
  // per-width air_clock.efficiency_20/_40, so airtime_budget is a fraction
  // of what the link actually moves. Priced off nominal until 2026-09-17,
  // which is why budget 0.6 sat at ~99 % of real mcs2 capacity and spiked
  // (docs/bandwidth-sweep-findings-2026-09-17.md).
  const double rate_b = delivered_mbps(applied_.ladder[0], cfg_.air_clock);
  // The probe stream has its own slot and is deliberately NOT a term here
  // — a probe costs zero encoder writes.
  const double rate_e = delivered_mbps(applied_.ladder[1], cfg_.air_clock);
  // The commanded pair (Task 6, RC_VERSION 5) IS the source — no single
  // ov to fan out to both terms.
  double ovb = applied_.fec_ov_base, ove = applied_.fec_ov_enh;
  // Debug-HTTP per-layer override active: the layers fly THESE overheads
  // (not the commanded pair), so the budget target must be built from them.
  // Still gated by ovr_ (tests construct RcAgent with none): the ternary's
  // -1 short-circuits the `>= 0` check exactly as a null ovr_ does.
  const int ob = ovr_ ? ovr_->ovr_base_pct.load(std::memory_order_relaxed) : -1;
  const int oe = ovr_ ? ovr_->ovr_enh_pct.load(std::memory_order_relaxed) : -1;
  if (ob >= 0 && oe >= 0) { ovb = ob / 100.0; ove = oe / 100.0; }
  // Airtime: V * [f0*(1+ovb)/rate_b + (1-f0)*(1+ove)/rate_e] = budget.
  // Two terms, not one, because BASE and ENH can carry different overhead
  // — that part of the 2-stream shape stays; only the measured inputs are
  // gone. (A probe's different MCS is deliberately NOT a term: see rate_e.)
  const double denom = kShareBase * (1.0 + ovb) / rate_b +
                       (1.0 - kShareBase) * (1.0 + ove) / rate_e;
  double kbps = 1000.0 * cfg_.encoder.airtime_budget / denom;
  // NOTE the encoder has its own floor below this one: venc's apply_bitrate
  // rails at VENC_BITRATE_MIN_KBPS (1000), so a configured bitrate_min_kbps
  // under 1000 is silently re-clamped there and this policy's "structural
  // minimum" would not be the one in force.
  kbps = std::clamp(kbps, static_cast<double>(cfg_.encoder.bitrate_min_kbps),
                     static_cast<double>(cfg_.encoder.bitrate_max_kbps));
  // Low-power cap (spec 2026-09-20): min(), so FAILSAFE's floor and the
  // cap compose; config validation keeps the cap inside the encoder clamp.
  if (low_power_active_)
    kbps = std::min(kbps, static_cast<double>(cfg_.low_power.bitrate_kbps));
  int kbps_i = round_to_100(kbps);

  bool decrease = have_last_bitrate_ && kbps_i < last_bitrate_kbps_;
  bool changed = !have_last_bitrate_ || kbps_i != last_bitrate_kbps_;
  bool throttled =
      have_last_bitrate_eval_ && now_ms - last_bitrate_eval_ms_ < 1000;
  // Every piece of "what the encoder is currently running" state is latched
  // ONLY when the verb reports success — including the throttle timestamp,
  // so a failed apply is not merely remembered as pending but is retried at
  // the first opportunity rather than one second later. A failed call
  // therefore leaves the agent exactly as it was, and the ordinary
  // changed/decrease logic re-issues the same value on the next policy
  // tick (i.e. the next RCF). Latching unconditionally would recreate the
  // waybeam wedge in-process: one dropped MI call and the encoder stays on
  // the old rate for the rest of the flight, because `changed` reads false
  // forever after.
  bool failed = false;
  // Frame rate, fps BEFORE bitrate: the bitrate write's IDR then seeds the
  // stream at the new rate. Only when the mode is configured on -- a
  // disabled mode never touches the encoder's boot fps. Latched on
  // success only, like every other verb; no 1 Hz throttle (that exists
  // for the bitrate hysteresis case).
  if (cfg_.low_power.enable) {
    const int target_fps = low_power_active_ ? cfg_.low_power.fps
                                             : static_cast<int>(cfg_.venc.core.fps);
    if (force || commanded_fps_ != target_fps) {
      if (act_.set_fps(target_fps)) commanded_fps_ = target_fps;
      else failed = true;
    }
  }
  if (force || decrease || (changed && !throttled)) {
    if (act_.set_bitrate_kbps(kbps_i)) {
      last_bitrate_kbps_ = kbps_i;
      have_last_bitrate_ = true;
      last_bitrate_eval_ms_ = now_ms;
      have_last_bitrate_eval_ = true;
    } else {
      failed = true;
    }
  }

  bool now_low = kbps_i < cfg_.encoder.roi_threshold_kbps;
  if (now_low != roi_low_) {
    // Same rule, same reason: flipping roi_low_ before knowing the encoder
    // took the QP would make the transition un-repeatable.
    if (act_.set_roi_qp(now_low ? cfg_.encoder.roi_qp_low : cfg_.encoder.roi_qp_normal))
      roi_low_ = now_low;
    else
      failed = true;
  }
  // Latched for tick()'s retry path. Cleared, not merely left alone, on a
  // clean run: the re-assert must go back to its 5 s cadence once the
  // encoder is taking values again, not retry on every tick forever.
  verb_apply_failed_ = failed;
}

// Escalates shed_level_ (0..3) by one step the instant tx_drops rises since
// the last tick (congestion is assumed the moment drops increase — no
// debounce on the way up), and decays it by exactly one step down per 2s
// clean window (2000ms with no further drop-rise); each step-down restarts
// the 2s window, so recovering from level 3 to level 0 takes three separate
// 2s clean windows in a row, not one. shed_level_ still ranges 0..3 for its
// own escalate/decay cadence, but the 2-stream space (spec
// 2026-08-29-airtime-balance-uep) leaves only one droppable layer (sid 1,
// enh) -- any level >= 1 sheds it and there is no second layer for level
// >= 2 to additionally shed (see reapply_with_shed()). Any level change
// reapplies the current op (shed folded in) so the change reaches the
// actuator immediately.
//
// Reaching level 3 also cut the encoder bitrate by 30% until 2026-08-28.
// That cut was removed: it wrote act_.set_bitrate_kbps(last * 0.7) straight
// to the actuator, bypassing run_bitrate_policy's
// clamp(bitrate_min_kbps, bitrate_max_kbps), and it compounded because it
// overwrote last_bitrate_kbps_ with its own output -- so each re-entry into
// level 3 multiplied the already-cut value (1300 -> 910 -> 637 -> 446 ->
// 312). An RCF repairs that within ~1s in LINKED, but tick() never calls
// run_bitrate_policy, so in FAILSAFE/RENDEZVOUS -- where the op is
// MAX_RANGE = mcs0 -- nothing restored it and the ratchet ran unopposed
// under the configured floor. run_bitrate_policy is now the ONLY writer of
// the encoder bitrate, so bitrate_min_kbps is a structural minimum
// (test 10e). Shedding layers remains the guard's whole job: bounded 0..3,
// self-decaying, no ratchet.
//
// Two triggers, OR'd:
//  - health.tx_drops = TxStats::failed (main.cpp): USB bulk-OUT
//    submission/completion FAILURES -- a sick dongle. Edge-triggered on the
//    counter rising.
//  - health.txq_depth >= txq_cap * kTxqShedNum/kTxqShedDen: TxQueue
//    backpressure -- over-driving the air. Level-triggered, because a full
//    queue is the condition itself, not an event. This is the one that
//    matters in flight: the encoder overshoots its command on a scene
//    change (17-22 Mb/s against 16 commanded, flight-0011 2026-09-03), the
//    queue runs to its cap, and drop-oldest throws bodies away with the
//    guard idle. Those drops are indistinguishable from RF loss at the GS
//    (expected symbol, never arrived), which books them as residual and
//    demotes -- 5->4->3->2 in 450 ms at 36 dB SNR, each step an IDR into a
//    queue that was already full. Shedding enh at half-cap lands BEFORE the
//    first drop, and a shed is invisible to the ladder (no s3 traffic, no
//    s3 decision -- LadderController::s3_usable). Half the cap is burst
//    headroom: one agg-6 feed batch is a few bodies, one IDR at rung 5 is
//    ~60-70 bodies; the tick is 100 ms and the queue drains 200+ bodies/s
//    at any rung, so a false shed costs one 2 s enh gap, a late one costs
//    the cascade. TxQueue drop-oldest (txq.drops) and RadioTx::drops()
//    stay telemetry-only: by the time either moves the damage is done.
void RcAgent::run_congestion_guard(uint64_t now_ms, const RadioHealth& health) {
  bool drops_rose = have_last_tx_drops_ && health.tx_drops > last_tx_drops_;
  if (!have_last_tx_drops_) drops_rose = health.tx_drops > 0;
  have_last_tx_drops_ = true;
  last_tx_drops_ = health.tx_drops;

  static constexpr size_t kTxqShedNum = 1, kTxqShedDen = 2;
  const bool txq_pressure =
      health.txq_cap > 0 &&
      health.txq_depth * kTxqShedDen >= health.txq_cap * kTxqShedNum;

  bool changed = false;
  if (drops_rose || txq_pressure) {
    last_drop_rise_ms_ = now_ms;
    have_last_drop_rise_ = true;
    if (shed_level_ < 3) {
      ++shed_level_;
      changed = true;
    }
  } else if (have_last_drop_rise_ && now_ms - last_drop_rise_ms_ >= 2000) {
    if (shed_level_ > 0) {
      --shed_level_;
      changed = true;
      last_drop_rise_ms_ = now_ms;  // restart the 2s window for the next step-down
    }
  }

  if (changed) reapply_with_shed();
}

void RcAgent::on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms) {
  int type = rc::frame_type(body, len);
  if (type == rc::T_DISC) {
    auto d = rc::parse_disc(body, len);
    if (!d.has_value()) return;
    // Auto channel select (spec 2026-10-03-auto-channel-set §2/§6): a DISC
    // is a proposal (Disc.op_channel), not a command -- we agree to it when
    // it's a member of our channel set; a non-member is answered with the
    // current channel instead (the GS logs ack_override). The agreement
    // rides the ack and the pending pair; the move itself happens only once
    // that pair is promoted (see T_RCF).
    const uint8_t agreed = member_(d->op_channel) ? d->op_channel : channel_;
    if (!rc::verify_control(body, len, cfg_.link.key, rc::TagCtx{})) {
      // Wrong key on the GS. Answer so the GS can SHOW it (spec §8) instead
      // of looking like the stale-caps deadlock; change nothing else.
      auth_reject_.store(true, std::memory_order_relaxed);
      act_.send_control(rc::pack_disc_ack(
          make_disc_ack(d->vrx_nonce, 0, rc::kAckKeyMismatch, d->seq, agreed)));
      return;
    }
    // A DISC only elicits an ack (spec §6): no op, no retune, no LINKED
    // entry, no watchdog refresh. The first RCF that verifies under the
    // acked pair does those. The ack itself still goes out on every DISC,
    // LINKED included: a rebooted GS's keep-alive is the only way it
    // re-learns chip_caps (stale-caps deadlock, 2026-08-12).
    uint32_t vtx;
    if (state_ == State::LINKED && current_.valid && d->vrx_nonce == current_.vrx_nonce) {
      vtx = current_.vtx_nonce;                       // keep-alive: same answer
    } else if (pending_.valid && d->vrx_nonce == pending_.vrx_nonce) {
      vtx = pending_.vtx_nonce;                       // lost-ack retry: same answer
      pending_.agreed_ch = agreed;
    } else {
      // New GS process, or our own GS heard again while we are NOT linked
      // (failsafe/rendezvous): a fresh pair, so its seq32 restarts on both
      // ends (the GS resets on a NEW vtx_nonce; a > 1 h gap would otherwise
      // desync the wrap count).
      vtx = fresh_vtx_nonce_();
      pending_ = Session{d->vrx_nonce, vtx, true, 0, false, agreed};
    }
    act_.send_control(rc::pack_disc_ack(make_disc_ack(d->vrx_nonce, vtx, 0, d->seq, agreed)));
    return;
  }

  if (type == rc::T_RCF) {
    auto r = rc::parse_rcf(body, len);
    if (!r.has_value()) return;
    uint32_t seq32 = 0;
    if (verify_rcf_(body, len, *r, current_, &seq32)) {
      // in-session
    } else if (verify_rcf_(body, len, *r, pending_, &seq32)) {
      // Promotion (spec §6 step 4): pending becomes current. A session swap
      // while LINKED touches nothing but the pair (op-thrash rule); from
      // any other state this RCF's own apply path below enters LINKED.
      current_ = pending_;
      pending_ = Session{};
      publish_session_();
      session_promoted_ = true;
      // Session boundary: a new GS session's hop epoch and IDR epoch
      // numbering start over, so the old session's latches must not
      // swallow its first hop order or IDR request.
      have_hop_ = false;
      hop_epoch_ = 0;
      hop_ch_ = 0;
      idr_epoch_seen_ = 0;
      idr_gs_pending_ = false;
      if (current_.agreed_ch != 0 && current_.agreed_ch != channel_)
        deferred_move_ch_ = current_.agreed_ch;          // after main's promote Telem
    } else {
      // Wrong key, wrong pair, stale or replayed seq: drop, flag for the
      // next Telem, never move the seq tracker. No log (spec §7).
      auth_reject_.store(true, std::memory_order_relaxed);
      return;
    }
    current_.last_seq32 = seq32;
    current_.have_seq = true;
    ++rcf_accepted_;
    // Any accepted RCF confirms an in-flight move (spec §6: "the first GS
    // frame received after the move confirms it") -- it arrived on the
    // channel we retuned to, so there's nothing left for tick()'s fallback
    // to guard against.
    if (move_pending_) act_.remember_channel(channel_);   // the GS heard us here: persist
    move_pending_ = false;

    PhyMode mode;
    uint8_t mcs, bw;
    rc::decode_profile(r->profile, mode, mcs, bw);
    auto ladder = rc::ladder_from(mode, mcs, bw, cfg_.radio.ldpc);

    State prev_state = state_;
    apply_ladder_op(ladder, r->fec_overhead_base, r->fec_overhead_enh, r->probe_profile);

    // In-flight hop order (spec 2026-09-14 §1): a NEW (epoch, ch) PAIR moves
    // us; the same pair again is a no-op; hop_ch 0 is a pre-hop GS. The order
    // in this very RCF must not be confirmed by itself, so move_pending_ is
    // re-armed AFTER the clear above; the next RCF heard on the new channel
    // clears it, and move_confirm_ms sends us home if none arrives.
    if (r->hop_ch != 0 &&
        (!have_hop_ || r->hop_epoch != hop_epoch_ || r->hop_ch != hop_ch_)) {
      have_hop_ = true;
      hop_epoch_ = r->hop_epoch;
      hop_ch_ = r->hop_ch;
      // The pair is still recorded above regardless, so a non-member order
      // is not re-evaluated on every repeat of the same (epoch, ch).
      if (member_(r->hop_ch) && r->hop_ch != channel_) {
        act_.retune(r->hop_ch, "hop");
        move_from_ch_ = channel_;
        channel_ = r->hop_ch;
        move_pending_ = true;
        move_at_ms_ = now_ms;
      }
    }

    // VTX recorder wish (spec 2026-09-26). Unknown (maburgs just restarted,
    // no player message yet) leaves the recorder alone; only a received
    // known-off stops it. Latched on success, retried on the next RCF.
    if (r->rec & rc::kRecKnown) {
      const int want = (r->rec & rc::kRecOn) ? 1 : 0;
      if (want != rec_applied_ && act_.set_record(want == 1)) rec_applied_ = want;
    }

    // GS-requested IDR (spec 2026-09-28): a CHANGED epoch is a request.
    // Marked seen on receipt -- pending stays set until tick() serves it, so
    // any number of bumps collapse into one IDR. No silent adopt of the
    // session's first epoch: a DISC link-up sends no IDR, and a page that
    // asked before the link came up needs one.
    if (r->idr_epoch != idr_epoch_seen_) {
      idr_epoch_seen_ = r->idr_epoch;
      idr_gs_pending_ = true;
    }

    if (prev_state == State::BOOT || prev_state == State::RENDEZVOUS)
      link_established_ = true;
    state_ = State::LINKED;
    last_fb_ms_ = now_ms;
    have_last_fb_ = true;

    bool entering_linked = prev_state == State::FAILSAFE || prev_state == State::RENDEZVOUS;
    // Through the shared pacer (§4) like every other IDR producer. The
    // 100 ms floor cannot suppress this in practice — re-entering LINKED
    // requires having spent at least failsafe_ms (>= 1 s) outside it — but
    // routing it here is what makes "RcAgent is the only IDR authority"
    // true rather than aspirational.
    if (entering_linked && idr_due(now_ms, /*chain=*/false)) {
      act_.request_idr();
      // This link-up IDR already covers any GS request pending on this same
      // RCF (spec 2026-09-28 fix round 1): FAILSAFE entry resets
      // idr_epoch_seen_ to 0, so the first RCF back reads the page's
      // (unchanged) epoch as a fresh change and arms idr_gs_pending_ right
      // above -- without this, that pending request survives to the next
      // tick and fires a redundant second IDR ~100 ms later. Leave
      // idr_epoch_seen_ alone: the epoch itself is still correctly seen, so
      // the same epoch in a later RCF stays "not a new request".
      idr_gs_pending_ = false;
    }

    // RCFs that transition into LINKED (from RENDEZVOUS or FAILSAFE) force
    // the bitrate policy so the new operating point's bitrate takes effect
    // immediately; steady-state LINKED RCFs go through the normal
    // throttle+hysteresis gate.
    run_bitrate_policy(now_ms, /*force=*/entering_linked);
    return;
  }
  // Anything else (DISC_ACK, unknown/corrupt) is ignored.
}

void RcAgent::tick(uint64_t now_ms, const RadioHealth& health) {
  // A promoted session's agreed channel (spec 2026-10-01 §6 step 5). Run
  // here, not on the promoting RCF, so main has sent one Telem saying
  // LINKED on the CURRENT channel first -- the GS follows on it.
  if (deferred_move_ch_ != 0) {
    const uint8_t ch = deferred_move_ch_;
    deferred_move_ch_ = 0;
    act_.retune(ch, "disc");
    move_from_ch_ = channel_;
    channel_ = ch;
    move_pending_ = true;
    move_at_ms_ = now_ms;
  }
  if (state_ == State::BOOT) {
    apply_max_range(now_ms);
    state_ = State::RENDEZVOUS;
    return;
  }

  if (move_pending_ && now_ms - move_at_ms_ >= static_cast<uint64_t>(cfg_.link.move_confirm_ms)) {
    // Unconfirmed move (spec 2026-10-03 §3): nothing from the GS on the new
    // channel. Back to the channel we came from -- where the GS found us,
    // or where a withdrawing GS still is -- and stay there. There is no
    // home to fall through to.
    if (move_from_ch_ != 0 && move_from_ch_ != channel_) {
      act_.retune(move_from_ch_, "move_unconfirmed");
      channel_ = move_from_ch_;
    }
    move_from_ch_ = 0;
    move_pending_ = false;
    if (state_ == State::LINKED) apply_max_range(now_ms);
    state_ = State::RENDEZVOUS;
    clear_sessions_();
    have_hop_ = false;
    hop_epoch_ = 0;
    hop_ch_ = 0;
    idr_epoch_seen_ = 0;
    idr_gs_pending_ = false;
  }

  // Chain-break intake, evaluated against the state as of this tick's ENTRY
  // — deliberately ahead of the failsafe/rendezvous timers below. The
  // encoder broke its reference chain while the link was up, and the tick
  // that notices a missed feedback deadline is the tick most likely to be
  // carrying that break; deferring the heal until after the state machine
  // has moved us to FAILSAFE would silently discard exactly the IDR that
  // matters. The LINKED gate stays, so a break raised while genuinely
  // unlinked is consumed and dropped rather than queued — affordable only
  // because the encoder's own GOP is the backstop: at the shipped
  // venc.gop_s = 0.5 (bundle/mabur.default.toml since b05c60f) an unhealed
  // chain break self-clears within ~0.5 s. That backstop is only as short as
  // gop_s: raise it and the safety net stretches with it, at which point
  // dropping refused requests instead of deferring them needs re-arguing.
  if (chain_break_pending_.exchange(false, std::memory_order_relaxed) &&
      state_ == State::LINKED && idr_due(now_ms, /*chain=*/true)) {
    act_.request_idr();
  }

  // GS-requested IDR (spec 2026-09-28), same tick-entry state rule as the
  // chain-break consumer above. Unlike a chain break, a refused request is
  // DEFERRED (pending survives to the next tick): the requester is frozen
  // and waiting, and dropping would cost it a full retry interval. Still
  // bounded by idr_due's 100 ms floor.
  if (idr_gs_pending_ && state_ == State::LINKED && idr_due(now_ms, /*chain=*/false)) {
    act_.request_idr();
    idr_gs_pending_ = false;
    ++idr_gs_total_;
  }

  if (state_ == State::LINKED) {
    if (have_last_fb_ && now_ms - last_fb_ms_ >= static_cast<uint64_t>(cfg_.link.failsafe_ms)) {
      apply_max_range(now_ms);
      state_ = State::FAILSAFE;
      // Failsafe entry clears both sessions (spec 2026-10-01 §7): a kept
      // pair would leave every RCF the GS sent during the fade replayable
      // for the life of the process. Recovery: the GS's next keep-alive
      // DISC gets a fresh pair; its RCFs set auth_reject until then
      // (<= ~1 s, one beacon_keepalive_ms).
      clear_sessions_();
      have_hop_ = false;
      hop_epoch_ = 0;
      hop_ch_ = 0;
      idr_epoch_seen_ = 0;
      idr_gs_pending_ = false;
      // Rebase the rendezvous_ms timer from the moment failsafe was
      // entered (not the last real feedback), so a link silent since t=0
      // with failsafe_ms=1000/rendezvous_ms=30000 falls back to
      // RENDEZVOUS at t=1000+30000=31000, not t=30000.
      last_fb_ms_ = now_ms;
    }
  } else if (state_ == State::FAILSAFE) {
    if (have_last_fb_ &&
        now_ms - last_fb_ms_ >= static_cast<uint64_t>(cfg_.link.rendezvous_ms)) {
      state_ = State::RENDEZVOUS;
    }
  }

  // Arm-state intake sits HERE — below the unconfirmed-move and
  // failsafe-entry branches, directly above the re-assert — so that its
  // forced policy run is the LAST one of the tick. Both of those branches
  // call apply_max_range() -> run_bitrate_policy(force=true) with no
  // ran_this_tick guard of their own, so an arm change coincident with one
  // of them still costs two policy runs in the same millisecond; what this
  // ordering buys is that the run which lands on the encoder last is the one
  // carrying BOTH the new operating point and the new low-power state, and
  // that its last_policy_ms_ stamp makes the re-assert below a no-op for
  // this tick. (Collapsing the coincident pair into one write would mean
  // taking the policy call out of apply_max_range — a restructure, not a
  // fix.) The intake must still run before the re-assert: on an ordinary
  // tick nothing else has run the policy, and the re-assert must see the
  // arm state this tick, not next.
  intake_arm_state_(now_ms);

  // Periodic re-assert of the encoder verbs (kReassertMs). run_bitrate_policy
  // otherwise runs ONLY on an RCF, a DISC, or a max-range entry, which makes
  // "retry on the next policy tick" a promise the agent cannot keep in the
  // states where it matters most: in FAILSAFE there are no RCFs by
  // definition, so a verb that failed on the failsafe entry itself stayed
  // failed for up to rendezvous_ms (30 s) with the encoder flooding a
  // mcs0-sized pipe at the previous rung's rate. The same gap let anything
  // that wrote the encoder behind RcAgent's back — the debug endpoint's
  // POST /venc/set?bitrate= above all — persist until the ladder happened to
  // change rung.
  //
  // force=true is required, not incidental: with force=false an unchanged
  // computed target is a deliberate no-op (the changed/decrease gate), so a
  // re-assert would send nothing at all — which is the entire bug. force
  // re-applies the CURRENT computed target, including FAILSAFE's floor
  // (applied_ is the max-range op while in FAILSAFE, so the value stamped
  // over is the correct one for the state, never a stale LINKED rate).
  //
  // Cadence: kReassertMs since the last bitrate the encoder actually ACCEPTED
  // (last_bitrate_eval_ms_ is latched only inside the success branch), so a
  // busy LINKED link that keeps genuinely changing rung never adds a
  // re-assert on top, while a parked one gets exactly one every 5 s. A failed
  // verb short-circuits to the next tick.
  //
  // RENDEZVOUS is excluded from the CADENCE: it is the pre-link state where
  // no GS has been heard from, MAX_RANGE was applied once on the BOOT tick,
  // and there is nothing to defend the value against. verb_apply_failed_
  // now retries in RENDEZVOUS too -- a refused low-power transition verb on
  // the ground has no RCF to carry its retry (spec 2026-09-20).
  //
  // ran_this_tick keeps it to at most one policy run per distinct now_ms: a
  // tick that has ALREADY run the policy (the failsafe entry a few lines up,
  // or an RCF the agent loop drained at the same millisecond) must not
  // immediately run it again — most visibly when that run FAILED, where the
  // retry belongs on the next tick, not back-to-back on this one.
  bool ran_this_tick = have_last_policy_ && last_policy_ms_ == now_ms;
  if (!ran_this_tick) {
    const bool defended = state_ == State::LINKED || state_ == State::FAILSAFE;
    bool reassert_due = verb_apply_failed_ ||
                        (defended && (!have_last_bitrate_eval_ ||
                                      now_ms - last_bitrate_eval_ms_ >= kReassertMs));
    if (reassert_due) run_bitrate_policy(now_ms, /*force=*/true);
  }

  run_congestion_guard(now_ms, health);
}

rc::DiscAck RcAgent::make_disc_ack(uint32_t vrx_nonce, uint32_t vtx_nonce, uint8_t flags,
                                   uint16_t seq, uint8_t agreed) const {
  DiscAck ack;
  ack.vrx_nonce = vrx_nonce;
  ack.vtx_nonce = vtx_nonce;
  ack.flags = flags;
  // Frame wire is the only video format maburd speaks; the bit stays on the
  // wire (one legal value) so a GS can still refuse a peer that lacks it.
  // CAP_TELEMETRY: this drone also sends T_TELEM frames on its uplink
  // (spec 2026-07-26 drone-telemetry) — display-grade only, not a gate.
  // CAP_CALIBRATE: this build understands T_CAL_CMD/T_CAL_RESULT (Task 11
  // wires them up in main.cpp) -- a real gate, unlike CAP_TELEMETRY:
  // gs/src/cal_session.cpp's start() refuses a session outright without it.
  ack.chip_caps = rc::CAP_FRAME_WIRE | rc::CAP_TELEMETRY | rc::CAP_CALIBRATE;
  // agreed is member_(op_channel) ? the DISC's proposed op_channel : the
  // current channel (spec 2026-10-03-auto-channel-set §2/§6) -- computed by
  // the caller, which also stores it in the pending pair whose promotion
  // drives the actual retune, so the ack and the move can never disagree
  // about what was agreed to.
  ack.agreed_channel = agreed;
  ack.agreed_width = cfg_.radio.width;
  ack.seq = seq;
  return ack;
}

}  // namespace mabur
