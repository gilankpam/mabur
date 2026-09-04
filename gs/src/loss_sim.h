#pragma once
#include <cstdint>

namespace maburgs {

// BENCH RIG — compiled only under MABUR_LOSS_SIM (CMake option, default
// OFF; prod maburgs contains none of this). Mainlined 2026-08-29 so
// resilience gates can build it with -DMABUR_LOSS_SIM=ON instead of
// merging a scaffolding branch.
//
// Injected per-stream packet loss for eyeballing SVC-T degradation: dial loss
// on s3 (the enhance layer) and watch the video while the rest of the link
// runs for real. Two-state Gilbert-Elliott per (card, stream), so loss can be
// bursty rather than an independent coin flip — the w32 sliding window is far
// more sensitive to bursts, and Bernoulli loss flatters it.
//
// Given target steady-state loss L and mean burst length B (consecutive lost
// bodies):
//   r = P(BAD->GOOD) = 1/B          -> mean run in BAD is geometric, 1/r = B
//   p = P(GOOD->BAD) = r*L/(1-L)    -> steady-state P(BAD) = p/(p+r) = L
// B == 1 collapses to a plain per-body coin flip at rate L.
//
// burst <= 1 fast path: at B == 1, r == 1 always (a geometric distribution
// only has mean exactly 1 if its success probability is exactly 1), and for
// L >= 0.5 the formula above then drives p to exactly 1 too. Once a
// transition probability is exactly 1, "u < p" with u drawn from [0, 1) is
// unconditionally true, so the two-state machine stops depending on the RNG
// at all and degenerates into a deterministic GOOD/BAD alternation shared by
// every (card, sid) pair regardless of its own LCG stream (verified: this is
// what "loss == 1.0" and "loss == 0.5, burst == 1.0" both hit). For B == 1
// the two-state machinery adds nothing anyway — a BAD state can never
// survive to the next call — so should_drop() bypasses it and draws directly
// against the target rate: a literal per-body coin flip, which is exactly
// what the model above claims B == 1 reduces to, without a p/r pair that can
// saturate.
//
// Feasibility (B > 1): a chain with recovery rate r = 1/B can reach at most
// steady-state loss B/(B+1) before p would need to exceed 1 — beyond that,
// "u < p" saturates the same way and the delivered loss silently caps at
// B/(B+1) no matter how high L is dialed (e.g. loss=0.90, burst=3 would
// silently deliver ~0.75). configure() refuses to lie about this: it keeps
// the requested loss exact and raises burst to the minimum feasible value
// instead (the value that makes p land on exactly 1.0), and burst() reports
// that raised value truthfully rather than the operator's original request.
// Below the B/(B+1) threshold, burst passes through unchanged.
//
// State is per (card, stream): real fading is independent per card, which is
// what the multi-card union exists to exploit. Two cards at 20% each yield
// ~4% union loss — that is the behaviour under test, not a bug. It does mean
// the rate set here is NOT the rate the FEC decoder sees, so LossControl
// reports both and offers `eff=` to dial the union rate directly; see the
// nominal-vs-measured caveat in loss_control.h before writing any number
// from this rig into a findings document.
//
// Deterministic (fixed seed, LCG per pair) so a sweep is reproducible.
// Not thread-safe: lives in Aggregator, which is core-thread-only by contract.
class LossSim {
 public:
  // sids 0..5: 0/1 are the video BASE/ENH streams, 2/3 are unused since the
  // 4->2 stream collapse, 4 is MSP, 5 is the probe canary (spec 2026-09-04).
  static constexpr int kStreams = 6;
  static constexpr int kMaxCards = 8;

  // loss clamps to [0, 1]; burst clamps to >= 1. Out-of-range sid is ignored.
  // Setting loss to 0 disables that stream (and resets its burst to 1).
  void configure(int sid, double loss, double burst) {
    if (sid < 0 || sid >= kStreams) return;
    if (loss < 0.0) loss = 0.0;
    if (loss > 1.0) loss = 1.0;
    if (burst < 1.0) burst = 1.0;
    // Feasibility: a chain with recovery rate r = 1/burst can only reach
    // steady-state loss up to burst/(1+burst) before p = r*L/(1-L) would
    // need to exceed 1. Above that threshold, keep the requested loss exact
    // and raise burst to the minimum value that makes it feasible --
    // substituting burst = loss/(1-loss) back into burst/(1+burst) gives
    // exactly `loss`, so p lands on exactly 1.0 instead of silently
    // saturating there while under-delivering the requested loss.
    if (loss > 0.0 && loss < 1.0 && loss > burst / (1.0 + burst)) {
      burst = loss / (1.0 - loss);
    }
    Cfg& c = cfg_[sid];
    c.loss = loss;
    c.burst = loss > 0.0 ? burst : 1.0;
    // loss == 1 would divide by zero in the p formula; it means "always
    // BAD", so p = 1 and r = 0 -- the BAD state, once entered, is never
    // left. (For burst == 1 this is moot: should_drop()'s fast path bypasses
    // r/p entirely.)
    c.r = loss >= 1.0 ? 0.0 : 1.0 / c.burst;
    c.p = loss >= 1.0 ? 1.0 : c.r * loss / (1.0 - loss);
    // Re-entering a stream mid-sweep starts from a fixed state so each step
    // of a sweep begins the same way regardless of where the previous one
    // ended: GOOD in general, but BAD when loss == 1.0. That special case
    // matters because r == 0 there means BAD is never left once entered --
    // if the initial state stayed GOOD, the very first body would survive
    // (state is read before it transitions) and every body afterward would
    // still drop, silently missing "always drop" by exactly one body.
    for (int card = 0; card < kMaxCards; ++card) {
      st_[card][sid].bad = (loss >= 1.0);
    }
    any_ = false;
    for (int s = 0; s < kStreams; ++s) if (cfg_[s].loss > 0.0) any_ = true;
  }

  bool enabled() const { return any_; }
  double loss(int sid) const {
    return (sid >= 0 && sid < kStreams) ? cfg_[sid].loss : 0.0;
  }
  double burst(int sid) const {
    return (sid >= 0 && sid < kStreams) ? cfg_[sid].burst : 0.0;
  }
  uint64_t dropped(int sid) const {
    return (sid >= 0 && sid < kStreams) ? dropped_[sid] : 0;
  }

  // Decides this body's fate from the CURRENT state, then transitions. That
  // order is what makes steady-state P(drop) equal the configured loss.
  bool should_drop(int card, int sid) {
    if (!any_) return false;
    if (sid < 0 || sid >= kStreams) return false;
    if (card < 0 || card >= kMaxCards) return false;
    const Cfg& c = cfg_[sid];
    if (c.loss <= 0.0) return false;
    St& s = st_[card][sid];
    if (!s.seeded) {
      // Distinct nonzero start per (card, sid) so the streams never lock step.
      s.lcg = 0x9E3779B9u + static_cast<uint32_t>(card * kStreams + sid) * 2654435761u;
      s.seeded = true;
    }
    // burst == 1 means r == 1: every BAD run is exactly one body long, so the
    // GOOD->BAD draw is the only source of entropy left. At loss >= 0.5 the
    // formula in configure() drives p to exactly 1 too (r*L/(1-L) == 1 at
    // L == 0.5), and u is drawn from [0, 1) -- strictly less than any p == 1
    // -- so "u < p" is unconditionally true. With every (card, sid) pair
    // reset to GOOD in configure(), that turns should_drop into a
    // deterministic GOOD/BAD/GOOD/BAD.. alternation shared by every card
    // regardless of its own LCG stream: loss_one_drops_everything sees every
    // other call return false instead of true, and cards_are_independent
    // (loss 0.50) sees the two cards move in lockstep instead of ~L^2
    // jointly. Both are the same underlying loss of entropy, so both are
    // fixed the same way: for burst == 1 the two-state machinery adds
    // nothing (a BAD state can never survive to the next call anyway), so
    // draw straight from this pair's own stream against the target rate --
    // a literal per-body coin flip, matching the model's documented B == 1
    // special case, without going through a p/r pair that can saturate.
    if (c.burst <= 1.0) {
      const bool drop = unit(s.lcg) < c.loss;
      if (drop) ++dropped_[sid];
      return drop;
    }
    const bool drop = s.bad;
    const double u = unit(s.lcg);
    if (s.bad) { if (u < c.r) s.bad = false; }
    else       { if (u < c.p) s.bad = true;  }
    if (drop) ++dropped_[sid];
    return drop;
  }

 private:
  struct Cfg { double loss = 0.0, burst = 1.0, p = 0.0, r = 1.0; };
  struct St  { bool bad = false, seeded = false; uint32_t lcg = 0; };

  // Same LCG constants as FrameFileSource::card_drops, but drawing 24 bits so
  // the resolution is finer than that one's whole percent.
  static double unit(uint32_t& s) {
    s = s * 1664525u + 1013904223u;
    return static_cast<double>((s >> 8) & 0xFFFFFFu) / 16777216.0;
  }

  Cfg cfg_[kStreams];
  St st_[kMaxCards][kStreams];
  uint64_t dropped_[kStreams] = {};
  bool any_ = false;
};

}  // namespace maburgs
