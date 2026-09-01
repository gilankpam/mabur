#include "ladder_residual.h"

namespace maburgs {

// The BASE layer's post-FEC loss, from the FEC decoder's own abandonment
// count -- symbols that fell below the sliding-window horizon still unknown
// (SwDecoder::advance). Current-rung only: syms_abandoned_stale is the
// pre-transition debris the attribution watermark already accounted for.
//
// NOT the packet-level delivery window (window_counts_cur), which this
// replaced on 2026-09-02. That measure inferred loss from FRAG-seq gaps, and
// a gap is indistinguishable from a unit that merely has not completed yet:
// a sliding-window repair landing late completes an older unit after a newer
// one, and the next forward gap re-books the already-delivered unit as a
// fresh expectation. In a window holding 3-6 units that reads as 0.25 or
// 0.33 residual, and block 4 demotes on anything > 0 -- 31 spurious demotes
// in ~7 min on the bench with abn=0 the whole time. Symbol abandonment is
// pure seq arithmetic over a span and cannot be fooled by arrival order.
//
// The trade is detection lag: a lost symbol is booked only once the horizon
// passes it (seq_horizon 512 at ~6.4k sym/s is ~80 ms), where the packet
// measure fired on the next forward gap. The ENH layer's demote path has
// always accepted exactly this (see s3_resid_cur in gs/src/main.cpp).
//
// BASE only, deliberately: the packet measure pooled sid 0 and sid 1, a
// leftover from the 4-stream era when sid 0/1/2 were all base layers (see
// the "never-shed base layers" note on window_counts). Since the 2-stream
// flatten sid 1 is the SHED-ABLE enh layer, which has its own demote path
// in block 5a; pooling it here let enh loss demote through both.
ResidualCounts residual_counts(const mabur::UepDecoder& dec, int sid,
                               bool cur) {
  const auto s = dec.stats(sid);
  const uint64_t abandoned =
      cur ? s.syms_abandoned - s.syms_abandoned_stale : s.syms_abandoned;
  const uint64_t expected = s.syms_delivered + s.syms_recovered + abandoned;
  return ResidualCounts{expected - abandoned, expected};
}

ResidualCounts residual_counts_pooled(const mabur::UepDecoder& dec, bool cur) {
  const auto base = residual_counts(dec, 0, cur);
  const auto enh = residual_counts(dec, 1, cur);
  return ResidualCounts{base.arrived + enh.arrived,
                        base.expected + enh.expected};
}

int delivery_pct(ResidualCounts rc) {
  if (rc.expected == 0) return 100;  // idle layer is not a 0%-delivered layer
  const uint64_t pct = rc.arrived * 100 / rc.expected;
  return static_cast<int>(pct > 100 ? 100 : pct);
}

ResidualCounts ladder_residual_counts(const mabur::UepDecoder& dec) {
  return residual_counts(dec, 0, /*cur=*/true);
}

}  // namespace maburgs
