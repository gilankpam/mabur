// The rung-transition edge-detect wiring (gs/src/transition_edge.h): which
// instant-demote windows get settle-blanked at an op change. Before
// 2026-09-05 only the base (s1) residual window was blanked; the enh (s3)
// window kept the abandonment-horizon's late booking of old-rung loss and
// re-fired the demote the tick the controller's s3_settle_ms gate opened
// (flights 20/21: 13 of 14 s3_residual cascades double-stepped at exactly
// 300 ms and were promoted straight back). The dry-run e2e never runs the
// ladder, so this is the host-side pin for that wiring.
#include "mtest.h"
#include "transition_edge.h"
#include "mabur/uep_encoder.h"
using namespace maburgs;

namespace {

mabur::UepDecoder make_dec() {
  std::array<mabur::UepLayerCfg, 2> layers{};
  for (auto& l : layers) {
    l.fec = mabur::SwConfig{512, 8, 0.5};
    l.blocks_per_body = 1;
  }
  return mabur::UepDecoder(layers);
}

OpPoint op_at(int mcs, double ov = 1.0) {
  OpPoint o;
  o.mcs = mcs;
  o.overhead_base = ov;
  return o;
}

// Old-rung loss sitting in a window: 40 % abandoned, would demote for 500 ms.
void book_loss(S1LossWindow& w, double t) {
  w.add(0, 0, t - 100.0);
  w.add(100, 60, t);
}

}  // namespace

TEST(mcs_edge_blanks_both_demote_windows) {
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500);
  TransitionEdge edge;
  CHECK(edge.on_tick(op_at(5), dec, s1, s3, 1000.0));  // first sight arms both
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(s1.sample(2000.0).valid && s1.sample(2000.0).loss > 0.0);
  CHECK(s3.sample(2000.0).valid && s3.sample(2000.0).loss > 0.0);

  // Demote 5 -> 4 at t=2050: the debris must be gone from BOTH decision
  // inputs at once -- s3 was the one left dirty until 2026-09-05.
  CHECK(edge.on_tick(op_at(4), dec, s1, s3, 2050.0));
  CHECK(!s1.sample(2050.0).valid);
  CHECK(!s3.sample(2050.0).valid);

  // Horizon-lag booking of old-rung loss inside the settle is swallowed on
  // both, and after the settle the window is still empty (this is what the
  // controller's 300 ms s3 gate then reads: nothing, not the old rung).
  s1.add(200, 120, 2100.0);
  s3.add(200, 120, 2100.0);
  CHECK(!s1.sample(2350.0).valid);
  CHECK(!s3.sample(2350.0).valid);

  // Fresh current-rung loss after the settle still demotes.
  s3.add(300, 170, 2400.0);
  auto fresh = s3.sample(2400.0);
  CHECK(fresh.valid);
  CHECK(fresh.loss > 0.49 && fresh.loss < 0.51);
}

TEST(overhead_only_step_blanks_base_not_enh) {
  // An FEC re-key without a PHY change is a sid-0 edge only (same-MCS
  // fallback on the decoder); the enh layer keeps measuring.
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500);
  TransitionEdge edge;
  edge.on_tick(op_at(4, 1.0), dec, s1, s3, 1000.0);
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(edge.on_tick(op_at(4, 0.5), dec, s1, s3, 2050.0));
  CHECK(!s1.sample(2050.0).valid);
  CHECK(s3.sample(2050.0).valid);
}

TEST(no_change_never_blanks) {
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500);
  TransitionEdge edge;
  edge.on_tick(op_at(3), dec, s1, s3, 1000.0);
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(!edge.on_tick(op_at(3), dec, s1, s3, 2050.0));  // static pin / steady
  CHECK(s1.sample(2050.0).valid);
  CHECK(s3.sample(2050.0).valid);
}

TEST(settle_is_shorter_than_the_s3_gate) {
  // The controller refuses to read s3 for s3_settle_ms (300); the window
  // blank must expire before that, or a genuine continuing fade could never
  // be seen at the gate's first read.
  CHECK(TransitionEdge::kResidSettleMs < 300.0);
}

MTEST_MAIN
