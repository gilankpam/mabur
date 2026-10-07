// The rung-transition edge-detect wiring (gs/src/transition_edge.h): which
// instant-demote windows get settle-blanked at an op change. Before
// 2026-09-05 only the base (s1) residual window was blanked; the enh (s3)
// window kept the abandonment-horizon's late booking of old-rung loss and
// re-fired the demote the tick the controller's s3_settle_ms gate opened
// (flights 20/21: 13 of 14 s3_residual cascades double-stepped at exactly
// 300 ms and were promoted straight back). The dry-run e2e never runs the
// ladder, so this is the host-side pin for that wiring.
// The util windows left the edge on 2026-09-05 (arrival tracker) and came
// back on 2026-10-06 (flight 0026): attribution kept debris out of them,
// but nothing emptied the 500 ms of legitimately booked old-rung loss the
// cascade then re-decided on. They are cleared, not settle-blanked.
#include <type_traits>

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

OpPoint op_at(int mcs, double ov = 1.0, int bw = 20) {
  OpPoint o;
  o.mcs = mcs;
  o.overhead_base = ov;
  o.bw = bw;
  return o;
}

// Old-rung loss sitting in a window: 40 % abandoned, would demote for 500 ms.
void book_loss(S1LossWindow& w, double t) {
  w.add(0, 0, t - 100.0);
  w.add(100, 60, t);
}

}  // namespace

TEST(mcs_edge_blanks_both_residual_windows) {
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500), u1(500), u3(500);
  TransitionEdge edge;
  CHECK(edge.on_tick(op_at(5), dec, s1, s3, u1, u3, 1000.0));  // first sight arms both
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(s1.sample(2000.0).valid && s1.sample(2000.0).loss > 0.0);
  CHECK(s3.sample(2000.0).valid && s3.sample(2000.0).loss > 0.0);

  // Demote 5 -> 4 at t=2050: both residual inputs cleared at once.
  CHECK(edge.on_tick(op_at(4), dec, s1, s3, u1, u3, 2050.0));
  CHECK(!s1.sample(2050.0).valid);
  CHECK(!s3.sample(2050.0).valid);

  // Horizon-lag booking of old-rung abandonment inside the settle is
  // swallowed, and after the settle the window is still empty.
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

TEST(mcs_edge_clears_both_util_windows_without_a_swallow) {
  // Flight 0026 (2026-10-06): the 500 ms util windows were never cleared at
  // a transition, only the residual ones, so in the fade regime (100 ms
  // confirm, 150 ms spacing) one real demote kept stepping every 150 ms on
  // the same old-rung bookings -- u collapsed to 0 in one tick exactly
  // 500 ms after the booking, two or three rungs lower. The edge now clears
  // the two util windows too. No settle swallow: the arrival tracker's
  // open boundary already books every post-edge old-rung seq as stale, so
  // the window must start taking the new rung's entries at once.
  TransitionEdge e;
  auto dec = make_dec();
  S1LossWindow s1r, s3r, s1u, s3u;
  e.on_tick(op_at(5), dec, s1r, s3r, s1u, s3u, 0.0);
  book_loss(s1u, 1000.0);
  book_loss(s3u, 1000.0);
  CHECK(s1u.sample(1000.0).valid && s1u.sample(1000.0).loss > 0.3);
  CHECK(e.on_tick(op_at(4), dec, s1r, s3r, s1u, s3u, 1050.0));
  CHECK(!s1u.sample(1050.0).valid);          // old-rung loss gone
  CHECK(!s3u.sample(1050.0).valid);
  s1u.add(200, 150, 1050.0);                 // an add AT the edge tick is kept (10 lost of 100)
  s1u.add(300, 200, 1100.0);                 // 50 lost of 100 -> 60/200 over the window
  CHECK(s1u.sample(1100.0).valid);
  CHECK(s1u.sample(1100.0).loss > 0.29 && s1u.sample(1100.0).loss < 0.31);
}

TEST(overhead_only_step_blanks_base_not_enh) {
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500), u1(500), u3(500);
  TransitionEdge edge;
  edge.on_tick(op_at(4, 1.0), dec, s1, s3, u1, u3, 1000.0);
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(edge.on_tick(op_at(4, 0.5), dec, s1, s3, u1, u3, 2050.0));
  CHECK(!s1.sample(2050.0).valid);
  CHECK(s3.sample(2050.0).valid);
}

TEST(width_only_step_blanks_both_residual_windows) {
  // 20/3 -> 40/3 (2026-09-24 40 MHz rungs): same MCS, different PHY rate
  // and airtime -- a real rung transition, so both edges fire.
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500), u1(500), u3(500);
  TransitionEdge edge;
  edge.on_tick(op_at(3, 1.0, 20), dec, s1, s3, u1, u3, 1000.0);
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(edge.on_tick(op_at(3, 1.0, 40), dec, s1, s3, u1, u3, 2050.0));
  CHECK(!s1.sample(2050.0).valid);
  CHECK(!s3.sample(2050.0).valid);
  CHECK(!edge.on_tick(op_at(3, 1.0, 40), dec, s1, s3, u1, u3, 2100.0));   // and only once
}

TEST(no_change_never_blanks) {
  auto dec = make_dec();
  S1LossWindow s1(500), s3(500), u1(500), u3(500);
  TransitionEdge edge;
  edge.on_tick(op_at(3), dec, s1, s3, u1, u3, 1000.0);
  book_loss(s1, 2000.0);
  book_loss(s3, 2000.0);
  CHECK(!edge.on_tick(op_at(3), dec, s1, s3, u1, u3, 2050.0));
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
