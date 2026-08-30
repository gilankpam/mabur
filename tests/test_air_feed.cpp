#include <cmath>

#include "mtest.h"
#include "../drone/src/air_feed.h"
using namespace mabur;

static void feed_frames(AirFeed& f, double len_b, double mult_b,
                        double len_e, double mult_e, int n = 64) {
  for (int i = 0; i < n; ++i) {
    f.on_frame(0, (size_t)len_b, (size_t)(len_b * mult_b));
    f.on_frame(1, (size_t)len_e, (size_t)(len_e * mult_e));
  }
}

TEST(unseeded_until_both_sides_fed) {
  AirFeed f(nullptr);
  CHECK(!f.seeded());
  f.on_frame(0, 1000, 1500);
  CHECK(!f.seeded());  // only sid 0 fed so far
  f.on_frame(1, 2000, 3000);
  CHECK(f.seeded());
}

TEST(out_may_be_null) {
  // Hot-loop-less construction (tests, or before the feed is wired up):
  // on_frame must not crash with no AirFeedOut to publish into.
  AirFeed f(nullptr);
  feed_frames(f, 1000, 1.5, 2000, 1.5);
  CHECK(f.seeded());
}

TEST(publishes_share_against_ewma_len) {
  AirFeedOut out;
  AirFeed f(&out);
  feed_frames(f, 1000, 1.6, 2000, 1.5);
  CHECK(std::abs(out.share_base.load() - 1000.0 / 3000.0) < 0.01);
}

TEST(excess_measured_against_the_applied_anchor) {
  AirFeedOut out;
  AirFeed f(&out);
  f.set_applied(0.5, 0.3);  // the op pair currently flying (Task 6 apply)
  feed_frames(f, 1000, 1.5, 2000, 1.6);
  // excess_s = emit/len - (1 + anchor_s)
  CHECK(std::abs(out.excess_base.load() - (1.5 - 1.5)) < 1e-3);   // 0
  CHECK(std::abs(out.excess_enh.load() - (1.6 - 1.3)) < 1e-3);    // 0.3
  CHECK(std::abs(out.ov_base.load() - 0.5) < 1e-3);
  CHECK(std::abs(out.ov_enh.load() - 0.3) < 1e-3);
}

TEST(excess_defaults_to_zero_anchor_before_any_apply) {
  AirFeedOut out;
  AirFeed f(&out);
  feed_frames(f, 1000, 1.2, 2000, 1.2);
  CHECK(std::abs(out.ov_base.load() - 0.0) < 1e-3);
  CHECK(std::abs(out.excess_base.load() - 0.2) < 1e-3);  // 1.2 - (1+0)
}

// Brief step 1: armed debug-HTTP override wins the anchor the publish
// reports, even though an op pair was separately applied (bench sweeps
// override main.cpp's hot-loop application of the op pair -- the
// published ov_* / excess_* must track what's ACTUALLY flying, not the
// stale op pair).
TEST(armed_override_is_published_as_the_anchor) {
  AirFeedOut out;
  out.ovr_base_pct.store(100);
  out.ovr_enh_pct.store(100);
  AirFeed f(&out);
  f.set_applied(0.5, 0.3);  // op pair present, but the override wins
  feed_frames(f, 1000, 1.5, 2000, 1.5);
  CHECK(std::abs(out.ov_base.load() - 1.0) < 1e-3);
  CHECK(std::abs(out.ov_enh.load() - 1.0) < 1e-3);
}

TEST(half_armed_override_does_not_win) {
  AirFeedOut out;
  out.ovr_base_pct.store(100);
  out.ovr_enh_pct.store(-1);  // only one side armed -> override does not apply
  AirFeed f(&out);
  f.set_applied(0.5, 0.3);
  feed_frames(f, 1000, 1.5, 2000, 1.5);
  CHECK(std::abs(out.ov_base.load() - 0.5) < 1e-3);
  CHECK(std::abs(out.ov_enh.load() - 0.3) < 1e-3);
}

// Fix round 1 review finding: clearing the debug-HTTP override has no
// bounded re-assert to fall back on (run_bitrate_policy's 5s reassert only
// re-sends bitrate/roi_qp, never UEP layer overhead), so main.cpp's hot
// loop now detects the armed->cleared transition itself and calls
// set_applied() with the op pair EXACTLY ONCE at the transition (mirroring
// the same apply_op_to_uep + set_applied pairing an op change uses). This
// simulates that hot-loop sequence directly against AirFeed and checks the
// published anchors stay truthful at every stage, including many frames
// after the transition with NO further set_applied call -- proving the
// fix does not need a per-frame reapply to hold.
TEST(armed_then_cleared_transition_reapplies_once) {
  AirFeedOut out;
  AirFeed f(&out);
  f.set_applied(0.5, 0.3);  // op pair applied (Task 6's apply_op_to_uep)

  // Override armed: the encoder flies 1.0/1.0, frames measured against it.
  out.ovr_base_pct.store(100);
  out.ovr_enh_pct.store(100);
  feed_frames(f, 1000, 2.2, 2000, 2.2);
  CHECK(std::abs(out.ov_base.load() - 1.0) < 1e-3);
  CHECK(std::abs(out.ov_enh.load() - 1.0) < 1e-3);
  CHECK(std::abs(out.excess_base.load() - 0.2) < 1e-3);  // 2.2 - (1+1.0)

  // Override cleared. The hot loop's transition handler calls set_applied
  // with the op pair exactly once, right on the tick the atomics flip --
  // not lazily, not waiting on the next op change or any timer.
  out.ovr_base_pct.store(-1);
  out.ovr_enh_pct.store(-1);
  f.set_applied(0.5, 0.3);

  // Many subsequent frames, now actually encoded at the op pair (mult
  // 1.5), with NO further set_applied call: the anchor must hold at the
  // op pair throughout, not drift or require re-poking every frame.
  feed_frames(f, 1000, 1.5, 2000, 1.5, /*n=*/200);
  CHECK(std::abs(out.ov_base.load() - 0.5) < 1e-3);
  CHECK(std::abs(out.ov_enh.load() - 0.3) < 1e-3);
  CHECK(std::abs(out.excess_base.load()) < 1e-3);        // 1.5 - (1+0.5) = 0
  CHECK(std::abs(out.excess_enh.load() - 0.2) < 1e-3);   // 1.5 - (1+0.3)
}

MTEST_MAIN
