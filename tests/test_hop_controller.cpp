#include "hop_controller.h"
#include "mtest.h"
using namespace maburgs;
static HopCfg cfg(bool en = true) { HopCfg c; c.enable = en; return c; }
static VerdictOut interfered(int ref = 5) { VerdictOut o; o.v = Verdict::Interfered; o.trigger = true; o.ref_rung = ref; return o; }
static VerdictOut healthy() { return VerdictOut{}; }
static HopTick T(double t, VerdictOut v, std::optional<uint8_t> best, uint8_t cur, bool video = false, int lead = 1) {
  HopTick k; k.now_ms = t; k.verdict = v; k.best = best; k.cur_op = cur; k.video_on_target = video; k.lead_card = lead; return k;
}
TEST(trigger_orders_best_and_video_confirms_then_verify_passes) {
  HopController h(cfg(), 136);
  auto a = h.tick(T(1000, interfered(5), 149, 136));
  CHECK(a.kind == HopAction::Order && a.target == 149 && a.epoch == 1 && a.restore_rung == 5 && a.lead_card == 1);
  CHECK(h.hop_ch() == 149 && h.state() == HopState::Ordered);
  a = h.tick(T(1080, interfered(5), 149, 136, /*video=*/true));
  CHECK(a.kind == HopAction::Confirm && h.state() == HopState::Verifying);
  for (double t = 1100; t < 2100; t += 150) CHECK(h.tick(T(t, healthy(), 120, 149)).kind == HopAction::None);
  CHECK(h.tick(T(2150, healthy(), 120, 149)).kind == HopAction::None);
  CHECK(h.state() == HopState::Idle && h.hops() == 1);
  auto ev = h.take_events();
  REQUIRE(ev.size() >= 3);
  CHECK(ev[0].kind == "order" && ev[1].kind == "lead_confirm" && ev.back().kind == "verify_pass");
}
TEST(no_video_withdraws_and_backs_off_target) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));
  auto a = h.tick(T(1600, interfered(), 149, 136));
  CHECK(a.kind == HopAction::Withdraw && a.target == 136 && a.epoch == 2);
  CHECK(h.hop_ch() == 136 && h.state() == HopState::Idle);
  auto bo = h.backed_off(1601);
  REQUIRE(bo.size() == 1); CHECK(bo[0] == 149);
  CHECK(h.backed_off(1600 + 30000 + 1).empty());
}
TEST(verify_fail_hops_again_immediately_to_next_best) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));
  h.tick(T(1080, interfered(), 149, 136, true));
  auto a = h.tick(T(1300, interfered(), 165, 149));     // still interfered on 149; ranker now says 165
  CHECK(a.kind == HopAction::Order && a.target == 165 && a.epoch == 2 && a.restore_rung == 5);
  auto bo = h.backed_off(1301); CHECK(bo.size() == 1 && bo[0] == 149);
}
TEST(exhausted_goes_home_then_holds) {
  HopController h(cfg(), 136);
  auto a = h.tick(T(1000, interfered(), std::nullopt, 149));
  CHECK(a.kind == HopAction::Order && a.target == 136);
  h.tick(T(1080, interfered(), std::nullopt, 149, true));
  a = h.tick(T(1300, interfered(), std::nullopt, 136));
  CHECK(a.kind == HopAction::Hold && h.state() == HopState::Hold && h.holds() == 1);
}
TEST(cooldown_and_per_minute_cap) {
  HopController h(cfg(), 136);
  double t = 0;
  for (int n = 0; n < 4; ++n) {                       // 4 confirmed hops
    CHECK(h.tick(T(t += 3000, interfered(), 149, 136)).kind == HopAction::Order);
    h.tick(T(t += 50, interfered(), 149, 136, true));
    t += 1100; h.tick(T(t, healthy(), 120, 149));     // verify passes
    h.tick(T(t += 100, healthy(), 120, 149));
  }
  CHECK(h.tick(T(t += 500, interfered(), 149, 136)).kind == HopAction::None);    // cooldown
  CHECK(h.tick(T(t += 2500, interfered(), 149, 136)).kind == HopAction::Hold);   // 4/min cap
}
TEST(one_card_retunes_after_repeats) {
  HopController h(cfg(), 136);
  HopTick k = T(1000, interfered(), 149, 136, false, /*lead=*/-1); k.n_cards = 1;
  CHECK(h.tick(k).kind == HopAction::Order);
  k.now_ms = 1200; k.rcf_sent_since_order = 4; CHECK(h.tick(k).kind == HopAction::None);
  k.now_ms = 1250; k.rcf_sent_since_order = 5;
  auto a = h.tick(k); CHECK(a.kind == HopAction::OneCardRetune && a.target == 149);
  k.now_ms = 1300; CHECK(h.tick(k).kind == HopAction::None);       // once
  k.now_ms = 1350; k.video_on_target = true; CHECK(h.tick(k).kind == HopAction::Confirm);
}
TEST(disabled_logs_but_never_acts) {
  HopController h(cfg(false), 136);
  CHECK(h.tick(T(1000, interfered(), 149, 136)).kind == HopAction::None);
  CHECK(h.hop_ch() == 0);
  auto ev = h.take_events(); REQUIRE(ev.size() == 1); CHECK(ev[0].kind == "would_order");
}
TEST(ordered_state_never_reorders_without_confirm_or_withdraw) {
  HopController h(cfg(), 136);
  CHECK(h.tick(T(1000, interfered(), 149, 136)).kind == HopAction::Order);
  CHECK(h.tick(T(1100, interfered(), 149, 136)).kind == HopAction::None);   // still Ordered, before confirm_ms, no video
  CHECK(h.state() == HopState::Ordered);
}
TEST(verify_fail_retries_count_against_rate_cap) {
  HopController h(cfg(), 136);
  double t = 1000;
  CHECK(h.tick(T(t, interfered(), 149, 136)).kind == HopAction::Order);              // order #1
  t += 50; CHECK(h.tick(T(t, interfered(), 149, 136, true)).kind == HopAction::Confirm);
  t += 10; CHECK(h.tick(T(t, interfered(), 165, 149)).kind == HopAction::Order);      // retry #1 (order #2)
  t += 10; CHECK(h.tick(T(t, interfered(), 165, 149, true)).kind == HopAction::Confirm);
  t += 10; CHECK(h.tick(T(t, interfered(), 40, 149)).kind == HopAction::Order);       // retry #2 (order #3)
  t += 10; CHECK(h.tick(T(t, interfered(), 40, 149, true)).kind == HopAction::Confirm);
  t += 10; CHECK(h.tick(T(t, interfered(), 44, 149)).kind == HopAction::Order);       // retry #3 (order #4, hits the cap)
  t += 10; CHECK(h.tick(T(t, interfered(), 44, 149, true)).kind == HopAction::Confirm);
  t += 10;
  auto a = h.tick(T(t, interfered(), 48, 149));                                       // retry #4: cap already at 4/min
  CHECK(a.kind == HopAction::Hold);
  CHECK(h.state() == HopState::Hold);
  CHECK(h.holds() == 1);
}
MTEST_MAIN
