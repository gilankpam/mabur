#include "hop_controller.h"
#include "mtest.h"
using namespace maburgs;
static HopCfg cfg(bool en = true) { HopCfg c; c.enable = en; return c; }
static VerdictOut interfered(int ref = 5) { VerdictOut o; o.v = Verdict::Interfered; o.trigger = true; o.ref_rung = ref; return o; }
static VerdictOut healthy() { return VerdictOut{}; }
// A verdict with no span stamped on it is treated as one HopVerdict would
// have produced for the window ending at this very tick -- i.e. FRESH.
// Staleness (a cached pre-hop verdict re-fed after the hop landed, which
// is what main.cpp actually does every ~10 ms between 150 ms windows) is
// spelled out explicitly with measured() below, never implied.
static HopTick T(double t, VerdictOut v, std::optional<uint8_t> best, uint8_t cur, bool video = false, int lead = 1) {
  HopTick k; k.now_ms = t;
  if (v.t_ms == 0) { v.t_start_ms = t; v.t_ms = t; }
  k.verdict = v; k.best = best; k.cur_op = cur; k.video_on_target = video; k.lead_card = lead; return k;
}
// The same verdict object, stamped as having been MEASURED over the window
// [t_start, t_end] -- used to hand the controller a verdict older than the
// hop it is verifying.
static VerdictOut measured(VerdictOut v, double t_start, double t_end) {
  v.t_start_ms = t_start; v.t_ms = t_end; return v;
}
// Whether one channel is currently backed off (the fled channel is backed
// off too since the trigger-flees rule, so "nothing backed off" is no longer
// the way to say "the landed channel was not penalised").
static bool backed(const HopController& h, uint8_t ch, double now) {
  for (auto c : h.backed_off(now)) if (c == ch) return true;
  return false;
}
TEST(trigger_orders_best_and_video_confirms_then_verify_passes) {
  HopController h(cfg(), 136);
  auto a = h.tick(T(1000, interfered(5), 149, 136));
  CHECK(a.kind == HopAction::Order && a.target == 149 && a.epoch == 1 && a.restore_rung == 5 && a.lead_card == 1);
  CHECK(h.hop_ch() == 149 && h.state() == HopState::Ordered);
  a = h.tick(T(1080, interfered(5), 149, 136, /*video=*/true));
  CHECK(a.kind == HopAction::Confirm && h.state() == HopState::Verifying);
  for (double t = 1100; t < 2100; t += 150) CHECK(h.tick(T(t, healthy(), 120, 149)).kind == HopAction::None);
  // VerifyPass is the caller's cue to thaw HopVerdict's frozen references
  // (spec section 2's "or after a hop's verify window ends"); before the
  // fix the verify window closed with no action at all and
  // HopVerdict::reset() had no caller anywhere in the tree.
  CHECK(h.tick(T(2150, healthy(), 120, 149)).kind == HopAction::VerifyPass);
  CHECK(h.tick(T(2300, healthy(), 120, 149)).kind == HopAction::None);   // once, not every tick
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
  // The failed target AND the channel the trigger fled (136) are backed
  // off; both expire on the same 30 s first-repeat schedule.
  auto bo = h.backed_off(1601);
  REQUIRE(bo.size() == 2); CHECK(bo[0] == 136 && bo[1] == 149);
  CHECK(h.backed_off(1600 + 30000 + 1).empty());
}
TEST(verify_fail_hops_again_immediately_to_next_best) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));
  h.tick(T(1080, interfered(), 149, 136, true));
  auto a = h.tick(T(1300, interfered(), 165, 149));     // still interfered on 149; ranker now says 165
  CHECK(a.kind == HopAction::Order && a.target == 165 && a.epoch == 2 && a.restore_rung == 5);
  auto bo = h.backed_off(1301);   // the fled 136 and the failed 149
  CHECK(bo.size() == 2 && bo[0] == 136 && bo[1] == 149);
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
  // Each failing verdict lands past the landing settle (a window starting
  // inside it is the transition's own debris and does not count).
  const double kPast = 160;
  t += kPast; CHECK(h.tick(T(t, interfered(), 165, 149)).kind == HopAction::Order);   // retry #1 (order #2)
  t += 10; CHECK(h.tick(T(t, interfered(), 165, 149, true)).kind == HopAction::Confirm);
  t += kPast; CHECK(h.tick(T(t, interfered(), 40, 149)).kind == HopAction::Order);    // retry #2 (order #3)
  t += 10; CHECK(h.tick(T(t, interfered(), 40, 149, true)).kind == HopAction::Confirm);
  t += kPast; CHECK(h.tick(T(t, interfered(), 44, 149)).kind == HopAction::Order);    // retry #3 (order #4, hits the cap)
  t += 10; CHECK(h.tick(T(t, interfered(), 44, 149, true)).kind == HopAction::Confirm);
  t += kPast;
  auto a = h.tick(T(t, interfered(), 48, 149));                                       // retry #4: cap already at 4/min
  CHECK(a.kind == HopAction::Hold);
  CHECK(h.state() == HopState::Hold);
  CHECK(h.holds() == 1);
}
// C1, the real staleness condition main.cpp produces and neither the old
// unit tests nor gs_e2e could: main.cpp recomputes a verdict every
// hop.window_ms (150 ms) but ticks this controller every ~10 ms, so for up
// to a full window after a Confirm the cached VerdictOut is the one
// measured BEFORE the hop, on the old channel -- Interfered by
// construction, since that is why we hopped. Acting on it failed the
// verify of a perfectly good channel ~10 ms after landing, backed it off
// for 30 s and marched on to the next candidate.
TEST(stale_pre_hop_interfered_does_not_break_the_verify_window) {
  HopController h(cfg(), 136);
  VerdictOut pre_hop = measured(interfered(5), 850, 1000);   // the window the order was placed on
  CHECK(h.tick(T(1000, pre_hop, 149, 136)).kind == HopAction::Order);
  CHECK(h.tick(T(1080, pre_hop, 149, 136, /*video=*/true)).kind == HopAction::Confirm);
  // Every tick for a whole window_ms after the confirm re-feeds that same
  // cached verdict. None of them may touch the hop.
  for (double t = 1090; t < 1230; t += 10) {
    CHECK(h.tick(T(t, pre_hop, 165, 149)).kind == HopAction::None);
    CHECK(h.state() == HopState::Verifying);
  }
  CHECK(!backed(h, 149, 1230));   // the channel we just landed on is NOT backed off
  // A verdict genuinely measured after the landing still fails the verify.
  auto a = h.tick(T(1400, measured(interfered(5), 1250, 1400), 165, 149));
  CHECK(a.kind == HopAction::Order && a.target == 165);
  CHECK(backed(h, 149, 1401));   // now it is: a genuine post-landing failure
}
// The boundary: a window that merely ENDS after the confirm gathered most
// of its deltas on the old channel, so it is still stale.
TEST(verify_window_rejects_a_window_that_straddles_the_confirm) {
  HopController h(cfg(), 136);
  h.tick(T(1000, measured(interfered(5), 850, 1000), 149, 136));
  h.tick(T(1080, measured(interfered(5), 850, 1000), 149, 136, true));   // confirm at 1080
  CHECK(h.tick(T(1160, measured(interfered(5), 1000, 1150), 165, 149)).kind == HopAction::None);
  CHECK(h.state() == HopState::Verifying);
  CHECK(!backed(h, 149, 1161));
}
// C3: idle_tick() runs from Hold as well as Idle, so a held controller
// with the trigger still latched re-entered the hold branch on every ~10 ms
// control tick -- one H line into scan.log AND one stderr line per tick
// (~10 KB/s each), and hop.holds on the sideport becoming a six-digit ramp.
// A hold is a state: log its edges.
TEST(hold_is_one_episode_not_one_event_per_tick) {
  HopController h(cfg(), 136);
  for (double t = 1000; t < 2000; t += 10) {
    auto a = h.tick(T(t, interfered(), std::nullopt, 136));   // at home, nothing ranked
    CHECK(a.kind == HopAction::Hold);
  }
  CHECK(h.state() == HopState::Hold);
  CHECK(h.holds() == 1);                    // episodes, not ticks (was 100)
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);                  // was 100
  CHECK(ev[0].kind == "hold_exhausted");
}
// ... and the episode has to be able to END, or hop.state reads "hold" for
// the rest of the flight: every other way out of Hold runs through order(),
// which needs a live trigger.
TEST(hold_ends_when_the_trigger_clears_and_says_how_long) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), std::nullopt, 136));
  (void)h.take_events();
  CHECK(h.tick(T(1500, healthy(), std::nullopt, 136)).kind == HopAction::None);
  CHECK(h.state() == HopState::Idle);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "hold_end");
  CHECK(ev[0].elapsed_ms == 500);
  // Idle with no trigger stays quiet: hold_end is an edge too.
  h.tick(T(1600, healthy(), std::nullopt, 136));
  CHECK(h.take_events().empty());
}
TEST(re_entering_a_hold_after_it_ended_counts_a_second_episode) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), std::nullopt, 136));
  h.tick(T(1500, healthy(), std::nullopt, 136));
  h.tick(T(2000, interfered(), std::nullopt, 136));
  CHECK(h.state() == HopState::Hold);
  CHECK(h.holds() == 2);
}
MTEST_MAIN

// ---- session lost mid-hop (bench 2026-09-24, GS session 0207) ----------
// run_radio() only ticks this controller while the link is in SESSION, and
// ChannelPlan::tick() ignores link loss entirely while a hop is in flight.
// A session that dropped while an order was still unconfirmed therefore
// left BOTH frozen forever: lead card parked on the target, trailing card
// on the old op, no split_home, while the drone had long since gone home
// on move_confirm_ms. on_session_lost() is the falling-edge exit.
TEST(session_loss_while_ordered_withdraws_the_hop) {
  HopController h(cfg(), 136);
  CHECK(h.tick(T(1000, interfered(), 40, 128)).kind == HopAction::Order);
  auto a = h.on_session_lost(1050, 128);
  CHECK(a.kind == HopAction::Withdraw && a.target == 128 && a.epoch == 2);
  CHECK(h.state() == HopState::Idle && h.hop_ch() == 128);
  bool forty = false;
  for (auto c : h.backed_off(1051)) forty = forty || c == 40;
  CHECK(forty);   // the unconfirmed target failed, same as a confirm_ms withdraw
  auto ev = h.take_events();
  REQUIRE(!ev.empty());
  CHECK(ev.back().kind == "session_lost" && ev.back().target == 40);
}
TEST(session_loss_when_idle_does_nothing) {
  HopController h(cfg(), 136);
  CHECK(h.on_session_lost(1000, 136).kind == HopAction::None);
  CHECK(h.state() == HopState::Idle && h.take_events().empty());
}
TEST(session_loss_while_verifying_ends_the_verify) {
  // Confirmed already, so ChannelPlan has moved op_ and its own link-loss
  // path works; the controller must just not resume a stale verify.
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));
  CHECK(h.tick(T(1080, interfered(), 149, 136, true)).kind == HopAction::Confirm);
  CHECK(h.on_session_lost(1200, 149).kind == HopAction::None);
  CHECK(h.state() == HopState::Idle);
}
TEST(session_loss_is_shadow_only_when_disabled) {
  HopController h(cfg(false), 136);
  h.tick(T(1000, interfered(), 40, 128));
  CHECK(h.on_session_lost(1050, 128).kind == HopAction::None);
  auto ev = h.take_events();
  REQUIRE(!ev.empty());
  CHECK(ev.back().kind == "would_session_lost");
}

// The deadlock itself, with the real ChannelPlan: the recorded sequence
// (op 128, order 40 on card 0, session drops before any confirm). The GS
// must have a card on home within split_after_ms, where the drone waits.
#include "channel_plan.h"
TEST(session_loss_mid_hop_lets_the_plan_split_home) {
  ChannelPlan plan(ChannelPlanCfg{136, 2, 5000, 300, 20});
  plan.on_ack(0, 128, 128);                  // link lives on 128
  plan.tick(100, true);
  HopController h(cfg(), 136);
  auto a = h.tick(T(1000, interfered(), 40, 128, false, /*lead=*/0));
  REQUIRE(a.kind == HopAction::Order);
  plan.hop_order(1000, a.target, a.lead_card);
  // session drops: the caller stops ticking h; the falling edge calls this
  a = h.on_session_lost(1100, plan.op());
  REQUIRE(a.kind == HopAction::Withdraw);
  plan.hop_withdraw(1100);
  bool home = false;
  for (double t = 1100; t <= 1100 + 5000 + 100; t += 10) {
    plan.tick(t, false);
    home = home || plan.desired(0) == 136 || plan.desired(1) == 136;
  }
  CHECK(home);
}

// ---- never retry the channel being fled (same session) -----------------
// Only failed TARGETS were backed off. The channel the trigger fled stayed
// eligible, and the in-flight ranker (event counts over 5 ms dwells) scored
// the jammed 144 a clean 25, so when 128's verify failed the retry went
// straight back into the jam.
TEST(trigger_backs_off_the_channel_it_flees) {
  HopController h(cfg(), 136);
  CHECK(h.tick(T(1000, interfered(), 128, 144)).kind == HopAction::Order);
  bool fled = false;
  for (auto c : h.backed_off(1001)) fled = fled || c == 144;
  CHECK(fled);
}
TEST(verify_fail_never_retries_the_channel_it_fled) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 128, 144));           // flee the jam on 144
  h.tick(T(1062, interfered(), 128, 144, true));     // landed on 128
  auto a = h.tick(T(1300, interfered(), 144, 128));  // 128 "fails"; ranker offers 144
  CHECK(a.kind == HopAction::Order && a.target != 144);
  CHECK(a.target == 136);                            // nothing else ranked: home
}

// ---- the verify ignores the landing's own debris (bench 2026-09-24) ----
// The first verify window that counted used to start ANY time after the
// confirm, and the hop's own retune gap is still being repaired then: 40
// failed its verify on 85 then 32 recovered symbols in windows starting
// 27 ms after landing, with 0 % loss. A window must start at least the
// settle (kHopSettleBlankMs, the same one the loss window is blanked for)
// after the confirm to count.
TEST(verify_ignores_a_window_that_starts_inside_the_landing_settle) {
  HopController h(cfg(), 136);
  h.tick(T(1000, measured(interfered(5), 850, 1000), 149, 136));
  h.tick(T(1080, measured(interfered(5), 850, 1000), 149, 136, true));   // confirm at 1080
  // starts 27 ms after landing: repairs of the transition, not the channel
  CHECK(h.tick(T(1260, measured(interfered(5), 1107, 1257), 165, 149)).kind == HopAction::None);
  CHECK(h.state() == HopState::Verifying);
  // starts after confirm + settle: genuine, fails the verify
  auto a = h.tick(T(1410, measured(interfered(5), 1257, 1407), 165, 149));
  CHECK(a.kind == HopAction::Order && a.target == 165);
}

// ---- nor go home when home is the channel being fled -------------------
// Run 1 of the fix bench: the jam was on home (136). 136 was correctly
// backed off as the fled channel, but when 144, 128 and 40 each failed
// their verify, the "nothing ranked: go home" fallback ordered 136 anyway
// -- straight back into the jam. A backed-off home is not a candidate.
TEST(verify_fail_does_not_fall_back_to_a_backed_off_home) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));             // flee the jam on home
  h.tick(T(1080, interfered(), 149, 136, true));       // landed on 149
  auto a = h.tick(T(1400, measured(interfered(), 1250, 1400), std::nullopt, 149));
  CHECK(a.kind != HopAction::Order || a.target != 136);
  CHECK(a.kind == HopAction::Hold);
}
TEST(fresh_trigger_does_not_fall_back_to_a_backed_off_home) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));             // flee 136 (backs it off)
  h.tick(T(1080, interfered(), 149, 136, true));
  h.tick(T(2200, healthy(), std::nullopt, 149));       // verify passes on 149
  // cooldown over, 149 now jammed, nothing ranked, home still backed off
  auto a = h.tick(T(5000, interfered(), std::nullopt, 149));
  CHECK(a.kind != HopAction::Order || a.target != 136);
}
