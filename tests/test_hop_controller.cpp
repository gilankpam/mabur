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

// ---- Task 11: hop-logic fixes after the 2026-09-26 long-frame jam run ----
static VerdictOut blocked_here(int ref = 5) {
  VerdictOut o = interfered(ref); o.evidence = kEvImpaired | kEvBlocked; return o;
}
static VerdictOut raised_here(int ref = 5) {
  VerdictOut o = interfered(ref); o.evidence = kEvImpaired | kEvRaised; return o;
}
static bool has(const std::vector<uint8_t>& v, uint8_t ch) {
  for (auto c : v) if (c == ch) return true;
  return false;
}
// (a) A blocked home is not the "nothing ranked" fallback: ordering it is
// ordering a channel the dwells already read as jammed.
// Revert (drop `!home_blocked` from home_available()): both ticks Order 136.
TEST(blocked_home_is_not_a_fallback) {
  HopController h(cfg(), 136);
  HopTick k = T(1000, interfered(), std::nullopt, 149); k.home_blocked = true;
  auto a = h.tick(k);
  CHECK(a.kind == HopAction::Hold);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "hold_exhausted");
  // ...and not the verify-fail retry's fallback either.
  HopController g(cfg(), 136);
  g.tick(T(1000, interfered(), 149, 120));
  g.tick(T(1080, interfered(), 149, 120, true));
  HopTick f = T(1400, measured(interfered(), 1250, 1400), std::nullopt, 149); f.home_blocked = true;
  auto b = g.tick(f);
  CHECK(b.kind == HopAction::Hold);
}
// (d) Stuck on a blocked op with nothing ranked and home unavailable: the
// escape (a fled, unblocked channel) is ordered instead of a hold.
// Revert (drop the escape branch in idle_tick): Hold, hold_exhausted.
TEST(escape_from_blocked_op_into_fled_channel) {
  HopController h(cfg(), 136);
  HopTick k = T(1000, blocked_here(), std::nullopt, 136);   // on home: home unavailable
  k.escape = 112; k.escape_score = 7;
  auto a = h.tick(k);
  CHECK(a.kind == HopAction::Order && a.target == 112 && a.restore_rung == 5);
  CHECK(h.state() == HopState::Ordered);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "escape" && ev[0].target == 112 && ev[0].score == 7);
  CHECK(backed(h, 136, 1001));                          // the blocked op is fled
  CHECK(!has(h.backed_off_failed(1001), 136));          // ...fled, not failed
  // the escape target goes through the normal confirm/verify flow
  CHECK(h.tick(T(1080, blocked_here(), std::nullopt, 136, true)).kind == HopAction::Confirm);
  CHECK(h.state() == HopState::Verifying);
}
// Revert (escape without the kEvBlocked gate): Order 112.
TEST(no_escape_when_op_not_blocked) {
  HopController h(cfg(), 136);
  HopTick k = T(1000, raised_here(), std::nullopt, 136);
  k.escape = 112;
  auto a = h.tick(k);
  CHECK(a.kind == HopAction::Hold);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "hold_exhausted");
}
// main.cpp passes backed_off_failed() as the escape's skip list, so a
// verify-failed channel is never an escape while a fled one is.
// Revert (backed_off_failed returns every entry, or flee() records
// verify_failed): 144 appears in the failed list.
TEST(backed_off_failed_lists_only_verify_failed_channels) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 128, 144));             // flee 144
  h.tick(T(1062, interfered(), 128, 144, true));       // landed on 128
  auto a = h.tick(T(1300, interfered(), 40, 128));     // 128 fails its verify -> 40
  REQUIRE(a.kind == HopAction::Order && a.target == 40);
  auto failed = h.backed_off_failed(1301);
  CHECK(failed.size() == 1 && has(failed, 128));
  auto all = h.backed_off(1301);
  CHECK(all.size() == 2 && has(all, 144) && has(all, 128));
  // a withdraw (confirm timeout) is a failure too
  h.tick(T(1300 + 600, interfered(), 40, 128));
  CHECK(has(h.backed_off_failed(1901), 40));
  CHECK(!has(h.backed_off_failed(1901), 144));
  // expiry applies to the failed view the same way
  CHECK(h.backed_off_failed(1300 + 60000 + 1000).empty());
}
// A later back-off of a fled channel for a verify failure takes the new
// reason (and keeps doubling).
// Revert (back_off() keeps the first reason): 144 stays "fled".
TEST(back_off_reason_is_overwritten_by_a_later_failure) {
  HopController g(cfg(), 136);
  g.tick(T(1000, interfered(), 128, 144));             // flee 144 (k=1, fled)
  g.tick(T(1062, interfered(), 128, 144, true));
  g.tick(T(2200, healthy(), std::nullopt, 128));       // verify passes on 128
  CHECK(!has(g.backed_off_failed(2201), 144));
  g.tick(T(40000, interfered(), 144, 128));            // 144's first backoff expired: order 144
  g.tick(T(40062, interfered(), 144, 128, true));
  g.tick(T(40300, interfered(), std::nullopt, 144));   // 144 fails its verify
  CHECK(has(g.backed_off_failed(40301), 144));
  // doubled: k=2 -> 60 s from the failure, still backed off at +59 s
  CHECK(has(g.backed_off(40300 + 59000), 144));
}
// Revert (no hop-cap check before the verify-fail escape): Order 112.
TEST(escape_respects_hop_cap) {
  HopController h(cfg(), 136);
  double t = 1000;
  const double kPast = 160;
  CHECK(h.tick(T(t, interfered(), 149, 136)).kind == HopAction::Order);               // #1
  t += 50; h.tick(T(t, interfered(), 149, 136, true));
  t += kPast; CHECK(h.tick(T(t, interfered(), 165, 149)).kind == HopAction::Order);   // #2
  t += 10; h.tick(T(t, interfered(), 165, 149, true));
  t += kPast; CHECK(h.tick(T(t, interfered(), 40, 149)).kind == HopAction::Order);    // #3
  t += 10; h.tick(T(t, interfered(), 40, 149, true));
  t += kPast; CHECK(h.tick(T(t, interfered(), 44, 149)).kind == HopAction::Order);    // #4: cap full
  t += 10; h.tick(T(t, interfered(), 44, 149, true));
  (void)h.take_events();
  t += kPast;
  HopTick k = T(t, blocked_here(), std::nullopt, 44);   // home 136 fled -> unavailable
  k.escape = 112;
  auto a = h.tick(k);
  CHECK(a.kind == HopAction::Hold);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "hold_cap");
}
// Revert (drop the escape branch in verifying_tick): Hold, verify_fail.
TEST(escape_after_verify_fail) {
  HopController h(cfg(), 136);
  h.tick(T(1000, interfered(), 149, 136));              // flee home 136
  h.tick(T(1080, interfered(), 149, 136, true));        // landed on 149
  (void)h.take_events();
  HopTick k = T(1400, measured(blocked_here(), 1250, 1400), std::nullopt, 149);
  k.escape = 112; k.escape_score = 3;
  auto a = h.tick(k);
  CHECK(a.kind == HopAction::Order && a.target == 112);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "escape" && ev[0].target == 112);
  CHECK(has(h.backed_off_failed(1401), 149));           // the failed target is still penalised
  // not blocked here: the old verify_fail hold stands
  HopController g(cfg(), 136);
  g.tick(T(1000, interfered(), 149, 136));
  g.tick(T(1080, interfered(), 149, 136, true));
  HopTick m = T(1400, measured(raised_here(), 1250, 1400), std::nullopt, 149);
  m.escape = 112;
  CHECK(g.tick(m).kind == HopAction::Hold);
}

// ---- Task 12 (f): undelivered orders are not channel failures ----------
// Bench 2026-09-26 (GS session 0232): the jammer next to the drone also
// jammed the uplink, so the order (carried by every RCF) never arrived;
// after confirm_ms the controller backed the clean target off as FAILED,
// the escape (which skips Failed) had nowhere to go, and the link held 30 s
// on the jammed op. While the op reads blocked, the order is given until
// confirm_extend_ms (3000) before it is withdrawn as undelivered.
static uint8_t evcount(const std::vector<HopEvent>& ev, const char* k) {
  uint8_t n = 0;
  for (const auto& e : ev) if (e.kind == k) ++n;
  return n;
}
// Revert (drop the extension branch in ordered_tick): Withdraw at 1500.
TEST(confirm_extends_while_op_blocked) {
  HopController h(cfg(), 136);
  REQUIRE(h.tick(T(1000, blocked_here(), 112, 144)).kind == HopAction::Order);
  (void)h.take_events();
  CHECK(h.tick(T(1500, blocked_here(), 112, 144)).kind == HopAction::None);
  CHECK(h.tick(T(2000, blocked_here(), 112, 144)).kind == HopAction::None);
  CHECK(h.tick(T(3900, blocked_here(), 112, 144)).kind == HopAction::None);
  CHECK(h.state() == HopState::Ordered && h.hop_ch() == 112);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);   // logged once, on entering the extension
  CHECK(ev[0].kind == "confirm_extend" && ev[0].target == 112 && ev[0].elapsed_ms == 500);
}
// Revert (record Failed on an extended withdraw): 112 is in
// backed_off_failed(); or keep "withdraw" as the event kind.
TEST(extension_expires_as_undelivered) {
  HopController h(cfg(), 136);
  h.tick(T(1000, blocked_here(), 112, 144));
  h.tick(T(1500, blocked_here(), 112, 144));
  (void)h.take_events();
  auto a = h.tick(T(4000, blocked_here(), 112, 144));
  CHECK(a.kind == HopAction::Withdraw && a.target == 144);
  CHECK(h.state() == HopState::Idle && h.hop_ch() == 144);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "withdraw_undelivered" && ev[0].target == 112 && ev[0].elapsed_ms == 3000);
  CHECK(!has(h.backed_off_failed(4001), 112));
  CHECK(has(h.backed_off(4001), 112));   // the normal ranker still skips it
}
// Today's behaviour when the op is not blocked, and with the key at 0.
// Revert (extend regardless of kEvBlocked): None at 1500.
TEST(unblocked_op_withdraws_at_confirm_ms) {
  HopController h(cfg(), 136);
  h.tick(T(1000, raised_here(), 112, 144));
  (void)h.take_events();
  CHECK(h.tick(T(1500, raised_here(), 112, 144)).kind == HopAction::Withdraw);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "withdraw");
  CHECK(has(h.backed_off_failed(1501), 112));
  // confirm_extend_ms 0 (<= confirm_ms): no extension even when blocked
  HopCfg c = cfg(); c.confirm_extend_ms = 0;
  HopController g(c, 136);
  g.tick(T(1000, blocked_here(), 112, 144));
  CHECK(g.tick(T(1500, blocked_here(), 112, 144)).kind == HopAction::Withdraw);
  CHECK(has(g.backed_off_failed(1501), 112));
  CHECK(evcount(g.take_events(), "confirm_extend") == 0);
}
// An extension that sees the op unblock withdraws then (not at 3000), and
// still as undelivered: the extension was entered for this order.
TEST(extension_ends_when_op_unblocks) {
  HopController h(cfg(), 136);
  h.tick(T(1000, blocked_here(), 112, 144));
  h.tick(T(1500, blocked_here(), 112, 144));
  (void)h.take_events();
  CHECK(h.tick(T(1800, raised_here(), 112, 144)).kind == HopAction::Withdraw);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "withdraw_undelivered");
}
// The escape skips only Failed channels, so an undelivered target is
// still an escape once the op reads blocked again.
// Revert (Undelivered counted in backed_off_failed): Hold, hold_exhausted.
TEST(escape_may_target_undelivered_channel) {
  HopController h(cfg(), 136);
  h.tick(T(1000, blocked_here(), 112, 144));   // flee 144, order 112
  h.tick(T(1500, blocked_here(), 112, 144));
  REQUIRE(h.tick(T(4000, blocked_here(), 112, 144)).kind == HopAction::Withdraw);
  (void)h.take_events();
  // after cooldown: nothing ranked (112 backed off), home 136 blocked
  HopTick k = T(6100, blocked_here(), std::nullopt, 144); k.home_blocked = true;
  k.escape = 112; k.escape_score = 4;
  auto a = h.tick(k);
  CHECK(a.kind == HopAction::Order && a.target == 112);
  auto ev = h.take_events();
  REQUIRE(ev.size() == 1);
  CHECK(ev[0].kind == "escape");
}
// Revert (check the timeout before video_on_target): no Confirm.
TEST(confirm_during_extension_proceeds_to_verifying) {
  HopController h(cfg(), 136);
  h.tick(T(1000, blocked_here(), 112, 144));
  h.tick(T(1500, blocked_here(), 112, 144));
  auto a = h.tick(T(2700, blocked_here(), 112, 144, /*video=*/true));
  CHECK(a.kind == HopAction::Confirm && a.target == 112);
  CHECK(h.state() == HopState::Verifying);
  auto ev = h.take_events();
  CHECK(ev.back().kind == "lead_confirm" && ev.back().elapsed_ms == 1700);
}
// A later verify_pass on the undelivered channel erases its back-off.
// Revert (drop backoff_.erase(landed) in verifying_tick): 112 stays backed off.
TEST(verify_pass_clears_undelivered_backoff) {
  HopController h(cfg(), 136);
  h.tick(T(1000, blocked_here(), 112, 144));
  h.tick(T(1500, blocked_here(), 112, 144));
  h.tick(T(4000, blocked_here(), 112, 144));   // withdraw_undelivered
  HopTick k = T(6100, blocked_here(), std::nullopt, 144); k.home_blocked = true; k.escape = 112;
  REQUIRE(h.tick(k).kind == HopAction::Order);
  REQUIRE(h.tick(T(6150, blocked_here(), std::nullopt, 144, true)).kind == HopAction::Confirm);
  CHECK(h.tick(T(7200, healthy(), std::nullopt, 112)).kind == HopAction::VerifyPass);
  CHECK(!has(h.backed_off(7201), 112));
}
