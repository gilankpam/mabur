#include "mtest.h"
#include "loss_sim.h"
#include "loss_control.h"
#ifdef MABUR_LOSS_SIM
// Only the Aggregator-hook tests below need these; see the #ifdef guarding
// them for why the whole file cannot assume this macro is set.
#include "frame_fixture.h"
#include "aggregator.h"
#include "mabur/uep_encoder.h"
#endif

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace maburgs;

// Statistical tests need enough draws that a correct implementation is
// comfortably inside tolerance and an off-by-one model is not. 200k bodies is
// ~1 hour of one stream at 60 fps; it runs in milliseconds.
static constexpr int kDraws = 200000;

TEST(loss_zero_never_drops) {
  LossSim s;
  CHECK(!s.enabled());
  for (int i = 0; i < 1000; ++i) CHECK(!s.should_drop(0, 3));
  CHECK(s.dropped(3) == 0);
}

TEST(configured_stream_reports_enabled) {
  LossSim s;
  s.configure(3, 0.10, 1.0);
  CHECK(s.enabled());
  CHECK(std::fabs(s.loss(3) - 0.10) < 1e-9);
  CHECK(std::fabs(s.burst(3) - 1.0) < 1e-9);
  // Other streams stay untouched.
  CHECK(s.loss(1) == 0.0);
  for (int i = 0; i < 1000; ++i) CHECK(!s.should_drop(0, 1));
}

TEST(bernoulli_rate_matches_target) {
  LossSim s;
  s.configure(3, 0.10, 1.0);
  int drops = 0;
  for (int i = 0; i < kDraws; ++i) if (s.should_drop(0, 3)) ++drops;
  const double measured = static_cast<double>(drops) / kDraws;
  CHECK(std::fabs(measured - 0.10) < 0.01);
  CHECK(s.dropped(3) == static_cast<uint64_t>(drops));
}

TEST(burst_mean_length_matches_target) {
  LossSim s;
  s.configure(3, 0.10, 5.0);
  int drops = 0, runs = 0;
  bool in_run = false;
  for (int i = 0; i < kDraws; ++i) {
    const bool d = s.should_drop(0, 3);
    if (d) { ++drops; if (!in_run) { ++runs; in_run = true; } }
    else in_run = false;
  }
  const double measured_loss = static_cast<double>(drops) / kDraws;
  const double measured_burst = static_cast<double>(drops) / runs;
  CHECK(std::fabs(measured_loss - 0.10) < 0.015);
  CHECK(std::fabs(measured_burst - 5.0) < 0.5);
}

// Real fading is independent per card — that is the whole point of the
// multi-card diversity union. Two cards at 50% must give ~25% joint loss, not
// 50%: if they were correlated the union would never recover a thing.
TEST(cards_are_independent) {
  LossSim s;
  s.configure(3, 0.50, 1.0);
  int both = 0;
  for (int i = 0; i < kDraws; ++i) {
    const bool a = s.should_drop(0, 3);
    const bool b = s.should_drop(1, 3);
    if (a && b) ++both;
  }
  const double joint = static_cast<double>(both) / kDraws;
  CHECK(std::fabs(joint - 0.25) < 0.02);
}

TEST(same_seed_same_sequence) {
  LossSim a, b;
  a.configure(3, 0.20, 3.0);
  b.configure(3, 0.20, 3.0);
  for (int i = 0; i < 5000; ++i) CHECK(a.should_drop(0, 3) == b.should_drop(0, 3));
}

TEST(loss_one_drops_everything) {
  LossSim s;
  s.configure(3, 1.0, 1.0);
  for (int i = 0; i < 1000; ++i) CHECK(s.should_drop(0, 3));
}

TEST(out_of_range_ids_never_drop) {
  LossSim s;
  s.configure(3, 1.0, 1.0);
  CHECK(!s.should_drop(0, 6));            // past kStreams (0..5 valid)
  CHECK(!s.should_drop(0, -1));
  CHECK(!s.should_drop(LossSim::kMaxCards, 3));
}

// sid 5 is the probe stream (spec 2026-09-04): configurable and dropped
// like any other stream, unlike sids past kStreams which are ignored.
TEST(probe_stream_sid5_is_configurable) {
  LossSim s;
  s.configure(5, 1.0, 1.0);
  CHECK(s.enabled());
  for (int i = 0; i < 10; ++i) CHECK(s.should_drop(0, 5));
  CHECK(!s.should_drop(0, 1));
}

TEST(clamps_out_of_range_config) {
  LossSim s;
  s.configure(3, 2.0, 0.1);               // loss > 1, burst < 1
  CHECK(s.loss(3) == 1.0);
  CHECK(s.burst(3) == 1.0);
  s.configure(3, -1.0, 3.0);              // negative loss
  CHECK(s.loss(3) == 0.0);
  CHECK(!s.enabled());
  s.configure(9, 0.5, 2.0);               // bad sid: ignored, no crash
  CHECK(!s.enabled());
}

// Fix round 1: a chain with recovery rate r = 1/burst can only reach
// steady-state loss up to burst/(1+burst) before p would need to exceed 1.
// loss=0.90, burst=3.0 is past that threshold (3/4 = 0.75) -- configure()
// must keep the requested loss exact and raise burst instead of silently
// capping delivered loss at 0.75.
TEST(saturation_raises_burst_keeps_loss) {
  LossSim s;
  s.configure(3, 0.90, 3.0);
  CHECK(std::fabs(s.burst(3) - 9.0) < 1e-6);
  int drops = 0, runs = 0;
  bool in_run = false;
  for (int i = 0; i < kDraws; ++i) {
    const bool d = s.should_drop(0, 3);
    if (d) { ++drops; if (!in_run) { ++runs; in_run = true; } }
    else in_run = false;
  }
  const double measured_loss = static_cast<double>(drops) / kDraws;
  const double measured_burst = static_cast<double>(drops) / runs;
  CHECK(std::fabs(measured_loss - 0.90) < 0.015);
  CHECK(std::fabs(measured_loss - 0.75) > 0.05);   // must NOT silently cap at B/(B+1)
  CHECK(std::fabs(measured_burst - 9.0) < 1.0);
}

// loss=1.0 with a multi-body burst must still drop the very first body: r=0
// means BAD is never left once entered, so the initial state must start BAD
// rather than reading one GOOD body before the state machine catches up.
TEST(loss_one_with_long_burst_drops_everything) {
  LossSim s;
  s.configure(3, 1.0, 5.0);
  for (int i = 0; i < 1000; ++i) CHECK(s.should_drop(0, 3));
}

// The feasibility rule must never perturb a pair that was already feasible:
// loss=0.10, burst=3.0 is well under the 3/4 threshold, so burst must pass
// through unchanged -- this is the normal operating range the bench sweep
// uses, and it must not get silently rewritten.
TEST(feasible_pair_leaves_burst_unchanged) {
  LossSim s;
  s.configure(3, 0.10, 3.0);
  CHECK(s.burst(3) == 3.0);
}

// The three tests below exercise the Aggregator hook (aggregator.h/.cpp),
// which only exists when MABUR_LOSS_SIM is defined (see gs/CMakeLists.txt) --
// unlike LossSim/LossControl above, Aggregator is not header-only, so this
// suite can only see its loss_sim() accessor when the library it links
// against (mabur_gs_core) was itself built with the same macro. Gated here
// so the rest of this file (the header-only classes) keeps compiling and
// running unconditionally in the default (MABUR_LOSS_SIM=OFF) build.
#ifdef MABUR_LOSS_SIM

// A dropped body must be invisible to EVERY counter. If injection leaks into
// the aggregator's accounting, the sideport lies to the operator during the
// very test the injector exists to support — so this invariant gets its own
// explicit test rather than being assumed.
static std::array<mabur::UepLayerCfg, 4> sim_layers() {
  std::array<mabur::UepLayerCfg, 4> L{};
  const double ov[4] = {1.00, 0.75, 0.50, 0.25};
  for (int s = 0; s < 4; ++s)
    L[s] = mabur::UepLayerCfg{mabur::SwConfig{64, 128, ov[s]}, 4};
  return L;
}

// Every fixture frame forced onto stream 3, so every body carries sid 3.
static std::vector<std::vector<uint8_t>> s3_bodies() {
  mabur::UepEncoder enc(sim_layers(), /*flush_ms=*/1'000'000'000ULL);
  std::vector<std::vector<uint8_t>> out;
  auto frames = mtest::load_frame_fixture(std::string(MABUR_FIXTURE_DIR) +
                                          "/frame_stream.bin");
  for (size_t i = 0; i < frames.size(); ++i) {
    auto unit = mtest::frame_unit(frames[i], static_cast<uint16_t>(i));
    for (auto& b : enc.add_frame(3, unit.data(), unit.size(), 0))
      out.push_back(std::move(b.body));
  }
  for (auto& b : enc.flush_all()) out.push_back(std::move(b.body));
  return out;
}

static mabur::node::RxBody sim_msg(uint8_t card, uint16_t seq,
                                   std::vector<uint8_t> body) {
  mabur::node::RxBody m;
  m.card_id = card; m.mac_seq = seq; m.crc_ok = true;
  m.rssi[0] = 38; m.rssi[1] = 40;
  m.snr[0] = 10; m.snr[1] = 25;
  m.mono_us = 1000u * seq;
  m.body = std::move(body);
  return m;
}

TEST(dropped_body_touches_no_counter) {
  const auto bodies = s3_bodies();
  REQUIRE(!bodies.empty());

  Aggregator agg(sim_layers(), 200, 512, 2);
  int frags = 0;
  agg.set_frag_sink([&](const mabur::DecodedFrag&) { ++frags; });
  agg.loss_sim().configure(3, 1.0, 1.0);   // drop every s3 body

  uint16_t seq = 0;
  for (const auto& b : bodies) agg.on_rx_body(sim_msg(0, seq++, b));

  const CardTrack& c = agg.card(0);
  CHECK(c.frames == 0);
  CHECK(c.rx_bytes == 0);
  CHECK(c.seq_received == 0);
  CHECK(c.seq_expected == 0);
  CHECK(c.video_bodies == 0);
  CHECK(c.has_ema == false);
  CHECK(c.cls[3].frames == 0);
  CHECK(c.cls[3].bytes == 0);
  CHECK(frags == 0);
  CHECK(agg.loss_sim().dropped(3) == bodies.size());
}

// The contrast case: with the sim off, the same bodies must flow normally.
// Without this, a should_drop() that always returned true would still pass
// the test above.
TEST(sim_off_leaves_path_untouched) {
  const auto bodies = s3_bodies();
  REQUIRE(!bodies.empty());

  Aggregator agg(sim_layers(), 200, 512, 2);
  int frags = 0;
  agg.set_frag_sink([&](const mabur::DecodedFrag&) { ++frags; });

  uint16_t seq = 0;
  for (const auto& b : bodies) agg.on_rx_body(sim_msg(0, seq++, b));

  const CardTrack& c = agg.card(0);
  CHECK(c.frames == bodies.size());
  CHECK(c.video_bodies == bodies.size());
  CHECK(c.seq_received == bodies.size());
  CHECK(c.cls[3].frames == bodies.size());
  CHECK(frags > 0);
  CHECK(agg.loss_sim().dropped(3) == 0);
}

// A corrupt body's peeked stream_id is untrustworthy, so injection must skip
// it rather than drop a randomly-misidentified stream.
TEST(crc_fail_bodies_are_not_injected) {
  const auto bodies = s3_bodies();
  REQUIRE(!bodies.empty());

  Aggregator agg(sim_layers(), 200, 512, 2);
  agg.set_frag_sink([](const mabur::DecodedFrag&) {});
  agg.loss_sim().configure(3, 1.0, 1.0);

  uint16_t seq = 0;
  for (const auto& b : bodies) {
    auto m = sim_msg(0, seq++, b);
    m.crc_ok = false;
    agg.on_rx_body(m);
  }
  CHECK(agg.loss_sim().dropped(3) == 0);
  CHECK(agg.card(0).frames == bodies.size());
  CHECK(agg.card(0).crc_fail == bodies.size());
}

#endif  // MABUR_LOSS_SIM

// The parser is exercised directly; the socket plumbing is not unit-tested
// (it is three POSIX calls and is proven by the bench run itself).
//
// Fix round 2: apply() now takes the card count, because injection is per-card
// and the loss the decoder sees is the union (percard^ncards). Every reply
// states both rates. The bench GS runs 2 cards, so that is the default here.
static constexpr int kBenchCards = 2;

TEST(control_parses_commands) {
  LossSim s;
  std::string reply;

  CHECK(LossControl::apply("s3 loss=8 burst=3", s, kBenchCards, &reply));
  CHECK(std::fabs(s.loss(3) - 0.08) < 1e-9);
  CHECK(std::fabs(s.burst(3) - 3.0) < 1e-9);
  // 8% per card on 2 cards is 0.64% to the decoder -- both numbers, labelled.
  CHECK(reply ==
        "ok s3 percard=8.00 eff=0.640 burst=3.0 ncards=2 note=eff-nominal");

  CHECK(LossControl::apply("s3 off", s, kBenchCards, &reply));
  CHECK(s.loss(3) == 0.0);
  CHECK(!s.enabled());
  CHECK(reply ==
        "ok s3 percard=0.00 eff=0.000 burst=1.0 ncards=2 note=eff-nominal");

  // burst defaults to 1 when omitted
  CHECK(LossControl::apply("s1 loss=5", s, kBenchCards, &reply));
  CHECK(std::fabs(s.loss(1) - 0.05) < 1e-9);
  CHECK(std::fabs(s.burst(1) - 1.0) < 1e-9);

  CHECK(LossControl::apply("off", s, kBenchCards, &reply));
  CHECK(!s.enabled());
  CHECK(reply == "ok all streams zero");

  // status is a query: it must report state, not change it (ruling: apply()
  // returns true only when state actually changed). All streams are zero at
  // this point (the "off" above), so the full string is deterministic.
  CHECK(!LossControl::apply("status", s, kBenchCards, &reply));
  CHECK(reply ==
        "ok ncards=2 s0[percard=0.00 eff=0.000 burst=1.0] "
        "s1[percard=0.00 eff=0.000 burst=1.0] "
        "s2[percard=0.00 eff=0.000 burst=1.0] "
        "s3[percard=0.00 eff=0.000 burst=1.0] "
        "s4[percard=0.00 eff=0.000 burst=1.0] "
        "s5[percard=0.00 eff=0.000 burst=1.0] "
        "drops=0,0,0,0,0,0 note=eff-nominal");
}

// FINDING 1: `loss=` keeps meaning PER-CARD, unchanged. On 2 cards a dialled
// 20% delivers ~4% to the decoder, and the reply must say so rather than
// letting the operator read 20% as the loss under test.
TEST(control_loss_is_still_per_card) {
  LossSim s;
  std::string reply;
  CHECK(LossControl::apply("s3 loss=20", s, kBenchCards, &reply));
  CHECK(std::fabs(s.loss(3) - 0.20) < 1e-12);   // per-card, verbatim
  CHECK(reply ==
        "ok s3 percard=20.00 eff=4.000 burst=1.0 ncards=2 note=eff-nominal");

  // Same per-card rate, three cards: the per-card setting must not move, only
  // the reported union rate.
  CHECK(LossControl::apply("s3 loss=20", s, 3, &reply));
  CHECK(std::fabs(s.loss(3) - 0.20) < 1e-12);
  CHECK(reply ==
        "ok s3 percard=20.00 eff=0.800 burst=1.0 ncards=3 note=eff-nominal");
}

// FINDING 1: `eff=` solves per-card = eff^(1/ncards). One card is the identity
// case, two and three are the ones the bench actually meets.
TEST(control_eff_solves_per_card) {
  LossSim s;
  std::string reply;

  // 1 card: union == per-card, so eff= and loss= must agree exactly.
  CHECK(LossControl::apply("s3 eff=20", s, 1, &reply));
  CHECK(std::fabs(s.loss(3) - 0.20) < 1e-9);
  CHECK(reply ==
        "ok s3 percard=20.00 eff=20.000 burst=1.0 ncards=1 note=eff-nominal");

  // 2 cards: sqrt(0.20) = 0.4472
  CHECK(LossControl::apply("s3 eff=20", s, 2, &reply));
  CHECK(std::fabs(s.loss(3) - std::sqrt(0.20)) < 1e-12);
  CHECK(std::fabs(s.loss(3) - 0.4472136) < 1e-6);
  CHECK(reply ==
        "ok s3 percard=44.72 eff=20.000 burst=1.0 ncards=2 note=eff-nominal");

  // 3 cards: cbrt(0.20) = 0.5848. Note the reported burst: a per-card rate
  // above 50% is infeasible for a burst-1 chain, so LossSim::configure keeps
  // the rate exact and raises burst to loss/(1-loss) = 1.4 (pre-existing
  // behaviour, unchanged here). The reply reports the raised value, which is
  // exactly why it is reported at all -- dialling by eff= reaches per-card
  // rates over 50% routinely, so the operator must be able to see it.
  CHECK(LossControl::apply("s3 eff=20", s, 3, &reply));
  CHECK(std::fabs(s.loss(3) - std::pow(0.20, 1.0 / 3.0)) < 1e-12);
  CHECK(std::fabs(s.loss(3) - 0.5848035) < 1e-6);
  CHECK(reply ==
        "ok s3 percard=58.48 eff=20.000 burst=1.4 ncards=3 note=eff-nominal");

  // eff=0 is off.
  CHECK(LossControl::apply("s3 eff=0", s, 2, &reply));
  CHECK(s.loss(3) == 0.0);
  CHECK(!s.enabled());
}

// FINDING 1: the whole planned sweep must round-trip -- every dialled
// effective step has to come back out of the reply as the same number, on both
// the ok reply and a follow-up status, or the sweep log records a level that
// was never in force.
TEST(control_eff_round_trips_through_both_replies) {
  const double steps[] = {2.0, 5.0, 10.0, 20.0, 35.0, 50.0};
  for (const double eff : steps) {
    for (const int n : {1, 2, 3}) {
      LossSim s;
      std::string reply;
      char cmd[64];
      std::snprintf(cmd, sizeof(cmd), "s3 eff=%.0f", eff);
      CHECK(LossControl::apply(cmd, s, n, &reply));

      // The stored per-card rate is the exact nth root...
      CHECK(std::fabs(s.loss(3) - std::pow(eff / 100.0, 1.0 / n)) < 1e-12);
      // ...and raising it back by n lands on the dialled effective rate.
      CHECK(std::fabs(std::pow(s.loss(3), n) * 100.0 - eff) < 1e-9);

      // Both replies must print that same effective figure (%.3f).
      char want_eff[32];
      std::snprintf(want_eff, sizeof(want_eff), "eff=%.3f", eff);
      CHECK(reply.find(want_eff) != std::string::npos);
      char want_percard[32];
      std::snprintf(want_percard, sizeof(want_percard), "percard=%.2f",
                    std::pow(eff / 100.0, 1.0 / n) * 100.0);
      CHECK(reply.find(want_percard) != std::string::npos);

      CHECK(!LossControl::apply("status", s, n, &reply));
      CHECK(reply.find(std::string("s3[") + want_percard + " " + want_eff) !=
            std::string::npos);
      char want_ncards[24];
      std::snprintf(want_ncards, sizeof(want_ncards), "ok ncards=%d", n);
      CHECK(reply.rfind(want_ncards, 0) == 0);
    }
  }
}

// FINDING 1: dialling both at once is ambiguous -- token order would silently
// decide which knob moved. Reject, and change nothing.
TEST(control_rejects_loss_and_eff_together) {
  LossSim s;
  std::string reply;
  CHECK(!LossControl::apply("s3 loss=20 eff=20", s, kBenchCards, &reply));
  CHECK(reply == "err loss= and eff= are exclusive");
  CHECK(!s.enabled());
  CHECK(!LossControl::apply("s3 eff=20 loss=20", s, kBenchCards, &reply));
  CHECK(reply == "err loss= and eff= are exclusive");
  CHECK(!s.enabled());
}

// A card count LossSim cannot honour must not be used in the arithmetic:
// cards past kMaxCards never drop, so counting them would understate per-card
// and overstate eff. Clamp to [1, kMaxCards] and say so in the reply.
TEST(control_clamps_card_count) {
  LossSim s;
  std::string reply;
  CHECK(LossControl::apply("s3 loss=50", s, 0, &reply));
  CHECK(reply ==
        "ok s3 percard=50.00 eff=50.000 burst=1.0 ncards=1 note=eff-nominal");
  CHECK(LossControl::apply("s3 loss=50", s, 99, &reply));
  CHECK(reply.find("ncards=8") != std::string::npos);
}

// Fix round 1, IMPORTANT 1: the reply strings Task 4's Python driver parses
// mid-sweep were previously asserted only by prefix (`"ok s0="`), leaving
// the `drops=%llu,...` tail -- the exact thing that driver reads to track
// injected-drop counts across a sweep -- completely unpinned. A reformat of
// status() could break Task 4 with this whole suite green. Distinct
// loss/burst per stream plus a distinct, non-zero drop count on exactly one
// stream (driven deterministically via loss=1.0's burst<=1 fast path, see
// loss_one_drops_everything above) makes every field's position unambiguous:
// a bug that reordered or reformatted status() would fail this even though
// an all-zero state would not.
TEST(control_status_full_reply_pinned) {
  LossSim s;
  s.configure(0, 0.0, 1.0);
  s.configure(1, 1.0, 1.0);   // burst==1 fast path: should_drop() is deterministic
  s.configure(2, 0.20, 2.0);
  s.configure(3, 0.08, 3.0);
  for (int i = 0; i < 5; ++i) CHECK(s.should_drop(0, 1));  // 5 deterministic drops on s1

  std::string reply;
  CHECK(!LossControl::apply("status", s, kBenchCards, &reply));
  CHECK(reply ==
        "ok ncards=2 s0[percard=0.00 eff=0.000 burst=1.0] "
        "s1[percard=100.00 eff=100.000 burst=1.0] "
        "s2[percard=20.00 eff=4.000 burst=2.0] "
        "s3[percard=8.00 eff=0.640 burst=3.0] "
        "s4[percard=0.00 eff=0.000 burst=1.0] "
        "s5[percard=0.00 eff=0.000 burst=1.0] "
        "drops=0,5,0,0,0,0 note=eff-nominal");
}

// Fix round 1, IMPORTANT 2: loss=80/burst=3 is infeasible (80% exceeds the
// burst/(1+burst) = 75% ceiling a burst-3 chain can deliver), so
// LossSim::configure raises burst to the minimum feasible value -- here
// loss/(1-loss) = 0.8/0.2 = 4.0. The whole point of reporting sim.burst(sid)
// in the reply is that the operator sees what they actually got, not what
// they typed; both the immediate "ok" reply and a follow-up "status" must
// show the raised value, never the requested 3.0.
TEST(control_reports_effective_burst_on_saturation) {
  LossSim s;
  std::string reply;
  CHECK(LossControl::apply("s3 loss=80 burst=3", s, kBenchCards, &reply));
  CHECK(reply ==
        "ok s3 percard=80.00 eff=64.000 burst=4.0 ncards=2 note=eff-nominal");
  CHECK(std::fabs(s.burst(3) - 4.0) < 1e-9);

  CHECK(!LossControl::apply("status", s, kBenchCards, &reply));
  CHECK(reply ==
        "ok ncards=2 s0[percard=0.00 eff=0.000 burst=1.0] "
        "s1[percard=0.00 eff=0.000 burst=1.0] "
        "s2[percard=0.00 eff=0.000 burst=1.0] "
        "s3[percard=80.00 eff=64.000 burst=4.0] "
        "s4[percard=0.00 eff=0.000 burst=1.0] "
        "s5[percard=0.00 eff=0.000 burst=1.0] "
        "drops=0,0,0,0,0,0 "
        "note=eff-nominal");
}

TEST(control_rejects_junk) {
  LossSim s;
  std::string reply;
  CHECK(!LossControl::apply("", s, kBenchCards, &reply));
  CHECK(!LossControl::apply("   ", s, kBenchCards, &reply));
  CHECK(!LossControl::apply("s9 loss=5", s, kBenchCards, &reply));
  CHECK(!LossControl::apply("s3 loss=abc", s, kBenchCards, &reply));
  CHECK(!LossControl::apply("s3 eff=abc", s, kBenchCards, &reply));
  CHECK(!LossControl::apply("hello world", s, kBenchCards, &reply));
  CHECK(!LossControl::apply("s3", s, kBenchCards, &reply));
  CHECK(reply.rfind("err", 0) == 0);
  CHECK(!s.enabled());   // no junk command ever changed state
}

// Whitespace and trailing newline (what `echo | socat` sends) must work.
TEST(control_tolerates_whitespace) {
  LossSim s;
  std::string reply;
  CHECK(LossControl::apply("  s3   eff=4   burst=2  \n", s, kBenchCards, &reply));
  CHECK(std::fabs(s.loss(3) - 0.20) < 1e-12);   // sqrt(0.04) on 2 cards
  CHECK(std::fabs(s.burst(3) - 2.0) < 1e-9);

  CHECK(LossControl::apply("  s3   loss=12   burst=4  \n", s, kBenchCards, &reply));
  CHECK(std::fabs(s.loss(3) - 0.12) < 1e-9);
  CHECK(std::fabs(s.burst(3) - 4.0) < 1e-9);
}
MTEST_MAIN
