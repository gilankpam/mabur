#include "mtest.h"
#include "gs_overlay.h"
#include <string>
using namespace maburplay;

// --- thresholds -------------------------------------------------------
// Boundaries are inclusive at the OK end: "ok >= -65 dBm" means -65 is ok.

TEST(rssi_status_boundaries) {
  CHECK(rssi_status(-40.0) == Status::kOk);
  CHECK(rssi_status(-65.0) == Status::kOk);
  CHECK(rssi_status(-65.1) == Status::kCaution);
  CHECK(rssi_status(-80.0) == Status::kCaution);
  CHECK(rssi_status(-80.1) == Status::kCritical);
  CHECK(rssi_status(-120.0) == Status::kCritical);
}

TEST(snr_status_boundaries) {
  CHECK(snr_status(30.0) == Status::kOk);
  CHECK(snr_status(12.0) == Status::kOk);
  CHECK(snr_status(11.9) == Status::kCaution);
  CHECK(snr_status(6.0) == Status::kCaution);
  CHECK(snr_status(5.9) == Status::kCritical);
}

// Worst-of wins: a strong signal with terrible SNR is not a good link.
TEST(card_status_takes_the_worst_of_rssi_and_snr) {
  GsCard c;
  c.heard = true;
  c.rssi_dbm = -40.0;   // ok
  c.snr_db = 4.0;       // critical
  CHECK(card_status(c) == Status::kCritical);
  c.snr_db = 8.0;       // caution
  CHECK(card_status(c) == Status::kCaution);
  c.snr_db = 20.0;      // ok
  CHECK(card_status(c) == Status::kOk);
  c.rssi_dbm = -90.0;   // critical
  CHECK(card_status(c) == Status::kCritical);
}

// An unheard card has no status to report and must not read as critical --
// it renders unlit with "never heard" instead.
TEST(unheard_card_is_ok_status_and_zero_bars) {
  GsCard c;
  c.heard = false;
  CHECK(card_status(c) == Status::kOk);
  CHECK(card_bars(c) == 0);
}

TEST(rssi_bars_map_six_steps_over_minus90_to_minus45) {
  CHECK(rssi_bars(-95.0) == 0);
  CHECK(rssi_bars(-90.0) == 0);
  CHECK(rssi_bars(-40.0) == 6);
  CHECK(rssi_bars(-45.0) == 6);
  // -58 is (90-58)/45 = 0.711 of the range -> 4 of 6
  CHECK(rssi_bars(-58.0) == 4);
  // -71 is (90-71)/45 = 0.422 -> 3 of 6 (the handoff's caution sample)
  CHECK(rssi_bars(-71.0) == 3);
  // Monotone, and never out of range.
  int prev = -1;
  for (int d = -100; d <= -30; ++d) {
    const int b = rssi_bars((double)d);
    CHECK(b >= 0 && b <= 6);
    CHECK(b >= prev);
    prev = b;
  }
}

TEST(airtime_status_cautions_at_the_ceiling) {
  CHECK(airtime_status(0.0) == Status::kOk);
  CHECK(airtime_status(74.9) == Status::kOk);
  CHECK(airtime_status(75.0) == Status::kCaution);
  CHECK(airtime_status(120.0) == Status::kCaution);
}

// Zero post-FEC loss is the only OK state -- any unrecovered loss is
// picture damage, and >1% is where it becomes unflyable.
TEST(post_loss_status_treats_any_loss_as_at_least_caution) {
  CHECK(post_loss_status(0.0) == Status::kOk);
  CHECK(post_loss_status(0.01) == Status::kCaution);
  CHECK(post_loss_status(1.0) == Status::kCaution);
  CHECK(post_loss_status(1.01) == Status::kCritical);
}

// Critical uses the caution hue: the essential OSD has no third colour,
// escalation beyond it is the alert layer's job.
TEST(critical_renders_in_the_caution_colour) {
  CHECK(status_rgb(Status::kOk) == tok::kStatusOk);
  CHECK(status_rgb(Status::kCaution) == tok::kStatusCaution);
  CHECK(status_rgb(Status::kCritical) == tok::kStatusCaution);
}

// --- formatting -------------------------------------------------------

TEST(fmt_int_rounds_to_nearest) {
  CHECK(fmt_int(0.0) == "0");
  CHECK(fmt_int(60.4) == "60");
  CHECK(fmt_int(60.5) == "61");
  CHECK(fmt_int(-0.4) == "0");
}

TEST(fmt_one_dp_always_keeps_the_decimal) {
  CHECK(fmt_one_dp(0.0) == "0.0");
  CHECK(fmt_one_dp(2.14) == "2.1");
  CHECK(fmt_one_dp(2.15) == "2.2");
  CHECK(fmt_one_dp(24.6) == "24.6");
  CHECK(fmt_one_dp(100.0) == "100.0");
}

// RSSI uses a TRUE minus sign (U+2212), never a hyphen -- the design is
// explicit, and the hyphen is visually much shorter in this typeface.
TEST(fmt_signed_int_uses_a_true_minus_sign) {
  CHECK(fmt_signed_int(-58.0) == std::string(kMinus) + "58");
  CHECK(fmt_signed_int(-58.4) == std::string(kMinus) + "58");
  CHECK(fmt_signed_int(0.0) == "0");
  CHECK(fmt_signed_int(12.0) == "12");
  CHECK(fmt_signed_int(-58.0).find('-') == std::string::npos);
}

TEST(fmt_clock_is_mm_ss_and_saturates) {
  CHECK(fmt_clock(0) == "00:00");
  CHECK(fmt_clock(9) == "00:09");
  CHECK(fmt_clock(767) == "12:47");
  CHECK(fmt_clock(3599) == "59:59");
  // Past an hour it keeps counting minutes rather than wrapping to 00:00,
  // which would read as "recording just started" mid-flight.
  CHECK(fmt_clock(3600) == "60:00");
  CHECK(fmt_clock(5999) == "99:59");
  CHECK(fmt_clock(6000) == "99:59");   // saturate, never widen the box
  CHECK(fmt_clock(-5) == "00:00");
}

TEST(em_dash_pair_is_the_never_received_rendering) {
  CHECK(std::string(kEmDashPair) == "——");
}

// --- GsOverlay --------------------------------------------------------
#include "gs_draw.h"
#include "gs_font.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// Absolute paths to synthetic .gfont fixtures the build generated once (see
// tests/CMakeLists.txt), injected as compile definitions. The overlay needs
// every size it asks for, so the sets are:
//
//   GSFONT_DESIGN    19,21,22,24,26,34,38,56 -- the eight design sizes, i.e.
//                    exactly what layout() asks for at 1080p.
//   GSFONT_SCALED    the full 30-size bake set (kScaledSizes, below).
//   GSFONT_PARTIAL   19,24 -- most of the design set absent, so layout()
//                    must report a missing size.
//   GSFONT_OFFBYONE  18,21,23,25,33,37,55 -- every design size off by one,
//                    so pick()'s proportional tolerance has to bridge it.
//
// Read-only, and shared with the other .gfont tests: nothing here may
// remove or rewrite one.

// The bake set the shipped asset actually carries. Defined once in
// gs_scaled_sizes.h -- shared with test_gs_asset.cpp, which asserts the
// committed gs_osd.gfont bakes exactly this list -- so the synthetic
// fixture here and the real asset can never drift apart. The derivation
// and the history of getting it wrong twice live in that header.
// tests/CMakeLists.txt parses the same string out of that header to build
// GSFONT_SCALED; the first TEST below is what proves the parse right.
#include "gs_scaled_sizes.h"

// GSFONT_SCALED is generated by CMake from a regex over gs_scaled_sizes.h,
// so the C++ constant and the fixture agree only as long as that regex
// does. Checked here rather than assumed: a parse that silently dropped a
// size would turn every exact-resolution assertion below into a test of
// pick()'s tolerance instead, which is precisely the failure mode the
// header exists to rule out.
TEST(the_scaled_fixture_bakes_exactly_kScaledSizes) {
  GsFont f;
  std::string err;
  REQUIRE(f.load(GSFONT_SCALED, &err));
  int n = 0;
  const std::string list = kScaledSizes;
  for (size_t i = 0; i < list.size();) {
    size_t j = list.find(',', i);
    if (j == std::string::npos) j = list.size();
    const int px = std::atoi(list.substr(i, j - i).c_str());
    if (f.atlas(px) == nullptr) std::printf("  missing size %d\n", px);
    CHECK(f.atlas(px) != nullptr);
    ++n;
    i = j + 1;
  }
  CHECK(f.n_sizes() == n);  // ...and nothing the header does not list
}

struct FourReso { int w, h; };
constexpr FourReso kFourResolutions[] = {
    {1280, 720}, {1920, 1080}, {2560, 1440}, {3840, 2160}};

struct OverlayCanvas {
  std::vector<uint32_t> px;
  Surface s;
  OverlayCanvas(int w, int h) : px((size_t)w * h, 0u) {
    s.pixels = px.data(); s.width = w; s.height = h; s.stride_px = w;
  }
  int nonzero() const {
    int n = 0;
    for (uint32_t v : px) if (v) ++n;
    return n;
  }
};

static GsSnapshot nominal() {
  GsSnapshot s;
  s.mcs = 5;
  s.fec_pct = 25.0;
  s.air_pct = 61.0;
  s.pre_loss_pct = 2.1;
  s.post_loss_pct = 0.0;
  GsCard a; a.id = 0; a.heard = true; a.rssi_dbm = -58.0; a.snr_db = 18.0;
  GsCard b; b.id = 1; b.heard = true; b.rssi_dbm = -71.0; b.snr_db = 9.0;
  s.cards = {a, b};
  return s;
}

static GsPlayerState player_nominal() {
  GsPlayerState p;
  p.fps = 60.0; p.jitter_ms = 3.0; p.mbps = 24.6;
  p.rec.kind = RecState::Kind::kRecording;
  p.rec.elapsed_s = 767;
  return p;
}

TEST(layout_succeeds_at_the_design_resolution) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  CHECK(ov.layout(1920, 1080, &err));
  CHECK(err.empty());
}

// "19,24" stays far enough away that pick()'s 15% proportional tolerance
// (I3 in review) can't bridge it: at 1080p (scale 1.0, want == design_px)
// hero wants 56 and the nearest available is 24 -- 32 px off, ~57% of the
// request, versus a tolerance of floor(56*0.15)=8 px / 15%. Same shape of
// margin for primary (38 vs 24, 14 px / ~37% against an 5 px tolerance)
// and standard (34 vs 24, 10 px / ~29% against a 5 px tolerance). All
// three needed sizes miss by more than double their tolerance, so this
// fixture is not a boundary case that 15% could plausibly bridge later.
TEST(layout_fails_with_a_reason_when_a_size_is_missing) {
  const std::string fp = GSFONT_PARTIAL;  // most of the set absent
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  CHECK(!ov.layout(1920, 1080, &err));
  CHECK(!err.empty());
}

// Task 14 will bake the font across several resolutions, not just 1080p
// (the design doc: "at other resolutions the whole layer scales uniformly
// by height/1080"). layout() must actually succeed at all of them, not
// just the design resolution.
TEST(layout_succeeds_at_every_scaled_resolution) {
  const std::string fp = GSFONT_SCALED;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    CHECK(ov.layout(r.w, r.h, &err));
    CHECK(err.empty());
  }
}

// With the CORRECT bake set (kScaledSizes), pick() must resolve every one
// of layout()'s 7 named roles EXACTLY (d==0) at every one of the four
// resolutions -- not merely within pick()'s 15% tolerance. That tolerance
// exists as slop insurance for an imperfect real-world asset; it must
// never be load-bearing for the shipped configuration, and the ONLY way
// to prove that is to assert exact equality here rather than a distance
// bound that a wrong bake set could quietly satisfy anyway (which is
// exactly what happened with the first two kScaledSizes attempts -- see
// the comment in gs_scaled_sizes.h).
TEST(sizes_resolve_exactly_at_every_resolution) {
  const std::string fp = GSFONT_SCALED;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));

  struct Role { int design_px; GsFieldId id; };
  const Role roles[] = {
      {56, GsFieldId::kFpsValue},   {38, GsFieldId::kCard0Rssi},
      {26, GsFieldId::kLossArrow},  {24, GsFieldId::kJit},
      {22, GsFieldId::kCard0Id},    {19, GsFieldId::kLossLabel},
  };
  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    const double scale = r.h / 1080.0;
    for (const Role& role : roles) {
      const int want = (int)(role.design_px * scale + 0.5);
      const int got = ov.debug_field_atlas_px(role.id);
      REQUIRE(got > 0);
      CHECK(got == want);  // exact -- not "within tolerance"
    }
    // kRung (standard, 34 px design) is skipped at 720p: drop_top fires
    // there (see drop_top_fires_below_the_floor_not_above) and kRung is
    // never placed, so debug_field_atlas_px reads 0 -- an intentionally
    // dropped block, not a resolution failure.
    if (r.h != 720) {
      const int want = (int)(34 * scale + 0.5);
      const int got = ov.debug_field_atlas_px(GsFieldId::kRung);
      REQUIRE(got > 0);
      CHECK(got == want);
    }
  }
}

// A SEPARATE, deliberately-imprecise fixture -- every design size shifted
// 1 px off its exact value, with no exact match possible anywhere -- so
// pick() MUST rely on its 15% tolerance to succeed at all. This is what
// actually exercises the tolerance path: with the correct kScaledSizes
// above, every size resolves exactly and the tolerance branch in pick()
// never has anything to do. Keeping the two fixtures separate is
// deliberate -- a single fixture that's "sometimes exact, sometimes
// within tolerance" can't tell a reader (or a future edit) which
// property it's actually checking.
TEST(pick_tolerance_bridges_a_deliberately_off_by_one_font) {
  const std::string fp = GSFONT_OFFBYONE;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  CHECK(ov.debug_field_atlas_px(GsFieldId::kFpsValue) == 55);   // hero: 56 -> 55
  CHECK(ov.debug_field_atlas_px(GsFieldId::kCard0Rssi) == 37);  // primary: 38 -> 37
  CHECK(ov.debug_field_atlas_px(GsFieldId::kRung) == 33);       // standard: 34 -> 33
  CHECK(ov.debug_field_atlas_px(GsFieldId::kLossArrow) == 25);  // arrow: 26 -> 25
  CHECK(ov.debug_field_atlas_px(GsFieldId::kJit) == 23);        // secondary: 24 -> 23
  CHECK(ov.debug_field_atlas_px(GsFieldId::kCard0Id) == 21);    // cardid: 22 -> 21
  CHECK(ov.debug_field_atlas_px(GsFieldId::kLossLabel) == 18);  // label: 19 -> 18
}

// The handoff's responsive floor: below an 18 px RENDERED label, the top
// two blocks (rung/airtime/recording) are dropped and only the bottom two
// render. Previously dead code -- the fixed +-2 px tolerance (round 2's
// I3) meant no resolution in any plausible test font ever actually
// resolved label below 18 px. The 15% proportional tolerance makes it
// reachable: at 720p label wants (int)(19*2/3+0.5)=13 px, which IS baked
// in kScaledSizes, so drop_top fires for a real, non-contrived reason.
TEST(drop_top_fires_below_the_floor_not_above) {
  const std::string fp = GSFONT_SCALED;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));

  GsOverlay ov720(f);
  REQUIRE(ov720.layout(1280, 720, &err));
  // Top blocks: never placed at all, not merely inactive.
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kRung) == 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kAirValue) == 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kRec) == 0);
  // Bottom blocks: still render.
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kLossLabel) > 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kCard0Id) > 0);
  CHECK(ov720.debug_field_atlas_px(GsFieldId::kFpsValue) > 0);

  GsOverlay ov1080(f);
  REQUIRE(ov1080.layout(1920, 1080, &err));
  CHECK(ov1080.debug_field_atlas_px(GsFieldId::kRung) > 0);

  GsOverlay ov1440(f);
  REQUIRE(ov1440.layout(2560, 1440, &err));
  CHECK(ov1440.debug_field_atlas_px(GsFieldId::kRung) > 0);
}

TEST(first_update_draws_and_emits_rects) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  const int n = ov.update(nominal(), false, player_nominal(), c.s, &rects);
  CHECK(n > 0);
  CHECK(!rects.empty());
  CHECK(c.nonzero() > 0);
}

// The whole point of field-level tracking: an unchanged snapshot must cost
// nothing. A full repaint every 500 ms would put ~2.8 ms of quantize on a
// 2 ms loop.
TEST(identical_update_redraws_nothing) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);
  // update() documents `out` as NOT cleared -- callers batch GS and MSP
  // rects into one quantize_rects() call across an update. Testing "this
  // call added nothing" therefore means clearing between calls, the same
  // as any real caller does per pump-loop tick.
  rects.clear();
  const int n = ov.update(nominal(), false, player_nominal(), c.s, &rects);
  CHECK(n == 0);
  CHECK(rects.empty());
}

// One changed value must dirty ONE field, not the block and not the screen.
TEST(one_changed_value_dirties_one_field) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);
  rects.clear();  // see identical_update_redraws_nothing

  GsPlayerState p = player_nominal();
  p.jitter_ms = 7.0;  // JIT only
  const int n = ov.update(nominal(), false, p, c.s, &rects);
  CHECK(n == 1);
  CHECK(rects.size() == 1);
  // And the rect is small -- a field, not a block.
  CHECK(rects[0].w < 600);
  CHECK(rects[0].h < 120);
}

// The elapsed clock ticks once a second and must not drag anything with it.
TEST(rec_clock_tick_dirties_only_the_recording_field) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);
  GsPlayerState p = player_nominal();
  p.rec.elapsed_s = 768;
  CHECK(ov.update(nominal(), false, p, c.s, &rects) == 1);
}

// Staleness dims every LINK field but must leave the player-measured ones
// alone -- they are current by construction.
TEST(stale_dims_link_fields_and_leaves_player_fields_alone) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);
  rects.clear();
  const int n = ov.update(nominal(), true, player_nominal(), c.s, &rects);
  CHECK(n > 0);                 // link fields recoloured
  CHECK(n < ov.field_count());  // not everything -- but that alone is true
                                 // of nearly any partial update, so it says
                                 // nothing about WHICH half didn't redraw.

  // Specifically: no PLAYER-measured field is among the redrawn boxes --
  // fps/jit/mbps/rec must be untouched by a staleness change that has
  // nothing to do with how current the player's own measurements are.
  auto redrawn = [&](GsFieldId id) {
    const DirtyRect b = ov.debug_field_box(id);
    for (const DirtyRect& r : rects)
      if (r.x == b.x && r.y == b.y && r.w == b.w && r.h == b.h) return true;
    return false;
  };
  CHECK(!redrawn(GsFieldId::kFpsValue));
  CHECK(!redrawn(GsFieldId::kJit));
  CHECK(!redrawn(GsFieldId::kMbps));
  CHECK(!redrawn(GsFieldId::kRec));
  // And a LINK field genuinely did -- confirms the check above isn't
  // vacuously true because nothing redrew at all.
  CHECK(redrawn(GsFieldId::kRung));
}

// Never-received renders as an em-dash pair, NOT as zero. Substituting zero
// would paint a dead link as a perfect one.
TEST(absent_values_render_em_dashes_not_zero) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  GsSnapshot s;             // everything empty
  s.cards.clear();
  CHECK(ov.debug_field_text(s, false, GsPlayerState{}, GsFieldId::kLossPost) ==
        kEmDashPair);
  CHECK(ov.debug_field_text(s, false, GsPlayerState{}, GsFieldId::kAirValue) ==
        kEmDashPair);
}

TEST(zero_cards_draws_no_signal_block) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  GsSnapshot s = nominal();
  s.cards.clear();
  ov.update(s, false, player_nominal(), c.s, &rects);
  // No pixel lands in the bottom-left region at all -- no empty shell.
  int lit = 0;
  for (int y = 900; y < 1030; ++y)
    for (int x = 90; x < 560; ++x)
      if (c.px[(size_t)y * 1920 + x]) ++lit;
  CHECK(lit == 0);
}

// A card that is present but silent still renders its row.
TEST(unheard_card_renders_a_row_with_never_heard) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  GsSnapshot s = nominal();
  s.cards[1].heard = false;
  s.cards[1].rssi_dbm.reset();
  s.cards[1].snr_db.reset();
  // The em-dash pair, not the words: it has to fit a box sized for the widest
  // NUMERIC reading, or every heard row carries the unheard string's width as
  // dead space between the value and "dBm".
  CHECK(ov.debug_field_text(s, false, player_nominal(), GsFieldId::kCard1Rssi) ==
        kEmDashPair);
  // The rest of the row goes quiet, so the em-dash is unambiguous.
  CHECK(ov.debug_field_text(s, false, player_nominal(), GsFieldId::kCard1Unit).empty());
  CHECK(ov.debug_field_text(s, false, player_nominal(), GsFieldId::kCard1Snr).empty());
}

// The gap this closed: the RSSI box must be sized for the numeric, so a
// normal row's "dBm" sits right after the value rather than ~160 px away.
TEST(rssi_box_is_sized_for_the_numeric_not_the_unheard_text) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  const DirtyRect rssi = ov.debug_field_box(GsFieldId::kCard0Rssi);
  const DirtyRect unit = ov.debug_field_box(GsFieldId::kCard0Unit);
  const MaskAtlas* primary = f.atlas(38);
  REQUIRE(primary != nullptr);
  // Box width is the numeric's, plus the glyph cell's shadow pad.
  const int numeric_w = text_width(*primary, "\xE2\x88\x92" "100");
  CHECK(rssi.w <= numeric_w + primary->glyph_w);
  // And the unheard rendering fits inside it, so nothing draws past the box.
  CHECK(text_width(*primary, kEmDashPair) <= numeric_w);
  // "dBm" follows within one design gap of the numeric's right edge.
  CHECK(unit.x - (rssi.x + rssi.w) < 40);
}

// A changed card count re-lays the block out, so the next update is full --
// and, per C1 in review, the row that vanishes must actually be CLEARED,
// not just deactivated and left as the last thing drawn there.
TEST(card_count_change_forces_a_full_repaint) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;

  // Start from all kMaxCards slots active. Active rows are anchored to
  // `bottom`, so shrinking the count slides the SURVIVING rows down into
  // where the bottommost vanished rows used to be -- legitimate new
  // content, not a leftover. Only the TOPMOST row(s), which no active
  // slot ever re-occupies after shrinking to 2, give an unambiguous
  // "is the vanished pixel data actually gone" check.
  GsSnapshot four = nominal();
  four.cards.push_back(GsCard{2, true, -60.0, 15.0});
  four.cards.push_back(GsCard{3, true, -65.0, 12.0});
  REQUIRE((int)four.cards.size() == kMaxCards);
  ov.update(four, false, player_nominal(), c.s, &rects);
  const DirtyRect vacated = ov.debug_field_box(GsFieldId::kCard0Rssi);

  auto lit_in = [&](const DirtyRect& b) {
    int lit = 0;
    for (int y = std::max(0, b.y); y < std::min(c.s.height, b.y + b.h); ++y)
      for (int x = std::max(0, b.x); x < std::min(c.s.width, b.x + b.w); ++x)
        if (c.px[(size_t)y * c.s.width + x]) ++lit;
    return lit;
  };
  REQUIRE(lit_in(vacated) > 0);  // slot 0 really did draw there first

  const int n = ov.update(nominal(), false, player_nominal(), c.s, &rects);  // back to 2
  CHECK(n > 1);  // not just the one vanished row
  CHECK(lit_in(vacated) == 0);  // and it's actually erased, not just inactive
}

// Two field boxes must never overlap: draw_field_ clears a field's own box
// before drawing it, so an overlap means one field's clear can silently
// erase pixels that belong to a DIFFERENT, unrelated field. This is the
// invariant the whole per-field dirty-tracking design depends on -- review
// caught a real violation (kRung x kAirValue, kFpsValue x kMbps, from an
// earlier fix that shrank line spacing too aggressively) that no other
// test in this file would have noticed, since each field-level test only
// ever looks at ONE field at a time. Checked at all four resolutions: the
// scaled gaps and pitches recompute independently at each one, so an
// overlap introduced by a rounding edge case at, say, 1440p would not
// show up at 1080p.
//
// Deliberately scoped to ACTIVE fields only. An inactive card slot (past
// the reported count, or dropped by drop_top) keeps whatever box it was
// last assigned rather than being reset -- draw_field_ and
// repaint_intersecting are both gated on `active`, so nothing ever draws
// or clears through that box again while inactive, and a stale inactive
// box coinciding with an active one is inert, NOT a real collision. Do
// not "fix" a flagged inactive/anything pair here; it isn't a bug.
TEST(no_two_active_field_boxes_overlap_at_any_resolution) {
  const std::string fp = GSFONT_SCALED;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));

  auto overlaps = [](const DirtyRect& a, const DirtyRect& b) {
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
  };

  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));
    OverlayCanvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    // kMaxCards cards, so every card slot is active; drop_top may still
    // leave the top block's fields inactive (720p), which is exactly what
    // the active-only filter below exists to handle correctly.
    GsSnapshot s = nominal();
    s.cards.push_back(GsCard{2, true, -60.0, 15.0});
    s.cards.push_back(GsCard{3, true, -65.0, 12.0});
    REQUIRE((int)s.cards.size() == kMaxCards);
    ov.update(s, false, player_nominal(), c.s, &rects);

    std::vector<DirtyRect> boxes;
    for (int i = 0; i < ov.field_count(); ++i) {
      const GsFieldId id = (GsFieldId)i;
      if (ov.debug_field_active(id)) boxes.push_back(ov.debug_field_box(id));
    }
    int bad_pairs = 0;
    for (size_t i = 0; i < boxes.size(); ++i)
      for (size_t j = i + 1; j < boxes.size(); ++j)
        if (overlaps(boxes[i], boxes[j])) ++bad_pairs;
    CHECK(bad_pairs == 0);
  }
}

TEST(recording_states_render_distinctly) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  const GsSnapshot s = nominal();
  GsPlayerState p;
  p.rec.kind = RecState::Kind::kRecording;
  p.rec.elapsed_s = 767;
  CHECK(ov.debug_field_text(s, false, p, GsFieldId::kRec) ==
        std::string(kDotFilled) + " REC 12:47");
  p.rec.kind = RecState::Kind::kArmed;
  CHECK(ov.debug_field_text(s, false, p, GsFieldId::kRec) ==
        std::string(kDotHollow) + " REC --:--");
  p.rec.kind = RecState::Kind::kFault;
  CHECK(ov.debug_field_text(s, false, p, GsFieldId::kRec) ==
        std::string(kDotFilled) + " REC FAULT");
  p.rec.kind = RecState::Kind::kAbsent;
  CHECK(ov.debug_field_text(s, false, p, GsFieldId::kRec).empty());
}

// The MSP grid overlaps the GS corners. After MSP repaints over a field,
// that field must come back -- GS pixels always win a collision.
TEST(repaint_intersecting_redraws_only_overlapped_fields) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);

  // A rect covering the whole bottom-left corner.
  const DirtyRect msp{0, 880, 600, 200};
  std::vector<DirtyRect> out;
  const int n = ov.repaint_intersecting(&msp, 1, c.s, &out);
  CHECK(n > 0);
  CHECK(n < ov.field_count());
  CHECK(out.size() == (size_t)n);

  // A rect touching nothing repaints nothing.
  const DirtyRect middle{900, 500, 100, 100};
  out.clear();
  CHECK(ov.repaint_intersecting(&middle, 1, c.s, &out) == 0);
  CHECK(out.empty());
}

TEST(invalidate_forces_the_next_update_to_be_full) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);
  CHECK(ov.update(nominal(), false, player_nominal(), c.s, &rects) == 0);
  ov.invalidate();
  CHECK(ov.update(nominal(), false, player_nominal(), c.s, &rects) > 1);
}

// Every field must sit inside the 5 % title-safe inset -- the handoff's
// whole reason for the inset is that the outer band may be cropped.
TEST(all_fields_stay_inside_the_safe_inset) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  const DirtyRect b = ov.bounds();
  CHECK(b.x >= 96);
  CHECK(b.y >= 54);
  CHECK(b.x + b.w <= 1920 - 96);
  CHECK(b.y + b.h <= 1080 - 54);
}

// Row 2 is empty by design: the centre of frame carries no OSD.
TEST(nothing_is_drawn_in_the_centre_of_frame) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), c.s, &rects);
  int lit = 0;
  for (int y = 300; y < 780; ++y)
    for (int x = 400; x < 1520; ++x)
      if (c.px[(size_t)y * 1920 + x]) ++lit;
  CHECK(lit == 0);
}

// Same two assertions as the pair above, but at all four resolutions --
// the inset and centre-of-frame band both scale by height/1080, and a
// rounding mistake in that scaling (as opposed to the absolute-pixel
// arithmetic the 1080p-only tests exercise) would only show up off the
// design resolution.
TEST(safe_inset_and_centre_of_frame_hold_at_every_resolution) {
  const std::string fp = GSFONT_SCALED;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));

  for (const FourReso& r : kFourResolutions) {
    GsOverlay ov(f);
    REQUIRE(ov.layout(r.w, r.h, &err));

    // Safe inset: layout()'s own scaling rule (96/54 px at 1080p, scaled
    // by height/1080), not the hardcoded 1080p pixel pair -- bounds()
    // must stay inside it at every resolution, not just the design one.
    const double scale = r.h / 1080.0;
    const int inset_x = (int)(96 * scale + 0.5);
    const int inset_y = (int)(54 * scale + 0.5);
    const DirtyRect b = ov.bounds();
    CHECK(b.x >= inset_x);
    CHECK(b.y >= inset_y);
    CHECK(b.x + b.w <= r.w - inset_x);
    CHECK(b.y + b.h <= r.h - inset_y);

    // Centre of frame: the same fractional band as the 1080p-only test
    // (x:400-1520, y:300-780 of a 1920x1080 canvas), scaled by the same
    // proportion at this resolution.
    OverlayCanvas c(r.w, r.h);
    std::vector<DirtyRect> rects;
    ov.update(nominal(), false, player_nominal(), c.s, &rects);
    const int x0 = (int)(r.w * (400.0 / 1920.0));
    const int x1 = (int)(r.w * (1520.0 / 1920.0));
    const int y0 = (int)(r.h * (300.0 / 1080.0));
    const int y1 = (int)(r.h * (780.0 / 1080.0));
    int lit = 0;
    for (int y = y0; y < y1; ++y)
      for (int x = x0; x < x1; ++x)
        if (c.px[(size_t)y * r.w + x]) ++lit;
    CHECK(lit == 0);
  }
}

TEST(palette_seeds_cover_every_token) {
  size_t n = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n);
  REQUIRE(seeds != nullptr);
  CHECK(n > 0);
  // Each token must appear at full alpha somewhere in the seed set, or the
  // burned DVR quantizes GS pixels to the nearest MSP-atlas colour.
  const uint32_t want[] = {tok::kTextPrimary, tok::kTextSecondary, tok::kTextLabel,
                           tok::kTrack, tok::kStatusOk, tok::kStatusCaution,
                           tok::kStatusRec};
  for (uint32_t w : want) {
    bool found = false;
    for (size_t i = 0; i < n; ++i)
      if (seeds[i] == premul(w, 255)) { found = true; break; }
    CHECK(found);
  }
}

TEST(null_surface_update_is_safe) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));
  Surface null_s;
  std::vector<DirtyRect> rects;
  ov.update(nominal(), false, player_nominal(), null_s, &rects);
  CHECK(true);  // survived
}

// Robustness gap closed in Task 8: gs_snapshot.cpp's JSON accessors reject
// non-finite/non-numeric values but never bound a well-formed one's
// magnitude, and the formatters (fmt_int et al.) have no width limit of
// their own. draw_text clips only to the SURFACE, not to a field's box, and
// clear_region only ever clears the box -- so an absurd value (a corrupt or
// hostile datagram's air_pct: 1e300) would draw a ~300-character string
// straight through neighbouring fields and off the edge of the screen, and
// nothing would ever erase the overhang on a later frame. The chosen fix is
// to clamp every value into its sensible display range in state_of_()
// before formatting (see the comment above that switch), rather than
// adding a clip rect to draw_text: it keeps the fixed-width invariant the
// whole box-sizing scheme depends on exactly true instead of papering over
// one symptom of violating it. This asserts the invariant end to end: every
// lit pixel must fall inside SOME field's own declared box.
TEST(absurd_values_never_draw_outside_their_field_boxes) {
  const std::string fp = GSFONT_DESIGN;
  GsFont f;
  std::string err;
  REQUIRE(f.load(fp, &err));
  GsOverlay ov(f);
  REQUIRE(ov.layout(1920, 1080, &err));

  GsSnapshot s = nominal();
  s.mcs = 1000000000;
  s.fec_pct = 1e300;
  s.air_pct = 1e300;             // the brief's own example
  s.pre_loss_pct = -1e300;
  s.post_loss_pct = 1e300;
  s.cards[0].id = 999999999;
  s.cards[0].rssi_dbm = -1e300;
  s.cards[0].snr_db = 1e300;

  GsPlayerState p = player_nominal();
  p.fps = 1e300;
  p.jitter_ms = 1e300;
  p.mbps = -1e300;

  OverlayCanvas c(1920, 1080);
  std::vector<DirtyRect> rects;
  ov.update(s, false, p, c.s, &rects);
  CHECK(!rects.empty());  // the corruption didn't just silently draw nothing

  std::vector<DirtyRect> boxes;
  for (int i = 0; i < ov.field_count(); ++i)
    boxes.push_back(ov.debug_field_box((GsFieldId)i));
  auto covered = [&](int x, int y) {
    for (const DirtyRect& b : boxes)
      if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) return true;
    return false;
  };
  int stray = 0;
  for (int y = 0; y < c.s.height; ++y)
    for (int x = 0; x < c.s.width; ++x)
      if (c.px[(size_t)y * c.s.width + x] && !covered(x, y)) ++stray;
  CHECK(stray == 0);
}

MTEST_MAIN
