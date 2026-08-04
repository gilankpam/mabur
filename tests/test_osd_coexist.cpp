// The two overlays share one double-buffered surface. Every bug this file
// pins has the same shape: a piece of state that is per-buffer, refreshed by
// one publisher, and read after the OTHER publisher put that buffer on
// screen. They are invisible to a single-buffer test and were all found by
// measurement rather than by reading.
//
// Everything here drives maburplay::OsdComposer itself. The presenter is
// reduced to its actual contract -- two surfaces, a back index, and a swap
// that happens at COMMIT and only when something was published -- because
// that contract is the whole reason the bugs exist.
#include "mtest.h"

#include "gs_draw.h"
#include "gs_font.h"
#include "gs_overlay.h"
#include "mabur/msp_dp.h"
#include "osd_compose.h"
#include "osd_font.h"
#include "osd_palette.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace maburplay;

static constexpr int W = 1920, H = 1080;

// --- fonts ------------------------------------------------------------

static std::string gsfont_path() {
  static std::string p;
  if (!p.empty()) return p;
  p = std::string(std::tmpnam(nullptr)) + ".gfont";
  const std::string cmd = std::string("python3 ") + GEN_GSFONT + " --synthetic " + p +
                          " --sizes 13,14,15,16,17,19,21,22,23,24,25,26,28,29,32,34,35,"
                          "37,38,42,44,45,48,51,52,56,68,75,76,112 >/dev/null 2>&1";
  REQUIRE(std::system(cmd.c_str()) == 0);
  return p;
}

// 38x60 glyphs, so the 50x18 MSP grid is 1900x1080 -- 91% of a 1080p
// surface, close to what the shipped Betaflight font produces (~84%) and
// enough to reach every GS corner block, which is the point. Glyph gi is a
// solid opaque colour carrying gi in its low bits, so a probe pixel says
// WHICH screen a buffer is showing, not merely that it is showing one.
static std::string mspfont_path() {
  static std::string p;
  if (!p.empty()) return p;
  p = std::string(std::tmpnam(nullptr)) + ".mfont";
  std::FILE* f = std::fopen(p.c_str(), "wb");
  REQUIRE(f != nullptr);
  const int gw = 38, gh = 60;
  const uint32_t hdr[8] = {0x544E464DU, 1, (uint32_t)gw, (uint32_t)gh, 1024, 0, 0, 0};
  std::fwrite(hdr, sizeof(hdr), 1, f);
  std::vector<uint32_t> g((size_t)gw * gh);
  for (int gi = 0; gi < 1024; ++gi) {
    for (auto& px : g) px = 0xFF000000u | (uint32_t)gi;
    std::fwrite(g.data(), 4, g.size(), f);
  }
  std::fclose(f);
  return p;
}

static GsFont& gsfont() {
  static GsFont f;
  static bool once = false;
  if (!once) {
    std::string err;
    REQUIRE(f.load(gsfont_path(), &err));
    once = true;
  }
  return f;
}
static OsdFont& mspfont() {
  static OsdFont f;
  static bool once = false;
  if (!once) {
    std::string err;
    REQUIRE(f.load(mspfont_path(), &err));
    once = true;
  }
  return f;
}

// --- MSP screens ------------------------------------------------------

// Every cell of the grid set to `ch`, so the grid covers the GS corners and
// a probe anywhere in it identifies the screen.
static mabur::MspScreen full_screen(char ch) {
  std::vector<uint8_t> s;
  std::vector<uint8_t> clr = {2};
  mabur::msp_append_message(s, 182, clr.data(), clr.size());
  for (int r = 0; r < 18; ++r) {
    std::vector<uint8_t> ds = {3, (uint8_t)r, 0, 0};
    for (int c = 0; c < 50; ++c) ds.push_back((uint8_t)ch);
    mabur::msp_append_message(s, 182, ds.data(), ds.size());
  }
  std::vector<uint8_t> scr = {4};
  mabur::msp_append_message(s, 182, scr.data(), scr.size());

  mabur::MspParser parser;
  mabur::MspScreen screen;
  for (const auto& m : parser.feed(s.data(), s.size())) screen.apply(m);
  return screen;
}

// A pixel dead centre of the grid, which no GS field reaches.
static constexpr int kProbeX = 979, kProbeY = 570;
static uint32_t glyph_px(char ch) { return 0xFF000000u | (uint32_t)(uint8_t)ch; }

// --- GS inputs --------------------------------------------------------

static GsSnapshot snap_cards(int n) {
  GsSnapshot s;
  s.mcs = 5;
  s.fec_pct = 25.0;
  s.air_pct = 61.0;
  s.pre_loss_pct = 2.1;
  s.post_loss_pct = 0.0;
  for (int i = 0; i < n; ++i) {
    GsCard c;
    c.id = i;
    c.heard = true;
    c.rssi_dbm = -58.0 - i * 4.0;
    c.snr_db = 18.0 - i;
    s.cards.push_back(c);
  }
  return s;
}
static GsPlayerState player(double fps) {
  GsPlayerState p;
  p.fps = fps;
  p.jitter_ms = 3.0;
  p.mbps = 24.6;
  p.rec.kind = RecState::Kind::kRecording;
  p.rec.elapsed_s = 767;
  return p;
}

// --- the presenter's actual contract ----------------------------------

struct Buf {
  std::vector<uint32_t> px;
  Surface s;
  Buf() : px((size_t)W * H, 0u) { s = Surface{px.data(), W, H, W}; }
};

struct Rig {
  Buf buf[2];
  OsdRaster raster;
  OsdComposer comp;
  int back = 0;
  int front = -1;  // the buffer a commit last put on screen
  bool dirty = false;
  bool last_announce = false;

  explicit Rig(bool with_msp = true, bool with_gs = true) : raster(mspfont(), ScaleMode::kSharp) {
    if (with_msp) comp.set_raster(&raster);
    if (with_gs) {
      comp.set_gs(std::make_unique<GsOverlay>(gsfont()), std::make_unique<GsOverlay>(gsfont()),
                  std::make_unique<GsOverlay>(gsfont()));
      std::string err;
      REQUIRE(comp.gs_layout(W, H, &err));
    }
  }

  // One main-loop iteration's worth: the trigger, the composition, the
  // publish. Mirrors main.cpp's block, which is now only these four lines.
  bool step(const OsdComposeIn& in) {
    if (!comp.wants(in)) return false;
    const OsdComposeOut out = comp.compose(back, buf[back].s, in);
    last_announce = out.announce_blank;
    if (out.published) dirty = true;
    return true;
  }
  void commit() {
    if (dirty) {
      dirty = false;
      front = back;
      back ^= 1;
    }
  }
  // step() then commit(), which is what a live iteration followed by a video
  // frame does.
  bool tick(const OsdComposeIn& in) {
    const bool ran = step(in);
    commit();
    return ran;
  }
  uint32_t px(int b, int x, int y) const { return buf[b].px[(size_t)y * W + x]; }
};

static OsdComposeIn make_in(const mabur::MspScreen* scr, bool fresh, bool stale,
                            const GsSnapshot* snap, const GsPlayerState& ps, bool gs_dirty) {
  OsdComposeIn in;
  in.screen = scr;
  in.msp_fresh = fresh;
  in.msp_stale = stale;
  in.snap = snap;
  in.gs_ps = ps;
  in.gs_dirty = gs_dirty;
  return in;
}

// The picture the screen is supposed to be showing, composed from nothing.
static std::vector<uint32_t> reference(const OsdComposeIn& in, bool with_msp, bool with_gs,
                                       bool seen_screen = true) {
  Rig r(with_msp, with_gs);
  OsdComposeIn ref = in;
  // A virgin composer has never seen a screen, so it needs the fresh flag to
  // latch one; a blanked grid, or a run in which no screen has ever arrived,
  // is a grid it simply never draws.
  ref.msp_fresh = seen_screen && !in.msp_stale;
  ref.gs_dirty = true;
  r.comp.compose(0, r.buf[0].s, ref);
  return r.buf[0].px;
}

static int diff_px(const std::vector<uint32_t>& a, const std::vector<uint32_t>& b) {
  int n = 0;
  for (size_t i = 0; i < a.size(); ++i)
    if (a[i] != b[i]) ++n;
  return n;
}
static int diff_box(const std::vector<uint32_t>& a, const std::vector<uint32_t>& b,
                    const DirtyRect& r) {
  int n = 0;
  for (int y = r.y; y < r.y + r.h; ++y)
    for (int x = r.x; x < r.x + r.w; ++x) {
      if (y < 0 || y >= H || x < 0 || x >= W) continue;
      if (a[(size_t)y * W + x] != b[(size_t)y * W + x]) ++n;
    }
  return n;
}
// Summed over every active GS field box, which is where the collision lives.
static int diff_gs_fields(const Rig& r, int b, const std::vector<uint32_t>& ref, int* boxes) {
  GsOverlay probe(gsfont());
  std::string err;
  REQUIRE(probe.layout(W, H, &err));
  int n = 0, cnt = 0;
  for (int f = 0; f < (int)GsFieldId::kCount; ++f) {
    const GsFieldId id = (GsFieldId)f;
    if (!probe.debug_field_active(id)) continue;
    const DirtyRect box = probe.debug_field_box(id);
    if (box.w <= 0 || box.h <= 0) continue;
    ++cnt;
    n += diff_box(r.buf[b].px, ref, box);
  }
  if (boxes) *boxes = cnt;
  return n;
}

// ======================================================================
// (b) A card that disappears
// ======================================================================

TEST(a_vanished_card_row_is_cleared_from_both_buffers) {
  const GsSnapshot s4 = snap_cards(4), s2 = snap_cards(2);
  const GsPlayerState ps = player(60.0);
  Rig r(false, true);
  for (int i = 0; i < 2; ++i) r.tick(make_in(nullptr, false, false, &s4, ps, true));
  for (int i = 0; i < 4; ++i) r.tick(make_in(nullptr, false, false, &s2, ps, true));

  const auto ref = reference(make_in(nullptr, false, false, &s2, ps, true), false, true);
  CHECK(diff_px(r.buf[0].px, ref) == 0);
  CHECK(diff_px(r.buf[1].px, ref) == 0);
}

// Non-vacuity. The hazard is real and is a property of GsOverlay, not of the
// composer: repaint_intersecting() is gated on `active`, so it can never
// CLEAR anything, and one overlay clearing a vacated row clears it in
// whichever buffer happened to be back. Driven directly, not through the
// composer, so this control cannot drift with it.
TEST(one_gs_overlay_cannot_clear_the_buffer_it_was_not_drawing_into) {
  const GsSnapshot s4 = snap_cards(4), s2 = snap_cards(2);
  const GsPlayerState ps = player(60.0);
  GsOverlay one(gsfont());
  std::string err;
  REQUIRE(one.layout(W, H, &err));
  Buf b[2];
  int back = 0;
  auto step = [&](const GsSnapshot& sn) {
    std::vector<DirtyRect> d;
    one.update(sn, false, ps, b[back].s, &d);
    if (!d.empty()) back ^= 1;
  };
  for (int i = 0; i < 2; ++i) step(s4);
  for (int i = 0; i < 4; ++i) step(s2);
  const auto ref = reference(make_in(nullptr, false, false, &s2, ps, true), false, true);
  CHECK(diff_px(b[0].px, ref) + diff_px(b[1].px, ref) > 0);
}

// ======================================================================
// (c) The MSP raster over GS pixels
// ======================================================================

TEST(the_msp_grid_never_destroys_a_gs_field_in_either_buffer) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, false, false, &sn, ps, true));
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &sn, ps, false));

  const auto ref = reference(make_in(&scr, true, false, &sn, ps, true), true, true);
  int boxes = 0;
  CHECK(diff_gs_fields(r, 0, ref, &boxes) == 0);
  CHECK(diff_gs_fields(r, 1, ref, nullptr) == 0);
  CHECK(boxes >= 13);  // the fixture must actually have fields to destroy
  // ...and the MSP grid really is on the surface, so this is not passing
  // because nothing was drawn.
  CHECK(r.px(0, kProbeX, kProbeY) == glyph_px('X'));
  CHECK(r.px(1, kProbeX, kProbeY) == glyph_px('X'));
}

// Non-vacuity: OsdRaster::draw() clears a whole cell before blitting, so
// without a reclaim the grid takes every GS pixel under it.
TEST(without_a_reclaim_the_msp_grid_does_destroy_them) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  GsOverlay ov(gsfont());
  OsdRaster raster(mspfont(), ScaleMode::kSharp);
  std::string err;
  REQUIRE(ov.layout(W, H, &err));
  Buf b;
  ShadowGrid sh;
  ov.update(sn, false, ps, b.s, nullptr);
  raster.draw(scr, b.s, &sh);
  const auto ref = reference(make_in(&scr, true, false, &sn, ps, true), true, true);
  Rig probe(true, true);  // only to reuse diff_gs_fields' box enumeration
  probe.buf[0].px = b.px;
  CHECK(diff_gs_fields(probe, 0, ref, nullptr) > 0);
}

// ======================================================================
// C1 -- the stale blank must reach BOTH buffers
// ======================================================================

TEST(the_msp_stale_blank_reaches_the_screen) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &sn, ps, true));
  REQUIRE(r.px(0, kProbeX, kProbeY) == glyph_px('X'));
  REQUIRE(r.px(1, kProbeX, kProbeY) == glyph_px('X'));

  // The source goes quiet. Drive it until the composer stops asking.
  const OsdComposeIn stale = make_in(&scr, false, true, &sn, ps, false);
  int rounds = 0;
  while (r.step(stale)) {
    ++rounds;
    r.commit();
    REQUIRE(rounds < 8);  // it has to settle
  }
  CHECK(rounds == 1);                            // one composition reaches the screen
  CHECK(r.px(r.front, kProbeX, kProbeY) == 0u);  // and the screen is blank
  CHECK(r.comp.screen_blank());
}

// The screen is what the blank has to reach, and the screen is whichever
// buffer the next commit puts up -- so the guarantee is not "both buffers
// are blank now" but "no publish during a stale episode can put a lit grid
// up". The other buffer is cleaned in the composition that publishes it.
TEST(no_publish_during_a_stale_episode_scans_out_a_lit_grid) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &sn, ps, true));
  const OsdComposeIn stale = make_in(&scr, false, true, &sn, ps, false);
  while (r.step(stale)) r.commit();
  // Non-vacuity: the buffer NOT on screen still carries the grid, which is
  // exactly the frame that must never be committed.
  CHECK(r.px(r.front ^ 1, kProbeX, kProbeY) == glyph_px('X'));

  // Now the GS overlay publishes, repeatedly, while the source stays quiet.
  for (int i = 0; i < 6; ++i) {
    const OsdComposeIn gs = make_in(&scr, false, true, &sn, player(50.0 - i), true);
    REQUIRE(r.step(gs));
    r.commit();
    CHECK(r.px(r.front, kProbeX, kProbeY) == 0u);
  }
}

// N1. Publishing is what puts a blank on screen, and it publishes the BACK
// buffer -- so a trigger asking whether the back buffer is blank answers the
// wrong question. With one screen ever drawn and then silence, the back
// buffer has never held a grid, so it is already blank, so nothing composes,
// so nothing publishes, and the front keeps the frozen flight OSD past
// osd.stale_ms forever. No GS overlay here on purpose: an MSP-only run is
// the configuration shipping today and has no second publisher to rescue it.
TEST(a_stale_blank_reaches_the_screen_with_no_second_publisher) {
  const mabur::MspScreen scr = full_screen('X');
  Rig r(/*with_msp=*/true, /*with_gs=*/false);
  OsdComposeIn fresh;
  fresh.screen = &scr;
  fresh.msp_fresh = true;
  REQUIRE(r.tick(fresh));
  REQUIRE(r.px(r.front, kProbeX, kProbeY) == glyph_px('X'));
  // The deadlock precondition, read off the composer: the buffer a publish
  // would scan out is already blank while the one on screen is lit.
  REQUIRE(r.comp.blanked(r.back));
  REQUIRE(!r.comp.blanked(r.front));

  OsdComposeIn stale;
  stale.screen = &scr;
  stale.msp_stale = true;
  int rounds = 0;
  while (r.step(stale)) {
    ++rounds;
    r.commit();
    REQUIRE(rounds < 8);
  }
  CHECK(rounds >= 1);
  CHECK(r.px(r.front, kProbeX, kProbeY) == 0u);
}

// Non-vacuity, taken from the composer's own intermediate state rather than
// from a copy of the old code: after the FIRST blank composition the other
// buffer is still lit, which is exactly the frame a single `osd_blanked`
// latch would have left on screen forever. A GS publish then swaps to it.
TEST(one_blank_composition_leaves_the_other_buffer_lit) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &sn, ps, true));
  const int first = r.back;
  r.tick(make_in(&scr, false, true, &sn, ps, false));
  CHECK(r.px(first, kProbeX, kProbeY) == 0u);
  CHECK(r.px(first ^ 1, kProbeX, kProbeY) == glyph_px('X'));
  // And the composer knows it still owes that buffer a blank.
  CHECK(r.comp.blanked(first));
  CHECK(!r.comp.blanked(first ^ 1));
}

TEST(the_blank_is_announced_once_per_episode_not_once_per_buffer) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &sn, ps, true));
  int announces = 0;
  const OsdComposeIn stale = make_in(&scr, false, true, &sn, ps, false);
  while (r.step(stale)) {
    if (r.last_announce) ++announces;
    r.commit();
  }
  // ...including the GS publishes that clean the second buffer.
  for (int i = 0; i < 4; ++i) {
    if (r.step(make_in(&scr, false, true, &sn, player(50.0 - i), true)) && r.last_announce)
      ++announces;
    r.commit();
  }
  CHECK(announces == 1);
}

TEST(the_stale_blank_leaves_the_gs_overlay_intact_in_both_buffers) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &sn, ps, true));
  const OsdComposeIn stale = make_in(&scr, false, true, &sn, ps, false);
  while (r.step(stale)) r.commit();
  // One more publish, which is what cleans the buffer that was not back.
  r.tick(make_in(&scr, false, true, &sn, ps, true));

  // Blanked means blanked of MSP: the surface must now be EXACTLY the GS
  // overlay, in both buffers.
  const auto ref = reference(make_in(nullptr, false, false, &sn, ps, true), false, true);
  CHECK(diff_px(r.buf[0].px, ref) == 0);
  CHECK(diff_px(r.buf[1].px, ref) == 0);
}

// ======================================================================
// I1 -- a GS-only publish must not scan out a stale MSP grid
// ======================================================================

TEST(a_gs_only_publish_does_not_scan_out_a_stale_msp_grid) {
  const mabur::MspScreen a = full_screen('A'), b = full_screen('B');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  r.tick(make_in(&a, true, false, &sn, ps, true));  // screen A into one buffer
  r.tick(make_in(&b, true, false, &sn, ps, false)); // screen B into the other
  // A GS figure moves; no new MSP screen. This publishes the buffer that
  // last saw screen A.
  r.tick(make_in(&b, false, false, &sn, player(45.0), true));
  CHECK(r.px(r.front, kProbeX, kProbeY) == glyph_px('B'));
  // And again, the other way round.
  r.tick(make_in(&b, false, false, &sn, player(44.0), true));
  CHECK(r.px(r.front, kProbeX, kProbeY) == glyph_px('B'));
}

// Non-vacuity: the buffer really was a screen behind before the composition
// touched it. Read off the composer's own shadow bookkeeping.
TEST(the_other_buffer_is_a_screen_behind_until_it_is_composed) {
  const mabur::MspScreen a = full_screen('A'), b = full_screen('B');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState ps = player(60.0);
  Rig r;
  r.tick(make_in(&a, true, false, &sn, ps, true));
  const int behind = r.back;  // never saw screen A... nor anything
  r.tick(make_in(&b, true, false, &sn, ps, false));
  const int stale_buf = r.back;  // saw A, not B
  CHECK(r.px(stale_buf, kProbeX, kProbeY) == glyph_px('A'));
  CHECK(behind != stale_buf);  // tick() swapped; `behind` is the one just drawn
  // The very next composition into it, GS-triggered, brings it forward.
  r.step(make_in(&b, false, false, &sn, player(45.0), true));
  CHECK(r.px(stale_buf, kProbeX, kProbeY) == glyph_px('B'));
}

TEST(an_msp_publish_scans_out_the_current_gs_sample) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  const GsPlayerState v1 = player(60.0), v2 = player(45.0);
  Rig r;
  r.tick(make_in(&scr, true, false, &sn, v1, true));
  r.tick(make_in(&scr, true, false, &sn, v2, true));
  // An MSP-only publish into the buffer that last saw v1.
  r.tick(make_in(&scr, true, false, &sn, v2, false));
  const auto ref = reference(make_in(&scr, true, false, &sn, v2, true), true, true);
  Rig tmp(true, true);
  tmp.buf[0].px = r.buf[r.front].px;
  CHECK(diff_gs_fields(tmp, 0, ref, nullptr) == 0);
}

// ======================================================================
// The publish rule
// ======================================================================

// fps alternating 60/59 is the case a per-buffer diff cannot see: each
// buffer keeps matching its own previous composition, so nothing is drawn --
// and the buffer that is ON SCREEN is the other one, holding the other
// value. Gating the publish on "something changed" strands it there.
TEST(an_alternating_value_draws_nothing_yet_must_still_be_published) {
  const GsSnapshot sn = snap_cards(3);
  Rig r;
  int silent = 0;
  for (int i = 0; i < 8; ++i) {
    const GsPlayerState ps = player(i % 2 ? 59.0 : 60.0);
    const OsdComposeIn in = make_in(nullptr, false, false, &sn, ps, true);
    REQUIRE(r.comp.wants(in));
    const OsdComposeOut out = r.comp.compose(r.back, r.buf[r.back].s, in);
    if (out.gs_drawn == 0) ++silent;      // a "publish only if drawn" would skip
    CHECK(out.published);                 // the composer says publish anyway
    if (out.published) r.dirty = true;
    r.commit();
    const auto ref = reference(in, false, true);
    CHECK(diff_px(r.buf[r.front].px, ref) == 0);
  }
  CHECK(silent > 0);  // the control: some compositions really did draw nothing
}

// ======================================================================
// The burned DVR's index map
// ======================================================================

// Stand-in for BurnRecorder, which needs the MPP encoder and cannot build on
// the host. The incremental-then-fall-back-to-full rule is
// BurnRecorder::set_osd()'s, not a reinvention.
struct FakeBurn {
  OsdPalette pal;
  OsdIndexMap map;
  QuantizeCache qc;
  void feed(const Surface& s, const DirtyRect* r, size_t n) {
    if (!quantize_rects(s, pal, r, n, &map, &qc)) quantize(s, pal, &map, &qc);
  }
  int wrong_against(const Surface& s) const {
    OsdIndexMap truth;
    quantize(s, pal, &truth);
    int n = 0;
    for (size_t i = 0; i < truth.px.size(); ++i)
      if (truth.px[i] != map.px[i]) ++n;
    return n;
  }
};

static OsdPalette seeded_palette() {
  size_t n = 0;
  const uint32_t* seeds = GsOverlay::palette_seeds(&n);
  return build_palette(mspfont().native(), seeds, n);
}

TEST(the_burn_index_map_tracks_the_screen_through_an_alternating_value) {
  const GsSnapshot sn = snap_cards(3);
  Rig r(false, true);
  FakeBurn fb;
  fb.pal = seeded_palette();
  r.comp.set_burn_sink(
      [&fb](const Surface& s, const DirtyRect* p, size_t n) { fb.feed(s, p, n); });
  int worst = 0;
  for (int i = 0; i < 8; ++i) {
    const GsPlayerState ps = player(i % 2 ? 59.0 : 60.0);
    r.step(make_in(nullptr, false, false, &sn, ps, true));
    const int w = fb.wrong_against(r.buf[r.back].s);
    if (w > worst) worst = w;
    r.commit();
  }
  CHECK(worst == 0);
}

// Non-vacuity: the same sequence fed the rects a single per-buffer overlay
// yields. Driven directly, so it cannot drift with the composer.
TEST(a_per_buffer_rect_list_would_not_keep_the_burn_map_current) {
  const GsSnapshot sn = snap_cards(3);
  GsOverlay ov[2] = {GsOverlay(gsfont()), GsOverlay(gsfont())};
  std::string err;
  REQUIRE(ov[0].layout(W, H, &err));
  REQUIRE(ov[1].layout(W, H, &err));
  Buf b[2];
  FakeBurn fb;
  fb.pal = seeded_palette();
  int back = 0, worst = 0;
  for (int i = 0; i < 8; ++i) {
    const GsPlayerState ps = player(i % 2 ? 59.0 : 60.0);
    std::vector<DirtyRect> d;
    ov[back].update(sn, false, ps, b[back].s, &d);
    if (i == 0)
      fb.feed(b[back].s, nullptr, 0);  // sizes the map, as production's first call does
    else if (!d.empty())
      fb.feed(b[back].s, d.data(), d.size());
    const int w = fb.wrong_against(b[back].s);
    if (w > worst) worst = w;
    back ^= 1;
  }
  CHECK(worst > 0);
}

TEST(the_burn_map_survives_an_msp_collision_and_a_stale_blank) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  Rig r;
  FakeBurn fb;
  fb.pal = seeded_palette();
  r.comp.set_burn_sink(
      [&fb](const Surface& s, const DirtyRect* p, size_t n) { fb.feed(s, p, n); });
  int worst = 0;
  auto run = [&](const OsdComposeIn& in) {
    if (!r.step(in)) return;
    const int w = fb.wrong_against(r.buf[r.back].s);
    if (w > worst) worst = w;
    r.commit();
  };
  for (int i = 0; i < 4; ++i)
    run(make_in(&scr, true, false, &sn, player(60.0 - i), true));
  const OsdComposeIn stale = make_in(&scr, false, true, &sn, player(56.0), false);
  for (int i = 0; i < 4; ++i) run(stale);
  // GS publishes DURING the stale episode: these compose the buffer the
  // blank never reached, so the map has to follow them too.
  for (int i = 0; i < 4; ++i)
    run(make_in(&scr, false, true, &sn, player(56.0 - i), true));
  for (int i = 0; i < 4; ++i)
    run(make_in(&scr, true, false, &sn, player(50.0 - i), true));
  CHECK(worst == 0);
}

// N2. The burn's index map is refreshed per COMPOSITION, and a composition
// that publishes a blank may be one whose buffer was already clear -- it
// never held this episode's grid. Keying the burn's blank feed off "this
// buffer was cleared just now" misses exactly that composition, and the map
// then carries a flight OSD the screen has already dropped for the whole
// stale episode. The interleaving is the reviewer's: lit, blank, ONE screen
// back, quiet again, then a GS publish.
TEST(the_burn_map_does_not_keep_a_grid_the_screen_dropped) {
  const mabur::MspScreen scr = full_screen('X');
  const GsSnapshot sn = snap_cards(3);
  Rig r;
  FakeBurn fb;
  fb.pal = seeded_palette();
  r.comp.set_burn_sink(
      [&fb](const Surface& s, const DirtyRect* p, size_t n) { fb.feed(s, p, n); });
  int worst = 0, steps = 0;
  auto run = [&](const OsdComposeIn& in) {
    if (!r.step(in)) return;
    ++steps;
    const int w = fb.wrong_against(r.buf[r.back].s);
    if (w > worst) worst = w;
    r.commit();
  };
  // lit
  for (int i = 0; i < 3; ++i) run(make_in(&scr, true, false, &sn, player(60.0 - i), true));
  // the source goes quiet
  const OsdComposeIn quiet1 = make_in(&scr, false, true, &sn, player(57.0), false);
  while (r.comp.wants(quiet1)) run(quiet1);
  // exactly ONE screen comes back...
  run(make_in(&scr, true, false, &sn, player(56.0), false));
  // ...and it goes quiet again. This is the composition that publishes a
  // buffer which is already clear.
  const OsdComposeIn quiet2 = make_in(&scr, false, true, &sn, player(55.0), false);
  while (r.comp.wants(quiet2)) run(quiet2);
  // ...then a GS-only publish during the same episode.
  run(make_in(&scr, false, true, &sn, player(54.0), true));
  CHECK(steps >= 6);   // the interleaving must actually have been walked
  CHECK(worst == 0);
  // Non-vacuity: the screen really did drop the grid at the end, so there
  // was something for the map to be wrong about.
  CHECK(r.px(r.front, kProbeX, kProbeY) == 0u);
}

// ======================================================================
// A GS box that moves punches a hole in the MSP grid underneath
// ======================================================================

// The area a card row vacates. Taken from a throwaway overlay so the test
// knows the geometry without reimplementing place_card_row_().
static DirtyRect vacated_by_shrinking_cards(const GsPlayerState& ps) {
  GsOverlay probe(gsfont());
  std::string err;
  REQUIRE(probe.layout(W, H, &err));
  Buf b;
  probe.update(snap_cards(4), false, ps, b.s, nullptr);
  const DirtyRect four = probe.debug_field_box(GsFieldId::kCard0Rssi);
  probe.update(snap_cards(1), false, ps, b.s, nullptr);
  const DirtyRect one = probe.debug_field_box(GsFieldId::kCard0Rssi);
  // With one card the block hugs the bottom, so slot 0's four-card position
  // is well clear of it -- assert that rather than assume it.
  REQUIRE(four.y + four.h <= one.y);
  return four;
}

// GsOverlay clears a box it is about to vacate, and that erase goes straight
// through to the MSP glyphs underneath while the MSP shadow still records
// those cells as painted. Left alone it is a transparent hole in the flight
// OSD, the shape of a card row, lasting until whatever text was in those
// cells happens to change.
TEST(a_vacated_card_row_does_not_leave_a_hole_in_the_msp_grid) {
  const mabur::MspScreen scr = full_screen('X');
  const GsPlayerState ps = player(60.0);
  const DirtyRect vac = vacated_by_shrinking_cards(ps);
  const int px = vac.x + vac.w / 2, py = vac.y + vac.h / 2;

  const GsSnapshot s4 = snap_cards(4), s1 = snap_cards(1);
  Rig r;
  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, true, false, &s4, ps, true));
  // Non-vacuity: with four cards that pixel really is the overlay's, not the
  // grid's, so the check below is not measuring an area nothing ever used.
  CHECK(r.px(0, px, py) != glyph_px('X'));
  CHECK(r.px(1, px, py) != glyph_px('X'));

  for (int i = 0; i < 2; ++i) r.tick(make_in(&scr, false, false, &s1, ps, true));
  CHECK(r.px(0, px, py) == glyph_px('X'));
  CHECK(r.px(1, px, py) == glyph_px('X'));
}

// Non-vacuity for the repair itself: the raw sequence, with no composer,
// leaves the hole. This is what the flight OSD did before the repair.
TEST(without_the_repair_the_vacated_row_stays_transparent) {
  const mabur::MspScreen scr = full_screen('X');
  const GsPlayerState ps = player(60.0);
  const DirtyRect vac = vacated_by_shrinking_cards(ps);
  const int px = vac.x + vac.w / 2, py = vac.y + vac.h / 2;

  GsOverlay ov(gsfont());
  OsdRaster raster(mspfont(), ScaleMode::kSharp);
  std::string err;
  REQUIRE(ov.layout(W, H, &err));
  Buf b;
  ShadowGrid sh;
  raster.draw(scr, b.s, &sh);
  ov.update(snap_cards(4), false, ps, b.s, nullptr);
  ov.update(snap_cards(1), false, ps, b.s, nullptr);
  raster.draw(scr, b.s, &sh);  // the next screen: unchanged cells stay put
  CHECK(b.px[(size_t)py * W + px] == 0u);
}

// ======================================================================
// The invariant, over interleavings nobody thought to enumerate
// ======================================================================

// After every commit the front buffer must equal a from-scratch composition
// of the state at that moment -- in BOTH layers. Every bug above is one
// instance of this failing; the interleaving is what makes them hard to find
// by hand, so it is generated rather than chosen.
TEST(a_committed_buffer_always_matches_a_from_scratch_composition) {
  const mabur::MspScreen screens[3] = {full_screen('A'), full_screen('B'), full_screen('C')};
  int total_checked = 0;
  // Both topologies. MSP-only has no second publisher, which is what made
  // N1 unreachable there and invisible everywhere else.
  for (bool with_gs : {true, false}) {
    for (uint32_t seed : {0x2026u, 0x51ceu, 0xbeadu}) {
      Rig r(true, with_gs);
      FakeBurn fb;
      fb.pal = seeded_palette();
      r.comp.set_burn_sink(
          [&fb](const Surface& s, const DirtyRect* p, size_t n) { fb.feed(s, p, n); });
      uint32_t rng = seed;
      auto next = [&rng]() { return (rng = rng * 1103515245u + 12345u) >> 16; };

      int cur_screen = 0;
      bool stale = false, seen = false;
      int cards = 3;
      double fps = 60.0;
      GsSnapshot sn = snap_cards(cards);
      int checked = 0;
      for (int i = 0; i < 300; ++i) {
        const unsigned roll = next() % 10;
        bool fresh = false, gsd = false;
        if (roll < 4) {  // a new MSP screen
          cur_screen = (int)(next() % 3);
          fresh = true;
          seen = true;
          stale = false;
        } else if (roll < 5) {  // the MSP source goes quiet
          stale = true;
        } else if (roll < 7) {  // a GS figure moves
          fps = 40.0 + (double)(next() % 25);
          gsd = true;
        } else if (roll < 8) {  // the card set changes
          cards = (int)(next() % 5);
          sn = snap_cards(cards);
          gsd = true;
        }
        OsdComposeIn in = make_in(&screens[cur_screen], fresh, stale, with_gs ? &sn : nullptr,
                                  player(fps), gsd);
        if (!r.step(in)) continue;
        // The burned DVR's index map is a third surface and has to track the
        // one being scanned out, not merely the one it last heard about.
        const int map_wrong = fb.wrong_against(r.buf[r.back].s);
        if (map_wrong != 0)
          std::printf("  gs=%d seed %u iteration %d: burn map %d bytes wrong\n",
                      (int)with_gs, seed, i, map_wrong);
        CHECK(map_wrong == 0);
        r.commit();
        const auto ref = reference(in, true, with_gs, seen);
        const int wrong = diff_px(r.buf[r.front].px, ref);
        if (wrong != 0) {
          size_t k = 0;
          while (r.buf[r.front].px[k] == ref[k]) ++k;
          std::printf("  gs=%d seed %u iteration %d: %d px wrong, first at (%d,%d) %08X != %08X\n",
                      (int)with_gs, seed, i, wrong, (int)(k % W), (int)(k / W),
                      r.buf[r.front].px[k], ref[k]);
        }
        CHECK(wrong == 0);
        ++checked;
      }
      CHECK(checked > 50);  // the walk must actually have published things
      total_checked += checked;
    }
  }
  CHECK(total_checked > 300);
}

MTEST_MAIN
