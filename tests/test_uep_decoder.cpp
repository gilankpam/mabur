#include <map>
#include <random>
#include <set>
#include "mtest.h"
#include "frame_fixture.h"
#include "vectors.h"
#include "mabur/frame_wire.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
using namespace mabur;

// --- helpers copied verbatim from tests/test_uep_sw.cpp (self-contained) ---
namespace {

// One frame unit for `stream`: FrameHdr + one Annex-B NAL whose type/tid match
// the layer (type 32 = VPS -> critical/stream 0; type 1 = TRAIL_R with
// nuh_temporal_id_plus1 = stream for streams 1..3), then random payload.
std::vector<uint8_t> make_unit(int stream, uint32_t frame_id, size_t paylen,
                               std::mt19937& rng) {
  std::vector<uint8_t> unit(framewire::kFrameHdrLen);
  framewire::FrameHdr h;
  h.frame_id = static_cast<uint16_t>(frame_id);
  h.flags = stream == 0 ? framewire::kFlagIdr : 0;
  h.pts_us = frame_id * 16667u;
  framewire::pack_frame_hdr(h, unit.data());
  for (uint8_t b : {0x00, 0x00, 0x00, 0x01}) unit.push_back(b);
  if (stream == 0) {
    unit.push_back(32 << 1);  // VPS -> critical
    unit.push_back(1);
  } else {
    unit.push_back(1 << 1);   // TRAIL_R
    unit.push_back(static_cast<uint8_t>(stream));  // tid_plus1 -> stream 1..3
  }
  while (unit.size() < framewire::kFrameHdrLen + paylen)
    unit.push_back(static_cast<uint8_t>(rng()));
  return unit;
}

std::array<UepLayerCfg, 4> layers_for(int symbol_size, int bpb_base, int window) {
  std::array<UepLayerCfg, 4> layers{};
  const int bpb[4] = {bpb_base, bpb_base, bpb_base * 2, bpb_base * 2};
  for (int s = 0; s < 4; ++s) {
    layers[static_cast<size_t>(s)].fec =
        SwConfig{symbol_size, window, kUepRefOverhead[s]};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  return layers;
}

}  // namespace

// symbol_size=64, blocks_per_body=4 on every stream, per-stream overheads =
// the reference ladder (decoder ignores overhead; window is TX-side only).
static std::array<UepLayerCfg, 4> vec_layers() {
  std::array<UepLayerCfg, 4> L{};
  const double ov[4] = {1.00, 0.50, 0.50, 0.50};
  for (int s = 0; s < 4; ++s) L[s] = UepLayerCfg{SwConfig{64, 128, ov[s]}, 4};
  return L;
}

static std::vector<mtest::FrameRecord> fixture_frames() {
  return mtest::load_frame_fixture(std::string(MABUR_FIXTURE_DIR) + "/frame_stream.bin");
}

// The wire units maburd sends for the fixture, grouped per layer.
static std::map<int, std::vector<std::string>> fixture_by_stream() {
  std::map<int, std::vector<std::string>> out;
  auto frames = fixture_frames();
  for (size_t i = 0; i < frames.size(); ++i)
    out[frames[i].stream_id()].push_back(
        mtest::hex(mtest::frame_unit(frames[i], static_cast<uint16_t>(i))));
  return out;
}

// Bodies produced by a real UepEncoder over the fixture stream — the
// sliding-window scheme has no separate golden-vector wire format to pin
// (see test_uep.cpp), so the decoder is exercised against its own encoder's
// output, same as the drone/GS pairing on air.
static std::vector<UepBody> encode_fixture_bodies() {
  UepEncoder enc(vec_layers(), /*flush_ms=*/1'000'000'000ULL);
  std::vector<UepBody> bodies;
  auto frames = fixture_frames();
  for (size_t i = 0; i < frames.size(); ++i) {
    auto unit = mtest::frame_unit(frames[i], static_cast<uint16_t>(i));
    for (auto& b : enc.add_frame(frames[i].stream_id(), unit.data(), unit.size(), 0))
      bodies.push_back(std::move(b));
  }
  for (auto& b : enc.flush_all()) bodies.push_back(std::move(b));
  return bodies;
}

static void feed_and_check(bool duplicate_bodies) {
  auto bodies = encode_fixture_bodies();
  UepDecoder dec(vec_layers());
  mtest::FragCollector got;
  for (auto& b : bodies) {
    int reps = duplicate_bodies ? 2 : 1;
    for (int r = 0; r < reps; ++r)
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), 0)) got.add(d);
  }
  std::map<int, std::vector<std::string>> recovered;
  for (auto& [sid, unit] : got.completed()) recovered[sid].push_back(mtest::hex(unit));
  auto want = fixture_by_stream();
  for (auto& [sid, units] : want) {
    // Duplicates must NOT double output: a re-fed body's fragments are
    // dropped as known seqs inside SwDecoder, so no unit completes twice.
    REQUIRE(recovered[sid].size() == units.size());
    for (size_t i = 0; i < units.size(); ++i) CHECK(recovered[sid][i] == units[i]);
  }
  CHECK(dec.bodies_misrouted() == 0);
}

TEST(decodes_uep_encoder_bodies_to_fixture) { feed_and_check(false); }
TEST(duplicate_bodies_are_idempotent) { feed_and_check(true); }

TEST(garbage_body_is_misrouted_not_fatal) {
  UepDecoder dec(vec_layers());
  std::vector<uint8_t> junk(40, 0x5A);
  CHECK(dec.add_body(junk.data(), junk.size(), 0).empty());
  CHECK(dec.bodies_misrouted() == 1);
}

TEST(window_delivery_full_on_clean_stream) {
  auto bodies = encode_fixture_bodies();
  UepDecoder dec(vec_layers());
  for (auto& b : bodies) dec.add_body(b.body.data(), b.body.size(), 0);
  for (int s = 0; s < 4; ++s) CHECK(dec.window_delivery_pct(s) == 100);
  dec.reset_window();
  for (int s = 0; s < 4; ++s) CHECK(dec.window_delivery_pct(s) == 100);  // empty = 100
}

TEST(layer_stats_expose_recovered_arrived) {
  // Hold back one stream-1 body: later repairs recover its source symbols;
  // feeding it afterwards is the direct-copy-lost-the-race case and must
  // surface in LayerStats.syms_recovered_arrived for the s1 health feed.
  auto bodies = encode_fixture_bodies();
  UepDecoder dec(vec_layers());
  std::vector<uint8_t> held;
  for (auto& b : bodies) {
    if (held.empty() && b.stream_id == 1) { held = b.body; continue; }
    dec.add_body(b.body.data(), b.body.size(), 0);
  }
  REQUIRE(!held.empty());
  REQUIRE(dec.stats(1).syms_recovered >= 1);
  CHECK(dec.stats(1).syms_recovered_arrived == 0);
  dec.add_body(held.data(), held.size(), 0);
  CHECK(dec.stats(1).syms_recovered_arrived >= 1);
}

TEST(window_counts_accessor) {
  UepDecoder dec(vec_layers());
  auto [d, e] = dec.window_counts(0);
  CHECK(d == 0);
  CHECK(e == 0);
}

// Attribution harness: run frames through UepEncoder -> UepDecoder with a
// mid-stream transition; returns the decoder for counter assertions and
// collects emitted fragment bytes for the identity check.
namespace {
struct AttribRun {
  std::vector<std::vector<uint8_t>> frag_bytes;  // every DecodedFrag::frag, in order
  uint64_t abandoned = 0, abandoned_stale = 0;
  std::pair<uint64_t, uint64_t> wc_total{0, 0}, wc_cur{0, 0};
};

// drop_frames: frame indices (stream 1) whose bodies are all dropped.
// transition_at: frame index BEFORE which mark_transition fires; frames
// >= transition_at carry mcs_new, earlier ones mcs_old. use_attrib=false
// runs the identical stream with no mark_transition and all-unknown MCS.
AttribRun run_transition_sim(const std::set<int>& drop_frames, int transition_at,
                             bool use_attrib) {
  auto layers = layers_for(/*symbol_size=*/64, /*bpb_base=*/1, /*window=*/8);
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers, /*decode_deadline_ms=*/200);
  const uint8_t mcs_old = 5, mcs_new = 4;
  std::mt19937 rng(42);
  AttribRun r;
  uint64_t now = 1000;
  // Establish the pre-transition expected MCS first, like production does
  // on its first op: without a known previous MCS the real transition
  // below could not open its boundary (first-ever transitions use the
  // plain-fallback path by design).
  if (use_attrib) dec.mark_transition(1, mcs_old, now);
  for (int i = 0; i < 120; ++i) {
    if (use_attrib && i == transition_at)
      dec.mark_transition(1, mcs_new, now);
    const uint8_t mcs = use_attrib ? (i < transition_at ? mcs_old : mcs_new)
                                   : UepDecoder::kMcsUnknown;
    auto unit = make_unit(/*stream=*/1, static_cast<uint32_t>(i), 300, rng);
    auto bodies = enc.add_frame(1, unit.data(), unit.size(), now);
    for (auto& b : bodies) {
      if (drop_frames.count(i)) continue;
      for (auto& f : dec.add_body(b.body.data(), b.body.size(), now, mcs))
        r.frag_bytes.push_back(f.frag);
    }
    now += 5;
    auto flushed = enc.poll(now);
    for (auto& b : flushed) {
      if (drop_frames.count(i)) continue;
      for (auto& f : dec.add_body(b.body.data(), b.body.size(), now, mcs))
        r.frag_bytes.push_back(f.frag);
    }
  }
  dec.poll(now + 5000);
  const auto st = dec.stats(1);
  r.abandoned = st.syms_abandoned;
  r.abandoned_stale = st.syms_abandoned_stale;
  r.wc_total = dec.window_counts(1);
  r.wc_cur = dec.window_counts_cur(1);
  return r;
}
}  // namespace

TEST(uep_attrib_pre_transition_loss_books_stale_and_output_identical) {
  // Frames 47..49 lost entirely (killed in flight at the old op) with the
  // transition at frame 50: their symbols sit ABOVE the highest seq the
  // decoder saw at mark time, so this pins the first-new-frame close rule
  // (wm = first post-boundary seq - 1), not just the snapshot.
  const std::set<int> drop{47, 48, 49};
  auto attrib = run_transition_sim(drop, 50, /*use_attrib=*/true);
  auto legacy = run_transition_sim(drop, 50, /*use_attrib=*/false);
  // Bookkeeping-only: emitted fragment bytes identical with/without attribution.
  CHECK(attrib.frag_bytes == legacy.frag_bytes);
  CHECK(attrib.abandoned == legacy.abandoned);
  CHECK(attrib.abandoned > 0);
  CHECK(attrib.abandoned_stale == attrib.abandoned);  // all loss pre-transition
  CHECK(legacy.abandoned_stale == 0);                 // no watermark, no stale
  // Packet window: the lost units' expected slots book stale -> current
  // window sees full delivery.
  const auto [dc, ec] = attrib.wc_cur;
  CHECK(ec > 0);
  CHECK(dc == ec);
  const auto [dt, et] = attrib.wc_total;
  CHECK(et > ec);  // total window still carries the stale hole
}

TEST(uep_attrib_post_transition_loss_books_current) {
  // Transition at frame 30 (clean); frames 60..62 lost at the NEW mcs.
  const std::set<int> drop{60, 61, 62};
  auto r = run_transition_sim(drop, 30, /*use_attrib=*/true);
  CHECK(r.abandoned > 0);
  CHECK(r.abandoned_stale == 0);  // loss is the new rung's own
  const auto [dc, ec] = r.wc_cur;
  CHECK(ec > dc);  // current window shows the damage
}

TEST(uep_attrib_same_mcs_transition_plain_fallback) {
  // mark_transition to the SAME mcs the stream already runs: boundary must
  // not open (plain highest-seen watermark), pre-transition loss below the
  // snapshot still books stale.
  auto layers = layers_for(64, 1, 8);
  UepEncoder enc(layers, 15);
  UepDecoder dec(layers, 200);
  std::mt19937 rng(42);
  uint64_t now = 1000;
  auto feed = [&](int i, bool drop, uint8_t mcs) {
    auto unit = make_unit(1, static_cast<uint32_t>(i), 300, rng);
    auto bodies = enc.add_frame(1, unit.data(), unit.size(), now);
    for (auto& b : bodies)
      if (!drop) dec.add_body(b.body.data(), b.body.size(), now, mcs);
    now += 5;
    for (auto& b : enc.poll(now))
      if (!drop) dec.add_body(b.body.data(), b.body.size(), now, mcs);
  };
  dec.mark_transition(1, 5, now);            // establishes expected mcs 5
  for (int i = 0; i < 40; ++i) feed(i, i >= 35 && i <= 36, 5);
  dec.mark_transition(1, 5, now);            // same-MCS: overhead-only rung step
  for (int i = 40; i < 100; ++i) feed(i, false, 5);
  dec.poll(now + 5000);
  const auto st = dec.stats(1);
  CHECK(st.syms_abandoned > 0);
  CHECK(st.syms_abandoned_stale == st.syms_abandoned);  // below snapshot => stale
}

TEST(uep_attrib_expiry_disarms) {
  auto layers = layers_for(64, 1, 8);
  UepEncoder enc(layers, 15);
  UepDecoder dec(layers, 200);
  std::mt19937 rng(42);
  uint64_t now = 1000;
  dec.mark_transition(1, 5, now);
  dec.mark_transition(1, 4, now);  // opens (5 -> 4)
  now += 2000;                     // > 1000 ms boundary expiry
  // Loss entirely after expiry, still at "old" mcs 5: must book CURRENT
  // (boundary disarmed; a 2 s straggler is not transition debris).
  for (int i = 0; i < 100; ++i) {
    auto unit = make_unit(1, static_cast<uint32_t>(i), 300, rng);
    auto bodies = enc.add_frame(1, unit.data(), unit.size(), now);
    for (auto& b : bodies)
      if (!(i >= 40 && i <= 42))
        dec.add_body(b.body.data(), b.body.size(), now, 5);
    now += 5;
    for (auto& b : enc.poll(now))
      if (!(i >= 40 && i <= 42))
        dec.add_body(b.body.data(), b.body.size(), now, 5);
  }
  dec.poll(now + 5000);
  const auto st = dec.stats(1);
  CHECK(st.syms_abandoned > 0);
  CHECK(st.syms_abandoned_stale == 0);
}

// Fix-round addition (code review, 2026-08-14): pins the win_hwm fallback's
// safety property under a repair-cascade reorder entirely ON THE NEW RUNG,
// well after the boundary has closed (transition_at=10, drop starts at
// frame 47 — 37 frames and >180ms into the still-armed window, all bodies
// at mcs_new). This same window/drop-size combination is known (from the
// pre-transition test above) to make the underlying sliding-window FEC
// recover one of the three dropped units OUT OF ORDER — completing after a
// later unit and regressing last_seq — which is exactly the shape win_hwm
// exists to handle. Two properties must hold:
//  (a) the two units that never recover (permanently lost, all on the new
//      rung) must NOT be hidden by the stale split: expected_cur must
//      exceed delivered_cur.
//  (b) attribution must not perturb the preserved win_expected/win_delivered
//      arithmetic at all: totals identical to the same run with no
//      mark_transition/rx_mcs (use_attrib=false).
TEST(uep_attrib_current_loss_first_booking_never_suppressed) {
  const std::set<int> drop{47, 48, 49};
  auto attrib = run_transition_sim(drop, /*transition_at=*/10, /*use_attrib=*/true);
  auto legacy = run_transition_sim(drop, 10, /*use_attrib=*/false);
  CHECK(attrib.abandoned > 0);  // sanity: this scenario does lose symbols
  const auto [dc, ec] = attrib.wc_cur;
  CHECK(ec > dc);  // (a) genuine current-rung loss is visible, not hidden
  CHECK(attrib.wc_total == legacy.wc_total);  // (b) totals unchanged
}

// Final-review fix (2026-08-14): the boundary-expiry check in add_body must
// not underflow when now_ms lands BEFORE bnd_arm_ms (a body stamped
// microseconds before the marking iteration's clock capture can drain in
// the next iteration with an earlier-rounding ms stamp). Without the
// `now_ms > bnd_arm_ms` guard, `now_ms - bnd_arm_ms` wraps to a huge value,
// exceeds kBoundaryExpiryMs, and force-disarms a fresh boundary at exactly
// the wrong moment.
TEST(uep_attrib_expiry_guard_no_underflow) {
  auto layers = layers_for(64, 1, 8);
  UepEncoder enc(layers, 15);
  UepDecoder dec(layers, 200);
  std::mt19937 rng(42);
  uint64_t now = 500;
  dec.mark_transition(1, 5, now);   // establish a known prev mcs (5)
  now = 2000;
  dec.mark_transition(1, 4, now);   // opens boundary (5 -> 4), bnd_arm_ms = 2000

  auto feed = [&](int i, uint8_t mcs, uint64_t add_body_ms) {
    auto unit = make_unit(1, static_cast<uint32_t>(i), 300, rng);
    auto bodies = enc.add_frame(1, unit.data(), unit.size(), now);
    for (auto& b : bodies) dec.add_body(b.body.data(), b.body.size(), add_body_ms, mcs);
    now += 5;
    for (auto& b : enc.poll(now)) dec.add_body(b.body.data(), b.body.size(), add_body_ms, mcs);
  };
  // 1 ms BEFORE the arm stamp (now_ms=1999 < bnd_arm_ms=2000), old MCS ->
  // kPre-classifiable. Must NOT disarm the boundary.
  feed(0, 5, 1999);
  // A subsequent new-MCS body must still be able to close the boundary --
  // it can only do so if the boundary is still armed/open.
  feed(1, 4, now);
  CHECK(dec.last_boundary_close_ms(1) >= 0.0);
}

MTEST_MAIN
