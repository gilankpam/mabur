#include <map>
#include "mtest.h"
#include "frame_fixture.h"
#include "vectors.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
using namespace mabur;

// symbol_size=64, blocks_per_body=4 on every stream, per-stream overheads =
// the reference ladder (decoder ignores overhead; window is TX-side only).
static std::array<UepLayerCfg, 4> vec_layers() {
  std::array<UepLayerCfg, 4> L{};
  const double ov[4] = {1.00, 0.75, 0.50, 0.25};
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
MTEST_MAIN
