#include <cstdio>
#include <map>
#include "mtest.h"
#include "vectors.h"
#include "mabur/nal.h"
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

// Fixture RTP packets grouped per stream via the same classifier the drone used.
static std::map<int, std::vector<std::string>> fixture_by_stream() {
  std::map<int, std::vector<std::string>> out;
  std::string path = std::string(MABUR_FIXTURE_DIR) + "/rtp_stream.bin";
  FILE* f = fopen(path.c_str(), "rb");
  REQUIRE(f != nullptr);
  uint8_t hdr[2];
  while (fread(hdr, 1, 2, f) == 2) {
    size_t n = static_cast<size_t>(hdr[0] | (hdr[1] << 8));
    std::vector<uint8_t> pkt(n);
    REQUIRE(fread(pkt.data(), 1, n, f) == n);
    out[classify_rtp(pkt.data(), pkt.size())].push_back(mtest::hex(pkt));
  }
  fclose(f);
  return out;
}

// Bodies produced by a real UepEncoder over the fixture stream — the
// sliding-window scheme has no separate golden-vector wire format to pin
// (see test_uep.cpp), so the decoder is exercised against its own encoder's
// output, same as the drone/GS pairing on air.
static std::vector<UepBody> encode_fixture_bodies() {
  UepEncoder enc(vec_layers(), /*flush_ms=*/1'000'000'000ULL);
  std::vector<UepBody> bodies;
  std::string path = std::string(MABUR_FIXTURE_DIR) + "/rtp_stream.bin";
  FILE* f = fopen(path.c_str(), "rb");
  REQUIRE(f != nullptr);
  uint8_t hdr[2];
  while (fread(hdr, 1, 2, f) == 2) {
    size_t n = static_cast<size_t>(hdr[0] | (hdr[1] << 8));
    std::vector<uint8_t> pkt(n);
    REQUIRE(fread(pkt.data(), 1, n, f) == n);
    for (auto& b : enc.add_rtp(pkt.data(), pkt.size(), 0)) bodies.push_back(std::move(b));
  }
  fclose(f);
  for (auto& b : enc.flush_all()) bodies.push_back(std::move(b));
  return bodies;
}

static void feed_and_check(bool duplicate_bodies) {
  auto bodies = encode_fixture_bodies();
  UepDecoder dec(vec_layers());
  std::map<int, std::vector<std::string>> got;
  for (auto& b : bodies) {
    int reps = duplicate_bodies ? 2 : 1;
    for (int r = 0; r < reps; ++r)
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), 0))
        got[d.stream_id].push_back(mtest::hex(d.pkt));
  }
  auto want = fixture_by_stream();
  for (auto& [sid, pkts] : want) {
    REQUIRE(got[sid].size() == pkts.size());   // duplicates must NOT double output
    for (size_t i = 0; i < pkts.size(); ++i) CHECK(got[sid][i] == pkts[i]);
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

TEST(window_counts_accessor) {
  UepDecoder dec(vec_layers());
  auto [d, e] = dec.window_counts(0);
  CHECK(d == 0);
  CHECK(e == 0);
}
MTEST_MAIN
