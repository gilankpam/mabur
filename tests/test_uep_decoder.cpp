#include <cstdio>
#include <map>
#include "mtest.h"
#include "vectors.h"
#include "mabur/nal.h"
#include "mabur/uep_decoder.h"
using namespace mabur;

// uep.json config: k=8, symbol_size=64, blocks_per_body=4 on every stream,
// per-stream overheads = the reference ladder (decoder ignores overhead).
static std::array<UepLayerCfg, 4> vec_layers() {
  std::array<UepLayerCfg, 4> L{};
  const double ov[4] = {1.00, 0.75, 0.50, 0.25};
  for (int s = 0; s < 4; ++s) L[s] = UepLayerCfg{RsConfig{8, 64, ov[s]}, 4};
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

static void feed_and_check(bool duplicate_bodies) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/uep.json");
  UepDecoder dec(vec_layers());
  std::map<int, std::vector<std::string>> got;
  auto feed = [&](const nlohmann::json& arr) {
    for (auto& e : arr) {
      auto body = mtest::unhex(e["body"].get<std::string>());
      int reps = duplicate_bodies ? 2 : 1;
      for (int r = 0; r < reps; ++r)
        for (auto& d : dec.add_body(body.data(), body.size(), 0))
          got[d.stream_id].push_back(mtest::hex(d.pkt));
    }
  };
  feed(j["stream"]); feed(j["flush"]);
  auto want = fixture_by_stream();
  for (auto& [sid, pkts] : want) {
    REQUIRE(got[sid].size() == pkts.size());   // duplicates must NOT double output
    for (size_t i = 0; i < pkts.size(); ++i) CHECK(got[sid][i] == pkts[i]);
  }
  CHECK(dec.bodies_misrouted() == 0);
}

TEST(decodes_uep_vectors_to_fixture) { feed_and_check(false); }
TEST(duplicate_bodies_are_idempotent) { feed_and_check(true); }

TEST(garbage_body_is_misrouted_not_fatal) {
  UepDecoder dec(vec_layers());
  std::vector<uint8_t> junk(40, 0x5A);
  CHECK(dec.add_body(junk.data(), junk.size(), 0).empty());
  CHECK(dec.bodies_misrouted() == 1);
}

TEST(window_delivery_full_on_clean_stream) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/uep.json");
  UepDecoder dec(vec_layers());
  for (auto& e : j["stream"]) {
    auto body = mtest::unhex(e["body"].get<std::string>());
    dec.add_body(body.data(), body.size(), 0);
  }
  for (auto& e : j["flush"]) {
    auto body = mtest::unhex(e["body"].get<std::string>());
    dec.add_body(body.data(), body.size(), 0);
  }
  for (int s = 0; s < 4; ++s) CHECK(dec.window_delivery_pct(s) == 100);
  dec.reset_window();
  for (int s = 0; s < 4; ++s) CHECK(dec.window_delivery_pct(s) == 100);  // empty = 100
}
MTEST_MAIN
