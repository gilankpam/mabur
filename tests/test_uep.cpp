#include <array>
#include <cstdint>
#include <fstream>
#include <vector>

#include "mtest.h"
#include "vectors.h"
#include "mabur/uep_encoder.h"
#include "mabur/nal.h"
#include "mabur/sbi.h"
using namespace mabur;

namespace {
std::vector<std::vector<uint8_t>> load_rtp_stream(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::vector<std::vector<uint8_t>> pkts;
  while (true) {
    uint8_t lenb[2];
    f.read(reinterpret_cast<char*>(lenb), 2);
    if (f.gcount() != 2) break;
    uint16_t len = static_cast<uint16_t>(lenb[0]) | (static_cast<uint16_t>(lenb[1]) << 8);
    std::vector<uint8_t> pkt(len);
    f.read(reinterpret_cast<char*>(pkt.data()), len);
    if (f.gcount() != static_cast<std::streamsize>(len)) break;
    pkts.push_back(std::move(pkt));
  }
  return pkts;
}

std::array<UepLayerCfg, 4> make_layers(int k, int symbol_size, int blocks_per_body,
                                        const std::vector<double>& overheads) {
  std::array<UepLayerCfg, 4> layers;
  for (int sid = 0; sid < 4; ++sid) {
    layers[static_cast<size_t>(sid)].fec = RsConfig{k, symbol_size, overheads[static_cast<size_t>(sid)]};
    layers[static_cast<size_t>(sid)].blocks_per_body = blocks_per_body;
  }
  return layers;
}
}  // namespace

TEST(uep_encoder_matches_python_composed_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/uep.json");
  int k = j["k"].get<int>();
  int symbol_size = j["symbol_size"].get<int>();
  int blocks_per_body = j["blocks_per_body"].get<int>();
  std::vector<double> overheads;
  for (auto& o : j["overheads"]) overheads.push_back(o.get<double>());

  auto layers = make_layers(k, symbol_size, blocks_per_body, overheads);
  UepEncoder enc(layers, /*flush_ms=*/1'000'000'000ULL);

  auto pkts = load_rtp_stream(std::string(MABUR_FIXTURE_DIR) + "/rtp_stream.bin");
  REQUIRE(pkts.size() == j["classify"].size());

  std::vector<std::pair<int, std::string>> stream;
  for (size_t i = 0; i < pkts.size(); ++i) {
    int expect_sid = j["classify"][i].get<int>();
    int actual_sid = classify_rtp(pkts[i].data(), pkts[i].size());
    CHECK(actual_sid == expect_sid);

    auto bodies = enc.add_rtp(pkts[i].data(), pkts[i].size(), /*now_ms=*/0);
    for (auto& b : bodies) stream.emplace_back(b.stream_id, mtest::hex(b.body));
  }

  REQUIRE(stream.size() == j["stream"].size());
  for (size_t i = 0; i < stream.size(); ++i) {
    auto& c = j["stream"][i];
    CHECK(stream[i].first == c["sid"].get<int>());
    CHECK(stream[i].second == c["body"].get<std::string>());
  }

  auto flushed = enc.flush_all();
  REQUIRE(flushed.size() == j["flush"].size());
  for (size_t i = 0; i < flushed.size(); ++i) {
    auto& c = j["flush"][i];
    CHECK(flushed[i].stream_id == c["sid"].get<int>());
    CHECK(mtest::hex(flushed[i].body) == c["body"].get<std::string>());
  }
}

TEST(uep_layer_overhead_math_and_clamps) {
  CHECK(uep_layer_overhead(0, 0.25) == 1.0);
  CHECK(uep_layer_overhead(3, 0.25) == 0.25);
  CHECK(uep_layer_overhead(3, 1.0) == 1.0);
  // Clamp low: sid 3 (REF 0.25) * cmd tiny -> below 0.125 clamps to 0.125.
  CHECK(uep_layer_overhead(3, 0.001) == 0.125);
  // Clamp high: sid 0 (REF 1.00) * cmd huge -> above 2.0 clamps to 2.0.
  CHECK(uep_layer_overhead(0, 100.0) == 2.0);
}

TEST(uep_set_shed_drops_stream_and_counts) {
  std::vector<double> overheads = {1.0, 0.75, 0.5, 0.25};
  auto layers = make_layers(8, 64, 4, overheads);
  UepEncoder enc(layers, /*flush_ms=*/1'000'000'000ULL);
  enc.set_shed(3, true);

  // Build a minimal RTP packet that classifies to stream 3 (tid 2, non-critical).
  // RTP header (12 bytes, no CSRC/ext) + HEVC NAL: type=1 (non-critical), tid byte -> tid 2.
  std::vector<uint8_t> pkt = {
      0x80, 0x61, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0xDE, 0xAD, 0xBE, 0xEF,
      0x02, 0x03, 0xAA, 0xBB, 0xCC};
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 3);

  CHECK(enc.dropped(3) == 0);
  auto out = enc.add_rtp(pkt.data(), pkt.size(), 0);
  CHECK(out.empty());
  CHECK(enc.dropped(3) == 1);

  auto out2 = enc.add_rtp(pkt.data(), pkt.size(), 1);
  CHECK(out2.empty());
  CHECK(enc.dropped(3) == 2);
}

TEST(uep_poll_after_idle_emits_short_block_with_kreal_lt_k) {
  std::vector<double> overheads = {1.0, 0.75, 0.5, 0.25};
  auto layers = make_layers(8, 64, 4, overheads);
  uint64_t flush_ms = 15;
  UepEncoder enc(layers, static_cast<int>(flush_ms));

  // One small non-critical, tid-0 packet -> stream 1. Leaves the layer with
  // pending (unflushed) data since a single packet doesn't fill k=8 symbols.
  std::vector<uint8_t> pkt = {
      0x80, 0x61, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0xDE, 0xAD, 0xBE, 0xEF,
      0x02, 0x01, 0xAA, 0xBB, 0xCC};
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 1);

  auto immediate = enc.add_rtp(pkt.data(), pkt.size(), 0);
  CHECK(immediate.empty());  // not enough to complete a block yet

  auto before_idle = enc.poll(flush_ms - 1);
  CHECK(before_idle.empty());

  auto after_idle = enc.poll(flush_ms + 1);
  REQUIRE(!after_idle.empty());
  CHECK(after_idle[0].stream_id == 1);
  // First envelope's kreal (RS header byte 4) must be < k (8): a short block.
  const auto& body = after_idle[0].body;
  REQUIRE(body.size() >= SBI_HDR_LEN + 2 + 5);
  uint8_t kreal = body[SBI_HDR_LEN + 2 + 4];
  CHECK(kreal < 8);
}

MTEST_MAIN
