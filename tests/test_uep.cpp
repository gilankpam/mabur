#include <array>
#include <cstdint>
#include <fstream>
#include <map>
#include <vector>

#include "mtest.h"
#include "vectors.h"
#include "mabur/uep_decoder.h"
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

std::array<UepLayerCfg, 4> make_layers(int symbol_size, int blocks_per_body,
                                        const std::vector<double>& overheads) {
  std::array<UepLayerCfg, 4> layers;
  for (int sid = 0; sid < 4; ++sid) {
    layers[static_cast<size_t>(sid)].fec =
        SwConfig{symbol_size, 128, overheads[static_cast<size_t>(sid)]};
    layers[static_cast<size_t>(sid)].blocks_per_body = blocks_per_body;
  }
  return layers;
}
}  // namespace

// Sliding-window envelopes are not byte-identical to the RS/Python-composed
// vectors in uep.json (different wire scheme entirely — see sw_wire.h), so
// this no longer pins body bytes against uep.json. classify_rtp routing is
// still scheme-agnostic and stays pinned against the vector; body content is
// verified as a full encode -> lossless channel -> decode round-trip against
// the same fixture (packet round-trips are scheme-agnostic).
TEST(uep_encoder_round_trips_fixture_through_decoder) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/uep.json");
  int symbol_size = j["symbol_size"].get<int>();
  int blocks_per_body = j["blocks_per_body"].get<int>();
  std::vector<double> overheads;
  for (auto& o : j["overheads"]) overheads.push_back(o.get<double>());

  auto layers = make_layers(symbol_size, blocks_per_body, overheads);
  UepEncoder enc(layers, /*flush_ms=*/1'000'000'000ULL);
  UepDecoder dec(layers, /*decode_deadline_ms=*/1'000'000'000ULL);

  auto pkts = load_rtp_stream(std::string(MABUR_FIXTURE_DIR) + "/rtp_stream.bin");
  REQUIRE(pkts.size() == j["classify"].size());

  std::map<int, std::vector<std::string>> want, got;
  for (size_t i = 0; i < pkts.size(); ++i) {
    int expect_sid = j["classify"][i].get<int>();
    int actual_sid = classify_rtp(pkts[i].data(), pkts[i].size());
    CHECK(actual_sid == expect_sid);
    want[actual_sid].push_back(mtest::hex(pkts[i]));

    auto bodies = enc.add_rtp(pkts[i].data(), pkts[i].size(), /*now_ms=*/0);
    for (auto& b : bodies)
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), 0))
        got[d.stream_id].push_back(mtest::hex(d.pkt));
  }
  for (auto& b : enc.flush_all())
    for (auto& d : dec.add_body(b.body.data(), b.body.size(), 0))
      got[d.stream_id].push_back(mtest::hex(d.pkt));

  for (auto& [sid, expect_pkts] : want) {
    REQUIRE(got[sid].size() == expect_pkts.size());
    for (size_t i = 0; i < expect_pkts.size(); ++i)
      CHECK(got[sid][i] == expect_pkts[i]);
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
  auto layers = make_layers(64, 4, overheads);
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

TEST(uep_poll_after_idle_seals_pending_symbol) {
  std::vector<double> overheads = {1.0, 0.75, 0.5, 0.25};
  auto layers = make_layers(64, 4, overheads);
  uint64_t flush_ms = 15;
  UepEncoder enc(layers, static_cast<int>(flush_ms));

  // One small non-critical, tid-0 packet -> stream 1. Leaves the layer with
  // pending (unflushed) data since a single packet doesn't fill a symbol.
  std::vector<uint8_t> pkt = {
      0x80, 0x61, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0xDE, 0xAD, 0xBE, 0xEF,
      0x02, 0x01, 0xAA, 0xBB, 0xCC};
  CHECK(classify_rtp(pkt.data(), pkt.size()) == 1);

  auto immediate = enc.add_rtp(pkt.data(), pkt.size(), 0);
  CHECK(immediate.empty());  // not enough to seal a symbol yet

  auto before_idle = enc.poll(flush_ms - 1);
  CHECK(before_idle.empty());

  auto after_idle = enc.poll(flush_ms + 1);
  REQUIRE(!after_idle.empty());
  CHECK(after_idle[0].stream_id == 1);
  // Sealed tail carries a source envelope: sw::kSwHeaderLen + symbol_size.
  const auto& body = after_idle[0].body;
  CHECK(body.size() >= SBI_HDR_LEN + 2 + mabur::sw::kSwHeaderLen);
}

MTEST_MAIN
