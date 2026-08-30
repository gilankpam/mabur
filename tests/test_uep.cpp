#include <array>
#include <cstdint>
#include <fstream>
#include <map>
#include <vector>

#include "mtest.h"
#include "frame_fixture.h"
#include "vectors.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
#include "mabur/nal.h"
#include "mabur/sbi.h"
using namespace mabur;

namespace {
std::array<UepLayerCfg, 2> make_layers(int symbol_size, int blocks_per_body,
                                        const std::vector<double>& overheads) {
  std::array<UepLayerCfg, 2> layers;
  for (int sid = 0; sid < 2; ++sid) {
    layers[static_cast<size_t>(sid)].fec =
        SwConfig{symbol_size, 128, overheads[static_cast<size_t>(sid)]};
    layers[static_cast<size_t>(sid)].blocks_per_body = blocks_per_body;
  }
  return layers;
}
}  // namespace

// Sliding-window envelopes have no Python reference (different wire scheme
// entirely — see sw_wire.h), so body bytes aren't pinned against uep.json.
// What stays pinned is classify_frame's per-frame layer routing; unit content
// is verified as a full encode -> lossless channel -> decode round-trip.
TEST(uep_encoder_round_trips_fixture_through_decoder) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/uep.json");
  int symbol_size = j["symbol_size"].get<int>();
  int blocks_per_body = j["blocks_per_body"].get<int>();
  std::vector<double> overheads;
  for (auto& o : j["overheads"]) overheads.push_back(o.get<double>());

  auto layers = make_layers(symbol_size, blocks_per_body, overheads);
  UepEncoder enc(layers, /*flush_ms=*/1'000'000'000ULL);
  UepDecoder dec(layers);

  auto frames = mtest::load_frame_fixture(std::string(MABUR_FIXTURE_DIR) +
                                          "/frame_stream.bin");
  REQUIRE(frames.size() == j["classify"].size());

  std::map<int, std::vector<std::string>> want;
  mtest::FragCollector got;
  for (size_t i = 0; i < frames.size(); ++i) {
    const int expect_sid = j["classify"][i].get<int>();
    const int sid = mabur::classify_frame(frames[i].annexb.data(), frames[i].annexb.size());
    CHECK(sid == expect_sid);

    auto unit = mtest::frame_unit(frames[i], static_cast<uint16_t>(i));
    want[sid].push_back(mtest::hex(unit));
    for (auto& b : enc.add_frame(sid, unit.data(), unit.size(), /*now_ms=*/0))
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), 0)) got.add(d);
  }
  for (auto& b : enc.flush_all())
    for (auto& d : dec.add_body(b.body.data(), b.body.size(), 0)) got.add(d);

  std::map<int, std::vector<std::string>> recovered;
  for (auto& [sid, unit] : got.completed()) recovered[sid].push_back(mtest::hex(unit));
  for (auto& [sid, expect_units] : want) {
    REQUIRE(recovered[sid].size() == expect_units.size());
    for (size_t i = 0; i < expect_units.size(); ++i)
      CHECK(recovered[sid][i] == expect_units[i]);
  }
}

TEST(set_overhead_is_literal_per_layer) {
  std::array<UepLayerCfg, 2> layers{};
  for (auto& l : layers) l.fec = SwConfig{332, 32, 0.5};
  UepEncoder uep(layers, 15);
  uep.set_layer_overhead(0, 0.28);
  uep.set_layer_overhead(1, 0.80);
  uep.set_overhead(0.5);  // resets both
  // behavioral proof: encode one identical frame per layer and compare
  // emitted byte counts — equal overhead => equal emitted bytes.
  std::vector<uint8_t> frame(4000, 0xAB);
  auto b0 = uep.add_frame(0, frame.data(), frame.size(), 1000);
  auto b1 = uep.add_frame(1, frame.data(), frame.size(), 1000);
  size_t n0 = 0, n1 = 0;
  for (auto& b : b0) n0 += b.body.size();
  for (auto& b : b1) n1 += b.body.size();
  CHECK(n0 == n1);
}

TEST(uep_set_shed_drops_stream_and_counts) {
  std::vector<double> overheads = {1.0, 0.75};
  auto layers = make_layers(64, 4, overheads);
  UepEncoder enc(layers, /*flush_ms=*/1'000'000'000ULL);
  enc.set_shed(1, true);

  // One frame unit routed to the shed (enhance) layer: FrameHdr + a TRAIL_N
  // slice NAL (type 0 -> classify_frame's enh sid 1 in the 2-stream space).
  std::vector<uint8_t> unit(framewire::kFrameHdrLen, 0);
  framewire::pack_frame_hdr(framewire::FrameHdr{}, unit.data());
  for (uint8_t b : {0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0xAA, 0xBB, 0xCC})
    unit.push_back(b);
  CHECK(classify_frame(unit.data() + framewire::kFrameHdrLen,
                       unit.size() - framewire::kFrameHdrLen) == 1);

  CHECK(enc.dropped(1) == 0);
  CHECK(enc.add_frame(1, unit.data(), unit.size(), 0).empty());
  CHECK(enc.dropped(1) == 1);

  CHECK(enc.add_frame(1, unit.data(), unit.size(), 1).empty());
  CHECK(enc.dropped(1) == 2);
}

TEST(uep_poll_has_nothing_to_seal_after_a_frame) {
  // add_frame seals the window AND flushes the SBI group at frame end, so a
  // synchronous encoder has no tail left for the idle flush to find. This is
  // the guard on that contract: were the frame-end seal dropped, the frame's
  // tail would surface here (one flush_ms later) instead of on the wire
  // immediately. poll() itself stays live for the async FEC worker, whose
  // repair envelopes surface at a later drain (see test_uep_sw).
  std::vector<double> overheads = {1.0, 0.75};
  auto layers = make_layers(64, 4, overheads);
  const int flush_ms = 15;
  UepEncoder enc(layers, flush_ms);

  std::vector<uint8_t> unit(framewire::kFrameHdrLen + 500, 0x5A);
  framewire::pack_frame_hdr(framewire::FrameHdr{}, unit.data());
  auto sealed = enc.add_frame(1, unit.data(), unit.size(), 0);
  REQUIRE(!sealed.empty());
  CHECK(sealed[0].stream_id == 1);
  // Sealed body carries a source envelope: sw::kSwHeaderLen + symbol_size.
  CHECK(sealed[0].body.size() >= SBI_HDR_LEN + 2 + mabur::sw::kSwHeaderLen);

  CHECK(enc.poll(flush_ms - 1).empty());
  CHECK(enc.poll(flush_ms + 1).empty());
}

MTEST_MAIN
