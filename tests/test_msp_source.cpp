#include "mtest.h"
#include "mabur/msp_source.h"
#include "mabur/msp_dp.h"
#include "mabur/sbi.h"
#include "mabur/sw_decoder.h"
#include "mabur/sw_wire.h"
#include <functional>
#include <string>
using namespace mabur;

// Reuse the frame builders from the codec test's shape.
static void append_msp(std::vector<uint8_t>& s, uint8_t cmd, std::vector<uint8_t> payload) {
  msp_append_message(s, cmd, payload.data(), payload.size());
}
static std::vector<uint8_t> screen_blob(const std::string& text) {
  std::vector<uint8_t> s;
  append_msp(s, 182, {2});                                  // CLEAR
  std::vector<uint8_t> ds = {3, 0, 0, 0};                   // DRAW_STRING row0 col0
  for (char c : text) ds.push_back(static_cast<uint8_t>(c));
  append_msp(s, 182, ds);
  append_msp(s, 182, {4});                                  // DRAW_SCREEN
  return s;
}

TEST(rate_gate_limits_forwards) {
  MspSourceCfg cfg;               // 1 Hz
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });

  auto blob = screen_blob("ABC");
  // Three complete screens within the same second: only the first forwards.
  src.on_serial_bytes(blob.data(), blob.size(), 1000);
  src.on_serial_bytes(blob.data(), blob.size(), 1200);
  src.on_serial_bytes(blob.data(), blob.size(), 1900);
  CHECK(src.snapshots_sent() == 1);
  CHECK(src.snapshots_gated() == 2);
  // A screen a full second later forwards again.
  src.on_serial_bytes(blob.data(), blob.size(), 2000);
  CHECK(src.snapshots_sent() == 2);
}

TEST(emitted_bodies_are_msp_stream) {
  MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  auto blob = screen_blob("HELLO");
  src.on_serial_bytes(blob.data(), blob.size(), 1000);
  REQUIRE(!bodies.empty());
  for (auto& b : bodies)
    CHECK(sbi_peek_stream_id(b.data(), b.size()) == kMspStreamId);
}

TEST(clean_roundtrip_reassembles_snapshot) {
  MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  auto blob = screen_blob("HELLO");
  src.on_serial_bytes(blob.data(), blob.size(), 1000);

  // Decode side: sbi_unpack -> SwDecoder -> reassembled snapshot bytes.
  int block_payload = cfg.symbol_size + static_cast<int>(sw::kSwHeaderLen);
  SwDecoder dec(SwConfig{cfg.symbol_size, cfg.window, 0.0});
  std::vector<std::vector<uint8_t>> snaps;
  for (auto& body : bodies) {
    auto u = sbi_unpack(body.data(), body.size(), block_payload);
    for (auto& env : u.survivors)
      for (auto& pkt : dec.add_symbol(env.data(), env.size(), 1000))
        snaps.push_back(pkt);
  }
  REQUIRE(snaps.size() == 1);
  // Replaying the reassembled snapshot reproduces the screen.
  MspScreen s; MspParser p;
  for (auto& m : p.feed(snaps[0].data(), snaps[0].size())) s.apply(m);
  CHECK(s.cell(0, 0) == 'H');
  CHECK(s.cell(0, 4) == 'O');
}

TEST(single_body_loss_recovers_via_repair) {
  MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  // Two snapshots a second apart -> sources + tail repairs across a window.
  src.on_serial_bytes(screen_blob("AAA").data(), screen_blob("AAA").size(), 1000);
  src.on_serial_bytes(screen_blob("BBB").data(), screen_blob("BBB").size(), 2000);
  REQUIRE(bodies.size() >= 3);  // >= 2 sources + >= 1 repair

  int block_payload = cfg.symbol_size + static_cast<int>(sw::kSwHeaderLen);
  SwDecoder dec(SwConfig{cfg.symbol_size, cfg.window, 0.0});
  int delivered = 0;
  // Drop the FIRST body (a source); repairs must recover it.
  for (size_t i = 1; i < bodies.size(); ++i) {
    auto u = sbi_unpack(bodies[i].data(), bodies[i].size(), block_payload);
    for (auto& env : u.survivors)
      delivered += static_cast<int>(dec.add_symbol(env.data(), env.size(), 2000).size());
  }
  CHECK(delivered >= 2);  // both snapshots recovered despite the dropped source
}

MTEST_MAIN
