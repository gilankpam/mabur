#include "mtest.h"
#include "msp_sink.h"
#include "mabur/msp_source.h"
#include "mabur/msp_dp.h"
#include "mabur/sbi.h"
#include <string>
#include <vector>
using namespace mabur;

static std::vector<uint8_t> screen_blob(const std::string& text) {
  std::vector<uint8_t> s;
  std::vector<uint8_t> clr = {2};
  msp_append_message(s, 182, clr.data(), clr.size());        // CLEAR
  std::vector<uint8_t> ds = {3, 0, 0, 0};
  for (char c : text) ds.push_back(static_cast<uint8_t>(c));
  msp_append_message(s, 182, ds.data(), ds.size());          // DRAW_STRING
  std::vector<uint8_t> scr = {4};
  msp_append_message(s, 182, scr.data(), scr.size());       // DRAW_SCREEN
  return s;
}

TEST(sink_reassembles_snapshot_to_udp_bytes) {
  MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  auto blob = screen_blob("HELLO");
  src.on_serial_bytes(blob.data(), blob.size(), 1000);

  std::vector<std::vector<uint8_t>> out;
  maburgs::MspSink sink(cfg.symbol_size, cfg.window,
                        [&](const uint8_t* d, size_t n){ out.emplace_back(d, d + n); });
  for (auto& b : bodies) sink.on_body(b.data(), b.size(), 1000);

  REQUIRE(out.size() == 1);
  MspScreen s; MspParser p;
  for (auto& m : p.feed(out[0].data(), out[0].size())) s.apply(m);
  CHECK(s.cell(0, 0) == 'H');
  CHECK(s.cell(0, 4) == 'O');
}

MTEST_MAIN;
