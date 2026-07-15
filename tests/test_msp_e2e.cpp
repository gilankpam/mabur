#include "mtest.h"
#include "msp_sink.h"
#include "mabur/msp_source.h"
#include "mabur/msp_dp.h"
#include <string>
#include <vector>
using namespace mabur;

static std::vector<uint8_t> screen_blob(const std::string& text) {
  std::vector<uint8_t> s;
  std::vector<uint8_t> clr = {2}, ds = {3, 0, 0, 0}, scr = {4};
  for (char c : text) ds.push_back(static_cast<uint8_t>(c));
  msp_append_message(s, 182, clr.data(), clr.size());
  msp_append_message(s, 182, ds.data(), ds.size());
  msp_append_message(s, 182, scr.data(), scr.size());
  return s;
}

TEST(e2e_clean_delivers_every_snapshot) {
  MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  for (int i = 0; i < 5; ++i) {
    auto blob = screen_blob("FRAME" + std::to_string(i));
    src.on_serial_bytes(blob.data(), blob.size(), 1000u * (i + 1));
  }
  int out = 0;
  maburgs::MspSink sink(cfg.symbol_size, cfg.window,
                        [&](const uint8_t*, size_t){ ++out; });
  for (auto& b : bodies) sink.on_body(b.data(), b.size(), 6000);
  CHECK(out == 5);
}

TEST(e2e_survives_periodic_body_loss) {
  MspSourceCfg cfg;
  std::vector<std::vector<uint8_t>> bodies;
  MspSource src(cfg, [&](const uint8_t* b, size_t n){ bodies.emplace_back(b, b + n); });
  const int N = 10;
  for (int i = 0; i < N; ++i) {
    auto blob = screen_blob("F" + std::to_string(i));
    src.on_serial_bytes(blob.data(), blob.size(), 1000u * (i + 1));
  }
  int out = 0;
  maburgs::MspSink sink(cfg.symbol_size, cfg.window,
                        [&](const uint8_t*, size_t){ ++out; });
  // Drop every 5th body; the per-snapshot repair recovers isolated losses.
  // Each snapshot = 1 source + 1 repair on separate bodies; dropping every
  // 5th never drops an adjacent source+repair pair, and the window-16
  // repair overlap recovers isolated losses. If MspSourceCfg defaults
  // (blocks_per_body=1 / one-symbol-per-snapshot) change, revisit this
  // stride.
  int idx = 0;
  for (auto& b : bodies) {
    if (idx++ % 5 != 0) sink.on_body(b.data(), b.size(), 11000);
  }
  // Not every snapshot need survive, but the large majority must.
  CHECK(out >= N - 2);
}

MTEST_MAIN;
