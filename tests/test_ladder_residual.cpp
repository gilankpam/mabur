// The ladder's block-4 instant-demote input: post-FEC (residual) loss.
//
// This wiring had NO coverage before 2026-09-02 -- test_vrx_controller.cpp
// hand-builds LinkHealth, so nothing exercised decoder -> residual ->
// controller. That gap is how a reorder double-count reached the demote
// path: on the bench it fired 31 spurious demotes in ~7 minutes while the
// FEC decoder reported abn=0 (nothing was ever actually lost).
#include <random>
#include <vector>
#include "mtest.h"
#include "ladder_residual.h"
#include "mabur/frame_wire.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
using namespace mabur;

namespace {

// One frame unit, small enough to ride a single symbol. stream 0 = BASE
// (VPS -> critical), stream 1 = ENH (TRAIL_R carrying the temporal id).
std::vector<uint8_t> make_unit_for(int stream, uint32_t frame_id, size_t paylen,
                                   std::mt19937& rng) {
  std::vector<uint8_t> unit(framewire::kFrameHdrLen);
  framewire::FrameHdr h;
  h.frame_id = static_cast<uint16_t>(frame_id);
  h.flags = stream == 0 ? framewire::kFlagIdr : 0;
  h.pts_us = frame_id * 16667u;
  framewire::pack_frame_hdr(h, unit.data());
  for (uint8_t b : {0x00, 0x00, 0x00, 0x01}) unit.push_back(b);
  if (stream == 0) {
    unit.push_back(32 << 1);  // VPS -> critical/base
    unit.push_back(1);
  } else {
    unit.push_back(1 << 1);  // TRAIL_R
    unit.push_back(static_cast<uint8_t>(stream));
  }
  while (unit.size() < framewire::kFrameHdrLen + paylen)
    unit.push_back(static_cast<uint8_t>(rng()));
  return unit;
}

std::vector<uint8_t> make_unit(uint32_t frame_id, size_t paylen,
                               std::mt19937& rng) {
  return make_unit_for(0, frame_id, paylen, rng);
}

std::array<UepLayerCfg, 2> one_symbol_layers() {
  std::array<UepLayerCfg, 2> layers{};
  for (int s = 0; s < 2; ++s) {
    layers[static_cast<size_t>(s)].fec = SwConfig{512, 8, 0.5};
    layers[static_cast<size_t>(s)].blocks_per_body = 1;
  }
  return layers;
}

}  // namespace

// A unit that COMPLETES after a later one is not loss -- it is what a
// sliding-window FEC repair does every time it lands late (the bench saw
// 6643 repairs and 29 late-arriving originals over one run). Units 0, 1 and
// 2 all arrive here; nothing is dropped; the demote input must read clean.
TEST(reorder_completion_is_not_residual_loss) {
  auto layers = one_symbol_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers);
  std::mt19937 rng(7);
  uint64_t now = 1000;

  std::vector<std::vector<uint8_t>> body;
  for (uint32_t i = 0; i < 3; ++i) {
    auto unit = make_unit(i, 64, rng);
    auto bodies = enc.add_frame(0, unit.data(), unit.size(), now);
    // bodies[0] is the unit's source symbol; the rest are its FEC repairs,
    // withheld so completion order is exactly delivery order here.
    REQUIRE(!bodies.empty());
    body.push_back(bodies[0].body);
    now += 5;
  }

  // Deliver 0, then 2, then 1: unit 1 completes last, out of seq order.
  for (int i : {0, 2, 1})
    dec.add_body(body[static_cast<size_t>(i)].data(),
                 body[static_cast<size_t>(i)].size(), now);

  const auto rc = maburgs::ladder_residual_counts(dec);
  CHECK(rc.expected > 0);
  CHECK(rc.arrived == rc.expected);
}

// The same accounting must still see REAL loss: unit 1 never arrives.
// Enough units follow it to carry the decoder's horizon (4 * window = 32
// symbols) past the hole -- a genuinely lost symbol is only booked once it
// ages out, which is the detection lag the symbol-level measure trades for
// being immune to arrival order.
TEST(dropped_unit_is_residual_loss) {
  auto layers = one_symbol_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers);
  std::mt19937 rng(7);
  uint64_t now = 1000;

  for (uint32_t i = 0; i < 60; ++i) {
    auto unit = make_unit(i, 64, rng);
    auto bodies = enc.add_frame(0, unit.data(), unit.size(), now);
    REQUIRE(!bodies.empty());
    now += 5;
    if (i == 1) continue;  // unit 1 dropped in flight
    dec.add_body(bodies[0].body.data(), bodies[0].body.size(), now);
  }

  const auto rc = maburgs::ladder_residual_counts(dec);
  CHECK(rc.expected > 0);
  CHECK(rc.arrived < rc.expected);
}

// The pooled observability view is exactly the two layers summed -- the
// sideport/OSD number must not silently be base-only, nor double-count.
TEST(pooled_residual_counts_sum_both_layers) {
  auto layers = one_symbol_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers);
  std::mt19937 rng(11);
  uint64_t now = 1000;

  for (uint32_t i = 0; i < 20; ++i) {
    for (int s = 0; s < 2; ++s) {
      auto unit = make_unit_for(s, i, 64, rng);
      auto bodies = enc.add_frame(s, unit.data(), unit.size(), now);
      REQUIRE(!bodies.empty());
      dec.add_body(bodies[0].body.data(), bodies[0].body.size(), now);
    }
    now += 5;
  }

  const auto base = maburgs::residual_counts(dec, 0, /*cur=*/false);
  const auto enh = maburgs::residual_counts(dec, 1, /*cur=*/false);
  const auto pooled = maburgs::residual_counts_pooled(dec, /*cur=*/false);
  CHECK(base.expected > 0);
  CHECK(enh.expected > 0);
  CHECK(pooled.expected == base.expected + enh.expected);
  CHECK(pooled.arrived == base.arrived + enh.arrived);
}

// An idle layer reads fully delivered, not 0% -- the old packet-level
// window_delivery_pct returned 100 on an empty window and the sideport's
// layer_delivery_pct consumers depend on that.
TEST(delivery_pct_is_100_when_nothing_expected) {
  CHECK(maburgs::delivery_pct(maburgs::ResidualCounts{0, 0}) == 100);
  CHECK(maburgs::delivery_pct(maburgs::ResidualCounts{3, 4}) == 75);
  CHECK(maburgs::delivery_pct(maburgs::ResidualCounts{4, 4}) == 100);
}

MTEST_MAIN
