#pragma once
// tests/body_gen.h — header-only drone-side video body generator for GS
// tests that need a REAL Aggregator/UepDecoder driven through its counters
// (test_link_health, the web GS tests). Bodies come out of the drone's own
// UepEncoder the way tests/test_frame_e2e.cpp drives it: framewire AUs
// (FrameHdr + Annex-B), both layers (even AUs base, odd AUs enh), sealed per
// frame. Everything is deterministic except the encoder's random start seq,
// which no GS consumer can observe.
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "aggregator.h"
#include "mabur/frame_wire.h"
#include "mabur/node.h"
#include "mabur/sw_wire.h"
#include "mabur/uep_encoder.h"

// The production GS geometry: both layers SwConfig{332, 128, 0.5}, bpb 4
// (Config::uep_layers(), spike/wasm/gsweb_core.cpp).
inline std::array<mabur::UepLayerCfg, 2> gen_layers() {
  std::array<mabur::UepLayerCfg, 2> l{};
  for (auto& x : l) {
    x.fec = mabur::SwConfig{332, 128, 0.5};
    x.blocks_per_body = 4;
  }
  return l;
}

// n_aus access units, one every dt_ms starting at mono 0 (AU i's bodies all
// carry mono_us = i * dt_ms * 1000). crc_ok, phy_valid, snr {40,40}, rssi
// {60,60}; mac_seq advances per body INCLUDING dropped ones, so a drop reads
// as a per-card seq gap too. drop_every = k > 0 drops each k-th body.
inline std::vector<mabur::node::RxBody> gen_bodies(int n_aus, double dt_ms,
                                                   int drop_every) {
  mabur::UepEncoder enc(gen_layers(), 15);
  std::vector<mabur::node::RxBody> out;
  uint16_t mac_seq = 0;
  uint64_t body_idx = 0;
  for (int i = 0; i < n_aus; ++i) {
    const double t_ms = i * dt_ms;
    const int sid = i % 2;  // base / enh alternation (SVC-T cadence)
    // Annex-B: one slice NAL (IDR for AU 0, TRAIL otherwise), 2400 B
    // payload so every AU spans several bodies.
    std::vector<uint8_t> ab = {0, 0, 0, 1,
                               static_cast<uint8_t>((i == 0 ? 19 : 1) << 1),
                               static_cast<uint8_t>(sid + 1)};
    for (int b = 0; b < 2400; ++b)
      ab.push_back(static_cast<uint8_t>((b * 131 + i * 7) & 0xFF));
    std::vector<uint8_t> unit(mabur::framewire::kFrameHdrLen + ab.size());
    mabur::framewire::FrameHdr h;
    h.frame_id = static_cast<uint16_t>(i);
    h.flags = i == 0 ? mabur::framewire::kFlagIdr : 0;
    h.pts_us = static_cast<uint32_t>(t_ms * 1000.0);
    mabur::framewire::pack_frame_hdr(h, unit.data());
    std::memcpy(unit.data() + mabur::framewire::kFrameHdrLen, ab.data(), ab.size());
    for (auto& b : enc.add_frame(sid, unit.data(), unit.size(),
                                 static_cast<uint64_t>(t_ms))) {
      ++body_idx;
      const uint16_t seq = mac_seq;
      mac_seq = static_cast<uint16_t>((mac_seq + 1) & 0x0FFF);
      if (drop_every > 0 && body_idx % static_cast<uint64_t>(drop_every) == 0)
        continue;
      mabur::node::RxBody m;
      m.card_id = 0;
      m.mono_us = static_cast<uint64_t>(t_ms * 1000.0);
      m.rssi[0] = m.rssi[1] = 60;
      m.snr[0] = m.snr[1] = 40;
      m.phy_valid = true;
      m.crc_ok = true;
      m.mac_seq = seq;
      m.body = std::move(b.body);
      out.push_back(std::move(m));
    }
  }
  return out;
}

// gen_bodies shifted to start at t0_ms and fed into agg; returns the last
// AU's timestamp (ms).
inline double feed_video(maburgs::Aggregator& agg, int n_aus, double t0_ms,
                         double dt_ms, int drop_every = 0) {
  for (auto m : gen_bodies(n_aus, dt_ms, drop_every)) {
    m.mono_us += static_cast<uint64_t>(t0_ms * 1000.0);
    agg.on_rx_body(m);
  }
  return t0_ms + (n_aus - 1) * dt_ms;
}
