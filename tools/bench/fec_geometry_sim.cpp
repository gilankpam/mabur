#include <cstdio>
#include <cstdint>
#include <random>
#include <vector>

#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"

using namespace mabur;

static int run(int symbol_size, int bpb_base, int n_pkts, double drop_pct, int burst_len) {
  std::array<UepLayerCfg, 4> enc_layers{};
  std::array<UepLayerCfg, 4> dec_layers{};
  const int bpb[4] = {bpb_base, bpb_base, bpb_base * 2, bpb_base * 2};
  for (int s = 0; s < 4; ++s) {
    enc_layers[s].fec = RsConfig{8, symbol_size, kUepRefOverhead[s]};
    enc_layers[s].blocks_per_body = bpb[s];
    dec_layers[s].fec = RsConfig{8, symbol_size, kUepRefOverhead[s]};
    dec_layers[s].blocks_per_body = 4;  // unused on decode
  }
  UepEncoder enc(enc_layers, /*flush_ms=*/15);
  std::mt19937 drop_rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  int burst_left = 0;
  uint64_t dropped = 0, bodies_n = 0;
  auto keep = [&]() {
    ++bodies_n;
    if (burst_left > 0) { --burst_left; ++dropped; return false; }
    if (u(drop_rng) * 100.0 < drop_pct) {
      ++dropped;
      if (burst_len > 1) burst_left = burst_len - 1;
      return false;
    }
    return true;
  };
  UepDecoder dec(dec_layers, /*block_max_age_ms=*/2000);

  std::mt19937 rng(42);
  uint64_t now = 1000;
  uint64_t sent = 0, got = 0;
  // Realistic-ish RTP: seq increments, mixed sizes (1400 mostly, some small),
  // marker/timestamps irrelevant here. classify_rtp routes by NAL: craft a
  // simple non-critical HEVC FU payload so everything lands on stream 1 (T0).
  for (int i = 0; i < n_pkts; ++i) {
    const size_t paylen = (i % 7 == 6) ? 300 : 1388;
    std::vector<uint8_t> pkt(12 + paylen);
    pkt[0] = 0x80; pkt[1] = 0x60;
    pkt[2] = static_cast<uint8_t>(i >> 8); pkt[3] = static_cast<uint8_t>(i & 0xFF);
    // HEVC FU (type 49), non-start non-end fragment of a trail NAL (type 1)
    pkt[12] = 49 << 1; pkt[13] = 1; pkt[14] = 1;  // fu header: type 1, mid
    for (size_t b = 15; b < pkt.size(); ++b) pkt[b] = static_cast<uint8_t>(rng());
    ++sent;
    auto bodies = enc.add_rtp(pkt.data(), pkt.size(), now);
    for (auto& b : bodies)
      if (keep()) got += dec.add_body(b.body.data(), b.body.size(), now).size();
    if (i % 30 == 29) {
      now += 15;
      for (auto& b : enc.poll(now))
        if (keep()) got += dec.add_body(b.body.data(), b.body.size(), now).size();
    }
  }
  now += 100;
  for (auto& b : enc.poll(now))
    if (keep()) got += dec.add_body(b.body.data(), b.body.size(), now).size();
  dec.poll(now + 5000);
  const auto st1 = dec.stats(1);
  const auto st0 = dec.stats(0);
  std::printf("sym=%d bpb=%d drop=%.1f%%xburst%d dropped=%llu/%llu | ", symbol_size, bpb_base, drop_pct, burst_len,
              (unsigned long long)dropped, (unsigned long long)bodies_n);
  std::printf("sent=%llu delivered=%llu (%.2f%%) s1[u=%llu fe=%llu] s0[u=%llu fe=%llu] misrouted=%llu\n",
              (unsigned long long)sent, (unsigned long long)got,
              100.0 * got / sent,
              (unsigned long long)st1.blocks_unrecoverable, (unsigned long long)st1.frag_evicted,
              (unsigned long long)st0.blocks_unrecoverable, (unsigned long long)st0.frag_evicted,
              (unsigned long long)dec.bodies_misrouted());
  return got == sent ? 0 : 1;
}

int main() {
  run(340, 4, 20000, 0.0, 1);
  run(340, 4, 20000, 5.0, 1);
  run(340, 4, 20000, 10.0, 1);
  run(340, 4, 20000, 1.0, 4);
  return 0;
}
