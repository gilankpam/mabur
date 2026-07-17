// Loss-injection matrix for the sliding-window FEC through the full UEP
// pipeline (UepEncoder -> lossy channel -> UepDecoder). Replaces the retired
// block-FEC time-diversity buffer matrix: time diversity now comes from
// overlapping repair windows, so sources never wait — the checks here pin
// delivery under the same random/burst loss the old scheme was built for
// (bench 2026-07-13, docs/handover-video-delivery.md §2).
#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "mabur/fec_worker.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
#include "mtest.h"

using namespace mabur;

namespace {

struct SimResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t bodies = 0, bodies_dropped = 0;
};

std::array<UepLayerCfg, 4> layers_for(int symbol_size, int bpb_base, int window) {
  std::array<UepLayerCfg, 4> layers{};
  const int bpb[4] = {bpb_base, bpb_base, bpb_base * 2, bpb_base * 2};
  for (int s = 0; s < 4; ++s) {
    layers[static_cast<size_t>(s)].fec =
        SwConfig{symbol_size, window, kUepRefOverhead[s]};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  return layers;
}

SimResult run_sim(int symbol_size, int bpb_base, int window, int n_pkts,
                  double drop_pct, int burst_len, FecWorker* worker = nullptr) {
  auto layers = layers_for(symbol_size, bpb_base, window);
  UepEncoder enc(layers, /*flush_ms=*/15, worker);
  UepDecoder dec(layers, /*decode_deadline_ms=*/200);

  std::mt19937 drop_rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  int burst_left = 0;
  SimResult r;
  auto keep = [&]() {
    ++r.bodies;
    if (burst_left > 0) { --burst_left; ++r.bodies_dropped; return false; }
    if (u(drop_rng) * 100.0 < drop_pct) {
      ++r.bodies_dropped;
      if (burst_len > 1) burst_left = burst_len - 1;
      return false;
    }
    return true;
  };

  std::mt19937 rng(42);
  uint64_t now = 1000;
  auto consume = [&](std::vector<UepBody>& bodies) {
    for (auto& b : bodies)
      if (keep())
        r.delivered += dec.add_body(b.body.data(), b.body.size(), now).size();
  };
  for (int i = 0; i < n_pkts; ++i) {
    const size_t paylen = (i % 7 == 6) ? 300 : 1388;
    std::vector<uint8_t> pkt(12 + paylen);
    pkt[0] = 0x80; pkt[1] = 0x60;
    pkt[2] = static_cast<uint8_t>(i >> 8);
    pkt[3] = static_cast<uint8_t>(i & 0xFF);
    pkt[12] = 49 << 1; pkt[13] = 1; pkt[14] = 1;  // HEVC FU -> stream 1 (T0)
    for (size_t b = 15; b < pkt.size(); ++b)
      pkt[b] = static_cast<uint8_t>(rng());
    ++r.sent;
    auto bodies = enc.add_rtp(pkt.data(), pkt.size(), now);
    consume(bodies);
    if (i % 30 == 29) {
      now += 15;
      auto flushed = enc.poll(now);
      consume(flushed);
    }
  }
  now += 100;
  auto tail = enc.flush_all();
  consume(tail);
  dec.poll(now + 5000);
  return r;
}

double pct(const SimResult& r) {
  return r.sent ? 100.0 * static_cast<double>(r.delivered) /
                      static_cast<double>(r.sent)
                : 0.0;
}

// Mixed per-layer geometry: stream 0 is the bench's critical-NAL config
// (sym 164/bpb 4), streams 1..3 share the "global" 1312-symbol config at
// bpb 1 — the matrix burst_sim.cpp sweeps as "mixed-164/1312".
std::array<UepLayerCfg, 4> mixed_layers() {
  std::array<UepLayerCfg, 4> layers{};
  const int sym[4] = {164, 1312, 1312, 1312};
  const int bpb[4] = {4, 1, 1, 1};
  for (int s = 0; s < 4; ++s) {
    layers[static_cast<size_t>(s)].fec = SwConfig{sym[s], 64, kUepRefOverhead[s]};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  return layers;
}

// RTP+HEVC packet that classify_rtp routes to the requested stream (same
// convention as tools/bench/burst_sim.cpp::make_rtp): stream 0 -> NAL type
// 32 (VPS, critical), streams 1..3 -> NAL type 1 (TRAIL_R) with
// nuh_temporal_id_plus1 = stream.
std::vector<uint8_t> make_stream_rtp(int stream, uint32_t seq, std::mt19937& rng) {
  const size_t paylen = 1388;
  std::vector<uint8_t> pkt(12 + paylen);
  pkt[0] = 0x80; pkt[1] = 0x60;
  pkt[2] = static_cast<uint8_t>(seq >> 8);
  pkt[3] = static_cast<uint8_t>(seq & 0xFF);
  if (stream == 0) {
    pkt[12] = 32 << 1;  // NAL type 32 = VPS -> critical, stream 0
    pkt[13] = 1;
  } else {
    pkt[12] = 1 << 1;   // NAL type 1 = TRAIL_R
    pkt[13] = static_cast<uint8_t>(stream);  // tid_plus1 -> stream 1..3
  }
  for (size_t b = 14; b < pkt.size(); ++b)
    pkt[b] = static_cast<uint8_t>(rng());
  return pkt;
}

// Per-layer sent/delivered counters, plus a running index of the next body
// (across all layers) so a caller can drop a fixed, deterministic run of
// consecutive bodies mid-stream.
struct MixedResult {
  uint64_t sent[4] = {0, 0, 0, 0};
  uint64_t delivered[4] = {0, 0, 0, 0};
};

// Feeds n_pkts round-robined across all 4 layers through the real
// UepEncoder -> (optional drop window) -> UepDecoder pipeline, following the
// same feed/poll/flush/drain cadence as run_sim(). drop_start/drop_count
// name a single deterministic window of consecutive *bodies* (not packets)
// to withhold from the decoder, mid-run; pass drop_count == 0 for the
// loss-free case.
MixedResult run_mixed_sim(int n_pkts, uint64_t drop_start, uint64_t drop_count) {
  auto layers = mixed_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers, /*decode_deadline_ms=*/200);

  MixedResult r;
  uint64_t body_idx = 0;
  auto consume = [&](std::vector<UepBody>& bodies, uint64_t now) {
    for (auto& b : bodies) {
      uint64_t idx = body_idx++;
      if (drop_count > 0 && idx >= drop_start && idx < drop_start + drop_count)
        continue;
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), now))
        ++r.delivered[d.stream_id];
    }
  };

  std::mt19937 rng(42);
  uint64_t now = 1000;
  for (int i = 0; i < n_pkts; ++i) {
    int stream = i % 4;
    auto pkt = make_stream_rtp(stream, static_cast<uint32_t>(i), rng);
    ++r.sent[stream];
    auto bodies = enc.add_rtp(pkt.data(), pkt.size(), now);
    consume(bodies, now);
    if (i % 30 == 29) {
      now += 15;
      auto flushed = enc.poll(now);
      consume(flushed, now);
    }
  }
  now += 100;
  auto tail = enc.flush_all();
  consume(tail, now);
  dec.poll(now + 5000);
  return r;
}

}  // namespace

TEST(lossless_delivers_everything) {
  auto r = run_sim(164, 8, 128, 5000, 0.0, 1);
  CHECK(r.delivered == r.sent);
}

TEST(survives_random_frame_loss) {
  // The bench geometry (symbol 164 / bpb 8) at 5% random body loss — the
  // scenario that killed the old block-FEC scheme at ~91%.
  auto r = run_sim(164, 8, 128, 20000, 5.0, 1);
  CHECK(pct(r) >= 99.5);
}

TEST(survives_heavier_random_loss) {
  auto r = run_sim(164, 8, 128, 20000, 10.0, 1);
  CHECK(pct(r) >= 99.0);
}

TEST(survives_burst_loss) {
  // 1% loss events x 4-body bursts (T0: 32 consecutive symbols lost;
  // window 128 at overhead 0.75 tolerates bursts up to ~54).
  auto r = run_sim(164, 8, 128, 20000, 1.0, 4);
  CHECK(pct(r) >= 99.0);
}

TEST(wider_window_survives_longer_bursts) {
  // 6-body bursts = 48 consecutive T0 symbols: window 32 cannot span the
  // hole (budget ~13), window 128 can (~54).
  auto w32 = run_sim(164, 8, 32, 20000, 1.0, 6);
  auto w128 = run_sim(164, 8, 128, 20000, 1.0, 6);
  CHECK(pct(w128) >= 99.0);
  CHECK(pct(w128) > pct(w32));
}

TEST(alternate_geometry) {
  auto r = run_sim(340, 4, 128, 20000, 5.0, 1);
  CHECK(pct(r) >= 99.0);
}

TEST(mixed_symbol_sizes_lossfree_delivery) {
  // ~2000 packets round-robined across all 4 layers of the mixed geometry
  // (sym 164/1312/1312/1312, bpb 4/1/1/1, window 64), zero loss -> every
  // layer must deliver exactly what it sent.
  auto r = run_mixed_sim(2000, /*drop_start=*/0, /*drop_count=*/0);
  for (int s = 0; s < 4; ++s) CHECK(r.delivered[s] == r.sent[s]);
}

TEST(mixed_symbol_sizes_burst_within_budget) {
  // Same traffic; drop 8 consecutive bodies once, mid-run. bpb 1 on layers
  // 1..3 means those bodies interleave source/repair envelopes for whichever
  // layer they land on, so 8 consecutive bodies costs a given layer well
  // under its worst-case symbol loss. Layer 3 (sym 1312/bpb1/window 64,
  // overhead 0.25) has the tightest guarantee band: burst_sim.cpp's
  // self-check gate proves this exact geometry lossless up to B<=14
  // consecutive bodies on single-layer stream-3 traffic, so 8 is comfortably
  // within budget for every layer here (layer 0's bpb 4 gives it an even
  // wider margin per body). Deterministic, fixed start index (not seeded
  // random).
  auto r = run_mixed_sim(2000, /*drop_start=*/500, /*drop_count=*/8);
  for (int s = 0; s < 4; ++s) CHECK(r.delivered[s] == r.sent[s]);
}

// Single stream-1 pipeline (sym 1312 / bpb 1 / window 64) exposing the
// decoder so we can assert the loss-accounting invariant, not just delivery.
// Drops a fixed run of bodies starting at `drop_start`.
struct AcctResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t syms_delivered = 0, syms_recovered = 0, syms_abandoned = 0;
  uint64_t frag_evicted = 0;
};

AcctResult run_stream1_acct(int n_pkts, uint64_t drop_start, uint64_t drop_count) {
  auto layers = mixed_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers, /*decode_deadline_ms=*/200);

  AcctResult r;
  uint64_t body_idx = 0;
  uint64_t now = 1000;
  auto consume = [&](std::vector<UepBody>& bodies) {
    for (auto& b : bodies) {
      uint64_t idx = body_idx++;
      if (drop_count > 0 && idx >= drop_start && idx < drop_start + drop_count)
        continue;
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), now))
        if (d.stream_id == 1) ++r.delivered;
    }
  };

  std::mt19937 rng(3);
  for (int i = 0; i < n_pkts; ++i) {
    auto pkt = make_stream_rtp(1, static_cast<uint32_t>(i), rng);  // all stream 1
    ++r.sent;
    auto bodies = enc.add_rtp(pkt.data(), pkt.size(), now);
    consume(bodies);
    if (i % 30 == 29) { now += 15; auto f = enc.poll(now); consume(f); }
  }
  now += 100;
  auto tail = enc.flush_all();
  consume(tail);
  dec.poll(now + 5000);

  auto st = dec.stats(1);
  r.syms_delivered = st.syms_delivered;
  r.syms_recovered = st.syms_recovered;
  r.syms_abandoned = st.syms_abandoned;
  r.frag_evicted = st.frag_evicted;
  return r;
}

// REPRODUCES the hardware "uncounted RTP loss" bug (bigsymbol-bug-report.md).
// Withholding the OPENING body of a stream-1 (sym 1312 / bpb 1) run costs one
// RTP packet — but the decoder anchors base_ at the FIRST source it *sees*, so
// the lost leading source(s) sit permanently below base_ and are NEVER booked
// as syms_abandoned (nor frag_evicted). That is the exact hardware signature:
// packets vanish while every loss counter stays frozen. The invariant every
// non-abandoned source must be delivered/recovered is violated silently.
//
// This is the systematic, sustained mechanism (not a one-off tail): it fires on
// every stream (re)join and after any burst that wipes the leading sources of a
// re-anchoring window — matching the measured 2.6-3.65% stream-1 loss.
TEST(opening_loss_is_accounted) {
  // Withhold the first two bodies of the run. That loses the leading
  // source(s) of stream 1 — but the decoder sets base_ to the FIRST source it
  // actually sees, so those lost sources sit permanently below base_ and are
  // never booked as syms_abandoned (nor caught by frag_evicted). FEC cannot
  // recover them either (their covering repairs arrive before any anchor and
  // are dropped as stale). Result: RTP packets vanish with every loss counter
  // frozen — the measured hardware signature.
  auto r = run_stream1_acct(400, /*drop_start=*/0, /*drop_count=*/2);
  const uint64_t missing = r.sent - r.delivered;
  const uint64_t counted_loss = r.syms_abandoned + r.frag_evicted;
  // Every missing RTP packet MUST be accounted by some loss counter.
  // FAILS today: missing >= 1 while counted_loss == 0 (uncounted vanish).
  CHECK(missing == 0 || counted_loss >= missing);
}

// Async worker through the full UEP pipeline: zero loss must deliver 100%
// (repairs trailing their sources is invisible to the decoder), and lossy
// delivery must match the sync pipeline within noise (the drop pattern
// shifts because repairs pack into different bodies, hence the tolerance).
TEST(uep_async_no_loss_delivers_everything) {
  FecWorker worker;
  SimResult r = run_sim(164, 8, 64, 3000, 0.0, 1, &worker);
  CHECK(r.delivered == r.sent);
}

TEST(uep_async_recovers_under_loss_like_sync) {
  SimResult sync_r = run_sim(164, 8, 64, 4000, 5.0, 3);
  FecWorker worker;
  SimResult async_r = run_sim(164, 8, 64, 4000, 5.0, 3, &worker);
  CHECK(pct(async_r) >= pct(sync_r) - 1.0);
}

MTEST_MAIN
