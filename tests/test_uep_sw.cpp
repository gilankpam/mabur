// Loss-injection matrix for the sliding-window FEC through the full UEP
// pipeline (UepEncoder -> lossy channel -> UepDecoder). Replaces the retired
// block-FEC time-diversity buffer matrix: time diversity now comes from
// overlapping repair windows, so sources never wait — the checks here pin
// delivery under the same random/burst loss the old scheme was built for.
#include <array>
#include <chrono>
#include <cstdint>
#include <thread>
#include <random>
#include <vector>

#include "frame_fixture.h"
#include "mabur/fec_worker.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "mabur/frame_wire.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
#include "mtest.h"

using namespace mabur;

namespace {

// sent/delivered count whole UNITS (frames): the encoder fragments each unit
// and the decoder emits raw fragments, so delivery is measured by units the
// receiver can fully reassemble — mtest::FragCollector does that assembly, the
// same job FrameStream does on the GS.
struct SimResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t bodies = 0, bodies_dropped = 0;
  uint64_t backlog_drops = 0;  // stream-1 repairs forfeited at kMaxBacklogJobs
};

// One frame unit for `stream`: FrameHdr + one Annex-B NAL whose type matches
// the layer (type 32 = VPS -> critical/stream 0; type 1 = TRAIL_R with
// nuh_temporal_id_plus1 = stream for stream 1), then random payload.
std::vector<uint8_t> make_unit(int stream, uint32_t frame_id, size_t paylen,
                               std::mt19937& rng) {
  std::vector<uint8_t> unit(framewire::kFrameHdrLen);
  framewire::FrameHdr h;
  h.frame_id = static_cast<uint16_t>(frame_id);
  h.flags = stream == 0 ? framewire::kFlagIdr : 0;
  h.pts_us = frame_id * 16667u;
  framewire::pack_frame_hdr(h, unit.data());
  for (uint8_t b : {0x00, 0x00, 0x00, 0x01}) unit.push_back(b);
  if (stream == 0) {
    unit.push_back(32 << 1);  // VPS -> critical
    unit.push_back(1);
  } else {
    unit.push_back(1 << 1);   // TRAIL_R
    unit.push_back(static_cast<uint8_t>(stream));  // tid_plus1 -> stream 1..3
  }
  while (unit.size() < framewire::kFrameHdrLen + paylen)
    unit.push_back(static_cast<uint8_t>(rng()));
  return unit;
}

std::array<UepLayerCfg, 2> layers_for(int symbol_size, int bpb_base, int window) {
  std::array<UepLayerCfg, 2> layers{};
  const int bpb[2] = {bpb_base, bpb_base * 2};
  for (int s = 0; s < 2; ++s) {
    layers[static_cast<size_t>(s)].fec = SwConfig{symbol_size, window, 0.5};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  return layers;
}

SimResult run_sim(int symbol_size, int bpb_base, int window, int n_pkts,
                  double drop_pct, int burst_len, FecWorker* worker = nullptr) {
  auto layers = layers_for(symbol_size, bpb_base, window);
  UepEncoder enc(layers, /*flush_ms=*/15, worker);
  UepDecoder dec(layers);

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
  mtest::FragCollector got;
  auto consume = [&](std::vector<UepBody>& bodies) {
    for (auto& b : bodies)
      if (keep())
        for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) got.add(d);
  };
  for (int i = 0; i < n_pkts; ++i) {
    const size_t paylen = (i % 7 == 6) ? 300 : 1388;
    auto unit = make_unit(/*stream=*/1, static_cast<uint32_t>(i), paylen, rng);
    ++r.sent;
    auto bodies = enc.add_frame(1, unit.data(), unit.size(), now);
    consume(bodies);
    // Simulated time has no wall-clock gaps between frames, so a starved
    // worker (a loaded CI runner) falls kMaxBacklogJobs behind and the
    // encoder forfeits repairs by design. Let it catch up each frame: the
    // async checks compare recovery, not how a slow core degrades.
    if (worker)
      while (worker->depth() != 0) std::this_thread::yield();
    if (i % 30 == 29) {
      now += 15;
      auto flushed = enc.poll(now);
      consume(flushed);
    }
  }
  now += 100;
  auto tail = enc.flush_all();
  consume(tail);
  r.delivered = got.completed().size();
  r.backlog_drops = enc.take_fec_gauge(1).backlog_drops;
  return r;
}

double pct(const SimResult& r) {
  return r.sent ? 100.0 * static_cast<double>(r.delivered) /
                      static_cast<double>(r.sent)
                : 0.0;
}

// Mixed per-layer geometry: stream 0 is the bench's critical-NAL config
// (sym 164/bpb 4), stream 1 uses the "global" 1312-symbol config at
// bpb 1 — the matrix burst_sim.cpp sweeps as "mixed-164/1312".
std::array<UepLayerCfg, 2> mixed_layers() {
  std::array<UepLayerCfg, 2> layers{};
  const int sym[2] = {164, 1312};
  const int bpb[2] = {4, 1};
  for (int s = 0; s < 2; ++s) {
    layers[static_cast<size_t>(s)].fec = SwConfig{sym[s], 64, 0.5};
    layers[static_cast<size_t>(s)].blocks_per_body = bpb[s];
  }
  return layers;
}

// Per-layer sent/delivered counters, plus a running index of the next body
// (across all layers) so a caller can drop a fixed, deterministic run of
// consecutive bodies mid-stream.
struct MixedResult {
  uint64_t sent[2] = {0, 0};
  uint64_t delivered[2] = {0, 0};
};

// Feeds n_pkts round-robined across both layers through the real
// UepEncoder -> (optional drop window) -> UepDecoder pipeline, following the
// same feed/poll/flush/drain cadence as run_sim(). drop_start/drop_count
// name a single deterministic window of consecutive *bodies* (not packets)
// to withhold from the decoder, mid-run; pass drop_count == 0 for the
// loss-free case.
MixedResult run_mixed_sim(int n_pkts, uint64_t drop_start, uint64_t drop_count) {
  auto layers = mixed_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers);

  MixedResult r;
  uint64_t body_idx = 0;
  mtest::FragCollector got;
  auto consume = [&](std::vector<UepBody>& bodies, uint64_t now) {
    for (auto& b : bodies) {
      uint64_t idx = body_idx++;
      if (drop_count > 0 && idx >= drop_start && idx < drop_start + drop_count)
        continue;
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) got.add(d);
    }
  };

  std::mt19937 rng(42);
  uint64_t now = 1000;
  for (int i = 0; i < n_pkts; ++i) {
    int stream = i % 2;
    auto unit = make_unit(stream, static_cast<uint32_t>(i), 1388, rng);
    ++r.sent[stream];
    auto bodies = enc.add_frame(stream, unit.data(), unit.size(), now);
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
  for (auto& [sid, unit] : got.completed()) ++r.delivered[sid];
  return r;
}

// Every SW envelope of one body, in wire order — a thin wrapper over
// sbi_unpack so the frame-declaration tests below can walk a frame's
// bodies and classify each envelope as a source or a frame-end repair.
std::vector<sw::SwHeader> headers_of(const UepBody& b, int block_payload) {
  std::vector<sw::SwHeader> out;
  for (auto& env : sbi_unpack(b.body.data(), b.body.size(), block_payload).survivors) {
    sw::SwHeader h;
    if (sw::parse_header(env.data(), env.size(), &h)) out.push_back(h);
  }
  return out;
}

// Shared shape check for a 100-source-symbol enh frame (layers_for(164, 4,
// 64)): the frame-end batch is one body of 8 repairs, each spanning the
// full window and ending at the frame's last source, with exactly 16
// sources (the held tail) between the last mid-frame repair and that body.
void expect_hundred_sym_frame_tail(const std::vector<UepBody>& bodies, int block_payload) {
  std::vector<sw::SwHeader> flat;
  for (auto& b : bodies) {
    auto hs = headers_of(b, block_payload);
    flat.insert(flat.end(), hs.begin(), hs.end());
  }
  uint32_t last_src_seq = 0;
  for (auto& h : flat) if (!h.repair) last_src_seq = h.seq;

  REQUIRE(flat.size() >= 8);
  const auto tail_hdrs = headers_of(bodies.back(), block_payload);
  REQUIRE(tail_hdrs.size() == 8);
  for (auto& h : tail_hdrs) {
    CHECK(h.repair);
    CHECK(h.window_len == 64);
    CHECK(h.seq + h.window_len == last_src_seq + 1);
  }

  const size_t tail = flat.size() - 8;
  size_t run = 0;
  while (run < tail && !flat[tail - 1 - run].repair) ++run;
  CHECK(run == 16);
  REQUIRE(tail > run);
  CHECK(flat[tail - 1 - run].repair);  // the last mid-frame repair
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
  // window 128 at overhead 0.50 tolerates bursts up to ~42).
  auto r = run_sim(164, 8, 128, 20000, 1.0, 4);
  CHECK(pct(r) >= 99.0);
}

TEST(wider_window_survives_longer_bursts) {
  // 6-body bursts = 48 consecutive T0 symbols: window 32 cannot span the
  // hole (budget ~10), window 128 can (~42) -- re-derived for overhead 0.50
  // (was ~13 / ~54 at the pre-flatten overhead 0.75).
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
  // ~2000 packets round-robined across both layers of the mixed geometry
  // (sym 164/1312, bpb 4/1, window 64), zero loss -> every layer must
  // deliver exactly what it sent.
  auto r = run_mixed_sim(2000, /*drop_start=*/0, /*drop_count=*/0);
  for (int s = 0; s < 2; ++s) CHECK(r.delivered[s] == r.sent[s]);
}

TEST(mixed_symbol_sizes_burst_within_budget) {
  // Same traffic; drop 8 consecutive bodies once, mid-run. bpb 1 on layer 1
  // means those bodies interleave source/repair envelopes for whichever
  // layer they land on, so 8 consecutive bodies costs a given layer well
  // under its worst-case symbol loss. Layer 1 (sym 1312/bpb1/window 64,
  // overhead 0.50 since the UEP flatten, was 0.25) has the
  // tightest guarantee band: burst_sim.cpp's self-check gate proves this
  // exact geometry lossless up to B<=30 (was B<=14 pre-flatten)
  // consecutive bodies on single-layer stream-1 traffic, so 8 is comfortably
  // within budget for every layer here (layer 0's bpb 4 gives it an even
  // wider margin per body). Deterministic, fixed start index (not seeded
  // random).
  auto r = run_mixed_sim(2000, /*drop_start=*/500, /*drop_count=*/8);
  for (int s = 0; s < 2; ++s) CHECK(r.delivered[s] == r.sent[s]);
}

// Single stream-1 pipeline (sym 1312 / bpb 1 / window 64) exposing the
// decoder so we can assert the loss-accounting invariant, not just delivery.
// Drops a fixed run of bodies starting at `drop_start`.
struct AcctResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t syms_delivered = 0, syms_recovered = 0, syms_abandoned = 0;
};

AcctResult run_stream1_acct(int n_pkts, uint64_t drop_start, uint64_t drop_count) {
  auto layers = mixed_layers();
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers);

  AcctResult r;
  uint64_t body_idx = 0;
  uint64_t now = 1000;
  mtest::FragCollector got;
  auto consume = [&](std::vector<UepBody>& bodies) {
    for (auto& b : bodies) {
      uint64_t idx = body_idx++;
      if (drop_count > 0 && idx >= drop_start && idx < drop_start + drop_count)
        continue;
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) got.add(d);
    }
  };

  std::mt19937 rng(3);
  for (int i = 0; i < n_pkts; ++i) {
    auto unit = make_unit(1, static_cast<uint32_t>(i), 1388, rng);  // all stream 1
    ++r.sent;
    auto bodies = enc.add_frame(1, unit.data(), unit.size(), now);
    consume(bodies);
    if (i % 30 == 29) { now += 15; auto f = enc.poll(now); consume(f); }
  }
  now += 100;
  auto tail = enc.flush_all();
  consume(tail);

  for (auto& [sid, unit] : got.completed())
    if (sid == 1) ++r.delivered;
  auto st = dec.stats(1);
  r.syms_delivered = st.syms_delivered;
  r.syms_recovered = st.syms_recovered;
  r.syms_abandoned = st.syms_abandoned;
  return r;
}

// REPRODUCES the hardware "uncounted loss" bug (bigsymbol-bug-report.md).
// Withholding the OPENING body of a stream-1 (sym 1312 / bpb 1) run costs one
// unit — but the decoder anchors base_ at the FIRST source it *sees*, so the
// lost leading source(s) sit permanently below base_ and are NEVER booked as
// syms_abandoned. That is the exact hardware signature: units vanish while
// every loss counter stays frozen. The invariant every non-abandoned source
// must be delivered/recovered is violated silently.
//
// This is the systematic, sustained mechanism (not a one-off tail): it fires on
// every stream (re)join and after any burst that wipes the leading sources of a
// re-anchoring window — matching the measured 2.6-3.65% stream-1 loss.
TEST(opening_loss_is_accounted) {
  // Withhold the first two bodies of the run. That loses the leading
  // source(s) of stream 1 — but the decoder sets base_ to the FIRST source it
  // actually sees, so those lost sources sit permanently below base_ and are
  // never booked as syms_abandoned. FEC cannot recover them either (their
  // covering repairs arrive before any anchor and are dropped as stale).
  // Result: units vanish with every loss counter frozen — the measured
  // hardware signature.
  auto r = run_stream1_acct(400, /*drop_start=*/0, /*drop_count=*/2);
  const uint64_t missing = r.sent - r.delivered;
  // Every missing unit MUST be accounted by some loss counter.
  // FAILS today: missing >= 1 while syms_abandoned == 0 (uncounted vanish).
  CHECK(missing == 0 || r.syms_abandoned >= missing);
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
  CHECK(async_r.backlog_drops == 0);
  CHECK(pct(async_r) >= pct(sync_r) - 1.0);
}

// fec-join-delete: the frame-end flush no longer waits for
// the worker, so add_frame ships sources (plus whatever repairs are already
// built) and collect() harvests the rest once the worker finishes — as
// bodies of the right stream, with the packer's partial group flushed so
// nothing lingers until the next frame. 77 source symbols (> the window 64)
// so the frame's front still spends credit through the worker as usual —
// frame_hold_syms only holds its last 16 syms, built inline at flush() —
// leaving something for collect() to harvest.
TEST(uep_collect_harvests_repairs_the_frame_left_behind) {
  auto layers = layers_for(164, 4, 64);
  FecWorker worker;
  worker.set_held(true);
  UepEncoder enc(layers, 15, &worker);
  UepDecoder dec(layers);
  std::mt19937 rng(42);
  const uint64_t now = 1000;
  auto unit = make_unit(/*stream=*/1, 0, 77 * 156 - 8, rng);  // 77 symbols exactly
  auto frame_bodies = enc.add_frame(1, unit.data(), unit.size(), now);
  CHECK(!frame_bodies.empty());
  std::vector<UepBody> harvested;
  auto sink = [&](UepBody&& b) { harvested.push_back(std::move(b)); };
  enc.collect(sink);
  CHECK(harvested.empty());  // worker held: nothing finished yet
  worker.set_held(false);
  // Wait for the worker to finish the frame's repairs, then harvest.
  for (int i = 0; i < 3000 && harvested.empty(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    enc.collect(sink);
  }
  REQUIRE(!harvested.empty());
  for (auto& b : harvested) CHECK(b.stream_id == 1);
  // Once idle, a second harvest finds nothing pending anywhere.
  std::vector<UepBody> again;
  enc.collect([&](UepBody&& b) { again.push_back(std::move(b)); });
  CHECK(again.empty());
  // Everything decodes: sources + late repairs are one consistent stream.
  mtest::FragCollector got;
  for (auto& b : frame_bodies)
    for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) got.add(d);
  for (auto& b : harvested)
    for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) got.add(d);
  CHECK(got.completed().size() == 1);
  // Repairs did ship: the harvested bodies carry the frame's whole repair
  // budget (ov 0.5 over the frame's symbols + the tail repair), never zero.
  CHECK(enc.take_fec_gauge(1).jobs > 0);
}

// UepEncoder declares every frame (begin_frame/frame_hold_syms), so the
// frame's sources always close their body before the frame-end repairs
// ship in one of their own: one lost body cannot take a frame's tail
// sources together with the repairs that protect them.
TEST(small_frame_end_repairs_cover_the_whole_frame_in_their_own_body) {
  auto layers = layers_for(164, 4, 64);
  UepEncoder enc(layers, 15);
  std::mt19937 rng(42);
  auto unit = make_unit(/*stream=*/1, 0, 30 * 156 - 8, rng);  // 30 symbols exactly
  auto bodies = enc.add_frame(1, unit.data(), unit.size(), 1000);
  const int bp = static_cast<int>(sw::kSwHeaderLen) + 164;

  size_t n_src = 0, n_rep = 0;
  bool seen_repair = false;
  uint32_t last_src_seq = 0;
  std::vector<size_t> repair_body_sizes;
  for (auto& b : bodies) {
    auto hs = headers_of(b, bp);
    REQUIRE(!hs.empty());
    const bool is_repair = hs.front().repair;
    for (auto& h : hs) CHECK(h.repair == is_repair);  // no body mixes kinds
    if (is_repair) {
      seen_repair = true;
      repair_body_sizes.push_back(hs.size());
      for (auto& h : hs) {
        CHECK(h.window_len == 30);
        CHECK(h.seq + h.window_len == last_src_seq + 1);
        ++n_rep;
      }
    } else {
      CHECK(!seen_repair);  // every source body precedes every repair body
      for (auto& h : hs) { last_src_seq = h.seq; ++n_src; }
    }
  }
  CHECK(n_src == 30);
  CHECK(n_rep == 15);  // ceil(30 * 0.5), all held: nothing spent mid-frame
  REQUIRE(repair_body_sizes.size() == 2);
  CHECK(repair_body_sizes[0] == 8);
  CHECK(repair_body_sizes[1] == 7);
}

TEST(big_frame_tail_has_one_body_of_repairs_behind_it) {
  auto layers = layers_for(164, 4, 64);
  UepEncoder enc(layers, 15);
  std::mt19937 rng(42);
  auto unit = make_unit(/*stream=*/1, 0, 100 * 156 - 8, rng);  // 100 symbols exactly
  auto bodies = enc.add_frame(1, unit.data(), unit.size(), 1000);
  const int bp = static_cast<int>(sw::kSwHeaderLen) + 164;
  expect_hundred_sym_frame_tail(bodies, bp);
}

TEST(overhead_change_moves_the_tail_start) {
  auto layers = layers_for(164, 4, 64);
  UepEncoder enc(layers, 15);
  enc.set_layer_overhead(1, 0.25);
  std::mt19937 rng(42);
  auto unit = make_unit(/*stream=*/1, 0, 100 * 156 - 8, rng);  // 100 symbols exactly
  auto bodies = enc.add_frame(1, unit.data(), unit.size(), 1000);
  const int bp = static_cast<int>(sw::kSwHeaderLen) + 164;

  std::vector<sw::SwHeader> flat;
  for (auto& b : bodies) {
    auto hs = headers_of(b, bp);
    flat.insert(flat.end(), hs.begin(), hs.end());
  }
  const auto tail_hdrs = headers_of(bodies.back(), bp);
  REQUIRE(tail_hdrs.size() == 8);  // ceil(32 * 0.25) = 8, same frame-end body size
  for (auto& h : tail_hdrs) CHECK(h.repair);

  // T = ceil(8/0.25) = 32 sources follow the last mid-frame repair.
  const size_t tail = flat.size() - 8;
  size_t run = 0;
  while (run < tail && !flat[tail - 1 - run].repair) ++run;
  CHECK(run == 32);
}

TEST(shed_frame_leaves_no_frame_state) {
  auto layers = layers_for(164, 4, 64);
  UepEncoder enc(layers, 15);
  std::mt19937 rng(42);
  enc.set_shed(1, true);
  auto shed_unit = make_unit(/*stream=*/1, 0, 30 * 156 - 8, rng);
  CHECK(enc.add_frame(1, shed_unit.data(), shed_unit.size(), 1000).empty());
  enc.set_shed(1, false);

  auto unit = make_unit(/*stream=*/1, 1, 100 * 156 - 8, rng);
  auto bodies = enc.add_frame(1, unit.data(), unit.size(), 1000);
  const int bp = static_cast<int>(sw::kSwHeaderLen) + 164;
  expect_hundred_sym_frame_tail(bodies, bp);
}

// The whole point of a frame-end batch in its own body: losing the body
// carrying a frame's LAST sources must not lose the frame, because its
// repairs ship in a body of their own, never bundled with those sources.
TEST(late_loss_recovered_by_the_frame_end_batch) {
  auto layers = layers_for(164, 4, 64);
  UepEncoder enc(layers, 15);
  UepDecoder dec(layers);
  std::mt19937 rng(42);
  auto unit = make_unit(/*stream=*/1, 0, 30 * 156 - 8, rng);  // 30 symbols exactly
  auto bodies = enc.add_frame(1, unit.data(), unit.size(), 1000);
  const int bp = static_cast<int>(sw::kSwHeaderLen) + 164;

  // The last body whose envelopes are sources carries the frame's last
  // source(s).
  size_t drop_idx = bodies.size();
  for (size_t i = 0; i < bodies.size(); ++i) {
    auto hs = headers_of(bodies[i], bp);
    REQUIRE(!hs.empty());
    if (!hs.front().repair) drop_idx = i;
  }
  REQUIRE(drop_idx < bodies.size());

  mtest::FragCollector got;
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (i == drop_idx) continue;  // one whole body dropped
    for (auto& d : dec.add_body(bodies[i].body.data(), bodies[i].body.size(), 1000))
      got.add(d);
  }
  CHECK(got.completed().size() == 1);
}

MTEST_MAIN
