// Host-side burst-loss simulator for the sliding-window UEP pipeline.
// Real UepEncoder -> body-loss channel -> real UepDecoder, fake clock.
// Quantifies residual per-layer frame loss vs burst length across
// symbol_size/bpb/window configs (spec: 2026-07-15-per-layer-symbol-size).
// 2-stream space since Task 3 (airtime-balance-uep): sid 0 = critical/base,
// sid 1 = bulk/enhance (the old sid 1..3 bulk streams collapsed to one).
//
// Modes:
//   burst_sim               full sweep, human table + CSV lines ("CSV,...")
//   burst_sim --self-check  acceptance gates only, exit 0/1 (run by ctest)
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "mabur/frame_wire.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"

using namespace mabur;

namespace {

struct LayerSpec {
  std::array<int, 2> sym;
  std::array<int, 2> bpb;
  int window;
  const char* name;
};

// Literal overhead (Task 3): 0.5 on both layers is the flattened value the
// old kUepRefOverhead ladder carried at every index (2026-08-29 UEP
// flatten), so this reproduces the exact SwConfig geometry pre-Task-3
// layers_for() built via kUepRefOverhead[s] — no ×2 rule needed here, this
// constant was already the literal applied overhead, never a scaled cmd
// value.
std::array<UepLayerCfg, 2> layers_for(const LayerSpec& sp) {
  std::array<UepLayerCfg, 2> layers{};
  for (int s = 0; s < 2; ++s) {
    layers[(size_t)s].fec = SwConfig{sp.sym[(size_t)s], sp.window, 0.5};
    layers[(size_t)s].blocks_per_body = sp.bpb[(size_t)s];
  }
  return layers;
}

// One frame unit for the wanted layer, as maburd sends it: FrameHdr + a single
// Annex-B NAL whose header matches the layer (stream 0 -> NAL type 32 (VPS,
// critical); stream 1 -> type 1 (TRAIL_R) with nuh_temporal_id_plus1 = 1).
// The NAL payload carries [stream u8 | seq u32 LE] for exact delivery
// accounting.
std::vector<uint8_t> make_unit(int stream, uint32_t seq, size_t total_len) {
  std::vector<uint8_t> p(total_len, 0xC5);
  framewire::FrameHdr h;
  h.frame_id = (uint16_t)seq;
  h.flags = stream == 0 ? framewire::kFlagIdr : 0;
  h.pts_us = seq * 16667u;
  framewire::pack_frame_hdr(h, p.data());
  const size_t off = framewire::kFrameHdrLen;
  p[off] = 0x00; p[off + 1] = 0x00; p[off + 2] = 0x00; p[off + 3] = 0x01;
  if (stream == 0) {
    p[off + 4] = (uint8_t)(32 << 1);  // NAL type 32 = VPS
    p[off + 5] = 1;                   // tid_plus1 = 1
  } else {
    p[off + 4] = (uint8_t)(1 << 1);   // NAL type 1 = TRAIL_R
    p[off + 5] = (uint8_t)stream;     // tid_plus1 1 -> stream 1
  }
  p[off + 6] = (uint8_t)stream;
  p[off + 7] = (uint8_t)(seq & 0xFF);
  p[off + 8] = (uint8_t)((seq >> 8) & 0xFF);
  p[off + 9] = (uint8_t)((seq >> 16) & 0xFF);
  p[off + 10] = (uint8_t)((seq >> 24) & 0xFF);
  return p;
}

bool read_tag(const std::vector<uint8_t>& unit, int* stream, uint32_t* seq) {
  const size_t off = framewire::kFrameHdrLen + 6;  // start code + NAL header
  if (unit.size() < off + 5) return false;
  *stream = unit[off];
  *seq = (uint32_t)unit[off + 1] | ((uint32_t)unit[off + 2] << 8) |
         ((uint32_t)unit[off + 3] << 16) | ((uint32_t)unit[off + 4] << 24);
  return true;
}

// Reassembles whole units from the raw FRAG fragments UepDecoder emits — the
// job FrameStream does on the GS, reduced to what delivery accounting needs.
class UnitAssembler {
 public:
  // Returns the completed unit, or an empty vector while fragments are missing.
  std::vector<uint8_t> add(const DecodedFrag& f) {
    if (f.frag.size() < Fragmenter::kHdrLen) return {};
    const uint16_t fseq = (uint16_t)(f.frag[0] | (f.frag[1] << 8));
    const uint16_t idx = (uint16_t)(f.frag[2] | (f.frag[3] << 8));
    const uint16_t count = (uint16_t)(f.frag[4] | (f.frag[5] << 8));
    if (count == 0) return {};
    const uint32_t key = ((uint32_t)f.stream_id << 16) | fseq;
    auto& e = pending_[key];
    e.count = count;
    e.chunks[idx].assign(f.frag.begin() + (long)Fragmenter::kHdrLen, f.frag.end());
    if (e.chunks.size() != count) return {};
    std::vector<uint8_t> unit;
    for (uint16_t i = 0; i < count; ++i) {
      auto c = e.chunks.find(i);
      if (c == e.chunks.end()) return {};
      unit.insert(unit.end(), c->second.begin(), c->second.end());
    }
    pending_.erase(key);
    return unit;
  }

 private:
  struct Entry {
    std::map<uint16_t, std::vector<uint8_t>> chunks;
    uint16_t count = 0;
  };
  std::map<uint32_t, Entry> pending_;
};

// Loss channel interface: returns true if this body is dropped.
struct PeriodicBurst {
  int burst_len;             // bodies killed per event
  uint64_t period_ms = 250;  // one event per period
  uint64_t next_ms = 250;
  int killing = 0;
  bool drop(uint64_t now_ms) {
    if (killing > 0) { --killing; return true; }
    if (now_ms >= next_ms) {
      next_ms += period_ms;
      killing = burst_len - 1;
      return true;
    }
    return false;
  }
};

struct GilbertElliott {
  // Mean burst 5 bodies (q=0.2), average loss ~3% (p = 0.03*q/0.97).
  double p = 0.00619, q = 0.2;
  bool bad = false;
  std::mt19937 rng{99};
  bool drop(uint64_t) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    bad = bad ? (u(rng) >= q) : (u(rng) < p);
    return bad;
  }
};

struct LayerResult {
  uint64_t sent = 0, delivered = 0;
  uint64_t max_latency_ms = 0;
};

struct SimOut {
  std::array<LayerResult, 2> layer;
  uint64_t bodies = 0, dropped = 0;
};

// Video-shaped traffic at ~9.1 Mbps for dur_ms of fake time: whole frames at
// 62.5 fps, one IDR frame per second on stream 0 (critical), everything else
// on stream 1 (bulk/enhance — the old t1/t2/t3 mix collapsed to the single
// enhance layer under the 2-stream space). single_layer >= 0 sends ALL
// frames on that stream (self-check mode needs a crisp single-layer budget
// edge).
template <typename Loss>
SimOut run(const std::array<UepLayerCfg, 2>& layers, Loss loss,
           uint64_t dur_ms, int single_layer = -1) {
  UepEncoder enc(layers, /*flush_ms=*/15);
  UepDecoder dec(layers, /*decode_deadline_ms=*/200, /*seq_horizon=*/512);
  SimOut out{};
  UnitAssembler asm_;
  std::map<std::pair<int, uint32_t>, uint64_t> sent_at;
  uint32_t seq[2] = {0, 0};
  // Traffic shape: whole frames, one per kFrameMs, which is what the frame-shm
  // ingest path actually sends. 18200 B every 16 ms = 9.1 Mbps at 62.5 fps.
  // Frame SIZE matters to this sim beyond bitrate: add_frame seals the window
  // at every frame end (one tail repair per frame), so modelling video as many
  // small units would inflate redundancy far past what the layer's overhead
  // setting buys — 1200 B units made the bulk stream lossless out to B=80.
  const uint64_t kFrameMs = 16;
  const size_t kFrameBytes = 18200;
  // Frames keep flowing for kCooldownMs past dur_ms but are NOT counted: the
  // last counted frame needs following traffic for its sliding window to
  // complete, else it shows as a permanent loss that is an artifact of the sim
  // ending rather than of the channel (before this, every B >= 5 lost exactly
  // frame N-1 and nothing else).
  const uint64_t kCooldownMs = 500;
  // Runs a batch of encoder-produced bodies through the loss channel and the
  // decoder, crediting delivery/latency per reassembled unit. Shared by the
  // live add_frame() path and the idle-timeout poll() path — both carry real
  // traffic subject to the same channel.
  bool counting = true;
  uint32_t counted_upto[2] = {0, 0};  // valid once counting == false
  auto sink = [&](std::vector<UepBody>& bodies, uint64_t now) {
    for (auto& b : bodies) {
      ++out.bodies;
      if (loss.drop(now)) { ++out.dropped; continue; }
      for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) {
        auto unit = asm_.add(d);
        int ds;
        uint32_t dq;
        if (unit.empty() || !read_tag(unit, &ds, &dq)) continue;
        if (!counting && dq >= counted_upto[(size_t)ds]) continue;  // cooldown
        ++out.layer[(size_t)ds].delivered;
        uint64_t lat = now - sent_at[{ds, dq}];
        if (lat > out.layer[(size_t)ds].max_latency_ms)
          out.layer[(size_t)ds].max_latency_ms = lat;
      }
    }
  };
  auto feed = [&](const std::vector<uint8_t>& unit, uint64_t now) {
    int st;
    uint32_t sq;
    read_tag(unit, &st, &sq);
    sent_at[{st, sq}] = now;
    if (counting) ++out.layer[(size_t)st].sent;
    auto bodies = enc.add_frame(st, unit.data(), unit.size(), now);
    sink(bodies, now);
  };
  uint64_t frame_i = 0;
  for (uint64_t now = 1; now <= dur_ms + kCooldownMs; ++now) {
    if (counting && now > dur_ms) {
      for (int s = 0; s < 2; ++s) counted_upto[(size_t)s] = seq[s];
      counting = false;
    }
    if (now % kFrameMs == 0) {
      // Non-single-layer mode: every frame is bulk (stream 1) except one
      // IDR per second of video on the critical layer (stream 0), replacing
      // that interval's P frame (a real GOP boundary, not extra traffic).
      int st = single_layer >= 0 ? single_layer : 1;
      if (single_layer < 0 && frame_i % 60 == 0) st = 0;
      feed(make_unit(st, seq[st]++, kFrameBytes), now);
      ++frame_i;
    }
    if (now % 100 == 0) {
      auto polled = enc.poll(now);
      sink(polled, now);
      dec.poll(now);
    }
  }
  // Drain: flush encoder, let deadlines expire, final poll. The flush_all()
  // tail is the encoder's teardown flush, not live traffic the burst/GE
  // channel is mid-cycle on — running it through loss.drop() re-applies a
  // channel event that's "due" from the live phase to a one-off drain body,
  // an artifact of the sim ending rather than a channel loss (confirmed via
  // instrumentation: the tail body's drop coincided with the periodic
  // channel's next scheduled kill on every affected config). Deliver it
  // unconditionally, matching a real deployment where there is no "end".
  uint64_t now = dur_ms + kCooldownMs;
  for (auto& b : enc.flush_all()) {
    ++out.bodies;
    for (auto& d : dec.add_body(b.body.data(), b.body.size(), now)) {
      auto unit = asm_.add(d);
      int ds;
      uint32_t dq;
      if (!unit.empty() && read_tag(unit, &ds, &dq))
        ++out.layer[(size_t)ds].delivered;
    }
  }
  dec.poll(now + 500);
  return out;
}

double residual_pct(const LayerResult& l) {
  return l.sent ? 100.0 * (double)(l.sent - l.delivered) / (double)l.sent : 0.0;
}

int self_check() {
  int failures = 0;
  auto expect = [&](bool ok, const char* what) {
    if (!ok) { std::printf("SELF-CHECK FAIL: %s\n", what); ++failures; }
  };
  const LayerSpec big{{164, 1312}, {4, 1}, 64, "big"};
  const LayerSpec cur{{164, 164}, {8, 8}, 64, "cur"};
  // Gate 1: B=1 -> zero residual loss for both configs (60s).
  for (const LayerSpec* sp : {&big, &cur}) {
    auto o = run(layers_for(*sp), PeriodicBurst{1}, 60000);
    for (int s = 0; s < 2; ++s)
      expect(residual_pct(o.layer[(size_t)s]) == 0.0, "B=1 zero loss");
  }
  // Gate 2: guarantee-region budget edge, single-layer stream 1 (the bulk
  // layer — old sid 3 pre-Task-3, ov 0.50 since the 2026-08-29 UEP flatten,
  // was 0.25), sym1312/bpb1/w64.
  // L <= W*ov/(1+ov) = 64*0.50/1.50 = 21.33 lost SOURCES is a one-sided
  // sufficiency bound, same as before. The body mix changed with ov: the
  // emitted-body stream interleaves one repair per 1/ov sources, i.e. the
  // source fraction of a burst is 1/(1+ov) — at ov=0.25 that was the 0.8
  // ("SSSSR") this gate used to spell out; at ov=0.50 it is 1/1.5 = 0.667
  // ("SSRSR"-ish), so a burst of B bodies now kills ~0.667B sources (and
  // ~0.333B repairs). Converting sources back to bodies divides by that
  // same fraction, so the (1+ov) terms cancel exactly and the guaranteed
  // body budget is just W*ov = 64*0.50 = 32 — no longer a coincidence that
  // the old 12.8/0.8 = 16 matched W*ov = 64*0.25 = 16 too. 32 bodies ≈ 31
  // at that mix, minus 1 body alignment margin -> every B <= 30 must be
  // lossless. Recovery beyond the bound (GE suffix-chaining) is allowed,
  // so the first lossy B may land past it — assert it exists in [31, 51]
  // (measured edge: B=48 under whole-frame traffic with the flattened
  // ladder, lossless confirmed through B=47; was B=21 under the pre-flatten
  // ov=0.25 ladder — see the 2026-08-29 UEP-flatten commit for that history;
  // geometry unchanged by the Task 3 sid 3->1 renumbering, sym1312/bpb1/w64
  // is exactly the same layer as before).
  int first_lossy = -1;
  for (int B = 2; B <= 51; B += 1) {
    auto o = run(layers_for(big), PeriodicBurst{B}, 60000, /*single_layer=*/1);
    double r = residual_pct(o.layer[1]);
    if (B <= 30) expect(r == 0.0, "B<=30 lossless");
    if (r > 0.0) { first_lossy = B; break; }
  }
  expect(first_lossy >= 31 && first_lossy <= 51, "budget edge in [31,51]");
  // Gate 3: monotonic-ish degradation for the current config, B in {2,8,16}.
  double prev = -1.0;
  for (int B : {2, 8, 16}) {
    auto o = run(layers_for(cur), PeriodicBurst{B}, 60000, 1);
    double r = residual_pct(o.layer[1]);
    expect(r >= prev - 0.05, "monotonic degradation");
    prev = r;
  }
  std::printf(failures ? "self-check: FAIL (%d)\n" : "self-check: OK\n",
              failures);
  return failures ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--self-check") return self_check();

  const std::vector<LayerSpec> specs = {
      {{164, 164}, {8, 8}, 64, "deployed-164"},
      {{164, 164}, {8, 8}, 128, "deployed-164-w128"},
      {{656, 656}, {4, 4}, 64, "global-656"},
      {{1312, 1312}, {2, 2}, 64, "global-1312-bpb2"},
      {{164, 1312}, {4, 1}, 64, "mixed-164/1312"},
      {{164, 1312}, {4, 2}, 64, "mixed-164/1312-bpb2"},
      {{164, 656}, {4, 2}, 64, "mixed-164/656"},
  };
  const std::array<int, 8> bursts = {1, 2, 4, 8, 12, 16, 24, 32};

  // ---- periodic-burst sweep: collect all results first, then print ----
  struct PeriodicRow {
    std::string name;
    std::array<double, 8> bulk_residual;
    double s0_worst;
  };
  std::vector<PeriodicRow> periodic_rows;
  for (const auto& sp : specs) {
    PeriodicRow row;
    row.name = sp.name;
    row.s0_worst = 0.0;
    for (size_t i = 0; i < bursts.size(); ++i) {
      auto o = run(layers_for(sp), PeriodicBurst{bursts[i]}, 60000);
      // report bulk residual: the enhance layer (stream 1)
      uint64_t sent = o.layer[1].sent, del = o.layer[1].delivered;
      double r = sent ? 100.0 * (double)(sent - del) / (double)sent : 0.0;
      row.bulk_residual[i] = r;
      if (residual_pct(o.layer[0]) > row.s0_worst)
        row.s0_worst = residual_pct(o.layer[0]);
    }
    periodic_rows.push_back(row);
  }

  std::printf("== periodic bursts (kill B consecutive bodies every 250ms, "
              "60s @ 9.1Mbps) ==\n");
  std::printf("%-22s |", "config");
  for (int B : bursts) std::printf("  B=%-3d", B);
  std::printf(" | s0 worst\n");
  for (const auto& row : periodic_rows) {
    std::printf("%-22s |", row.name.c_str());
    for (double r : row.bulk_residual) std::printf(" %6.2f", r);
    std::printf(" %6.2f\n", row.s0_worst);
  }

  // ---- gilbert-elliott sweep: collect, then print ----
  struct GeRow {
    std::string name;
    double bulk, s0;
    uint64_t maxlat;
  };
  std::vector<GeRow> ge_rows;
  for (const auto& sp : specs) {
    auto o = run(layers_for(sp), GilbertElliott{}, 120000);
    uint64_t sent = o.layer[1].sent, del = o.layer[1].delivered;
    GeRow row;
    row.name = sp.name;
    row.bulk = sent ? 100.0 * (double)(sent - del) / (double)sent : 0.0;
    row.s0 = residual_pct(o.layer[0]);
    row.maxlat = o.layer[1].max_latency_ms;
    ge_rows.push_back(row);
  }

  std::printf("== gilbert-elliott (~3%% avg, mean burst 5 bodies, 120s) ==\n");
  for (const auto& row : ge_rows) {
    std::printf("%-22s | bulk %6.3f%% | s0 %6.3f%% | maxlat s1 %llu ms\n",
                row.name.c_str(), row.bulk, row.s0,
                (unsigned long long)row.maxlat);
  }

  // ---- machine-readable CSV block ----
  // CSV,<config>,<model>,<B>,<bulk residual %>. GE has no burst-length knob,
  // so its rows carry B=0 as a placeholder.
  std::printf("== csv ==\n");
  for (const auto& row : periodic_rows)
    for (size_t i = 0; i < bursts.size(); ++i)
      std::printf("CSV,%s,periodic,%d,%.4f\n", row.name.c_str(), bursts[i],
                  row.bulk_residual[i]);
  for (const auto& row : ge_rows)
    std::printf("CSV,%s,ge,0,%.4f\n", row.name.c_str(), row.bulk);

  return 0;
}
