// Offline A/B of the frame-end repair placement (fec.repairs_with_frame)
// on a recorded frame sequence. Not a test; --target frame_fec_sim.
//
//   frame_fec_sim FRAMES [drop_pct=4] [seed=1] [rate_mbps=13.5] [burst=1]
//
// FRAMES: one line per frame, "t_us sid n_symbols" (production time, UEP
// layer, source symbols). Both modes encode the same frames with the real
// SwEncoder and decode with the real SwDecoder; only the frame-end repairs
// differ:
//   old: the tail repair is built on the FEC worker and harvested at the
//        5 ms ring-read timeout, as its own body (the drone's collect path);
//   old+0: the same repairs as old, shipped at once (placement alone);
//   new: the frame-end repairs are built inline and ship as their own body
//        right behind the frame's last source body.
// Mid-frame credited repairs go inline with the sources in both modes.
// The air loses whole bodies: each body is dropped with drop_pct, in runs
// of `burst` bodies. Bodies are serialised FIFO on one air at rate_mbps
// plus a fixed per-PPDU cost. Reported per mode: frames never completed,
// and for frames that lost a source, the wait from the end of the frame's
// last source body to its completion.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"

using namespace mabur;

namespace {

constexpr int kSymbol = 332;
int kWindow = 32;
constexpr int kBpb = 4;
constexpr double kOverhead[2] = {0.5, 0.25};  // rung-0 bundle pair
constexpr double kHarvestUs = 5000;           // ring-read timeout
constexpr double kPpduUs = 150;               // preamble + IFS per body
size_t g_repair_bpb = kBpb;                   // new: repairs per frame-end body

struct Frame {
  double t;
  int sid;
  int n;
};

struct Body {
  double ready;
  int sid;
  std::vector<std::vector<uint8_t>> envs;
  size_t frame;  // owning frame index
};

struct Result {
  size_t frames = 0, lossy = 0, incomplete = 0, bodies = 0, repairs = 0;
  double air_pct = 0;
  std::vector<double> wait_ms;
};

Result run(const std::vector<Frame>& frames, bool with_frame, double harvest_us, double drop,
           unsigned seed, double rate_mbps, int burst) {
  std::vector<SwEncoder> enc;
  for (int s = 0; s < 2; ++s)
    enc.emplace_back(SwConfig{kSymbol, kWindow, kOverhead[s], with_frame});
  const std::vector<uint8_t> pkt(static_cast<size_t>(SwConfig{kSymbol}.max_packet_size()), 0x5a);

  std::vector<Body> bodies;
  std::vector<std::pair<uint64_t, uint64_t>> seqs(frames.size());
  Result r;
  for (size_t i = 0; i < frames.size(); ++i) {
    const Frame& f = frames[i];
    auto& e = enc[static_cast<size_t>(f.sid)];
    std::vector<std::vector<uint8_t>> group;
    auto into_group = [&](const uint8_t* p, size_t n) { group.emplace_back(p, p + n); };
    auto close_group = [&](double ready, size_t bpb = kBpb) {
      for (size_t k = 0; k < group.size(); k += bpb) {
        Body b{ready, f.sid, {}, i};
        for (size_t j = k; j < std::min(group.size(), k + bpb); ++j) b.envs.push_back(group[j]);
        bodies.push_back(std::move(b));
      }
      group.clear();
    };
    const uint64_t s0 = e.sources_out();
    const uint64_t r0 = e.repairs_out();
    for (int k = 0; k < f.n; ++k) e.add_packet(pkt.data(), pkt.size(), into_group);
    e.seal(into_group);
    seqs[i] = {s0, e.sources_out()};
    close_group(f.t);
    e.flush(into_group);
    close_group(f.t + harvest_us, with_frame ? g_repair_bpb : kBpb);
    r.repairs += e.repairs_out() - r0;
  }
  std::stable_sort(bodies.begin(), bodies.end(),
                   [](const Body& a, const Body& b) { return a.ready < b.ready; });
  r.bodies = bodies.size();

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<SwDecoder> dec;
  for (int s = 0; s < 2; ++s) dec.emplace_back(SwConfig{kSymbol, kWindow, kOverhead[s]});
  std::vector<std::map<uint64_t, double>> known(2);  // seq -> delivery time, us
  std::vector<double> src_end(frames.size(), 0.0);
  std::vector<bool> lost_src(frames.size(), false);
  double air = 0, busy = 0;
  int burst_left = 0;
  for (const Body& b : bodies) {
    size_t bytes = 13;
    for (auto& env : b.envs) bytes += 2 + env.size();
    const double cost = kPpduUs + static_cast<double>(bytes) * 8.0 / rate_mbps;
    air = std::max(air, b.ready) + cost;
    busy += cost;
    bool dropped = false;
    if (burst_left > 0) {
      --burst_left;
      dropped = true;
    } else if (u(rng) < drop) {
      burst_left = burst - 1;
      dropped = true;
    }
    bool has_src = false;
    for (auto& env : b.envs) {
      sw::SwHeader h;
      sw::parse_header(env.data(), env.size(), &h);
      if (!h.repair) has_src = true;
    }
    if (has_src) {
      src_end[b.frame] = std::max(src_end[b.frame], air);
      if (dropped) lost_src[b.frame] = true;
    }
    if (dropped) continue;
    auto& d = dec[static_cast<size_t>(b.sid)];
    for (auto& env : b.envs) {
      d.add_symbol(env.data(), env.size(), static_cast<uint64_t>(air / 1000));
      for (uint32_t s : d.last_out_seqs()) known[static_cast<size_t>(b.sid)].emplace(s, air);
    }
  }

  r.air_pct = 100.0 * busy / std::max(1.0, frames.back().t - frames.front().t);
  for (size_t i = 0; i < frames.size(); ++i) {
    ++r.frames;
    const auto& k = known[static_cast<size_t>(frames[i].sid)];
    double done = 0;
    bool complete = true;
    for (uint64_t s = seqs[i].first; s < seqs[i].second; ++s) {
      auto it = k.find(static_cast<uint32_t>(s));
      if (it == k.end()) { complete = false; break; }
      done = std::max(done, it->second);
    }
    if (!complete) { ++r.incomplete; continue; }
    if (!lost_src[i]) continue;
    ++r.lossy;
    r.wait_ms.push_back(std::max(0.0, done - src_end[i]) / 1000.0);
  }
  std::sort(r.wait_ms.begin(), r.wait_ms.end());
  return r;
}

double pct(const std::vector<double>& v, double q) {
  if (v.empty()) return NAN;
  return v[std::min(v.size() - 1, static_cast<size_t>(q * static_cast<double>(v.size())))];
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: frame_fec_sim FRAMES [drop_pct] [seed] [rate_mbps] [burst]\n");
    return 2;
  }
  std::vector<Frame> frames;
  std::ifstream in(argv[1]);
  for (std::string line; std::getline(in, line);) {
    std::istringstream ss(line);
    Frame f{};
    if (ss >> f.t >> f.sid >> f.n && f.sid >= 0 && f.sid < 2 && f.n > 0) frames.push_back(f);
  }
  const double drop = (argc > 2 ? std::atof(argv[2]) : 4.0) / 100.0;
  const unsigned seed = argc > 3 ? static_cast<unsigned>(std::atoi(argv[3])) : 1;
  const double rate = argc > 4 ? std::atof(argv[4]) : 13.5;
  const int burst = argc > 5 ? std::max(1, std::atoi(argv[5])) : 1;
  if (argc > 7) kWindow = std::atoi(argv[7]);
  if (argc > 6) g_repair_bpb = static_cast<size_t>(std::max(1, std::atoi(argv[6])));
  std::printf("frames=%zu drop=%.1f%% burst=%d rate=%.1f Mb/s seed=%u\n", frames.size(),
              drop * 100, burst, rate, seed);
  struct Mode {
    const char* name;
    bool with_frame;
    double harvest_us;
  };
  // old: as the drone runs it; old+0: the same repairs, shipped at once
  // (placement alone); new: repairs_with_frame (placement + credit spend).
  for (const Mode& m : {Mode{"old", false, kHarvestUs}, Mode{"old+0", false, 0.0},
                        Mode{"new", true, 0.0}}) {
    const Result r = run(frames, m.with_frame, m.harvest_us, drop, seed, rate, burst);
    size_t over4 = 0, over8 = 0;
    for (double w : r.wait_ms) {
      over4 += w > 4.0;
      over8 += w > 8.3;
    }
    std::printf(
        "%-5s air=%.0f%% bodies=%zu repairs=%zu incomplete=%zu (%.2f%%) lossy=%zu wait_ms "
        "p50=%.2f p90=%.2f p99=%.2f >4ms=%.1f%% >8.3ms=%.1f%%\n",
        m.name, r.air_pct, r.bodies, r.repairs, r.incomplete,
        100.0 * static_cast<double>(r.incomplete) / static_cast<double>(r.frames), r.lossy,
        pct(r.wait_ms, .5), pct(r.wait_ms, .9), pct(r.wait_ms, .99),
        100.0 * static_cast<double>(over4) / std::max<double>(1, static_cast<double>(r.lossy)),
        100.0 * static_cast<double>(over8) / std::max<double>(1, static_cast<double>(r.lossy)));
  }
  return 0;
}
