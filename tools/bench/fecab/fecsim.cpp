// Monte Carlo of mabur FEC repair placement: encoder policy per sw_encoder.cpp,
// decoder as streaming GF(256) RREF over the unknown symbols (coefficients only).
// Body = bpb envelopes; losses hit bodies. Base/enhance alternate, one SwEncoder each.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
using namespace std;

static uint8_t GMUL[256][256], GINV[256];
static void gf_init() {
  uint8_t e[512]; int l[256]; int x = 1;
  for (int i = 0; i < 255; i++) { e[i] = x; l[x] = i; x <<= 1; if (x & 256) x ^= 0x11D; }
  for (int i = 255; i < 512; i++) e[i] = e[i - 255];
  for (int a = 1; a < 256; a++) { GINV[a] = e[255 - l[a]]; for (int b = 1; b < 256; b++) GMUL[a][b] = e[l[a] + l[b]]; }
}

enum Policy { CONT, TAIL, DYN };
struct Arm {
  string name; Policy p; int W;  // CONT/TAIL: window. DYN: floor on the window (0 = none)
  int m;                          // DYN: overlap beyond the frame
  double delay_frac;              // DYN: share of the batch shipped one frame period later
  int cap = 0;                    // DYN: above this window fall back to the tail rule with W = cap
};
struct Scen { string name; int kind; double eff; int burst; };  // kind 0 fixed burst, 1 flight mix, 2 time fades
struct Env { int layer; bool repair; uint32_t seq; int len; uint64_t key; };
struct Body { double ready; int order; vector<Env> envs; double t; bool lost; };
struct Frame { int layer; uint32_t first; int n; double t0; double src_end; int missing; double done; };

struct Stat { double frames=0,trunc=0,late10=0,late5=0,rows=0,rows_max=0,inline_max=0,recs=0,repairs=0,dec_ops=0,dec_max=0,bodies=0,lost=0;
  double f_base=0,t_base=0,f_enh=0,t_enh=0; };

static const int BPB = 4;
static const double TB = 130e-6;         // s per body on air (3.3 ms enhance span, MCS4/40)
static const double ROW_S = 0.95e-6;     // s per window row: 61 us per repair at 64 rows, drone, frame-end batch
static const double FP = 0.010;          // 100 fps, base and enhance alternate
static const double OV[2] = {0.5, 0.25};
static const double TGAP = 0.050;

static void run(const Arm& a, const Scen& s, double nbar, uint64_t seed, int nfr, Stat& st) {
  mt19937_64 r(seed);
  lognormal_distribution<double> ln(log(nbar) - 0.5 * 0.35 * 0.35, 0.35);
  uniform_real_distribution<double> U(0, 1);
  vector<Body> bodies; vector<Frame> frames; frames.reserve(nfr);
  uint32_t seq[2] = {0, 0}; int cnt[2] = {0, 0}; double credit[2] = {0, 0}; bool tailp[2] = {false, false};
  uint64_t key = 1; double worker_free = 0, hot_free = 0; int order = 0;
  for (int f = 0; f < nfr; f++) {
    int L = f & 1; double t0 = f * FP; double ov = OV[L];
    int n = max(1, (int)lround(ln(r)));
    if (L == 0 && f % 200 == 0) n = min(310, n * 3);  // an occasional big (IDR-like) frame
    Frame fr{L, seq[L], n, t0, 0, n, -1};
    vector<Env> src, rep; vector<double> rep_ready; double frame_rows = 0, inl = 0;
    if (worker_free < t0) worker_free = t0;
    if (hot_free < t0) hot_free = t0;
    auto mk = [&](int len, double& clock, bool inline_b) {
      Env e{L, true, seq[L] - (uint32_t)len, len, key++};
      rep.push_back(e); clock += len * ROW_S; rep_ready.push_back(clock); frame_rows += len; if (inline_b) inl += len * ROW_S;
    };
    bool fallback = a.p == DYN && a.cap > 0 && n + a.m > a.cap;
    Policy P = fallback ? TAIL : a.p; int W = fallback ? a.cap : a.W;
    if (P == CONT || P == TAIL) {
      size_t hold = 0;
      if (P == TAIL) {
        if (n <= W) hold = n;
        else hold = min<size_t>(min<size_t>((size_t)ceil(BPB / ov - 1e-9), W), n);
      }
      size_t front = n - hold;
      for (int i = 0; i < n; i++) {
        src.push_back({L, false, seq[L], 0, 0}); seq[L]++; cnt[L] = min(cnt[L] + 1, W); tailp[L] = true;
        credit[L] += ov;
        if (P == TAIL && (size_t)(i + 1) > front) continue;
        while (credit[L] >= 1.0) { mk(cnt[L], worker_free, false); tailp[L] = false; credit[L] -= 1.0; }
      }
      if (P == TAIL) {
        int k = credit[L] > 1e-9 ? (int)ceil(credit[L] - 1e-9) : 0;
        for (int i = 0; i < k; i++) mk(cnt[L], hot_free, true);
        credit[L] = 0;
      } else if (tailp[L] && cnt[L] > 0) { mk(cnt[L], worker_free, false); tailp[L] = false; }
    } else {
      for (int i = 0; i < n; i++) { src.push_back({L, false, seq[L], 0, 0}); seq[L]++; cnt[L] = min(cnt[L] + 1, 255); }
      int w = min(255, max(a.W, n + a.m));
      int len = min(cnt[L], w);
      int k = (int)ceil(n * ov - 1e-9);
      for (int i = 0; i < k; i++) mk(len, hot_free, true);
    }
    int nd = (a.p == DYN && a.delay_frac > 0) ? (int)floor(rep.size() * a.delay_frac) : 0;
    for (size_t i = 0; i < src.size(); i += BPB) {
      Body b; b.ready = t0; b.order = order++; b.lost = false; b.t = 0;
      for (size_t j = i; j < min(src.size(), i + BPB); j++) b.envs.push_back(src[j]);
      bodies.push_back(move(b));
    }
    size_t split = rep.size() - nd;
    for (int part = 0; part < 2; part++) {
      size_t lo = part ? split : 0, hi = part ? rep.size() : split;
      for (size_t i = lo; i < hi; i += BPB) {
        Body b; size_t e = min(hi, i + BPB); b.ready = rep_ready[e - 1] + (part ? 2 * FP : 0); b.order = order++; b.lost = false; b.t = 0;
        for (size_t j = i; j < e; j++) b.envs.push_back(rep[j]);
        bodies.push_back(move(b));
      }
    }
    st.rows += frame_rows; st.rows_max = max(st.rows_max, frame_rows); st.inline_max = max(st.inline_max, inl);
    st.repairs += rep.size();
    frames.push_back(fr);
  }
  stable_sort(bodies.begin(), bodies.end(), [](const Body& a, const Body& b) { return a.ready < b.ready; });
  double air = 0;
  for (auto& b : bodies) { air = max(air, b.ready); b.t = air + TB; air = b.t; }

  // Losses.
  if (s.kind == 0 || s.kind == 1) {
    double meanlen = s.kind == 0 ? s.burst : 0.7 * 1 + 0.3 * 8.5;
    double pstart = s.eff / (meanlen * (1 - s.eff) + s.eff);
    uniform_int_distribution<int> longb(5, 12);
    for (size_t i = 0; i < bodies.size();) {
      if (U(r) < pstart) {
        int l = s.kind == 0 ? s.burst : (U(r) < 0.7 ? 1 : longb(r));
        for (int j = 0; j < l && i < bodies.size(); j++, i++) bodies[i].lost = true;
      } else i++;
    }
  } else {
    // Time-domain fades: 2..15 ms outages covering eff of the wall clock.
    double t = 0, tend = nfr * FP + 1; vector<pair<double,double>> out;
    double mean_d = 8.5e-3, mean_gap = mean_d * (1 - s.eff) / s.eff;
    exponential_distribution<double> gap(1 / mean_gap); uniform_real_distribution<double> dur(2e-3, 15e-3);
    while (t < tend) { t += gap(r); double d = dur(r); out.push_back({t, t + d}); t += d; }
    size_t k = 0;
    for (auto& b : bodies) { while (k < out.size() && out[k].second < b.t) k++; if (k < out.size() && out[k].first <= b.t) b.lost = true; }
  }

  // Frame lookup by symbol, per layer.
  vector<int> fof[2];
  for (size_t i = 0; i < frames.size(); i++) for (int j = 0; j < frames[i].n; j++) fof[frames[i].layer].push_back((int)i);
  // src_end = nominal end of the frame's last source body.
  for (auto& b : bodies) for (auto& e : b.envs) if (!e.repair) { auto& F = frames[fof[e.layer][e.seq]]; F.src_end = max(F.src_end, b.t); }

  // Streaming RREF per layer, over unknown symbols only.
  struct Dec { vector<uint8_t> known; map<uint32_t, map<uint32_t,uint8_t>> rows; uint32_t newest = 0; } D[2];
  const uint32_t HOR = 512;
  D[0].known.assign(fof[0].size(), 0); D[1].known.assign(fof[1].size(), 0);
  mt19937_64 cr(seed ^ 0x9e3779b97f4a7c15ULL);
  vector<uint32_t> q;
  auto mark = [&](int L, uint32_t v, double t) {
    Dec& d = D[L]; q.assign(1, v);
    while (!q.empty()) {
      uint32_t u = q.back(); q.pop_back();
      if (d.known[u]) continue;
      d.known[u] = 1;
      Frame& F = frames[fof[L][u]];
      if (--F.missing == 0) F.done = t;
      d.rows.erase(u);
      for (auto it = d.rows.begin(); it != d.rows.end(); ++it) {
        it->second.erase(u);
        if (it->second.empty()) q.push_back(it->first);
      }
    }
  };
  for (auto& b : bodies) {
    st.bodies++; if (b.lost) { st.lost++; continue; }
    for (auto& e : b.envs) {
      int L = e.layer; Dec& d = D[L];
      uint32_t top = e.repair ? e.seq + e.len : e.seq + 1;
      if (top > d.newest) {
        d.newest = top;
        if (d.newest > HOR) { uint32_t fl = d.newest - HOR; while (!d.rows.empty() && d.rows.begin()->first < fl) d.rows.erase(d.rows.begin()); }
      }
      if (!e.repair) { mark(L, e.seq, b.t); continue; }
      if (d.newest > HOR && e.seq < d.newest - HOR) continue;
      double ops = 0; map<uint32_t,uint8_t> row;
      for (int i = 0; i < e.len; i++) { uint32_t v = e.seq + i; if (!d.known[v]) { uint8_t c = 0; while (!c) c = (uint8_t)(cr() & 0xFF); row[v] = c; } }
      if (row.empty()) continue;  // repairs_skipped
      ops += e.len;
      // Reduce against pivots.
      for (auto it = row.begin(); it != row.end();) {
        auto p = d.rows.find(it->first);
        if (p == d.rows.end()) { ++it; continue; }
        uint8_t f = it->second; uint32_t pv = it->first;
        it = row.erase(it);
        for (auto& kv : p->second) { uint8_t c = GMUL[f][kv.second]; auto q2 = row.find(kv.first); if (q2 == row.end()) row[kv.first] = c; else { q2->second ^= c; if (!q2->second) { if (q2 == it) it = row.erase(q2); else row.erase(q2); } } }
        ops += 1; (void)pv; it = row.begin();
      }
      if (row.empty()) { st.dec_ops += ops; continue; }
      uint32_t piv = row.begin()->first; uint8_t inv = GINV[row.begin()->second];
      row.erase(row.begin());
      for (auto& kv : row) kv.second = GMUL[inv][kv.second];
      // Back-substitute the new pivot out of existing rows.
      for (auto& R : d.rows) {
        auto c = R.second.find(piv); if (c == R.second.end()) continue;
        uint8_t f = c->second; R.second.erase(c); ops += 1;
        for (auto& kv : row) { uint8_t m = GMUL[f][kv.second]; auto q2 = R.second.find(kv.first); if (q2 == R.second.end()) R.second[kv.first] = m; else { q2->second ^= m; if (!q2->second) R.second.erase(q2); } }
      }
      d.rows[piv] = row;
      st.dec_ops += ops; st.dec_max = max(st.dec_max, ops);
      vector<uint32_t> solved; for (auto& R : d.rows) if (R.second.empty()) solved.push_back(R.first);
      for (uint32_t v : solved) { st.recs++; mark(L, v, b.t); }
    }
  }
  // Skip the last frames: their reach-forward repairs never ran.
  for (size_t i = 8; i + 8 < frames.size(); i++) {
    auto& F = frames[i]; st.frames++; (F.layer ? st.f_enh : st.f_base)++;
    bool tr = F.missing > 0 || F.done > F.src_end + TGAP;
    double late = F.missing > 0 ? 1e9 : F.done - F.src_end;
    if (tr) { st.trunc++; (F.layer ? st.t_enh : st.t_base)++; }
    if (late > 0.010) st.late10++;
    if (late > 0.005) st.late5++;
  }
}

int main(int argc, char** argv) {
  gf_init();
  int nfr = argc > 1 ? atoi(argv[1]) : 20000, seeds = argc > 2 ? atoi(argv[2]) : 6;
  vector<Arm> arms = {
    {"A cont w32", CONT, 32, 0, 0}, {"C cont w64", CONT, 64, 0, 0}, {"D tail w64", TAIL, 64, 0, 0},
    {"E n+0", DYN, 0, 0, 0}, {"E n+8", DYN, 0, 8, 0}, {"E n+16", DYN, 0, 16, 0}, {"E n+32", DYN, 0, 32, 0}, {"E n+64", DYN, 0, 64, 0},
    {"F max64 n+8", DYN, 64, 8, 0}, {"F max64 n+16", DYN, 64, 16, 0}, {"G n+16 delay25", DYN, 0, 16, 0.25},
    {"H n+16 cap128", DYN, 0, 16, 0, 128}, {"H n+32 cap128", DYN, 0, 32, 0, 128}, {"R max64 n+32 cap128", DYN, 64, 32, 0, 128},
  };
  vector<Scen> sc = {{"clean", 0, 0.0, 1}, {"2% b1", 0, 0.02, 1}, {"5% b3", 0, 0.05, 3}, {"5% flightmix", 1, 0.05, 0}, {"5% fades2-15ms", 2, 0.05, 0}};
  vector<double> nb = {30, 60, 88, 130};
  printf("nbar,scen,arm,trunc,trunc_ci,late10,late5,trunc_base,trunc_enh,rows_per_frame,rows_max,inline_max_ms,repairs_per_frame,dec_ops_per_rec,dec_max,loss\n");
  vector<thread> th;
  struct Job { size_t a, s, n; int seed; Stat st; };
  vector<Job> jobs;
  for (size_t n = 0; n < nb.size(); n++) for (size_t s = 0; s < sc.size(); s++) for (size_t a = 0; a < arms.size(); a++) for (int k = 0; k < seeds; k++) jobs.push_back({a, s, n, k, {}});
  std::atomic_size_t idx{0};
  for (int t = 0; t < 30; t++) th.emplace_back([&] { size_t i; while ((i = idx++) < jobs.size()) { auto& j = jobs[i]; run(arms[j.a], sc[j.s], nb[j.n], 1000 * j.seed + 17 * j.s + 13 * j.n + 1, nfr, j.st); } });
  for (auto& t : th) t.join();
  for (size_t n = 0; n < nb.size(); n++) for (size_t s = 0; s < sc.size(); s++) for (size_t a = 0; a < arms.size(); a++) {
    Stat tot; vector<double> tr;
    for (auto& j : jobs) if (j.a == a && j.s == s && j.n == n) {
      auto& x = j.st; tot.frames += x.frames; tot.trunc += x.trunc; tot.late10 += x.late10; tot.late5 += x.late5; tot.rows += x.rows; tot.rows_max = max(tot.rows_max, x.rows_max); tot.inline_max = max(tot.inline_max, x.inline_max); tot.repairs += x.repairs; tot.recs += x.recs; tot.dec_ops += x.dec_ops; tot.dec_max = max(tot.dec_max, x.dec_max); tot.bodies += x.bodies; tot.lost += x.lost; tot.f_base += x.f_base; tot.t_base += x.t_base; tot.f_enh += x.f_enh; tot.t_enh += x.t_enh;
      tr.push_back(100 * x.trunc / x.frames);
    }
    double m = 0, v = 0; for (double d : tr) m += d; m /= tr.size(); for (double d : tr) v += (d - m) * (d - m); double ci = 1.96 * sqrt(v / (tr.size() - 1) / tr.size());
    double F = tot.frames + 16 * seeds;
    printf("%.0f,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,%.2f,%.1f,%.1f,%.0f,%.2f\n", nb[n], sc[s].name.c_str(), arms[a].name.c_str(), 100 * tot.trunc / tot.frames, ci, 100 * tot.late10 / tot.frames, 100 * tot.late5 / tot.frames, 100 * tot.t_base / tot.f_base, 100 * tot.t_enh / tot.f_enh, tot.rows / F, tot.rows_max, tot.inline_max * 1000, tot.repairs / F, tot.recs > 0 ? tot.dec_ops / tot.recs : 0, tot.dec_max, 100 * tot.lost / max(1.0, tot.bodies));
  }
}
