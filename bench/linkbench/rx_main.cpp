// linkbench-rx — GS-side link bench receiver. Taps monitor mode on the
// chosen channel via maburgs::RadioFrontend (which already handles 8822E
// duplex bring-up, RSSI/SNR extraction and the USB advisory lock), decodes
// the bench FEC stream, and prints one iperf-style line per second plus a
// summary. Spec: docs/superpowers/specs/2026-07-13-linkbench-design.md.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "bench_wire.h"
#include "mabur/gf256.h"
#include "rx_pipeline.h"
#include "tx_pipeline.h"  // FecParams

#include "body_queue.h"
#include "radio_frontend.h"
#include "nhm_busy.h"

#include "SignalStop.h"
#include "mabur/cal_wire.h"
#include <map>
#include <tuple>
#include <utility>

namespace {

using namespace linkbench;

struct Args {
  int channel = 149;
  int bw = 20;  // 20, or 40 (channel = primary; secondary per mabur::ht40_offset)
  int card = 0;
  uint16_t usb_vid = 0x0bda, usb_pid = 0;
  int index = 0;
  FecParams fec;
  std::string json_path;
  int time_s = 0;  // 0 = until SIGINT
  // --wall: tally linkbench-tx --wall-sweep cal frames per (mcs, rel idx)
  // cell and print one "cell" line each at exit (stdout).
  bool wall = false;
  // --energy-ms N: every N ms print "E t_ms fa cca igi floor own foreign"
  // (GetRxEnergy(with_nhm) + decoded-frame deltas) to stdout.
  int energy_ms = 0;
  // --nhm-busy-ms N: every N ms print an "N" line for a busy-airtime NHM
  // window armed at the previous one (spec 2026-09-25-nhm-airtime spike).
  // --nhm-reset: also run a FA/CCA counter reset mid-window (spike q. b).
  int nhm_busy_ms = 0;
  bool nhm_reset = false;
  // --retune-bench a,b,..: after bring-up cycle FastRetune through the list
  // --retune-n times, printing "R from to us" per call, then exit.
  std::vector<int> retune_list;
  int retune_n = 10;
  // --rate-hist: tally (mac_seq parity, received HT MCS) for CRC-clean
  // canonical frames; printed as "H parity mcs count" at exit (stdout).
  bool rate_hist = false;
  // --range FILE: tally linkbench-tx --range-sweep frames per (cycle, bw,
  // mcs, rel) and append each finished cycle to FILE as
  // "C cycle bw mcs rel rx rssiA rssiB snrA snrB" (rssi/snr are sums over
  // rx) plus "F cycle crc_bad" — append-only, so it can be read live.
  std::string range_path;
};

void usage(const char* argv0) {
  std::fprintf(stderr,
    "usage: %s --channel N [--bw 20|40] [--card 0] [--usb-vid 0x0bda] [--usb-pid 0]\n"
    "  [--index 0] [--overhead 0.5] [--symbol-size 64] [--window 128] "
    "[--bpb 16]\n"
    "  [--json FILE] [--time S] [--wall] [--energy-ms N] [--nhm-busy-ms N] [--nhm-reset] [--rate-hist] [--range FILE]\n"
    "  [--retune-bench ch,ch,.. [--retune-n 10]]\n", argv0);
}

bool parse_args(int argc, char** argv, Args* a) {
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    auto next = [&](int* out) {
      if (i + 1 >= argc) return false;
      *out = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
      return true;
    };
    if (k == "--channel") { if (!next(&a->channel)) return false; }
    else if (k == "--bw") { if (!next(&a->bw) || (a->bw != 20 && a->bw != 40)) return false; }
    else if (k == "--card") { if (!next(&a->card)) return false; }
    else if (k == "--index") { if (!next(&a->index)) return false; }
    else if (k == "--overhead") {
      if (i + 1 >= argc) return false;
      a->fec.overhead = std::strtod(argv[++i], nullptr);
      if (a->fec.overhead <= 0) return false;
    }
    else if (k == "--symbol-size") { if (!next(&a->fec.symbol_size)) return false; }
    else if (k == "--bpb") { if (!next(&a->fec.bpb)) return false; }
    else if (k == "--window") { if (!next(&a->fec.window)) return false; }
    else if (k == "--json") {
      if (i + 1 >= argc) return false;
      a->json_path = argv[++i];
    }
    else if (k == "--time") { if (!next(&a->time_s)) return false; }
    else if (k == "--wall") { a->wall = true; }
    else if (k == "--rate-hist") { a->rate_hist = true; }
    else if (k == "--range") {
      if (i + 1 >= argc) return false;
      a->range_path = argv[++i];
    }
    else if (k == "--energy-ms") { if (!next(&a->energy_ms)) return false; }
    else if (k == "--nhm-busy-ms") { if (!next(&a->nhm_busy_ms)) return false; }
    else if (k == "--nhm-reset") { a->nhm_reset = true; }
    else if (k == "--retune-n") { if (!next(&a->retune_n)) return false; }
    else if (k == "--retune-bench") {
      if (i + 1 >= argc) return false;
      std::string v = argv[++i];
      size_t pos = 0;
      while (pos < v.size()) {
        size_t c = v.find(',', pos);
        if (c == std::string::npos) c = v.size();
        a->retune_list.push_back(std::atoi(v.substr(pos, c - pos).c_str()));
        pos = c + 1;
      }
      if (a->retune_list.empty()) return false;
    }
    else if (k == "--usb-vid") { int v; if (!next(&v)) return false; a->usb_vid = static_cast<uint16_t>(v); }
    else if (k == "--usb-pid") { int v; if (!next(&v)) return false; a->usb_pid = static_cast<uint16_t>(v); }
    else { return false; }
  }
  return true;
}

uint64_t mono_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse_args(argc, argv, &a)) {
    usage(argv[0]);
    return 2;
  }
  install_devourer_signal_handlers();

  std::fprintf(stderr,
               "linkbench-rx: ch %d bw %d card %d | fec window=%d overhead=%.2f "
               "symbol=%d bpb=%d | gf256=%s\n"
               "  air bytes = dot11+body (radiotap/PLCP/FCS excluded); rssi "
               "dBm ~= pwdb-110, chains A/B\n",
               a.channel, a.bw, a.card, a.fec.window, a.fec.overhead,
               a.fec.symbol_size, a.fec.bpb, mabur::gf::backend());

  FILE* jf = nullptr;
  if (!a.json_path.empty()) {
    jf = std::fopen(a.json_path.c_str(), "w");
    if (!jf) {
      std::fprintf(stderr, "error: cannot open %s\n", a.json_path.c_str());
      return 1;
    }
  }

  maburgs::BodyQueue queue;
  maburgs::RadioFrontend::Cfg fcfg;
  fcfg.usb_vid = a.usb_vid;
  fcfg.usb_pid = a.usb_pid;
  fcfg.index = a.index;
  fcfg.channel = static_cast<uint8_t>(a.channel);
  fcfg.width_mhz = static_cast<uint8_t>(a.bw);
  fcfg.card_id = static_cast<uint8_t>(a.card);
  maburgs::RadioFrontend fe(fcfg, queue);
  if (!fe.open_and_start()) {
    std::fprintf(stderr, "error: radio bring-up failed (no card / in use?)\n");
    if (jf) std::fclose(jf);
    return 1;
  }

  if (!a.retune_list.empty()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int from = a.channel;
    for (int n = 0; n < a.retune_n && !g_devourer_should_stop; ++n) {
      for (int ch : a.retune_list) {
        const uint64_t t = mono_us();
        fe.retune(static_cast<uint8_t>(ch));
        std::printf("R %d %d %llu\n", from, ch,
                    static_cast<unsigned long long>(mono_us() - t));
        from = ch;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    std::fflush(stdout);
    fe.stop();
    if (jf) std::fclose(jf);
    return 0;
  }

  RxPipeline pipe(a.fec);
  RxSnapshot prev;
  uint64_t last_mono_ms = 0;  // latest RxBody clock seen (decoder clock)
  // Idle timer starts at launch, not 0 — otherwise the "no bench frames for
  // 5s" warning would fire on the very first stats tick.
  uint64_t last_frame_us = mono_us();
  bool idle_warned = false;
  bool badcfg_hinted = false;
  const uint64_t t0 = mono_us();
  uint64_t next_stats = t0 + 1'000'000;
  const uint64_t deadline =
      a.time_s > 0 ? t0 + static_cast<uint64_t>(a.time_s) * 1'000'000 : 0;

  uint64_t next_energy = mono_us() + static_cast<uint64_t>(a.energy_ms) * 1000;
  maburgs::ScoutFrames prev_fr = fe.frames();
  uint64_t next_nhm = 0, mid_nhm = 0, last_nhm = mono_us();
  maburgs::ScoutFrames nhm_prev_fr = fe.frames();
  const uint16_t nhm_period =
      maburgs::nhm_period_4us(a.nhm_busy_ms > 10 ? a.nhm_busy_ms - 10 : a.nhm_busy_ms);
  if (a.nhm_busy_ms > 0) {
    fe.arm_nhm_busy(nhm_period);
    next_nhm = mono_us() + static_cast<uint64_t>(a.nhm_busy_ms) * 1000;
    mid_nhm = mono_us() + static_cast<uint64_t>(a.nhm_busy_ms) * 500;
  }
  struct WallCell { uint32_t rx = 0; double rssi0 = 0, rssi1 = 0; };
  std::map<std::pair<int, int>, WallCell> wall_cells;
  uint64_t wall_corrupt = 0;
  std::map<std::pair<int, int>, uint64_t> rate_hist;
  struct RangeCell { uint32_t rx = 0; double r0 = 0, r1 = 0, s0 = 0, s1 = 0; };
  std::map<std::tuple<int, int, int>, RangeCell> range_cur;
  int range_cycle = -1;
  uint64_t range_crc_bad = 0;
  FILE* rf = nullptr;
  if (!a.range_path.empty()) {
    rf = std::fopen(a.range_path.c_str(), "a");
    if (!rf) {
      std::fprintf(stderr, "error: cannot open %s\n", a.range_path.c_str());
      return 1;
    }
  }
  auto range_flush = [&] {
    if (!rf || range_cycle < 0) return;
    uint64_t n = 0;
    for (const auto& [k, c] : range_cur) {
      std::fprintf(rf, "C %d %d %d %d %u %.0f %.0f %.0f %.0f\n", range_cycle,
                   std::get<0>(k), std::get<1>(k), std::get<2>(k), c.rx,
                   c.r0, c.r1, c.s0, c.s1);
      n += c.rx;
    }
    std::fprintf(rf, "F %d %llu\n", range_cycle,
                 static_cast<unsigned long long>(range_crc_bad));
    std::fflush(rf);
    std::fprintf(stderr, "range: cycle %d flushed (%zu cells, %llu frames, %llu crc-bad)\n",
                 range_cycle, range_cur.size(), static_cast<unsigned long long>(n),
                 static_cast<unsigned long long>(range_crc_bad));
    range_cur.clear();
    range_crc_bad = 0;
  };
  std::vector<mabur::node::RxBody> batch;
  while (!g_devourer_should_stop) {
    const uint64_t now = mono_us();
    if (deadline && now >= deadline) break;
    if (!fe.alive()) {
      std::fprintf(stderr, "error: RX loop died — device unplugged?\n");
      break;
    }
    batch.clear();
    queue.drain(batch, /*timeout_ms=*/100);
    for (auto& m : batch) {
      if (a.rate_hist && m.crc_ok) ++rate_hist[{m.mac_seq & 1, m.mcs}];
      if (rf) {
        if (!m.crc_ok) { ++range_crc_bad; continue; }
        RangeFrameInfo ri;
        if (!parse_range_payload(m.body.data(), m.body.size(), &ri)) continue;
        if (static_cast<int>(ri.cycle) != range_cycle) {
          range_flush();
          range_cycle = ri.cycle;
        }
        auto& c = range_cur[{ri.bw, ri.mcs, ri.rel}];
        ++c.rx;
        c.r0 += m.rssi[0] - 110.0;
        c.r1 += m.rssi[1] - 110.0;
        c.s0 += m.snr[0];
        c.s1 += m.snr[1];
        last_frame_us = now;
        continue;
      }
      if (a.wall) {
        mabur::cal::CalFrameInfo ci;
        if (!m.crc_ok) { ++wall_corrupt; continue; }
        if (!mabur::cal::parse_cal_payload(m.body.data(), m.body.size(), &ci)) continue;
        auto& c = wall_cells[{ci.rate, ci.idx}];
        ++c.rx;
        c.rssi0 += m.rssi[0];
        c.rssi1 += m.rssi[1];
        last_frame_us = now;
        continue;
      }
      last_mono_ms = m.mono_us / 1000;
      const uint64_t before = pipe.snapshot().frames;
      pipe.on_body(m.body.data(), m.body.size(), m.mac_seq, m.crc_ok, m.rssi,
                   m.snr, last_mono_ms);
      if (pipe.snapshot().frames != before) {
        last_frame_us = now;
        idle_warned = false;
      }
    }
    if (last_mono_ms) pipe.expire(last_mono_ms);

    if (a.nhm_busy_ms > 0 && a.nhm_reset && mid_nhm && now >= mid_nhm) {
      (void)fe.read_energy_scout();   // spike (b): does a counter reset kill the window?
      mid_nhm = 0;
    }
    if (a.nhm_busy_ms > 0 && now >= next_nhm) {
      const maburgs::NhmBusyRead nb = fe.read_nhm_busy();
      const maburgs::ScoutFrames fr = fe.frames();
      const double own_pct = 100.0 * static_cast<double>(fr.own_air_us - nhm_prev_fr.own_air_us) /
                             static_cast<double>(now - last_nhm);
      last_nhm = now;
      std::printf("N %llu %d %u %u", static_cast<unsigned long long>((now - t0) / 1000),
                  nb.valid ? 1 : 0, nb.duration, nb.period);
      unsigned sum = 0;
      for (int i = 0; i < 12; ++i) { std::printf(" %u", nb.buckets[i]); sum += nb.buckets[i]; }
      const auto b83 = maburgs::nhm_busy_pct(nb, -83), b80 = maburgs::nhm_busy_pct(nb, -80),
                 b75 = maburgs::nhm_busy_pct(nb, -75);
      std::printf(" sum=%u busy83=%.1f busy80=%.1f busy75=%.1f own=%.1f own_n=%llu\n", sum,
                  b83 ? *b83 : -1.0, b80 ? *b80 : -1.0, b75 ? *b75 : -1.0, own_pct,
                  static_cast<unsigned long long>(fr.own - nhm_prev_fr.own));
      std::fflush(stdout);
      nhm_prev_fr = fr;
      fe.arm_nhm_busy(nhm_period);
      next_nhm += static_cast<uint64_t>(a.nhm_busy_ms) * 1000;
      mid_nhm = now + static_cast<uint64_t>(a.nhm_busy_ms) * 500;
    }
    if (a.energy_ms > 0 && now >= next_energy) {
      next_energy += static_cast<uint64_t>(a.energy_ms) * 1000;
      const maburgs::ScoutEnergy e = fe.read_energy(/*with_nhm=*/true);
      const maburgs::ScoutFrames fr = fe.frames();
      std::printf("E %llu %u %u %d %d %llu %llu\n",
                  static_cast<unsigned long long>((now - t0) / 1000),
                  e.fa_valid ? e.fa_ofdm : 0u, e.fa_valid ? e.cca_ofdm : 0u,
                  e.igi_valid ? e.igi : -1, e.floor_valid ? e.floor_dbm : 0,
                  static_cast<unsigned long long>(fr.own - prev_fr.own),
                  static_cast<unsigned long long>(fr.foreign - prev_fr.foreign));
      std::fflush(stdout);
      prev_fr = fr;
    }

    if (now >= next_stats) {
      next_stats += 1'000'000;
      const uint64_t t = (now - t0) / 1'000'000;
      const RxSnapshot cur = pipe.snapshot();
      const RxSnapshot d = snapshot_delta(cur, prev);
      prev = cur;
      if (d.frames > 0 || cur.frames > 0)
        std::fprintf(stderr, "%s\n", format_line(t, d).c_str());
      if (jf) {
        std::fprintf(jf, "%s\n", format_json(t, d).c_str());
        std::fflush(jf);
      }
      if (!badcfg_hinted && cur.sym_badcfg > 100 && cur.pkts == 0) {
        badcfg_hinted = true;
        std::fprintf(stderr,
                     "hint: %llu symbols dropped bad-cfg and nothing decodes "
                     "— --symbol-size mismatch with TX?\n",
                     static_cast<unsigned long long>(cur.sym_badcfg));
      }
      if (!idle_warned && now - last_frame_us > 5'000'000) {
        idle_warned = true;
        std::fprintf(stderr,
                     "no bench frames for 5s — check channel / TX running / "
                     "stream id\n");
      }
    }
  }

  const RxSnapshot s = pipe.snapshot();
  const double dur = (mono_us() - t0) / 1e6;
  const uint64_t ok_frames = s.frames - s.crc_bad;
  const double n = s.sig_frames ? static_cast<double>(s.sig_frames) : 1.0;
  std::fprintf(stderr,
      "--- summary (%.1fs) ---\n"
      "frames %llu (crc_bad %llu, mac_lost %llu = %.2f%%)\n"
      "air %.2f Mbps | goodput %.2f Mbps\n"
      "syms recovered %llu abandoned %llu | sub-blocks %llu crc-fail %llu | badcfg %llu\n"
      "pkts %llu / expected %llu (loss %.2f%%) pattern_bad %llu\n"
      "rssi %.1f/%.1f dBm  snr %.1f/%.1f dB (means over %llu frames)\n"
      "config: window=%d overhead=%.2f symbol=%d bpb=%d channel=%d\n",
      dur,
      static_cast<unsigned long long>(s.frames),
      static_cast<unsigned long long>(s.crc_bad),
      static_cast<unsigned long long>(s.mac_lost),
      (ok_frames + s.mac_lost)
          ? 100.0 * s.mac_lost / static_cast<double>(ok_frames + s.mac_lost)
          : 0.0,
      s.air_bytes * 8.0 / 1e6 / dur, s.good_bytes * 8.0 / 1e6 / dur,
      static_cast<unsigned long long>(s.syms_recovered),
      static_cast<unsigned long long>(s.syms_abandoned),
      static_cast<unsigned long long>(s.sub_blocks),
      static_cast<unsigned long long>(s.sub_crc_fail),
      static_cast<unsigned long long>(s.sym_badcfg),
      static_cast<unsigned long long>(s.pkts),
      static_cast<unsigned long long>(s.pkts_expected),
      s.pkts_expected
          ? 100.0 * (s.pkts_expected - s.pkts) / static_cast<double>(s.pkts_expected)
          : 0.0,
      static_cast<unsigned long long>(s.pattern_bad),
      s.rssi_sum[0] / n - 110.0, s.rssi_sum[1] / n - 110.0,
      s.snr_sum[0] / n, s.snr_sum[1] / n,
      static_cast<unsigned long long>(s.sig_frames),
      a.fec.window, a.fec.overhead, a.fec.symbol_size, a.fec.bpb, a.channel);

  if (rf) std::fclose(rf);  // the running (unfinished) cycle is dropped
  for (const auto& [k, n] : rate_hist)
    std::printf("H %d %d %llu\n", k.first, k.second, static_cast<unsigned long long>(n));
  if (a.wall) {
    for (const auto& [k, c] : wall_cells)
      std::printf("cell mcs %d rel %d rx %u rssi %.1f %.1f\n", k.first, k.second,
                  c.rx, c.rssi0 / c.rx - 110.0, c.rssi1 / c.rx - 110.0);
    std::printf("corrupt %llu\n", static_cast<unsigned long long>(wall_corrupt));
    std::fflush(stdout);
  }
  fe.stop();
  if (jf) std::fclose(jf);
  return 0;
}
