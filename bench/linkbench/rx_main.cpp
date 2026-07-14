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
#include <vector>

#include "bench_wire.h"
#include "mabur/gf256.h"
#include "rx_pipeline.h"
#include "tx_pipeline.h"  // FecParams

#include "body_queue.h"
#include "radio_frontend.h"

#include "SignalStop.h"

namespace {

using namespace linkbench;

struct Args {
  int channel = 149;
  int card = 0;
  uint16_t usb_vid = 0x0bda, usb_pid = 0;
  int index = 0;
  FecParams fec;
  std::string json_path;
  int time_s = 0;  // 0 = until SIGINT
};

void usage(const char* argv0) {
  std::fprintf(stderr,
    "usage: %s --channel N [--card 0] [--usb-vid 0x0bda] [--usb-pid 0]\n"
    "  [--index 0] [--overhead 0.5] [--symbol-size 64] [--window 128] "
    "[--bpb 16]\n"
    "  [--json FILE] [--time S]\n", argv0);
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
               "linkbench-rx: ch %d card %d | fec window=%d overhead=%.2f "
               "symbol=%d bpb=%d | gf256=%s\n"
               "  air bytes = dot11+body (radiotap/PLCP/FCS excluded); rssi "
               "dBm ~= pwdb-110, chains A/B\n",
               a.channel, a.card, a.fec.window, a.fec.overhead,
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
  fcfg.card_id = static_cast<uint8_t>(a.card);
  maburgs::RadioFrontend fe(fcfg, queue);
  if (!fe.open_and_start()) {
    std::fprintf(stderr, "error: radio bring-up failed (no card / in use?)\n");
    if (jf) std::fclose(jf);
    return 1;
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

  fe.stop();
  if (jf) std::fclose(jf);
  return 0;
}
