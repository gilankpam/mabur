// txagcbench-rx — GS-side sweep recorder. Taps monitor mode via
// maburgs::RadioFrontend (8822E duplex bring-up, per-chain RSSI/SNR, USB
// advisory lock) and writes one JSONL line per CRC-clean sweep frame; all
// judgment lives in analyze_sweep.py on the host. Corrupt frames are never
// attributed (their payload can't be trusted to name an index) — only
// counted, as a rig-health signal. rssi_* are raw pwdb (dBm ~= raw - 110);
// chain A is off-scale on the 8822E, the analyzer defaults to chain B.
// Spec: docs/superpowers/specs/2026-07-16-txagcbench-design.md.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sweep_wire.h"

#include "body_queue.h"
#include "radio_frontend.h"

#include "SignalStop.h"

namespace {

using namespace txagcbench;

struct Args {
  int channel = 149;
  int card = 0;
  int index = 0;
  int time_s = 0;  // 0 = until SIGINT
  uint16_t usb_vid = 0x0bda, usb_pid = 0;
  std::string out_path;
};

void usage(const char* argv0) {
  std::fprintf(stderr,
    "usage: %s [--channel 149] [--card 0] [--index 0] [--out FILE]\n"
    "  [--time S] [--usb-vid 0x0bda] [--usb-pid 0]\n", argv0);
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
    else if (k == "--time") { if (!next(&a->time_s)) return false; }
    else if (k == "--out") {
      if (i + 1 >= argc) return false;
      a->out_path = argv[++i];
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

  FILE* out = stdout;
  if (!a.out_path.empty()) {
    out = std::fopen(a.out_path.c_str(), "w");
    if (!out) {
      std::fprintf(stderr, "error: cannot open %s\n", a.out_path.c_str());
      return 1;
    }
  }

  std::fprintf(stderr, "txagcbench-rx: ch %d card %d -> %s\n", a.channel,
               a.card, a.out_path.empty() ? "stdout" : a.out_path.c_str());

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
    if (out != stdout) std::fclose(out);
    return 1;
  }

  uint64_t total = 0, crc_bad = 0, bench = 0;
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
      ++total;
      if (!m.crc_ok) { ++crc_bad; continue; }
      SweepInfo si;
      if (!parse_sweep_payload(m.body.data(), m.body.size(), &si)) continue;
      ++bench;
      std::fprintf(out,
                   "{\"idx\":%u,\"pass\":%u,\"seq\":%u,\"rssi_a\":%u,"
                   "\"rssi_b\":%u,\"snr_a\":%d,\"snr_b\":%d}\n",
                   si.idx, si.pass, si.seq, m.rssi[0], m.rssi[1],
                   m.snr[0], m.snr[1]);
    }
    if (now >= next_stats) {
      next_stats += 1'000'000;
      std::fflush(out);
      std::fprintf(stderr, "[%3llus] bench %llu | all frames %llu crc_bad %llu\n",
                   static_cast<unsigned long long>((now - t0) / 1'000'000),
                   static_cast<unsigned long long>(bench),
                   static_cast<unsigned long long>(total),
                   static_cast<unsigned long long>(crc_bad));
    }
  }

  std::fprintf(stderr,
               "--- summary (%.1fs) ---\n"
               "bench frames %llu | all frames on channel %llu, crc_bad %llu "
               "(all traffic, not only bench)\n",
               (mono_us() - t0) / 1e6,
               static_cast<unsigned long long>(bench),
               static_cast<unsigned long long>(total),
               static_cast<unsigned long long>(crc_bad));

  fe.stop();
  std::fflush(out);
  if (out != stdout) std::fclose(out);
  return 0;
}
