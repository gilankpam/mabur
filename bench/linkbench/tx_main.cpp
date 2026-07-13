// linkbench-tx — drone-side link bench transmitter. Generates a paced,
// sequenced test stream, encodes it with mabur's RS+SBI FEC (single stream,
// id 0xB0), and injects it via devourer at the chosen channel/MCS/power.
// Spec: docs/superpowers/specs/2026-07-13-linkbench-design.md. Bring-up
// order mirrors maburd run_real_mode (drone/src/main.cpp): InitWrite must
// complete before the first send or the bulk-OUT FIFO fills mid-DLFW and
// bricks TX for the session.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bench_wire.h"
#include "mabur/gf256.h"
#include "pacer.h"
#include "tx_pipeline.h"

#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

#include <libusb.h>

namespace {

using namespace linkbench;

struct Args {
  int channel = 149;
  int mcs = 5;
  uint64_t bitrate_bps = 8'000'000;
  int time_s = 0;  // 0 = until SIGINT
  FecParams fec;   // k/overhead/symbol_size/bpb/interleave defaults
  int size = 0;    // 0 = max_packet_size for the symbol size
  bool ldpc = false, stbc = false;
  std::string pwr_mode = "override";
  int pwr = 63;
  int pwr_offset_qdb = 0;
  uint16_t usb_vid = 0x0bda, usb_pid = 0;
  // Parallel sender threads = URBs in flight. The chip flow-controls sync
  // bulk URBs (~0.4 ms acceptance handshake + FIFO drain), so a single
  // blocking sender leaves the radio idle between host round-trips and caps
  // air throughput at ~23-32 Mbps regardless of MCS. ~4 saturates on the
  // HalMAC family (devourer docs/aggregation.md). 1 = strict on-air frame
  // order (threads can swap ≤3-frame URB batches; RX accounting tolerates).
  int tx_threads = 4;
};

void usage(const char* argv0) {
  std::fprintf(stderr,
    "usage: %s --channel N --mcs 0..7 --bitrate 8M [--time S]\n"
    "  [-k 8] [--overhead 0.5] [--symbol-size 64] [--bpb 16] [--interleave 0]\n"
    "  [--size B] [--ldpc] [--stbc]\n"
    "  [--pwr-mode override|none|offset] [--pwr 0..63] [--pwr-offset-qdb Q]\n"
    "  [--usb-vid 0x0bda] [--usb-pid 0] [--tx-threads 4]\n", argv0);
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
    else if (k == "--mcs") { if (!next(&a->mcs)) return false; }
    else if (k == "--bitrate") {
      if (i + 1 >= argc) return false;
      a->bitrate_bps = parse_rate_bps(argv[++i]);
      if (a->bitrate_bps == 0) return false;
    }
    else if (k == "--time") { if (!next(&a->time_s)) return false; }
    else if (k == "-k") { if (!next(&a->fec.k)) return false; }
    else if (k == "--overhead") {
      if (i + 1 >= argc) return false;
      a->fec.overhead = std::strtod(argv[++i], nullptr);
      if (a->fec.overhead <= 0) return false;
    }
    else if (k == "--symbol-size") { if (!next(&a->fec.symbol_size)) return false; }
    else if (k == "--bpb") { if (!next(&a->fec.bpb)) return false; }
    else if (k == "--interleave") { if (!next(&a->fec.interleave)) return false; }
    else if (k == "--size") { if (!next(&a->size)) return false; }
    else if (k == "--ldpc") { a->ldpc = true; }
    else if (k == "--stbc") { a->stbc = true; }
    else if (k == "--pwr-mode") {
      if (i + 1 >= argc) return false;
      a->pwr_mode = argv[++i];
      if (a->pwr_mode != "override" && a->pwr_mode != "none" &&
          a->pwr_mode != "offset") return false;
    }
    else if (k == "--pwr") { if (!next(&a->pwr)) return false; }
    else if (k == "--pwr-offset-qdb") { if (!next(&a->pwr_offset_qdb)) return false; }
    else if (k == "--usb-vid") { int v; if (!next(&v)) return false; a->usb_vid = static_cast<uint16_t>(v); }
    else if (k == "--usb-pid") { int v; if (!next(&v)) return false; a->usb_pid = static_cast<uint16_t>(v); }
    else if (k == "--tx-threads") { if (!next(&a->tx_threads)) return false; }
    else { return false; }
  }
  const int maxp = a->fec.symbol_size - 2;
  if (a->size == 0) a->size = maxp;
  if (a->size < static_cast<int>(kBenchPktHeader) || a->size > maxp) {
    std::fprintf(stderr, "error: --size must be in [%zu, %d] for symbol-size %d\n",
                 kBenchPktHeader, maxp, a->fec.symbol_size);
    return false;
  }
  if (a->mcs < 0 || a->mcs > 7) return false;
  if (a->tx_threads < 1 || a->tx_threads > 16) return false;
  return true;
}

// Same PID scan as maburd's open_usb_and_get_pid.
uint16_t open_usb(uint16_t vid, uint16_t configured_pid, libusb_context* ctx,
                  libusb_device_handle** out) {
  std::vector<uint16_t> pids;
  if (configured_pid != 0) pids.push_back(configured_pid);
  else pids = {0xa81a, 0x881a, 0x8812};
  for (uint16_t pid : pids) {
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (h) { *out = h; return pid; }
  }
  *out = nullptr;
  return 0;
}

uint64_t mono_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

// Estimated over-the-air bytes per app payload byte at this geometry: RS
// expansion × (per-symbol wire cost + per-body overhead share) over app
// bytes per symbol. Banner-grade math, not an airtime model.
double air_factor(const FecParams& f, int app_size, size_t radiotap_len) {
  const double n_over_k = static_cast<double>(f.rs().n()) / f.k;
  const double sym_wire = 2.0 + f.envelope_len();
  const double per_body =
      7.0 + kDot11HeaderLen + static_cast<double>(radiotap_len) + 4.0;  // +FCS
  const double body_share = per_body / f.effective_bpb();
  const double app_per_symbol = static_cast<double>(f.symbol_size) *
      (static_cast<double>(app_size) / (app_size + 2.0));
  return n_over_k * (sym_wire + body_share) / app_per_symbol;
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse_args(argc, argv, &a)) {
    usage(argv[0]);
    return 2;
  }
  install_devourer_signal_handlers();

  devourer::TxMode mode;
  mode.mode = devourer::TxMode::Mode::HT;
  mode.ht_mcs = static_cast<uint8_t>(a.mcs);
  mode.bw_mhz = 20;
  mode.ldpc = a.ldpc;
  mode.stbc = a.stbc;
  const std::vector<uint8_t> radiotap = devourer::build_stream_radiotap(mode);

  const double afac = air_factor(a.fec, a.size, radiotap.size());
  std::fprintf(stderr,
               "linkbench-tx: ch %d mcs %d %s%sbitrate %.2f Mbps app "
               "(~%.2f Mbps air, factor %.2f)\n"
               "  fec k=%d n=%d symbol=%d bpb=%d interleave=%d size=%d "
               "pwr-mode=%s pwr=%d gf256=%s\n",
               a.channel, a.mcs, a.ldpc ? "ldpc " : "", a.stbc ? "stbc " : "",
               a.bitrate_bps / 1e6, a.bitrate_bps / 1e6 * afac, afac,
               a.fec.k, a.fec.rs().n(), a.fec.symbol_size, a.fec.bpb,
               a.fec.interleave > 0 ? a.fec.effective_bpb() : 0, a.size,
               a.pwr_mode.c_str(), a.pwr,
               mabur::gf::backend());
  std::fprintf(stderr, "  tx-threads %d (URBs in flight)\n", a.tx_threads);

  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Warn);

  libusb_context* usb_ctx = nullptr;
  if (libusb_init(&usb_ctx) < 0) {
    std::fprintf(stderr, "error: libusb_init failed\n");
    return 1;
  }
  libusb_device_handle* handle = nullptr;
  const uint16_t pid = open_usb(a.usb_vid, a.usb_pid, usb_ctx, &handle);
  if (!handle) {
    std::fprintf(stderr, "error: no radio under VID 0x%04x\n", a.usb_vid);
    libusb_exit(usb_ctx);
    return 1;
  }
  std::fprintf(stderr, "opened device %04x:%04x\n", a.usb_vid, pid);

  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  if (devourer::claim_interface_then_reset(handle, 0, logger, /*do_reset=*/true,
                                           usb_lock) != 0) {
    std::fprintf(stderr, "error: claim_interface_then_reset failed\n");
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 1;
  }

  devourer::DeviceConfig dev_cfg;
  dev_cfg.rx.enable_with_tx = true;  // TX+RX duplex bring-up, as maburd
  dev_cfg.tx.usb_agg_max = 3;        // pack up to 3 frames per bulk-OUT URB

  WiFiDriver wifi_driver{logger};
  auto dev = wifi_driver.CreateRtlDevice(handle, usb_ctx, usb_lock, dev_cfg);
  if (!dev) {
    std::fprintf(stderr, "error: CreateRtlDevice failed\n");
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(usb_ctx);
    return 1;
  }

  std::fprintf(stderr, "bringing up TX on channel %d\n", a.channel);
  dev->InitWrite(SelectedChannel{static_cast<uint8_t>(a.channel), 0,
                                 CHANNEL_WIDTH_20});
  if (a.pwr_mode == "override") dev->SetTxPowerIndexOverride(a.pwr);
  else if (a.pwr_mode == "offset") dev->SetTxPowerOffsetQdb(a.pwr_offset_qdb);
  // "none": leave the efuse per-rate (per-MCS) calibration untouched.

  // TX hot loop in its own thread; main blocks in StartRxLoop (which
  // watches g_devourer_should_stop) exactly like maburd.
  std::thread tx_thread([&] {
    TxPipeline pipe(a.fec);
    // Burst must scale with rate: a fixed cap of a few KB divided by the
    // real loop period (1 ms nominal, 2+ ms under scheduler jitter) would
    // ceiling the offered load below the very link knee this bench exists
    // to find (MCS5 app knee ~16-17 Mbps vs a 3968-byte cap's ~16-31 Mbps
    // ceiling). 10 ms of rate keeps catch-up bursts bench-harmless.
    TokenBucket bucket(static_cast<double>(a.bitrate_bps) / 8.0,
                       std::max(static_cast<double>(a.size) * 64.0,
                                static_cast<double>(a.bitrate_bps) / 8.0 * 0.010));
    uint32_t app_seq = 0;
    uint16_t mac_seq = 0;
    uint64_t app_bytes = 0, air_bytes = 0;
    uint64_t iv_app = 0, iv_air = 0, iv_blk0 = 0;
    uint64_t iv_frm0 = 0, iv_fail0 = 0;
    const uint64_t t0 = mono_us();
    uint64_t next_stats = t0 + 1'000'000;
    const uint64_t deadline =
        a.time_s > 0 ? t0 + static_cast<uint64_t>(a.time_s) * 1'000'000 : 0;

    // Frame queue feeding N sender threads. Each sender blocks in its own
    // sync bulk transfer, so ~N URBs ride the endpoint at once — the deep
    // feed that keeps the chip's TX FIFO from idling during host round
    // trips (devourer docs/aggregation.md; sync bulk from multiple threads
    // is legal and simply queues). Bounded so a slow link backpressures the
    // pacer (token bucket overflows → offered load reflects reality).
    std::mutex qm;
    std::condition_variable q_fill, q_space;
    std::deque<std::vector<uint8_t>> q;
    bool q_done = false;
    constexpr size_t kQueueCap = 256;  // frames (~64-85 URBs of headroom)
    std::atomic<uint64_t> frames_ok{0}, frames_fail{0};
    std::vector<std::thread> senders;
    senders.reserve(static_cast<size_t>(a.tx_threads));
    for (int s = 0; s < a.tx_threads; ++s) {
      senders.emplace_back([&] {
        std::vector<std::vector<uint8_t>> batch;
        for (;;) {
          {
            std::unique_lock<std::mutex> lk(qm);
            q_fill.wait(lk, [&] { return q_done || !q.empty(); });
            if (q.empty()) return;  // q_done and drained
            // Batches of ≤3: the HalMAC per-transfer descriptor limit
            // devourer's send_packets aggregation packs into one URB.
            const size_t n = std::min<size_t>(3, q.size());
            for (size_t i = 0; i < n; ++i) {
              batch.push_back(std::move(q.front()));
              q.pop_front();
            }
          }
          q_space.notify_one();
          TxPacketView v[3];
          for (size_t i = 0; i < batch.size(); ++i)
            v[i] = {batch[i].data(), batch[i].size()};
          const size_t ok = dev->send_packets(v, batch.size());
          frames_ok += ok;
          frames_fail += batch.size() - ok;
          batch.clear();
        }
      });
    }

    std::vector<std::vector<uint8_t>> bodies;
    std::vector<std::vector<uint8_t>> frames;  // built, pending enqueue
    auto send_frames = [&] {
      std::unique_lock<std::mutex> lk(qm);
      for (auto& f : frames) {
        q_space.wait(lk, [&] { return q.size() < kQueueCap; });
        q.push_back(std::move(f));
        q_fill.notify_one();
      }
      frames.clear();
    };
    auto body_to_frame = [&](std::vector<uint8_t>& body) {
      std::vector<uint8_t> f;
      f.reserve(radiotap.size() + kDot11HeaderLen + body.size());
      f.insert(f.end(), radiotap.begin(), radiotap.end());
      auto hdr = build_dot11_header(mac_seq);
      mac_seq = static_cast<uint16_t>((mac_seq + 1) & 0x0FFF);
      f.insert(f.end(), hdr.begin(), hdr.end());
      f.insert(f.end(), body.begin(), body.end());
      air_bytes += f.size();
      iv_air += f.size();
      frames.push_back(std::move(f));
    };

    while (!g_devourer_should_stop) {
      const uint64_t now = mono_us();
      if (deadline && now >= deadline) break;
      bucket.advance(now);
      bodies.clear();
      while (bucket.spend(static_cast<size_t>(a.size))) {
        auto pkt = build_bench_packet(app_seq++, static_cast<size_t>(a.size));
        app_bytes += pkt.size();
        iv_app += pkt.size();
        pipe.add_packet(pkt.data(), pkt.size(), bodies);
      }
      for (auto& b : bodies) body_to_frame(b);
      if (!frames.empty()) send_frames();

      if (now >= next_stats) {
        next_stats += 1'000'000;
        const uint64_t t = (now - t0) / 1'000'000;
        const uint64_t blk = pipe.blocks_encoded();
        const uint64_t frm = frames_ok.load(), fail = frames_fail.load();
        std::fprintf(stderr,
                     "[%3llus] app %.2fM air %.2fM | frm %llu fail %llu | blk %llu%s\n",
                     static_cast<unsigned long long>(t), iv_app * 8.0 / 1e6,
                     iv_air * 8.0 / 1e6,
                     static_cast<unsigned long long>(frm - iv_frm0),
                     static_cast<unsigned long long>(fail - iv_fail0),
                     static_cast<unsigned long long>(blk - iv_blk0),
                     iv_app * 8.0 < a.bitrate_bps * 0.95 ? "  ** under target **"
                                                         : "");
        iv_app = iv_air = 0;
        iv_frm0 = frm;
        iv_fail0 = fail;
        iv_blk0 = blk;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    bodies.clear();
    pipe.flush(bodies);
    for (auto& b : bodies) body_to_frame(b);
    if (!frames.empty()) send_frames();
    {
      std::lock_guard<std::mutex> lk(qm);
      q_done = true;
    }
    q_fill.notify_all();
    for (auto& s : senders) s.join();
    const double dur = (mono_us() - t0) / 1e6;
    std::fprintf(stderr,
                 "done: %.1fs, %llu pkts (%.2f Mbps app), %llu frames "
                 "(%.2f Mbps air), %llu send-fail, %llu blocks, %zu oversize\n",
                 dur, static_cast<unsigned long long>(app_seq),
                 app_bytes * 8.0 / 1e6 / dur,
                 static_cast<unsigned long long>(frames_ok),
                 air_bytes * 8.0 / 1e6 / dur,
                 static_cast<unsigned long long>(frames_fail),
                 static_cast<unsigned long long>(pipe.blocks_encoded()),
                 pipe.oversize_drops());
    g_devourer_should_stop = true;
  });

  dev->StartRxLoop([](const Packet&) {});  // blocks until stop flag

  g_devourer_should_stop = true;
  if (tx_thread.joinable()) tx_thread.join();
  dev->Stop();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(usb_ctx);
  return 0;
}
