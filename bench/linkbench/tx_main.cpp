// linkbench-tx — drone-side link bench transmitter. Generates a paced,
// sequenced test stream, encodes it with mabur's sliding-window+SBI FEC
// (single stream, id 0xB0), and injects it via devourer at the chosen
// channel/MCS/power.
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
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "bench_wire.h"
#include "mabur/cal_wire.h"
#include "mabur/ht40.h"
#include "mabur/gf256.h"
#include "pacer.h"
#include "tx_pipeline.h"

#include "AmpduMode.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "TxMode.h"
#include "TxPower.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

#include <libusb.h>

namespace {

using namespace linkbench;

struct Args {
  int channel = 149;
  int bw = 20;  // 20, or 40 (channel = primary; secondary per mabur::ht40_offset)
  int mcs = 5;
  uint64_t bitrate_bps = 8'000'000;
  int time_s = 0;  // 0 = until SIGINT
  FecParams fec;   // overhead/symbol_size/window/bpb defaults
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
  // A-MPDU TX aggregation as maburd flies it (ampdu.max_num / max_time);
  // 0 = off (QoS-Data singles).
  int ampdu = 0;
  int ampdu_max_time = 32;
  // --wall-sweep: maburcal's per-rate TX-power wall sweep, FEC-free, at
  // this tool's --bw/--ldpc/--stbc. Every frame is a mabur::cal payload
  // stamped (mcs, rel idx); absolute index = the chip's anchor (mcs7 ref
  // with rate diffs zeroed, as maburd reads it) + rel. linkbench-rx --wall
  // tallies per cell (docs/bw40-sweep-findings-2026-09-23.md).
  bool no_cca = false;
  // --foreign-sa: flip the SA/BSSID so a GS books these frames as foreign
  // traffic (interferer role), not own.
  bool foreign_sa = false;  // --no-cca: MAC carrier sense off (maburd flies it ON)
  bool wall_sweep = false;
  int wall_lo = -41, wall_hi = 63;
  int wall_frames = 100;
  int wall_gap_us = 1000;
  int wall_settle_ms = 20;
  int wall_mcs_mask = 0xFF;
};

void usage(const char* argv0) {
  std::fprintf(stderr,
    "usage: %s --channel N [--bw 20|40] --mcs 0..7 --bitrate 8M [--time S]\n"
    "  [--overhead 0.5] [--symbol-size 64] [--window 128] [--bpb 16]\n"
    "  [--size B] [--ldpc] [--stbc]\n"
    "  [--pwr-mode override|none|offset] [--pwr 0..63] [--pwr-offset-qdb Q]\n"
    "  [--usb-vid 0x0bda] [--usb-pid 0] [--tx-threads 4]\n"
    "  [--ampdu N (max_num, 0=off)] [--ampdu-max-time 32]\n"
    "  [--no-cca] [--foreign-sa] [--wall-sweep [--wall-lo -41] [--wall-hi 63] [--wall-frames 100]\n"
    "   [--wall-gap-us 1000] [--wall-settle-ms 20] [--wall-mcs-mask 0xff]]\n", argv0);
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
    else if (k == "--mcs") { if (!next(&a->mcs)) return false; }
    else if (k == "--bitrate") {
      if (i + 1 >= argc) return false;
      a->bitrate_bps = parse_rate_bps(argv[++i]);
      if (a->bitrate_bps == 0) return false;
    }
    else if (k == "--time") { if (!next(&a->time_s)) return false; }
    else if (k == "--overhead") {
      if (i + 1 >= argc) return false;
      a->fec.overhead = std::strtod(argv[++i], nullptr);
      if (a->fec.overhead <= 0) return false;
    }
    else if (k == "--symbol-size") { if (!next(&a->fec.symbol_size)) return false; }
    else if (k == "--bpb") { if (!next(&a->fec.bpb)) return false; }
    else if (k == "--window") { if (!next(&a->fec.window)) return false; }
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
    else if (k == "--ampdu") { if (!next(&a->ampdu)) return false; }
    else if (k == "--ampdu-max-time") { if (!next(&a->ampdu_max_time)) return false; }
    else if (k == "--no-cca") { a->no_cca = true; }
    else if (k == "--foreign-sa") { a->foreign_sa = true; }
    else if (k == "--wall-sweep") { a->wall_sweep = true; }
    else if (k == "--wall-lo") { if (!next(&a->wall_lo)) return false; }
    else if (k == "--wall-hi") { if (!next(&a->wall_hi)) return false; }
    else if (k == "--wall-frames") { if (!next(&a->wall_frames)) return false; }
    else if (k == "--wall-gap-us") { if (!next(&a->wall_gap_us)) return false; }
    else if (k == "--wall-settle-ms") { if (!next(&a->wall_settle_ms)) return false; }
    else if (k == "--wall-mcs-mask") { if (!next(&a->wall_mcs_mask)) return false; }
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
  if (a->bw == 40 && mabur::ht40_offset(static_cast<uint8_t>(a->channel)) == 0) {
    std::fprintf(stderr, "error: channel %d has no 5 GHz HT40 pair\n", a->channel);
    return false;
  }
  if (a->wall_lo < -64 || a->wall_hi > 63 || a->wall_lo > a->wall_hi ||
      a->wall_frames < 1 || a->wall_frames > 65535) return false;
  if (a->tx_threads < 1 || a->tx_threads > 16) return false;
  if (a->ampdu < 0 || a->ampdu > 63 || a->ampdu_max_time < 0 || a->ampdu_max_time > 255) return false;
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

// Estimated over-the-air bytes per app payload byte at this geometry:
// sliding-window expansion (1 repair symbol per source symbol per unit of
// overhead) × (per-symbol wire cost + per-body overhead share) over app
// bytes per symbol. Banner-grade math, not an airtime model.
double air_factor(const FecParams& f, int app_size, size_t radiotap_len) {
  const double expansion = 1.0 + f.overhead;
  const double sym_wire = 2.0 + f.envelope_len();
  const double per_body =
      7.0 + kDot11HeaderLen + static_cast<double>(radiotap_len) + 4.0;  // +FCS
  const double body_share = per_body / f.bpb;
  const double app_per_symbol = static_cast<double>(f.symbol_size) *
      (static_cast<double>(app_size) / (app_size + 2.0));
  return expansion * (sym_wire + body_share) / app_per_symbol;
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
  mode.bw_mhz = static_cast<uint8_t>(a.bw);
  mode.ldpc = a.ldpc;
  mode.stbc = a.stbc;
  const std::vector<uint8_t> radiotap = devourer::build_stream_radiotap(mode);

  const double afac = air_factor(a.fec, a.size, radiotap.size());
  std::fprintf(stderr,
               "linkbench-tx: ch %d bw %d mcs %d %s%sbitrate %.2f Mbps app "
               "(~%.2f Mbps air, factor %.2f)\n"
               "  fec window=%d overhead=%.2f symbol=%d bpb=%d size=%d "
               "pwr-mode=%s pwr=%d gf256=%s\n",
               a.channel, a.bw, a.mcs, a.ldpc ? "ldpc " : "", a.stbc ? "stbc " : "",
               a.bitrate_bps / 1e6, a.bitrate_bps / 1e6 * afac, afac,
               a.fec.window, a.fec.overhead, a.fec.symbol_size, a.fec.bpb,
               a.size, a.pwr_mode.c_str(), a.pwr,
               mabur::gf::backend());
  std::fprintf(stderr, "  tx-threads %d (URBs in flight)\n", a.tx_threads);

  auto logger = std::make_shared<Logger>();
  // set_level() gates the diagnostic channel only; the JSON event stream is
  // gated solely by EventSink::enabled() (defaults to stdout, enabled,
  // flush-per-line). Without the disable(), jaguar3's per-URB "tx.agg" event
  // is one fwrite+fflush per bulk-OUT on the TX hot path — noise in the
  // measurement and ~1.5 MB/min of log for a run nothing here parses.
  logger->set_level(Logger::Level::Warn);
  logger->events().disable();

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
  dev_cfg.tuning.disable_cca = a.no_cca;

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
  const uint8_t ch = static_cast<uint8_t>(a.channel);
  dev->InitWrite(a.bw == 40 ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                            : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  if (a.pwr_mode == "override") dev->SetTxPowerIndexOverride(a.pwr);
  else if (a.pwr_mode == "offset") dev->SetTxPowerOffsetQdb(a.pwr_offset_qdb);
  // "none": leave the efuse per-rate (per-MCS) calibration untouched.

  // A-MPDU exactly as drone/src/main.cpp programs it (after InitWrite, before
  // the first send): no_ack + density 7, aggregate-fill timer max_time.
  if (a.ampdu > 0) {
    devourer::AmpduMode am;
    am.enabled = true;
    am.tid = 0;
    am.max_num = static_cast<uint8_t>(a.ampdu);
    am.density = 7;
    am.no_ack = true;
    am.max_time = static_cast<uint8_t>(a.ampdu_max_time);
    if (!dev->SetAmpduMode(am))
      std::fprintf(stderr, "warning: SetAmpduMode failed -- running un-aggregated\n");
    else
      std::fprintf(stderr, "A-MPDU ON (max_num=%d density=7 no-ack max_time=0x%02x)\n",
                   a.ampdu, a.ampdu_max_time);
  } else {
    std::fprintf(stderr, "A-MPDU OFF (QoS-Data singles)\n");
  }

  int anchor = -1;
  if (a.wall_sweep) {
    // maburd's read_anchor_idx(): zero the custom rate-diff table, then the
    // mcs7 index IS the chip's per-channel (and per-width) reference.
    dev->SetTxPowerRateDiffs(devourer::TxRateDiffsQdb{});
    const auto st = dev->GetTxPowerState();
    anchor = (st.valid && st.mcs7_index >= 0) ? st.mcs7_index : -1;
    std::fprintf(stderr, "wall-sweep: anchor %d (ch %d bw %d)\n", anchor,
                 a.channel, a.bw);
    std::printf("anchor %d ch %d bw %d ldpc %d stbc %d\n", anchor, a.channel,
                a.bw, a.ldpc ? 1 : 0, a.stbc ? 1 : 0);
    std::fflush(stdout);
    if (anchor < 0) {
      std::fprintf(stderr, "error: TXAGC anchor unreadable\n");
      return 1;
    }
  }
  auto wall_sweep = [&] {
    uint16_t mac_seq = 0;
    const uint64_t t0 = mono_us();
    for (int m = 0; m < 8 && !g_devourer_should_stop; ++m) {
      if (!(a.wall_mcs_mask & (1 << m))) continue;
      devourer::TxMode wm = mode;
      wm.ht_mcs = static_cast<uint8_t>(m);
      const std::vector<uint8_t> rt = devourer::build_stream_radiotap(wm);
      int sent_row = 0, fail_row = 0;
      for (int rel = a.wall_lo; rel <= a.wall_hi && !g_devourer_should_stop; ++rel) {
        dev->SetTxPowerIndexOverride(std::clamp(anchor + rel, 0, 127));
        std::this_thread::sleep_for(std::chrono::milliseconds(a.wall_settle_ms));
        uint64_t due = mono_us();
        for (int k = 0; k < a.wall_frames; ++k) {
          const auto body = mabur::cal::build_cal_payload(
              static_cast<uint8_t>(m), static_cast<int8_t>(rel),
              mabur::cal::kPhaseFine, static_cast<uint16_t>(k));
          std::vector<uint8_t> f;
          f.reserve(rt.size() + kDot11HeaderLen + body.size());
          f.insert(f.end(), rt.begin(), rt.end());
          const auto hdr = build_dot11_header(mac_seq);
          mac_seq = static_cast<uint16_t>((mac_seq + 1) & 0x0FFF);
          f.insert(f.end(), hdr.begin(), hdr.end());
          f.insert(f.end(), body.begin(), body.end());
          if (dev->send_packet(f.data(), f.size())) ++sent_row; else ++fail_row;
          due += static_cast<uint64_t>(a.wall_gap_us);
          const uint64_t now = mono_us();
          if (due > now) std::this_thread::sleep_for(std::chrono::microseconds(due - now));
        }
      }
      std::printf("row mcs %d sent %d fail %d\n", m, sent_row, fail_row);
      std::fflush(stdout);
      std::fprintf(stderr, "wall-sweep: mcs %d done (%d sent, %d fail) t=%.1fs\n",
                   m, sent_row, fail_row, (mono_us() - t0) / 1e6);
    }
    dev->SetTxPowerIndexOverride(-1);
    g_devourer_should_stop = true;
  };

  // TX hot loop in its own thread; main blocks in StartRxLoop (which
  // watches g_devourer_should_stop) exactly like maburd.
  std::thread tx_thread([&] {
    if (a.wall_sweep) { wall_sweep(); return; }
    // Random initial seq: a restarted linkbench-tx re-sending seq 0 within
    // SwDecoder's kResetSpan of the previous run's seqs is otherwise
    // dropped as stale for that run's lifetime (final-review Critical, see
    // sw_encoder.h / tx_pipeline.h). Tests keep the deterministic 0
    // default; only this live entry point randomizes.
    std::random_device rd;
    std::mt19937 seed_gen(rd());
    TxPipeline pipe(a.fec, std::uniform_int_distribution<uint32_t>()(seed_gen));
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
    uint64_t iv_app = 0, iv_air = 0, iv_src0 = 0;
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
      if (a.foreign_sa) { hdr[15] ^= 0xFF; hdr[21] ^= 0xFF; }
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
        const uint64_t src = pipe.sources_sent();
        const uint64_t frm = frames_ok.load(), fail = frames_fail.load();
        std::fprintf(stderr,
                     "[%3llus] app %.2fM air %.2fM | frm %llu fail %llu | src %llu%s\n",
                     static_cast<unsigned long long>(t), iv_app * 8.0 / 1e6,
                     iv_air * 8.0 / 1e6,
                     static_cast<unsigned long long>(frm - iv_frm0),
                     static_cast<unsigned long long>(fail - iv_fail0),
                     static_cast<unsigned long long>(src - iv_src0),
                     iv_app * 8.0 < a.bitrate_bps * 0.95 ? "  ** under target **"
                                                         : "");
        iv_app = iv_air = 0;
        iv_frm0 = frm;
        iv_fail0 = fail;
        iv_src0 = src;
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
                 "(%.2f Mbps air), %llu send-fail, %llu sources %llu repairs, "
                 "%zu oversize\n",
                 dur, static_cast<unsigned long long>(app_seq),
                 app_bytes * 8.0 / 1e6 / dur,
                 static_cast<unsigned long long>(frames_ok),
                 air_bytes * 8.0 / 1e6 / dur,
                 static_cast<unsigned long long>(frames_fail),
                 static_cast<unsigned long long>(pipe.sources_sent()),
                 static_cast<unsigned long long>(pipe.repairs_sent()),
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
