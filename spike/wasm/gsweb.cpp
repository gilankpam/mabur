// THROWAWAY SPIKE (branch wasm-spike): gsweb -- mabur's receive path for the
// browser. replay: maburd --dry-run --out file -> AU records (parity with
// maburgs --dry-run --out-aus). live: added in a later task.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <libusb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "IRtlRadio.h"
#include "RxPacket.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "body_queue.h"
#include "logger.h"
#include "mabur/ht40.h"
#include "mabur/hevc_params.h"
#ifdef __EMSCRIPTEN__
#include <emscripten/em_asm.h>
#endif

#include "au_file.h"
#include "gsweb_core.h"

namespace {

struct Opts {
  int symbol_size = 332;
  int drop_pct = 0;
  uint32_t seed = 1;
  size_t skip = 0;
};

void print_stats(const gsweb::CoreStats& s) {
  std::fprintf(stderr, "STAT bodies=%llu rc=%llu side=%llu aus=%llu trunc=%llu bad_cfg=%llu \n",
               (unsigned long long)s.bodies, (unsigned long long)s.rc,
               (unsigned long long)s.side, (unsigned long long)s.aus_complete,
               (unsigned long long)s.aus_truncated, (unsigned long long)s.bad_cfg);
}

// Mirrors maburgs FrameFileSource with cards=1: u32 LE length | radiotap |
// dot11 | body; per-frame LCG drop (seed+0); mono_us = (index+1)*900; the
// FrameStream clock is mono_us/1000 and poll() runs after every delivered
// body, then once at last+gap+1.
int run_replay(const char* in, const char* out, const Opts& o) {
  FILE* f = std::fopen(in, "rb");
  if (!f) { std::fprintf(stderr, "error: cannot read %s\n", in); return 2; }
  gsweb::AuFileWriter w;
  if (!w.open(out)) { std::fprintf(stderr, "error: cannot write %s\n", out); return 2; }
  gsweb::RxCore core(o.symbol_size, [&](gsweb::Au&& a) { w.write(a); });
  uint32_t rng = o.seed;
  uint64_t last_ms = 0;
  size_t index = 0;
  uint8_t lenb[4];
  while (std::fread(lenb, 1, 4, f) == 4) {
    const uint32_t len = lenb[0] | (lenb[1] << 8) | (uint32_t(lenb[2]) << 16) |
                         (uint32_t(lenb[3]) << 24);
    std::vector<uint8_t> frame(len);
    if (len == 0 || std::fread(frame.data(), 1, len, f) != len) break;
    const size_t i = index++;
    if (len < 4) continue;
    const size_t rl = frame[2] | (frame[3] << 8);
    if (rl + 1 > len) continue;
    // FrameFileSource discards malformed frames at load time, before any
    // LCG step: same dot11-length rule here, ahead of the rng.
    if (rl + ((frame[rl] == 0x88) ? 26 : 24) > len) continue;
    rng = rng * 1664525u + 1013904223u;
    const bool drop = static_cast<int>((rng >> 16) % 100) < o.drop_pct;
    if (drop || i < o.skip) continue;
    mabur::node::RxBody m;
    if (!gsweb::frame_to_body(frame.data() + rl, len - rl, true, m)) continue;
    m.mono_us = (i + 1) * 900;
    const uint64_t now_ms = m.mono_us / 1000;
    core.on_body(m, now_ms);
    core.poll(now_ms);
    last_ms = now_ms;
  }
  std::fclose(f);
  core.poll(last_ms + 50 + 1);
  print_stats(core.stats());
  return 0;
}

int64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t epoch_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

// USB delivery lateness (host arrival - chip RX TSF, above the per-second
// minimum), rxprobe's measure, on CRC-good own frames.
struct UsbLate {
  std::mutex mu;
  std::vector<int64_t> offs, hosts;
  uint32_t last_tsf = 0;
  int64_t tsf_hi = 0;
  void add(int64_t host, uint32_t tsf) {
    std::lock_guard<std::mutex> lk(mu);
    if (tsf < last_tsf) tsf_hi += int64_t{1} << 32;
    last_tsf = tsf;
    offs.push_back(host - (tsf_hi + tsf));
    hosts.push_back(host);
  }
};

void emit_au(gsweb::Au&& a) {
#ifdef __EMSCRIPTEN__
  // Copy out of the WASM heap on the worker thread, then free; the page
  // receives a plain transferable ArrayBuffer.
  auto* p = static_cast<uint8_t*>(std::malloc(a.data.empty() ? 1 : a.data.size()));
  if (!a.data.empty()) std::memcpy(p, a.data.data(), a.data.size());
  // hvcC for WebCodecs' `description`: WebCodecs refuses a non-IRAP key
  // chunk in Annex-B mode and the drone is GDR (parameter sets ride a
  // TRAIL_R refresh, never an IRAP), so the page configures with hvcC and
  // feeds length-prefixed chunks. Sent with every complete AU that carries
  // a VPS, built by mabur's own HevcParams (the DVR muxer's).
  static mabur::HevcParams hp;
  uint8_t* hv = nullptr;
  int hv_len = 0;
  if (a.complete) {
    bool has_vps = false;
    for (const auto& nal : mabur::split_nals(a.data.data(), a.data.size())) has_vps |= nal.type == 32;
    if (hp.feed(a.data.data(), a.data.size()) && has_vps) {
      const std::vector<uint8_t> rec = hp.hvcc();
      hv = static_cast<uint8_t*>(std::malloc(rec.size()));
      std::memcpy(hv, rec.data(), rec.size());
      hv_len = static_cast<int>(rec.size());
    }
  }
  MAIN_THREAD_ASYNC_EM_ASM(
      {
        const buf = HEAPU8.slice($0, $0 + $1).buffer;
        _free($0);
        let hvcc = null;
        if ($8) { hvcc = HEAPU8.slice($7, $7 + $8).buffer; _free($7); }
        Module['onAu'](buf, $2, $3, $4, $5, $6, hvcc);
      },
      p, static_cast<int>(a.data.size()), static_cast<double>(a.pts_us), a.sid,
      a.flags, a.complete ? 1 : 0, static_cast<double>(a.t_complete_us), hv, hv_len);
#else
  (void)a;
#endif
}

int run_live(uint8_t ch, int width, int secs, int symbol_size) {
  std::printf("gsweb live: ch %u width %d symbol %d\n", ch, width, symbol_size);
  std::fflush(stdout);
  const int64_t t0 = now_us(), e0 = epoch_us();
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) != 0) { std::printf("ERROR libusb_init\n"); return 1; }
  libusb_device** list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  libusb_device* dev = nullptr;
  for (ssize_t i = 0; i < n && !dev; ++i) {
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(list[i], &dd) != 0 || dd.idVendor != 0x0bda) continue;
    for (uint16_t p : {0xa81a, 0x881a, 0x8812}) if (dd.idProduct == p) dev = list[i];
  }
  libusb_device_handle* h = nullptr;
  if (!dev || libusb_open(dev, &h) != 0) { std::printf("ERROR no RTL card\n"); return 1; }
  libusb_free_device_list(list, 1);
  auto logger = std::make_shared<Logger>();
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (int rc = devourer::claim_interface_then_reset(h, 0, logger, true, lock); rc != 0) {
    std::printf("ERROR claim failed rc=%d (card busy?)\n", rc);
    return 1;
  }
  devourer::DeviceConfig cfg;
  cfg.rx.enable_with_tx = true;
  cfg.rx.keep_corrupted = true;
  cfg.tuning.disable_cca = false;
  cfg.tx.no_cancel_multipkt = true;
  cfg.usb.rx_zerocopy = false;
  WiFiDriver driver(logger);
  auto radio = driver.CreateRadio(h, ctx, lock, cfg);
  auto* rtl = dynamic_cast<IRtlRadio*>(radio.get());
  if (!rtl) { std::printf("ERROR unsupported chip\n"); return 1; }
  rtl->InitWrite(width == 40 ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                             : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  std::printf("TIMING init %.2f s\n", (now_us() - t0) / 1e6);
  std::fflush(stdout);

  maburgs::BodyQueue q;
  UsbLate late;
  std::thread rx([&] {
    rtl->StartRxLoop([&](const Packet& pkt) {
      const int64_t host = now_us();
      mabur::node::RxBody m;
      if (!gsweb::frame_to_body(pkt.Data.data(), pkt.Data.size(), !pkt.RxAtrib.crc_err, m))
        return;
      const uint16_t r = pkt.RxAtrib.data_rate;
      m.mcs = (r >= 0x0C && r <= 0x13) ? static_cast<uint8_t>(r - 0x0C) : 255;
      m.mono_us = static_cast<uint64_t>(host);
      if (m.crc_ok) late.add(host, pkt.RxAtrib.tsfl);
      q.push(std::move(m));
    });
  });

  // AU emission gaps at the core (> 25 ms after the previous AU): the same
  // measure the page takes at arrival, so a page-side gap with no core-side
  // gap is the browser's (worker / postMessage), not the link's.
  uint64_t au_gaps = 0;
  int64_t last_au = 0, gap_max = 0;
  gsweb::RxCore core(symbol_size, [&](gsweb::Au&& a) {
    const int64_t t = now_us();
    if (last_au && t - last_au > 25000) {
      ++au_gaps;
      gap_max = std::max(gap_max, t - last_au);
      std::printf("COREGAP at %.1f ms gap %.1f ms\n", (t - t0 + e0) / 1e3, (t - last_au) / 1e3);
    }
    last_au = t;
    emit_au(std::move(a));
  });
  std::vector<mabur::node::RxBody> batch;
  int64_t next_stat = now_us() + 1000000;
  for (int s = 0; secs == 0 || s < secs;) {
    batch.clear();
    q.drain(batch, 5);
    const uint64_t now_ms = static_cast<uint64_t>(now_us() / 1000);
    for (auto& m : batch) core.on_body(m, now_ms);
    core.poll(now_ms);
    if (now_us() < next_stat) continue;
    next_stat += 1000000;
    ++s;
    std::vector<int64_t> offs, hosts;
    { std::lock_guard<std::mutex> lk(late.mu); offs.swap(late.offs); hosts.swap(late.hosts); }
    int64_t p99 = 0, mx = 0;
    if (!offs.empty()) {
      const int64_t mn = *std::min_element(offs.begin(), offs.end());
      for (size_t i = 0; i < offs.size(); ++i) {
        offs[i] -= mn;
        if (offs[i] > 4000 && (i == 0 || offs[i - 1] <= 4000))
          std::printf("SPIKE at %.1f ms late %lld us\n", (hosts[i] - t0 + e0) / 1e3,
                      static_cast<long long>(offs[i]));
      }
      std::vector<int64_t> srt = offs;
      std::sort(srt.begin(), srt.end());
      p99 = srt[srt.size() * 99 / 100];
      mx = srt.back();
    }
    const gsweb::CoreStats st = core.stats();
    std::printf("STAT t=%d bodies=%llu aus=%llu trunc=%llu rc=%llu side=%llu bad_cfg=%llu "
                "qdrop=%llu usb_p99=%lld usb_max=%lld core_gaps=%llu core_gap_max=%lld\n",
                s, (unsigned long long)st.bodies, (unsigned long long)st.aus_complete,
                (unsigned long long)st.aus_truncated, (unsigned long long)st.rc,
                (unsigned long long)st.side, (unsigned long long)st.bad_cfg,
                (unsigned long long)q.dropped(), (long long)p99, (long long)mx,
                (unsigned long long)au_gaps, (long long)gap_max);
    std::fflush(stdout);
  }
  rtl->StopRxLoop();
  q.close();
  rx.join();
  std::printf("DONE\n");
  std::fflush(stdout);
  std::_Exit(0);
}

int usage() {
  std::fprintf(stderr,
               "usage: gsweb replay <in-bodies> <out-aus> [--symbol-size N] "
               "[--drop-pct P] [--seed S] [--skip N]\n"
               "       gsweb live [channel=136] [width=40] [seconds=0] [--symbol-size N]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) return usage();
  const std::string mode = argv[1];
  if (mode == "replay") {
    if (argc < 4) return usage();
    Opts o;
    for (int i = 4; i + 1 < argc; i += 2) {
      const std::string k = argv[i];
      if (k == "--symbol-size") o.symbol_size = std::atoi(argv[i + 1]);
      else if (k == "--drop-pct") o.drop_pct = std::atoi(argv[i + 1]);
      else if (k == "--seed") o.seed = static_cast<uint32_t>(std::atol(argv[i + 1]));
      else if (k == "--skip") o.skip = static_cast<size_t>(std::atol(argv[i + 1]));
      else return usage();
    }
    return run_replay(argv[2], argv[3], o);
  }
  if (mode == "live") {
    const uint8_t ch = argc > 2 ? static_cast<uint8_t>(std::atoi(argv[2])) : 136;
    const int width = argc > 3 ? std::atoi(argv[3]) : 40;
    const int secs = argc > 4 ? std::atoi(argv[4]) : 0;
    int ss = 332;
    for (int i = 5; i + 1 < argc; i += 2)
      if (std::string(argv[i]) == "--symbol-size") ss = std::atoi(argv[i + 1]);
    return run_live(ch, width, secs, ss);
  }
  return usage();
}
