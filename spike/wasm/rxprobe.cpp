// THROWAWAY SPIKE (branch wasm-spike): can devourer RX mabur video from a
// browser over WebUSB? The same source builds natively (libusb/usbfs) and
// with Emscripten (libusb's WebUSB backend) so the two runs are comparable
// on the same card, channel and drone.
//
// Per second it prints: frames, own CRC-good frames, CRC-failed frames, Mb/s,
// MAC-seq loss on own frames, and delivery jitter -- the host arrival time
// minus the chip's RX TSF stamp, relative to the window's minimum (i.e. how
// late past the best-case USB path each frame reached the host).
//
// usage: rxprobe [channel=136] [width=40] [seconds=30] [bench]

#include <libusb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "IRtlRadio.h"
#include "RxPacket.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "mabur/ht40.h"

namespace {
constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr uint16_t kPids[] = {0xa81a, 0x881a, 0x8812};

int64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct Window {
  uint64_t frames = 0, own = 0, crc = 0, bytes = 0, walked = 0;
  int64_t lost = 0;
  std::vector<int64_t> offs;  // host_us - tsf_us (unwrapped)
};

std::mutex g_mu;
Window g_win;
int g_last_seq = -1;
bool g_bench = false;
uint32_t g_last_tsf = 0;
int64_t g_tsf_hi = 0;

void on_packet(const Packet& pkt) {
  const int64_t host = now_us();
  std::lock_guard<std::mutex> lk(g_mu);
  ++g_win.frames;
  g_win.bytes += pkt.Data.size();
  if (pkt.RxAtrib.crc_err) { ++g_win.crc; return; }
  if (pkt.Data.size() < 24 || std::memcmp(pkt.Data.data() + 10, kSa, 6) != 0) return;
  ++g_win.own;
  // Seq walk on the drone's QoS-data (video/MSP) frames only: the GS's own
  // uplink frames share the SA but come from the GS card's seq counter.
  // g_bench (linkbench-tx): its frames are 0x40 like the GS's, so walk the
  // big ones only -- GS beacons/RCFs are small.
  if (g_bench ? (pkt.Data[0] != 0x40 || pkt.Data.size() < 1000) : pkt.Data[0] != 0x88) return;
  ++g_win.walked;
  const int seq = (pkt.Data[22] | (pkt.Data[23] << 8)) >> 4;
  if (g_last_seq >= 0) {
    const int gap = (seq - g_last_seq) & 0xfff;
    // Behind the high-water mark (linkbench's parallel senders reorder <=3
    // frames): it was booked missing when the mark jumped, so un-book it.
    if (gap >= 2048) { if (4096 - gap < 64) --g_win.lost; return; }
    if (gap > 1 && gap < 64) g_win.lost += gap - 1;
  }
  g_last_seq = seq;
  const uint32_t t = pkt.RxAtrib.tsfl;
  if (t < g_last_tsf) g_tsf_hi += (int64_t{1} << 32);
  g_last_tsf = t;
  g_win.offs.push_back(host - (g_tsf_hi + t));
}
}  // namespace

int main(int argc, char** argv) {
  const uint8_t ch = argc > 1 ? static_cast<uint8_t>(std::atoi(argv[1])) : 136;
  const int width = argc > 2 ? std::atoi(argv[2]) : 40;
  const int secs = argc > 3 ? std::atoi(argv[3]) : 30;
  g_bench = argc > 4 && std::strcmp(argv[4], "bench") == 0;
  std::printf("rxprobe: ch %u width %d secs %d%s\n", ch, width, secs, g_bench ? " (linkbench frames)" : "");

  const int64_t t0 = now_us();
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) != 0) { std::printf("libusb_init failed\n"); return 1; }
  libusb_device** list = nullptr;
  const ssize_t n = libusb_get_device_list(ctx, &list);
  libusb_device* dev = nullptr;
  for (ssize_t i = 0; i < n && !dev; ++i) {
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(list[i], &dd) != 0 || dd.idVendor != 0x0bda) continue;
    for (uint16_t p : kPids) if (dd.idProduct == p) dev = list[i];
  }
  libusb_device_handle* h = nullptr;
  if (!dev || libusb_open(dev, &h) != 0) { std::printf("no RTL card (%zd devices)\n", n); return 1; }
  libusb_free_device_list(list, 1);

  auto logger = std::make_shared<Logger>();
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  const bool do_reset = std::getenv("RXPROBE_NO_RESET") == nullptr;
  if (int rc = devourer::claim_interface_then_reset(h, 0, logger, do_reset, lock); rc != 0) {
    std::printf("claim failed rc=%d\n", rc);
    return 1;
  }
  const int64_t t_open = now_us();

  devourer::DeviceConfig cfg;
  cfg.rx.enable_with_tx = true;
  cfg.rx.keep_corrupted = true;
  cfg.tuning.disable_cca = false;
  cfg.tx.no_cancel_multipkt = true;
  cfg.usb.rx_zerocopy = false;
  WiFiDriver driver(logger);
  auto radio = driver.CreateRadio(h, ctx, lock, cfg);
  auto* rtl = dynamic_cast<IRtlRadio*>(radio.get());
  if (!rtl) { std::printf("unsupported chip\n"); return 1; }
  const int64_t t_create = now_us();
  rtl->InitWrite(width == 40 ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                             : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  const int64_t t_init = now_us();
  std::printf("TIMING open+claim+reset %.2f s, CreateRadio %.2f s, InitWrite %.2f s, total %.2f s\n",
              (t_open - t0) / 1e6, (t_create - t_open) / 1e6, (t_init - t_create) / 1e6,
              (t_init - t0) / 1e6);
  std::fflush(stdout);

  std::thread rx([&] { rtl->StartRxLoop(on_packet); });

  Window total;
  std::vector<int64_t> all_jit;
  for (int s = 0; s < secs; ++s) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Window w;
    { std::lock_guard<std::mutex> lk(g_mu); std::swap(w, g_win); }
    int64_t p50 = 0, p99 = 0, mx = 0;
    if (!w.offs.empty()) {
      const int64_t mn = *std::min_element(w.offs.begin(), w.offs.end());
      for (auto& o : w.offs) o -= mn;
      std::sort(w.offs.begin(), w.offs.end());
      p50 = w.offs[w.offs.size() / 2];
      p99 = w.offs[w.offs.size() * 99 / 100];
      mx = w.offs.back();
      if (s > 0) all_jit.insert(all_jit.end(), w.offs.begin(), w.offs.end());  // skip startup backlog
    }
    const double loss = w.walked + w.lost > 0 ? 100.0 * w.lost / (w.walked + w.lost) : 0;
    std::printf("t=%2d frames %5llu own %5llu crc %4llu %6.2f Mb/s loss %5.2f%% jit p50 %5lld p99 %6lld max %6lld us\n",
                s + 1, (unsigned long long)w.frames, (unsigned long long)w.own,
                (unsigned long long)w.crc, w.bytes * 8 / 1e6, loss, (long long)p50,
                (long long)p99, (long long)mx);
    std::fflush(stdout);
    total.frames += w.frames; total.own += w.own; total.crc += w.crc;
    total.bytes += w.bytes; total.lost += w.lost; total.walked += w.walked;
  }
  std::sort(all_jit.begin(), all_jit.end());
  const auto pct = [&](double q) {
    return all_jit.empty() ? 0LL : (long long)all_jit[std::min(all_jit.size() - 1, size_t(all_jit.size() * q))];
  };
  std::printf("SUMMARY frames %llu own %llu crc %llu avg %.2f Mb/s loss %.3f%% jit p50 %lld p99 %lld p99.9 %lld max %lld us\n",
              (unsigned long long)total.frames, (unsigned long long)total.own,
              (unsigned long long)total.crc, total.bytes * 8 / 1e6 / secs,
              total.walked + total.lost > 0 ? 100.0 * total.lost / (total.walked + total.lost) : 0.0,
              pct(0.5), pct(0.99), pct(0.999), all_jit.empty() ? 0LL : (long long)all_jit.back());
  std::fflush(stdout);
  rtl->StopRxLoop();
  rx.join();
  std::printf("DONE\n");
  std::fflush(stdout);
  std::_Exit(0);
}
