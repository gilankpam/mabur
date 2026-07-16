// txagcbench-tx — drone-side TXAGC gain-curve sweeper. One bring-up, then
// SetTxPowerIndexOverride walks [--lo, --hi] ascending (pass 1) and back
// descending (pass 2), injecting --frames raw sweep frames per index; each
// frame's payload carries the index it was sent at (sweep_wire.h). No FEC:
// attribution must be per-frame, and a frame lost below the decode floor is
// itself data. Bring-up order mirrors linkbench-tx / maburd run_real_mode
// (InitWrite before first send or the bulk-OUT FIFO bricks TX mid-DLFW).
// Spec: docs/superpowers/specs/2026-07-16-txagcbench-design.md.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "sweep_wire.h"

#include "RadiotapBuilder.h"
#include "SignalStop.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

#include <libusb.h>

namespace {

using namespace txagcbench;

struct Args {
  int channel = 149;
  int lo = 0, hi = 63;
  int frames = 50;
  int settle_ms = 100;
  int gap_us = 2000;
  int mcs = 0;
  // "override": SetTxPowerIndexOverride per index (flat power — the gain/wall
  // sweep). "none": never touch power; bring-up state stands, i.e. the efuse
  // refs + phy_reg_pg per-rate diffs stay live. Use with --lo 0 --hi 0 to
  // measure the as-deployed per-rate power (the stamped idx is then a dummy).
  std::string pwr_mode = "override";
  uint16_t usb_vid = 0x0bda, usb_pid = 0;
};

void usage(const char* argv0) {
  std::fprintf(stderr,
    "usage: %s [--channel 149] [--lo 0] [--hi 63, max 127] [--frames 50]\n"
    "  [--settle-ms 100] [--gap-us 2000] [--mcs 0] [--pwr-mode override|none]\n"
    "  [--usb-vid 0x0bda] [--usb-pid 0]\n", argv0);
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
    else if (k == "--lo") { if (!next(&a->lo)) return false; }
    else if (k == "--hi") { if (!next(&a->hi)) return false; }
    else if (k == "--frames") { if (!next(&a->frames)) return false; }
    else if (k == "--settle-ms") { if (!next(&a->settle_ms)) return false; }
    else if (k == "--gap-us") { if (!next(&a->gap_us)) return false; }
    else if (k == "--mcs") { if (!next(&a->mcs)) return false; }
    else if (k == "--pwr-mode") {
      if (i + 1 >= argc) return false;
      a->pwr_mode = argv[++i];
      if (a->pwr_mode != "override" && a->pwr_mode != "none") return false;
    }
    else if (k == "--usb-vid") { int v; if (!next(&v)) return false; a->usb_vid = static_cast<uint16_t>(v); }
    else if (k == "--usb-pid") { int v; if (!next(&v)) return false; a->usb_pid = static_cast<uint16_t>(v); }
    else { return false; }
  }
  if (a->lo < 0 || a->hi > 127 || a->lo > a->hi) return false;  // 7-bit TXAGC
  if (a->mcs < 0 || a->mcs > 7) return false;
  if (a->frames < 1 || a->settle_ms < 0 || a->gap_us < 0) return false;
  return true;
}

// Same PID scan as linkbench-tx / maburd's open_usb_and_get_pid.
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
  const std::vector<uint8_t> radiotap = devourer::build_stream_radiotap(mode);

  std::fprintf(stderr,
               "txagcbench-tx: ch %d mcs %d idx %d..%d frames/idx %d "
               "settle %dms gap %dus (2 passes, ~%.0fs)\n",
               a.channel, a.mcs, a.lo, a.hi, a.frames, a.settle_ms, a.gap_us,
               2.0 * (a.hi - a.lo + 1) *
                   (a.settle_ms / 1e3 + a.frames * a.gap_us / 1e6));

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
  dev_cfg.rx.enable_with_tx = true;  // duplex bring-up, as linkbench-tx/maburd

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

  std::thread tx_thread([&] {
    uint16_t mac_seq = 0, seq = 0;
    uint64_t sent = 0, fail = 0;
    auto run_index = [&](int idx, uint8_t pass) {
      // drain: let the previous index's last frame clear the TX FIFO before
      // repointing the power, so no frame transmits at power N+1 while
      // stamped N
      std::this_thread::sleep_for(std::chrono::microseconds(a.gap_us));
      if (a.pwr_mode == "override") dev->SetTxPowerIndexOverride(idx);
      // "none": bring-up power stands (efuse refs + per-rate diffs live).
      std::this_thread::sleep_for(std::chrono::milliseconds(a.settle_ms));
      for (int f = 0; f < a.frames && !g_devourer_should_stop; ++f) {
        const auto payload = build_sweep_payload(
            static_cast<uint8_t>(idx), pass, seq++,
            static_cast<uint8_t>(a.mcs));
        std::vector<uint8_t> frame;
        frame.reserve(radiotap.size() + kDot11HeaderLen + payload.size());
        frame.insert(frame.end(), radiotap.begin(), radiotap.end());
        const auto hdr = build_dot11_header(mac_seq);
        mac_seq = static_cast<uint16_t>((mac_seq + 1) & 0x0FFF);
        frame.insert(frame.end(), hdr.begin(), hdr.end());
        frame.insert(frame.end(), payload.begin(), payload.end());
        TxPacketView v{frame.data(), frame.size()};
        const size_t ok = dev->send_packets(&v, 1);
        sent += ok;
        fail += 1 - ok;
        std::this_thread::sleep_for(std::chrono::microseconds(a.gap_us));
      }
      std::fprintf(stderr, "pass %u idx %2d done (sent %llu fail %llu)\n",
                   pass, idx, static_cast<unsigned long long>(sent),
                   static_cast<unsigned long long>(fail));
    };
    for (int i = a.lo; i <= a.hi && !g_devourer_should_stop; ++i)
      run_index(i, 1);
    for (int i = a.hi; i >= a.lo && !g_devourer_should_stop; --i)
      run_index(i, 2);
    std::fprintf(stderr, "done: %llu frames sent, %llu send-fail\n",
                 static_cast<unsigned long long>(sent),
                 static_cast<unsigned long long>(fail));
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
