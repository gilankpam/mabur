// nhm-reset-probe -- does a 0x1eb4[25] counter reset clear an in-flight
// ArmNhmBusy window on Jaguar3?
//
// Brings one card up the way maburgs does (InitWrite + an RX loop on its own
// thread, so the coex thread runs its ~2 s phydm tick), then arms busy
// windows back to back and prints one line per window. Pair it with a
// square-wave jam (linkbench-tx --duty ON:OFF) whose period equals the
// window: an untouched window always spans one on- and one off-phase and
// reads ~on/(on+off) busy whatever its phase, while a window whose counters
// were cleared part-way reads only its tail and scatters towards 0 or 100.
//
//   --wipe-at-ms X   call GetRxEnergy(false) X ms after each arm (the same
//                    0x1eb4[25] pulse the coex tick and the scout read issue)
//   (no --wipe-at-ms: only the device's own ~2 s tick can touch a window)
//
// Output (stdout): w t_ms ready_ms valid sum busy_pct duration b0..b11
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "NoiseFloorMath.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "IRtlRadio.h"
#include "WiFiDriver.h"
#include "logger.h"
#include <libusb.h>

namespace {

struct Args {
  int channel = 144;
  int period_ms = 200;
  int windows = 200;
  int wipe_at_ms = 0;
  int busy_dbm = -83;
  uint16_t usb_pid = 0xa81a;
};

bool parse_args(int argc, char** argv, Args* a) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](int* out) {
      if (i + 1 >= argc) return false;
      *out = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
      return true;
    };
    int v = 0;
    if (k == "--channel") { if (!next(&a->channel)) return false; }
    else if (k == "--period-ms") { if (!next(&a->period_ms)) return false; }
    else if (k == "--windows") { if (!next(&a->windows)) return false; }
    else if (k == "--wipe-at-ms") { if (!next(&a->wipe_at_ms)) return false; }
    else if (k == "--busy-dbm") { if (!next(&a->busy_dbm)) return false; }
    else if (k == "--usb-pid") { if (!next(&v)) return false; a->usb_pid = static_cast<uint16_t>(v); }
    else return false;
  }
  return a->period_ms > 0 && a->period_ms <= 262 && a->windows > 0 &&
         a->wipe_at_ms >= 0 && a->wipe_at_ms < a->period_ms;
}

double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0).count();
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse_args(argc, argv, &a)) {
    std::fprintf(stderr,
                 "usage: %s [--channel 144] [--period-ms 200 (<=262)] "
                 "[--windows 200] [--wipe-at-ms 0] [--busy-dbm -83] "
                 "[--usb-pid 0xa81a]\n", argv[0]);
    return 2;
  }
  int first = -1;
  for (int i = 0; i < 11; ++i)
    if (devourer::nf::kNhmAbsThDbm[i] == a.busy_dbm) first = i + 1;
  if (first < 0) {
    std::fprintf(stderr, "error: --busy-dbm must be an NHM threshold edge\n");
    return 2;
  }

  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Warn);
  logger->events().disable();

  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, 0x0bda, a.usb_pid);
  if (!h) {
    std::fprintf(stderr, "error: no 0bda:%04x\n", a.usb_pid);
    return 1;
  }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lock) != 0) {
    std::fprintf(stderr, "error: claim failed\n");
    return 1;
  }
  devourer::DeviceConfig cfg;
  cfg.rx.enable_with_tx = true;  // the maburgs bring-up: InitWrite + RX loop
  cfg.tuning.disable_cca = false;
  cfg.usb.rx_zerocopy = false;
  WiFiDriver drv{logger};
  std::unique_ptr<IRtlRadio> dev;
  if (auto radio = drv.CreateRadio(h, ctx, lock, cfg);
      radio && dynamic_cast<IRtlRadio*>(radio.get()))
    dev.reset(static_cast<IRtlRadio*>(radio.release()));
  if (!dev) {
    std::fprintf(stderr, "error: CreateRadio failed (or not a Realtek radio)\n");
    return 1;
  }
  dev->InitWrite(SelectedChannel{static_cast<uint8_t>(a.channel), 0, CHANNEL_WIDTH_20});
  std::thread rx([&] { dev->StartRxLoop([](const Packet&) {}); });
  std::this_thread::sleep_for(std::chrono::seconds(1));

  const uint16_t period = static_cast<uint16_t>(a.period_ms * 250);
  const auto t0 = std::chrono::steady_clock::now();
  std::printf("# channel %d period %d ms wipe_at %d ms busy_dbm %d\n", a.channel,
              a.period_ms, a.wipe_at_ms, a.busy_dbm);
  std::printf("w t_ms ready_ms valid sum busy_pct duration b0 b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 b11\n");
  for (int w = 0; w < a.windows && !g_devourer_should_stop; ++w) {
    const auto ta = std::chrono::steady_clock::now();
    if (!dev->ArmNhmBusy(period)) {
      std::fprintf(stderr, "error: ArmNhmBusy refused\n");
      break;
    }
    if (a.wipe_at_ms > 0) {
      std::this_thread::sleep_until(ta + std::chrono::milliseconds(a.wipe_at_ms));
      (void)dev->GetRxEnergy(false);
    }
    std::this_thread::sleep_until(ta + std::chrono::milliseconds(a.period_ms));
    NhmBusy b{};
    double ready_ms = -1;
    while (ms_since(ta) < 3.0 * a.period_ms) {
      b = dev->ReadNhmBusy();
      if (b.valid) { ready_ms = ms_since(ta); break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    unsigned sum = 0, above = 0;
    for (int i = 0; i < 12; ++i) {
      sum += b.buckets[i];
      if (i >= first) above += b.buckets[i];
    }
    std::printf("%d %.1f %.1f %d %u %.1f %u", w, std::chrono::duration<double, std::milli>(ta - t0).count(),
                ready_ms, b.valid ? 1 : 0, sum, sum ? 100.0 * above / sum : -1.0,
                static_cast<unsigned>(b.duration));
    for (int i = 0; i < 12; ++i) std::printf(" %u", static_cast<unsigned>(b.buckets[i]));
    std::printf("\n");
    std::fflush(stdout);
  }
  dev->StopRxLoop();
  rx.join();
  dev->Stop();
  dev.reset();
  libusb_release_interface(h, 0);
  libusb_close(h);
  libusb_exit(ctx);
  return 0;
}
