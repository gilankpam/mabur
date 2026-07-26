#include "radio_frontend.h"

#include <libusb.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "TxMode.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "mabur/node.h"

namespace maburgs {
namespace {
constexpr size_t kDot11 = 24;
constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr uint16_t kScanPids[] = {0xa81a, 0x881a, 0x8812};

const std::vector<uint8_t>& max_range_radiotap() {
  static const std::vector<uint8_t> rt = [] {
    devourer::TxMode m;
    m.mode = devourer::TxMode::Mode::HT;
    m.ht_mcs = 0;
    m.bw_mhz = 20;
    m.ldpc = true;
    m.stbc = true;
    return devourer::build_stream_radiotap(m);
  }();
  return rt;
}

uint64_t mono_us_now() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
}  // namespace

std::vector<uint8_t> build_control_frame(uint16_t seq, const uint8_t* body,
                                         size_t len) {
  const auto& rt = max_range_radiotap();
  std::vector<uint8_t> f(rt.size() + kDot11 + len);
  std::memcpy(f.data(), rt.data(), rt.size());
  uint8_t* d = f.data() + rt.size();
  d[0] = 0x40;
  d[1] = 0x00;
  d[2] = 0x00;
  d[3] = 0x00;
  std::memset(d + 4, 0xff, 6);
  std::memcpy(d + 10, kSa, 6);
  std::memcpy(d + 16, kSa, 6);
  const uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  d[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  d[23] = static_cast<uint8_t>(seq_ctl >> 8);
  if (len) std::memcpy(d + kDot11, body, len);
  return f;
}

bool sa_canonical(const uint8_t* dot11, size_t len) {
  return len >= 16 && std::memcmp(dot11 + 10, kSa, 6) == 0;
}

// --- device management (mirrors drone/src/main.cpp bring-up) ----------------

RadioFrontend::RadioFrontend(Cfg cfg, BodyQueue& out) : cfg_(cfg), out_(out) {}
RadioFrontend::~RadioFrontend() { stop(); }

bool RadioFrontend::open_and_start() {
  if (libusb_init(&usb_ctx_) != 0) return false;
  // Find the index-th device matching vid + (pid or the scan list).
  libusb_device** list = nullptr;
  const ssize_t n = libusb_get_device_list(usb_ctx_, &list);
  int match = 0;
  libusb_device* dev = nullptr;
  for (ssize_t i = 0; i < n; ++i) {
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
    if (dd.idVendor != cfg_.usb_vid) continue;
    bool pid_ok = cfg_.usb_pid != 0 ? dd.idProduct == cfg_.usb_pid : false;
    if (cfg_.usb_pid == 0)
      for (uint16_t p : kScanPids) pid_ok = pid_ok || dd.idProduct == p;
    if (!pid_ok) continue;
    if (match++ == cfg_.index) { dev = list[i]; break; }
  }
  if (dev == nullptr || libusb_open(dev, &handle_) != 0) {
    if (list) libusb_free_device_list(list, 1);
    libusb_exit(usb_ctx_);
    usb_ctx_ = nullptr;
    handle_ = nullptr;
    return false;
  }
  libusb_free_device_list(list, 1);

  logger_ = std::make_shared<Logger>();
  int rc = devourer::claim_interface_then_reset(handle_, 0, logger_, /*do_reset=*/true, usb_lock_);
  if (rc != 0) {
    libusb_close(handle_);
    libusb_exit(usb_ctx_);
    handle_ = nullptr;
    usb_ctx_ = nullptr;
    return false;
  }

  devourer::DeviceConfig dev_cfg;
  dev_cfg.rx.enable_with_tx = true;  // TX+RX duplex: mandatory on the 8822E
  // Debug passthrough: devourer's env->config translation lives in its
  // examples/, not the library, so these two register-dump levers (used to
  // diff a live card against the vendor kernel's end state) must be wired
  // here explicitly. Inert unless the env vars are set.
  dev_cfg.debug.dump_canary = std::getenv("DEVOURER_DUMP_CANARY") != nullptr;
  dev_cfg.debug.bb_dump = std::getenv("DEVOURER_BB_DUMP") != nullptr;
  // (The 0x41e8 protect_pathb_agc knob was chased here too — exonerated:
  // the real path-B killer was the DPDT pin-mux, fixed by devourer's eFEM
  // pinmux port; see DEVOURER_DPDT_MODE in RtlJaguar3Device.)
  driver_ = std::make_unique<WiFiDriver>(logger_);
  device_ = driver_->CreateRtlDevice(handle_, usb_ctx_, usb_lock_, dev_cfg);
  if (!device_) { stop(); return false; }
  device_->InitWrite(SelectedChannel{cfg_.channel, 0, CHANNEL_WIDTH_20});
  ready_.store(true, std::memory_order_release);
  alive_.store(true, std::memory_order_release);
  rx_thread_ = std::thread([this] {
    device_->StartRxLoop([this](const Packet& pkt) { on_packet(pkt); });
    alive_.store(false, std::memory_order_release);
  });
  return true;
}

void RadioFrontend::on_packet(const Packet& pkt) {
  rx_frames_.fetch_add(1, std::memory_order_relaxed);
  if (pkt.Data.size() < kDot11 + 1) return;
  // Foreign traffic never reaches the queue: it polluted per-card EMAs and
  // the seq-loss walk (spec revision 2). CRC-failed frames pass — a corrupt
  // SA proves nothing, and they never fed EMAs/seq anyway.
  if (!pkt.RxAtrib.crc_err && !sa_canonical(pkt.Data.data(), pkt.Data.size())) {
    foreign_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  mabur::node::RxBody m;
  m.card_id = cfg_.card_id;
  m.mono_us = mono_us_now();
  m.rssi[0] = pkt.RxAtrib.rssi[0];
  m.rssi[1] = pkt.RxAtrib.rssi[1];
  m.snr[0] = pkt.RxAtrib.snr[0];
  m.snr[1] = pkt.RxAtrib.snr[1];
  m.crc_ok = !pkt.RxAtrib.crc_err;
  m.mac_seq = static_cast<uint16_t>(
      (static_cast<uint16_t>(pkt.Data[22] | (pkt.Data[23] << 8))) >> 4);
  m.body.assign(pkt.Data.begin() + kDot11, pkt.Data.end());
  out_.push(std::move(m));
}

bool RadioFrontend::send_control(const std::vector<uint8_t>& body) {
  if (!ready_.load(std::memory_order_acquire) || !device_) {
    tx_fail_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const auto frame = build_control_frame(tx_seq_, body.data(), body.size());
  tx_seq_ = static_cast<uint16_t>((tx_seq_ + 1) & 0xFFF);
  const bool ok = device_->send_packet(frame.data(), frame.size());
  (ok ? tx_frames_ : tx_fail_).fetch_add(1, std::memory_order_relaxed);
  return ok;
}

void RadioFrontend::stop() {
  if (device_ && alive_.load(std::memory_order_acquire)) device_->StopRxLoop();
  if (rx_thread_.joinable()) rx_thread_.join();
  if (device_) device_->Stop();
  device_.reset();
  driver_.reset();
  ready_.store(false, std::memory_order_release);
  alive_.store(false, std::memory_order_release);
  if (handle_) { libusb_release_interface(handle_, 0); libusb_close(handle_); handle_ = nullptr; }
  // Release the per-adapter advisory lock (claim_interface_then_reset filled
  // it) or the next open_and_start() on this same card refuses with "already
  // in use by another devourer process" — the process deadlocks against its
  // own stale lock and a replugged card can never reopen (bench 2026-07-12).
  usb_lock_.reset();
  if (usb_ctx_) { libusb_exit(usb_ctx_); usb_ctx_ = nullptr; }
}

bool RadioFrontend::ready() const { return ready_.load(std::memory_order_acquire); }
bool RadioFrontend::alive() const { return alive_.load(std::memory_order_acquire); }
uint64_t RadioFrontend::rx_frames() const { return rx_frames_.load(std::memory_order_relaxed); }
uint64_t RadioFrontend::foreign() const { return foreign_.load(std::memory_order_relaxed); }
uint64_t RadioFrontend::tx_frames() const { return tx_frames_.load(std::memory_order_relaxed); }
uint64_t RadioFrontend::tx_fail() const { return tx_fail_.load(std::memory_order_relaxed); }

}  // namespace maburgs
