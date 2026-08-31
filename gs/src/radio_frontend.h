#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "body_queue.h"
#include "logger.h"

// Forward declarations for devourer types
class WiFiDriver;
class IRtlDevice;
struct Packet;

namespace devourer {
class UsbDeviceLock;
}

namespace maburgs {

// Pure: MAX_RANGE radiotap + 24-byte dot11 probe-req header (canonical SA
// 57:42:75:05:d6:00, broadcast DA, seq<<4) + body. Mirrors drone radio_tx.cpp.
std::vector<uint8_t> build_control_frame(uint16_t seq, const uint8_t* body, size_t len);

// Pure: true when the dot11 header's SA (bytes 10..15) is the canonical
// mabur SA. Frames too short to carry an SA are not canonical.
bool sa_canonical(const uint8_t* dot11, size_t len);

// Pure: byte offset of the mabur body inside a dot11 frame, keyed on the
// frame-control type. QoS-Data (0x88, the post-A-MPDU drone wire) carries a
// 26-byte header; everything else (the legacy probe-req 0x40 wire, and any
// frame the SA filter passes) parses at the legacy 24-byte offset. Returns
// 0 when len cannot hold the header plus at least one body byte. seq_ctl
// sits at bytes 22-23 in BOTH layouts, so mac_seq extraction is unchanged.
size_t dot11_body_offset(const uint8_t* dot11, size_t len);

class RadioFrontend {
 public:
  struct Cfg {
    uint16_t usb_vid = 0x0bda;
    uint16_t usb_pid = 0;      // 0 = scan {0xa81a,0x881a,0x8812}
    int index = 0;             // ordinal among matching devices
    uint8_t channel = 149;
    uint8_t card_id = 0;
  };

  RadioFrontend(Cfg cfg, BodyQueue& out);
  ~RadioFrontend();                               // stop() if running
  bool open_and_start();                          // full bring-up; false on any failure
  void stop();                                    // StopRxLoop + join + release usb
  bool ready() const;                             // InitWrite completed
  bool alive() const;                             // RX loop thread still running
  uint64_t rx_frames() const;
  uint64_t tx_frames() const;  // control frames handed to the radio OK
  uint64_t tx_fail() const;    // send_control calls that returned false
  uint64_t foreign() const;   // CRC-clean frames dropped by the SA filter
  bool send_control(const std::vector<uint8_t>& body);  // false pre-ready/on error

 private:
  void on_packet(const Packet& pkt);

  Cfg cfg_;
  BodyQueue& out_;
  std::shared_ptr<Logger> logger_;
  libusb_context* usb_ctx_ = nullptr;
  libusb_device_handle* handle_ = nullptr;
  std::shared_ptr<WiFiDriver> driver_;
  std::shared_ptr<IRtlDevice> device_;
  std::thread rx_thread_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> alive_{false};
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> foreign_{0};
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<uint64_t> tx_fail_{0};
  uint16_t tx_seq_ = 0;
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock_;
};

}  // namespace maburgs
