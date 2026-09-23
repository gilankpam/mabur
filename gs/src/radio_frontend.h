#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "body_queue.h"
#include "card_scan.h"
#include "logger.h"
#include "scout_radio.h"

// Forward declarations for devourer types
class WiFiDriver;
class IRtlDevice;
struct Packet;

namespace devourer {
class UsbDeviceLock;
}

namespace maburgs {

// Pure: MAX_RANGE radiotap + 24-byte dot11 probe-req header (canonical SA
// 57:42:75:05:d6:00, broadcast DA, seq<<4) + body. Builds the GS's own
// uplink 0x40 control frame. The drone's video downlink switched to QoS-Data
// (A-MPDU), but the uplink RCF remains probe-req (unchanged by the wire change).
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

class RadioFrontend : public ScoutRadio {
 public:
  struct Cfg {
    uint16_t usb_vid = 0x0bda;
    uint16_t usb_pid = 0;      // 0 = scan {0xa81a,0x881a,0x8812}
    int index = 0;             // ordinal among matching devices
    uint8_t channel = 149;
    // RX width: 20, or 40 = HT40+ (channel is the primary/lower 20). Only the
    // linkbench HT40 sweep sets 40 (docs/bw40-sweep-findings-2026-09-23.md).
    uint8_t width_mhz = 20;
    uint8_t card_id = 0;
    // Set by the startup scan (card_scan.h): open the device at this
    // physical port instead of the index-th VID/PID match. Survives the
    // card re-enumerating at a new bus address, which the ordinal does not.
    bool by_port = false;
    ScannedCard port;
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

  // ScoutRadio interface: the scout thread's control plane on this card.
  bool retune(uint8_t ch) override;                 // FastRetune; false pre-ready
  ScoutEnergy read_energy(bool with_nhm) override;  // GetRxEnergy -> ScoutEnergy
  ScoutEnergy read_energy_scout() override;         // GetRxEnergyScout -> ScoutEnergy
  ScoutFrames frames() const override {
    return ScoutFrames{own_.load(std::memory_order_relaxed), foreign_.load(std::memory_order_relaxed)};
  }
  CardCaps caps() const { return caps_; }            // filled in open_and_start() after InitWrite
  uint8_t channel() const { return channel_.load(std::memory_order_acquire); }  // last channel handed to InitWrite/retune

 private:
  void on_packet(const Packet& pkt);

  // rx_pace gauge (usb-feed probe 2026-09-01, dq-spike findings §16): per
  // accepted body, the inter-arrival delta on TWO clocks — the chip's RX TSF
  // (RxAtrib.tsfl, µs at the antenna, upstream of ALL host processing) and
  // this host's mono_us stamp. Whichever clock carries the ~360 µs/body
  // spacing names the pace-setter (air/drone vs GS host). Pump-thread-owned
  // (one RadioFrontend per card), reported to stderr every 5 s. Deltas
  // > 5 ms are inter-burst gaps, counted but not folded into the hists.
  static constexpr int kPaceBuckets = 9;
  uint64_t rp_last_mono_ = 0;
  uint32_t rp_last_tsfl_ = 0;
  uint64_t rp_n_ = 0, rp_gaps_ = 0;
  uint64_t rp_tsfl_sum_ = 0, rp_host_sum_ = 0;
  uint64_t rp_tsfl_hist_[kPaceBuckets] = {};
  uint64_t rp_host_hist_[kPaceBuckets] = {};
  uint64_t rp_last_report_us_ = 0;

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
  std::atomic<uint64_t> own_{0};
  std::atomic<uint8_t> channel_{0};
  // What on_packet() stamps RxBody::rx_channel with: the channel this card
  // is KNOWN to have been tuned to when the frame arrived. Distinct from
  // channel_ (the commanded position, published after FastRetune returns)
  // because it is cleared to 0 BEFORE the retune starts, so every frame
  // delivered across the retune reads "unknown" rather than being
  // attributed to either side of it. Nothing but the stamp reads it, so
  // channel()'s existing readers are unaffected.
  std::atomic<uint8_t> rx_channel_{0};
  CardCaps caps_;
};

}  // namespace maburgs
