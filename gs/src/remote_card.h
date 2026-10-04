#pragma once
// RemoteCard: a TP-Link CPE510 running mabur-relay (protocol v3) as a
// maburgs radio card -- FRAMEs in over UDP, RCFs out as TX messages
// (spec 2026-10-02-maburgs-remote-card §2). RelayClient holds the protocol
// state; this class gives it a socket, an RX thread, the LinkCard surface
// and the daemon's never-give-up ownership policy.
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "body_queue.h"
#include "link_card.h"
#include "own_air.h"
#include "relay_client.h"
#include "relay_transport.h"

namespace maburgs {

class RemoteCard final : public LinkCard {
 public:
  struct Cfg {
    std::string addr;          // "host[:port]", port default 8310
    uint8_t channel = 149;
    uint8_t width_mhz = 20;    // 20|40; sec derives from channel (mabur::ht40_offset)
    uint8_t card_id = 0;
  };
  using OpenFn = std::function<std::unique_ptr<RelayTransport>(const std::string&, std::string&)>;
  using NowMsFn = std::function<uint64_t()>;
  // RelayClient gives up on TUNE 2.5 s after start() when not owner; a
  // daemon must not, so the client is restarted this often while refused.
  static constexpr uint64_t kRefusedRestartMs = 5000;

#ifndef __EMSCRIPTEN__
  RemoteCard(Cfg cfg, BodyQueue& out);                                // UDP + monotonic clock
#endif
  RemoteCard(Cfg cfg, BodyQueue& out, OpenFn open, NowMsFn now_ms);   // injectable (tests)
  ~RemoteCard() override;

  // The page's view of the relay (web_main.cpp maps these to its ERROR lines;
  // maburgs never gives up and ignores it). Read under mu_ at call time.
  enum class Health { Connecting, Owned, Refused, TuneFailed, Lost, Taken };
  Health health() const;

  // LinkCard
  bool open_and_start() override;
  void stop() override;
  bool ready() const override;              // owned_and_tuned() && !lost()
  bool alive() const override;              // RX thread up, a STATUS since (re)start, and !lost()
  CardCaps caps() const override;
  void tick(uint64_t now_ms) override;
  uint64_t rx_frames() const override { return rx_frames_.load(); }
  uint64_t foreign() const override { return foreign_.load(); }
  bool send_control(const std::vector<uint8_t>& body) override;
  uint64_t tx_frames() const override { return tx_frames_.load(); }
  uint64_t tx_fail() const override { return tx_fail_.load(); }
  bool set_width(uint8_t ch, uint8_t width_mhz) override;
  uint8_t channel() const override { return channel_.load(); }
  uint8_t width() const override { return width_.load(); }
  int tuned_central() override { return -1; }
  bool can_scout() const override { return false; }
  std::optional<RelayStatsIn> relay_stats() const override;

  // ScoutRadio
  bool retune(uint8_t ch) override;
  bool retune_width(uint8_t ch, uint8_t width_mhz) override { return set_width(ch, width_mhz); }
  ScoutEnergy read_energy(bool) override { return {}; }
  ScoutEnergy read_energy_scout() override { return {}; }
  ScoutFrames frames() const override;

 private:
  static uint8_t sec_for(uint8_t ch, uint8_t width_mhz);
  void rx_loop();
  void on_datagram(const uint8_t* b, size_t n);
  void log_transition(const char* what);

  Cfg cfg_;
  BodyQueue& out_;
  OpenFn open_;
  NowMsFn now_ms_;
  std::unique_ptr<RelayTransport> t_;
  mutable std::mutex mu_;          // guards c_ (RelayClient is not thread-safe)
  RelayClient c_;
  std::thread rx_;
  std::atomic<bool> running_{false}, stop_{false};
  std::atomic<uint8_t> channel_{0}, width_{20};
  std::atomic<uint64_t> rx_frames_{0}, foreign_{0}, own_{0}, own_air_us_{0};
  std::atomic<uint64_t> tx_frames_{0}, tx_fail_{0};
  std::atomic<uint32_t> reconnects_{0};
  uint64_t last_restart_ms_ = 0;
  bool opened_once_ = false;
  uint16_t tx_seq_ = 0;            // under mu_
  OwnAirAcc own_air_;              // RX thread only
  // last logged state, to log transitions once
  enum class St { Down, Waiting, Owned, Refused, Lost } last_st_ = St::Down;
};

}  // namespace maburgs
