#pragma once
// FakeCard: a LinkCard for ChannelCore tests. Every control-plane call is
// recorded; energy/frames are programmable; the clock is shared with the
// core through FakeClock.
#include <cstdint>
#include <string>
#include <vector>

#include "link_card.h"

struct FakeClock {
  uint64_t ms = 1000;
  uint64_t now_ms() const { return ms; }
  uint64_t now_us() const { return ms * 1000; }
  void sleep(int d) { if (d > 0) ms += static_cast<uint64_t>(d); }
};

struct FakeCard : maburgs::LinkCard {
  uint8_t ch = 0, width_mhz = 20;
  bool is_ready = true, is_alive = true, scout_ok = true, relay = false;
  bool retune_ok = true;
  // Relay-style: retune() takes the target at once (channel() reads it, as
  // RemoteCard's does) but ready() reads false until the test re-readies
  // the card (the TUNE's ~0.2 s to STATUS).
  bool retune_deferred = false;
  std::vector<std::string> calls;
  std::vector<std::vector<uint8_t>> sent;
  uint32_t cca_per_ms_on[256] = {};   // programmable busy rate per channel
  maburgs::ScoutFrames fr;
  uint64_t rx = 0, foreign_n = 0, tx = 0, txf = 0;
  FakeClock* clk = nullptr;
  uint64_t last_read = 0;

  // LinkCard
  bool open_and_start() override { calls.push_back("open"); is_alive = true; is_ready = true; return true; }
  void stop() override { calls.push_back("stop"); }
  bool ready() const override { return is_ready; }
  bool alive() const override { return is_alive; }
  maburgs::CardCaps caps() const override {
    maburgs::CardCaps c; c.valid = true; c.chip = relay ? "ath9k" : "RTL8822E"; c.gen = relay ? "CPE510" : "jaguar3";
    c.tx_chains = 2; c.rx_chains = 2; c.fast_retune = !relay;
    c.fa_ok = c.igi_ok = c.nhm_ok = !relay; c.snr_ok = !relay; return c;
  }
  uint64_t rx_frames() const override { return rx; }
  uint64_t foreign() const override { return foreign_n; }
  bool send_control(const std::vector<uint8_t>& body) override { sent.push_back(body); ++tx; return true; }
  uint64_t tx_frames() const override { return tx; }
  uint64_t tx_fail() const override { return txf; }
  bool set_width(uint8_t c, uint8_t w) override {
    calls.push_back("set_width " + std::to_string(c) + "/" + std::to_string(w));
    ch = c; width_mhz = w; return true;
  }
  uint8_t channel() const override { return ch; }
  uint8_t width() const override { return width_mhz; }
  int tuned_central() override { return -1; }
  bool can_scout() const override { return scout_ok && !relay; }
  // ScoutRadio
  bool retune(uint8_t c) override {
    calls.push_back("retune " + std::to_string(c));
    if (!retune_ok) return false;
    ch = c;
    if (retune_deferred) is_ready = false;
    return true;
  }
  bool retune_width(uint8_t c, uint8_t w) override {
    calls.push_back("retune_width " + std::to_string(c) + "/" + std::to_string(w));
    ch = c; width_mhz = w; return true;
  }
  maburgs::ScoutEnergy read_energy(bool with_nhm) override {
    maburgs::ScoutEnergy e; e.fa_valid = !relay;
    const uint64_t now = clk ? clk->ms : 0;
    e.cca_ofdm = static_cast<uint32_t>((now - last_read) * cca_per_ms_on[ch]);
    last_read = now;
    if (with_nhm && !relay) { e.floor_valid = true; e.floor_dbm = -95; }
    return e;
  }
  maburgs::ScoutEnergy read_energy_scout() override { maburgs::ScoutEnergy e; e.fa_valid = !relay; return e; }
  maburgs::ScoutFrames frames() const override { return fr; }
};
