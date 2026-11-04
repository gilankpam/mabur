#pragma once
// FakeCard: a LinkCard for ChannelCore tests. Every control-plane call is
// recorded; energy/frames are programmable; the clock is shared with the
// core through FakeClock.
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "link_card.h"
#include "sweep_types.h"

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
  int refuse_ch = -1;                 // retune() to this one channel fails
  // Relay-style: retune() takes the target at once (channel() reads it, as
  // RemoteCard's does) but ready() reads false until the test re-readies
  // the card (the TUNE's ~0.2 s to STATUS).
  bool retune_deferred = false;
  std::vector<std::string> calls;
  std::vector<std::vector<uint8_t>> sent;
  uint32_t cca_per_ms_on[256] = {};   // programmable busy rate per channel
  // Decoded foreign frames per ms on a channel, booked into fr.foreign by
  // read_energy() (the boot scout's observe reads frames() around it).
  uint32_t foreign_per_ms_on[256] = {};
  maburgs::ScoutFrames fr;
  uint64_t rx = 0, foreign_n = 0, tx = 0, txf = 0;
  FakeClock* clk = nullptr;
  uint64_t last_read = 0;

  // LinkCard
  bool open_and_start() override { calls.push_back("open"); is_alive = true; is_ready = true; return true; }
  void stop() override { calls.push_back("stop"); }
  // Called on every ready() read: a test sequences another thread against
  // the core's per-card walk through it.
  std::function<void()> on_ready;
  bool ready() const override { if (on_ready) on_ready(); return is_ready; }
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
    if (!retune_ok || c == refuse_ch) return false;
    ch = c;
    if (retune_deferred) is_ready = false;
    return true;
  }
  maburgs::ScoutEnergy read_energy(bool with_nhm) override {
    maburgs::ScoutEnergy e; e.fa_valid = !relay;
    const uint64_t now = clk ? clk->ms : 0;
    e.cca_ofdm = static_cast<uint32_t>((now - last_read) * cca_per_ms_on[ch]);
    fr.foreign += (now - last_read) * foreign_per_ms_on[ch];
    last_read = now;
    if (with_nhm && !relay) { e.floor_valid = true; e.floor_dbm = -95; }
    return e;
  }
  maburgs::ScoutEnergy read_energy_scout() override { maburgs::ScoutEnergy e; e.fa_valid = true; e.fa_ofdm = fa_next; fa_next = 0; return e; }
  maburgs::ScoutFrames frames() const override { return fr; }
  // Relay sweep / survey programmability.
  bool sweep_ok = true;
  std::vector<std::vector<uint8_t>> sweeps;         // channel lists passed to start_sweep
  std::optional<maburgs::SweepResult> pending_result;   // returned once by take_sweep_result
  bool sweep_in_flight = false;
  maburgs::SurveyWindow survey_win;                 // returned by read_survey_window
  uint32_t fa_next = 0;                             // returned (once) by read_energy_scout's fa_ofdm
  // NHM busy window programmability (ScoutRadio defaults are "unsupported").
  bool nhm_arm_ok = false;                          // arm_nhm_busy() result
  maburgs::NhmBusyRead nhm_read;                    // returned by read_nhm_busy()
  // Air that reads nhm_read's buckets under whatever period was last armed:
  // the op-window sampler and a scout dwell (each arms its own period) both
  // read it. Off, nhm_read.period decides which of them it is valid for.
  bool nhm_any_period = false;
  uint16_t nhm_armed_period = 0;
  // Per-channel NHM busy % override, read at whatever `ch` the card is
  // tuned to when the scout dwells on it: -1 (default, every entry) falls
  // back to the single nhm_arm_ok/nhm_read reading above, which is what
  // op's own (unchanging) channel uses.
  std::array<int, 256> nhm_busy_pct_on;
  FakeCard() { nhm_busy_pct_on.fill(-1); }
  bool arm_nhm_busy(uint16_t period_4us) override {
    calls.push_back("arm_nhm " + std::to_string(period_4us));
    nhm_armed_period = period_4us;
    if (nhm_busy_pct_on[ch] >= 0) return true;
    return nhm_arm_ok;
  }
  maburgs::NhmBusyRead read_nhm_busy() override {
    calls.push_back("read_nhm");
    if (nhm_busy_pct_on[ch] >= 0) {
      maburgs::NhmBusyRead r;
      r.valid = true;
      r.period = nhm_armed_period;
      r.buckets[11] = static_cast<uint8_t>(nhm_busy_pct_on[ch]);
      r.buckets[0] = static_cast<uint8_t>(100 - nhm_busy_pct_on[ch]);
      return r;
    }
    maburgs::NhmBusyRead r = nhm_read;
    if (nhm_any_period) r.period = nhm_armed_period;
    return r;
  }
  bool can_sweep() const override { return relay && sweep_ok; }
  bool start_sweep(const std::vector<uint8_t>& ch, uint8_t, uint8_t) override {
    if (!can_sweep() || !is_ready) return false;   // RemoteCard: running + owned_and_tuned
    sweeps.push_back(ch); sweep_in_flight = true; return true;
  }
  std::optional<maburgs::SweepResult> take_sweep_result() override {
    if (!pending_result) return std::nullopt;
    auto r = std::move(pending_result); pending_result.reset(); sweep_in_flight = false; return r;
  }
  bool sweeping() const override { return sweep_in_flight; }
  maburgs::SurveyWindow read_survey_window() override {
    calls.push_back("survey");
    return relay ? survey_win : maburgs::SurveyWindow{};
  }
};
