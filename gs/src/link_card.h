#pragma once
// LinkCard: what main.cpp drives on every radio card, USB (RadioFrontend)
// or CPE510 relay (RemoteCard). Spec 2026-10-02-maburgs-remote-card §1.
// ScoutRadio supplies retune()/retune_width()/read_energy*()/frames()/nhm.
#include <cstdint>
#include <optional>
#include <vector>

#include "relay_stats.h"
#include "scout_radio.h"
#include "sweep_types.h"

namespace maburgs {

class LinkCard : public ScoutRadio {
 public:
  // lifecycle
  virtual bool open_and_start() = 0;
  virtual void stop() = 0;
  virtual bool ready() const = 0;
  virtual bool alive() const = 0;
  virtual CardCaps caps() const = 0;
  // One call per core-loop pass. RadioFrontend: nothing to do.
  virtual void tick(uint64_t now_ms) { (void)now_ms; }
  // rx
  virtual uint64_t rx_frames() const = 0;
  virtual uint64_t foreign() const = 0;
  // tx
  virtual bool send_control(const std::vector<uint8_t>& body) = 0;
  virtual uint64_t tx_frames() const = 0;
  virtual uint64_t tx_fail() const = 0;
  // tuning; false = not applied, caller keeps its old cur_ch
  virtual bool set_width(uint8_t ch, uint8_t width_mhz) = 0;
  virtual uint8_t channel() const = 0;
  virtual uint8_t width() const = 0;
  virtual int tuned_central() = 0;   // -1 unknown
  // Energy reads exist (FA/CCA/NHM): may serve as boot or in-flight scout.
  virtual bool can_scout() const = 0;
  virtual std::optional<RelayStatsIn> relay_stats() const { return std::nullopt; }
  // Relay sweep (mabur-relay v4 SCAN, spec 2026-10-05-cpe-relay-hop §3):
  // the card surveys a channel list itself and reports later -- an async
  // burst, unlike ScoutRadio's synchronous dwells. USB cards: unsupported.
  // read_survey_window(): op-channel airtime over the span since the
  // previous call (relay SURVEY); invalid across a retune/sweep or < 100 ms.
  virtual bool can_sweep() const { return false; }
  virtual bool start_sweep(const std::vector<uint8_t>& ch, uint8_t passes, uint8_t observe_ms) { (void)ch; (void)passes; (void)observe_ms; return false; }
  virtual std::optional<SweepResult> take_sweep_result() { return std::nullopt; }
  virtual bool sweeping() const { return false; }
  virtual SurveyWindow read_survey_window() { return {}; }
};

}  // namespace maburgs
