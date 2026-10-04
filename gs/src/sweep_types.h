#pragma once
// A relay sweep (mabur-relay v4 SCAN_RESULT) decoded, and one verdict window
// of the relay's SURVEY op-channel airtime. Shared by relay_wire,
// RelayClient/RemoteCard and ChannelCore.
#include <cstdint>
#include <vector>

namespace maburgs {

struct SweepEntry {
  uint8_t ch = 0, pass = 0;
  bool valid = false;
  uint16_t active_ms = 0, busy_ms = 0, rx_ms = 0, foreign = 0, ofdm_err = 0;
};

struct SweepResult {
  uint16_t scan_id = 0;
  uint8_t status = 0, back_channel = 0, back_sec = 0;
  std::vector<SweepEntry> entries;
};

struct SurveyWindow {
  bool valid = false;
  double busy_pct = 0, rx_pct = 0;
};

}  // namespace maburgs
