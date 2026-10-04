#pragma once
// mabur-relay protocol v4 (CPE510 remote radio): the byte contract lives in
// ../mabur-openwrt docs/mabur-relay-protocol.md. Little-endian, byte by
// byte. Pure: no sockets, no clock.
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sweep_types.h"

namespace maburgs::relay {

constexpr uint16_t kMagic = 0x524D;
constexpr uint8_t kVer = 4;
enum Type : uint8_t { kFrame = 1, kHello = 2, kTune = 3, kStatus = 4, kTx = 5,
                      kSurvey = 6, kScan = 7, kScanResult = 8 };
constexpr size_t kHdrLen = 4, kFrameHdrLen = 20, kTuneLen = 8, kStatusLen = 51, kTxHdrLen = 6,
                 kSurveyLen = 32, kScanHdrLen = 9, kScanResultHdrLen = 10, kScanEntryLen = 13;
constexpr uint8_t kFlagBadFcs = 0x01, kFlagDropped = 0x02, kFlagPhyValid = 0x04;
constexpr uint8_t kFlagSgi = 0x08, kFlagStbc = 0x10;
constexpr uint8_t kTxLdpc = 0x01, kTxStbc = 0x02, kTxSgi = 0x04, kTxBw40 = 0x08;
constexpr uint8_t kMcsNone = 0xFF;
constexpr int8_t kDbmAbsent = -128;

struct FrameMeta {
  uint32_t seq = 0;
  uint8_t rx_channel = 0, sec = 0, flags = 0, mcs = kMcsNone;
  int8_t rssi[2] = {kDbmAbsent, kDbmAbsent}, noise[2] = {kDbmAbsent, kDbmAbsent};
  uint32_t tsf_lo = 0;
};

struct Status {
  uint16_t tune_id = 0;
  uint8_t state = 0, channel = 0, sec = 0, owner = 0, you_own = 0;
  uint32_t rx = 0, fwd = 0, foreign = 0, bad_fcs = 0, your_drops = 0, uptime_s = 0,
           tx = 0, tx_fail = 0, tx_refused = 0;
  uint32_t tx_scan_drop = 0;
};

struct Survey {
  uint8_t channel = 0, sec = 0;
  uint16_t gen = 0;
  uint32_t active_ms = 0, busy_ms = 0, rx_ms = 0, tx_ms = 0, ofdm_err = 0, foreign = 0;
};

std::vector<uint8_t> pack_hello();
std::vector<uint8_t> pack_tune(uint16_t tune_id, uint8_t channel, uint8_t sec);
std::vector<uint8_t> pack_tx(uint8_t mcs, uint8_t flags, const uint8_t* dot11, size_t len);
std::vector<uint8_t> pack_scan(uint16_t scan_id, uint8_t passes, uint8_t observe_ms,
                               const std::vector<uint8_t>& ch);
int msg_type(const uint8_t* b, size_t n);   // Type, or -1 (short / bad magic / ver != 4)
bool parse_frame(const uint8_t* b, size_t n, FrameMeta& m, const uint8_t*& dot11, size_t& dot11_len);
bool parse_status(const uint8_t* b, size_t n, Status& s);
bool parse_survey(const uint8_t* b, size_t n, Survey& s);
bool parse_scan_result(const uint8_t* b, size_t n, SweepResult& r);

}  // namespace maburgs::relay
