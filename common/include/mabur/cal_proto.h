#pragma once
// Plain-struct calibration protocol: what the drone's CalSweep and the GS's
// CalSession exchange, independent of how it travels on the wire. mav.h
// (cal_mav.h/cpp) is the only place that maps these onto the mabur MAVLink
// dialect -- these structs stay free of any MAVLink include so both
// engines, and their tests, can depend on this header without pulling the
// generated dialect in.
#include <array>
#include <cstdint>
#include <string>
#include <variant>

#include "mabur/cal_wire.h"

namespace mabur::cal {

// One sweep of a run: a kind (Wall or Capacity) at a bandwidth (20 or 40),
// and for Wall the channel to measure.
struct SweepSpec {
  Kind kind;
  uint8_t bw;
  uint8_t channel = 0;  // Wall only; 0 = the link's channel
};

// Ground -> drone: run a sweep. Wall measures the TX walls of `channel` at
// `bw` on the drone itself; Capacity dwells on each rate in mcs_mask.
struct SweepCmd {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  Kind kind = Kind::Capacity;
  uint8_t bw = 20;
  uint8_t channel = 0;       // Wall: the channel to measure
  uint16_t settle_ms = 300;  // Capacity
  uint16_t dwell_ms = 2000;  // Capacity
  uint8_t mcs_mask = 0xFF;   // Capacity
};

// BadChannel: a Wall channel outside radio.channels. BadWidth: a 40 MHz
// Wall on a radio tuned to 20. Unsupported: a chip that cannot measure.
enum class AckResult : uint8_t {
  Accepted = 0,
  AnchorUnreadable = 1,
  Busy = 2,
  Unsupported = 3,
  BadChannel = 4,
  BadWidth = 5
};
enum class WallResult : uint8_t { Ok = 0, Unparked = 1, TooFewPoints = 2, FloorTooHigh = 3, NotSaturated = 4, Deadline = 5, DeviceError = 6, Aborted = 7 };

// Drone -> ground: accept/refuse a SweepCmd, and how soon it starts.
struct SweepAck {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  AckResult result = AckResult::Accepted;
  uint16_t start_in_ms = 200;
  uint8_t flags = 0;
};

// Drone -> ground: a finished sweep's results. Capacity fills offered and
// blocked_pct; Wall fills wall_result and everything after it. walls are
// relative to `anchor`, the chip's TXAGC reference on that channel; the
// floor EVM is in 0.1 dB.
struct SweepDone {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  std::array<uint32_t, 8> offered{};
  std::array<uint8_t, 8> blocked_pct{};
  WallResult wall_result = WallResult::Ok;
  uint8_t channel = 0, bw = 0;
  std::array<int8_t, 8> walls{};
  int16_t floor_evm_x10 = 0;
  uint8_t anchor = 0;
};

// Ground -> drone: the efficiency table a capacity sweep produced, for the
// drone to adopt at `bw` (eff_x1000 0 = leave). `seq` names that sweep: a
// run may apply twice at one width, so (nonce, bw) alone cannot tell the
// second apply from a repeat of the first.
struct ApplyCmd {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  uint8_t bw = 20;
  std::array<uint16_t, 8> eff_x1000{};
};

// Drone -> ground: whether it adopted an ApplyCmd; nonce, seq and bw echo
// it. reason says why on a refusal; empty on success. At most 49 chars
// (the wire field is char[50], NUL-terminated).
struct Applied {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  uint8_t bw = 20;
  bool ok = false;
  std::string reason;
};

// Ground -> drone: cancel the in-progress sweep identified by nonce.
struct Abort {
  uint32_t nonce = 0;
};

using CalMsg = std::variant<SweepCmd, SweepAck, SweepDone, ApplyCmd, Applied, Abort>;

constexpr uint32_t kCalMsgIdLo = 60000, kCalMsgIdHi = 60005;

}  // namespace mabur::cal
