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

// One cell the drone is asked to sweep: a kind (Power or Capacity) at a
// bandwidth (20 or 40).
struct SweepSpec {
  Kind kind;
  uint8_t bw;
};

// Ground -> drone: run a sweep. idx_lo/idx_hi bound the rel power-index
// range (Power/Verify); mcs_mask selects which rates a Capacity sweep
// probes.
struct SweepCmd {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  Kind kind = Kind::Power;
  uint8_t bw = 20;
  uint16_t frames_per_cell = 20;
  uint16_t settle_ms = 5;
  uint16_t gap_us = 1000;
  uint16_t dwell_ms = 2000;
  int8_t idx_lo = -41;
  int8_t idx_hi = 63;
  uint8_t mcs_mask = 0xFF;
};

enum class AckResult : uint8_t { Accepted = 0, AnchorUnreadable = 1, Busy = 2, Unsupported = 3 };

// Drone -> ground: accept/refuse a SweepCmd, and how soon it starts.
struct SweepAck {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  AckResult result = AckResult::Accepted;
  uint16_t start_in_ms = 200;
  uint8_t flags = 0;
};

// Drone -> ground: a finished sweep's per-cell results, up to 8 cells.
struct SweepDone {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  std::array<uint32_t, 8> offered{};
  std::array<uint8_t, 8> blocked_pct{};
};

constexpr int8_t kWallLeave = -128;  // walls[i]: leave the config entry; eff_x1000 0 = leave

// Ground -> drone: the walls and/or efficiency table a sweep produced, for
// the drone to adopt at `bw`. `seq` names that sweep: a run applies twice
// at one width (power, then capacity), so (nonce, bw) alone cannot tell
// the second apply from a repeat of the first.
struct ApplyCmd {
  uint32_t nonce = 0;
  uint8_t seq = 0;
  uint8_t bw = 20;
  bool has_walls = false;
  bool has_eff = false;
  std::array<int8_t, 8> walls{kWallLeave, kWallLeave, kWallLeave, kWallLeave,
                              kWallLeave, kWallLeave, kWallLeave, kWallLeave};
  int8_t legacy = kWallLeave;
  std::array<uint16_t, 8> eff_x1000{};
};

// Drone -> ground: whether it adopted an ApplyCmd; nonce, seq and bw echo
// it. reason is empty on success; at most 49 chars (the wire field is
// char[50], NUL-terminated).
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
