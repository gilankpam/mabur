#pragma once
#include <cstdint>
#include <optional>
#include <vector>
#include "mabur/msp_dp.h"

namespace mabur {

// MSP v1 command 101, MSP_STATUS (Betaflight and iNav). Payload layout:
//   u16 cycleTime, u16 i2cErrors, u16 sensors, u32 flightModeFlags,
//   u8 profile, ... (27 bytes on the bench FC). BOXARM is bit 0 of
//   flightModeFlags in both firmwares. Only the flags word is read.
constexpr uint8_t MSP_CMD_STATUS = 101;

// True/false = the FC's arm state from a well-formed MSP_STATUS reply;
// nullopt for any other command or a payload too short to carry the
// flags word (< 11 bytes). Never throws.
std::optional<bool> msp_status_armed(const MspMessage& m);

// Appends the 6-byte MSP_STATUS request ($M< size 0, cmd 101, checksum)
// that makes the FC answer with the frame above. The OSD UART is full
// duplex; the DisplayPort push keeps streaming around it (spike 2026-09-19).
void msp_append_status_request(std::vector<uint8_t>& out);

}  // namespace mabur
