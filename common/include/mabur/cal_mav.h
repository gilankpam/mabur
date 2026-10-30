#pragma once
// Maps mabur::cal::CalMsg (cal_proto.h) onto the mabur MAVLink dialect's
// calibration messages (60000-60005). The only MAVLink TU for calibration
// traffic is common/src/cal_mav.cpp -- see mav.h's header comment for why
// that is safe despite the generated code's per-TU channel state.
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "mabur/cal_proto.h"
#include "mabur/mav.h"

namespace mabur::cal {

// One MAVLink 2 packet's wire bytes for `m`, from (sysid, compid) on `chan`.
std::vector<uint8_t> encode(const CalMsg& m, uint8_t sysid, uint8_t compid, uint8_t chan);

// nullopt for a non-cal id (is_cal_msgid(msg.msgid) is false).
std::optional<CalMsg> decode(const mavlink_message_t& msg);

bool is_cal_msgid(uint32_t msgid);

// Splits `bytes` (whole packets, back to back) with a fresh, stateless
// parse; true iff at least one packet parsed and every packet is a cal id,
// with no trailing garbage or partial packet left over.
bool all_packets_are_cal(const uint8_t* bytes, size_t n);

// Splits `bytes` (whole packets, back to back) with the same fresh,
// stateless parse and returns every cal packet in it, decoded, in order.
// Anything else -- a non-cal id, a bad CRC, a partial tail -- is skipped.
std::vector<CalMsg> decode_all(const uint8_t* bytes, size_t n);

}  // namespace mabur::cal
