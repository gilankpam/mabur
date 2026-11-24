#pragma once
// Maps the update lane's messages onto the mabur MAVLink dialect (ids
// 60006-60009) and the standard FILE_TRANSFER_PROTOCOL. The only MAVLink
// TU for update-lane traffic is common/src/ota_mav.cpp -- see mav.h's
// header comment for why that is safe despite per-TU channel state.
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "mabur/mav.h"
#include "mabur/sha256.h"

namespace mabur::ota {

enum class Cmd : uint8_t { kStatus = 0, kStart = 1, kAbort = 2 };
enum class State : uint8_t { kIdle = 0, kReceiving = 1, kTrial = 2, kCommitted = 3, kRolledBack = 4, kRefused = 5 };
enum class Reason : uint8_t { kNone = 0, kExited = 1, kNoLink = 2, kNoVideo = 3, kBadHash = 4,
                              kWrongBoard = 5, kBadConfig = 6, kBusy = 7, kNoFile = 8 };

constexpr size_t kFtpDataMax = 239;

struct IdentReq {};
struct Ident { std::string version, board, chip; };  // cut to 31/47/15 chars on encode
struct UpdateCmd { Cmd cmd = Cmd::kStatus; Sha256 sha256{}; };
struct Status { State state = State::kIdle; Reason reason = Reason::kNone; uint8_t trial_s = 0; };
// FILE_TRANSFER_PROTOCOL payload, MAVLink FTP layout:
// u16 seq | u8 session | u8 opcode | u8 size | u8 req_opcode | u8 burst_complete | u8 pad |
// u32 offset | data[239]. `size` is data.size() on encode and decode.
struct Ftp {
  uint16_t seq = 0;
  uint8_t session = 0, opcode = 0, size = 0, req_opcode = 0, burst_complete = 0;
  uint32_t offset = 0;
  std::vector<uint8_t> data;  // <= kFtpDataMax
};
using OtaMsg = std::variant<IdentReq, Ident, UpdateCmd, Status, Ftp>;

// One MAVLink 2 packet's wire bytes for `m`, from (sysid, compid) on `chan`.
// FTP goes out with target network/system/component 0.
std::vector<uint8_t> encode(const OtaMsg& m, uint8_t sysid, uint8_t compid, uint8_t chan);

// nullopt for anything but the five update-lane messages.
std::optional<OtaMsg> decode(const mavlink_message_t& msg);

// GS -> drone: IDENT_REQ, UPDATE, FILE_TRANSFER_PROTOCOL.
bool is_uplink_lane_msgid(uint32_t msgid);

// Stateless split of whole back-to-back packets; true iff at least one
// packet parsed, every one is an uplink-lane id, and nothing partial is left.
bool all_packets_uplink_lane(const uint8_t* bytes, size_t n);

// Every update-lane packet in `bytes`, decoded, in order; others skipped.
std::vector<OtaMsg> decode_all(const uint8_t* bytes, size_t n);

const char* reason_name(Reason r);  // "exited", "no_link", ...; "" for kNone
std::optional<Reason> reason_from_name(std::string_view s);

}  // namespace mabur::ota
