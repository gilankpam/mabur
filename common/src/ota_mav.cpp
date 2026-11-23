#include "mabur/ota_mav.h"

#include <algorithm>
#include <cstring>
#include <type_traits>

namespace mabur::ota {

namespace {

// Cut to fit a wire char[N] (N-1 chars + NUL), NUL padded so a stale tail
// can never read back as part of a shorter string.
void pack_str(const std::string& s, char* out, size_t n) {
  std::memset(out, 0, n);
  std::memcpy(out, s.data(), std::min(s.size(), n - 1));
}

std::string unpack_str(const char* p, size_t n) { return std::string(p, strnlen(p, n)); }

constexpr const char* kReasonNames[] = {"",          "exited",       "no_link",    "no_video",
                                        "bad_hash",  "wrong_board",  "bad_config", "busy",
                                        "no_file"};

}  // namespace

std::vector<uint8_t> encode(const OtaMsg& m, uint8_t sysid, uint8_t compid, uint8_t chan) {
  mavlink_message_t msg;
  std::visit(
      [&](auto&& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, IdentReq>) {
          mavlink_msg_mabur_ident_req_pack_chan(sysid, compid, chan, &msg, 0);
        } else if constexpr (std::is_same_v<T, Ident>) {
          char ver[32], board[48], chip[16];
          pack_str(v.version, ver, sizeof ver);
          pack_str(v.board, board, sizeof board);
          pack_str(v.chip, chip, sizeof chip);
          mavlink_msg_mabur_ident_pack_chan(sysid, compid, chan, &msg, ver, board, chip);
        } else if constexpr (std::is_same_v<T, UpdateCmd>) {
          mavlink_msg_mabur_update_pack_chan(sysid, compid, chan, &msg, static_cast<uint8_t>(v.cmd),
                                             v.sha256.data());
        } else if constexpr (std::is_same_v<T, Status>) {
          mavlink_msg_mabur_update_status_pack_chan(sysid, compid, chan, &msg,
                                                    static_cast<uint8_t>(v.state),
                                                    static_cast<uint8_t>(v.reason), v.trial_s);
        } else if constexpr (std::is_same_v<T, Ftp>) {
          uint8_t p[251] = {};
          const size_t n = std::min(v.data.size(), kFtpDataMax);
          p[0] = v.seq & 0xFF;
          p[1] = v.seq >> 8;
          p[2] = v.session;
          p[3] = v.opcode;
          p[4] = static_cast<uint8_t>(n);
          p[5] = v.req_opcode;
          p[6] = v.burst_complete;
          for (int i = 0; i < 4; ++i) p[8 + i] = (v.offset >> (8 * i)) & 0xFF;
          if (n) std::memcpy(p + 12, v.data.data(), n);
          mavlink_msg_file_transfer_protocol_pack_chan(sysid, compid, chan, &msg, 0, 0, 0, p);
        }
      },
      m);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  return std::vector<uint8_t>(buf, buf + n);
}

bool is_uplink_lane_msgid(uint32_t id) {
  return id == MAVLINK_MSG_ID_MABUR_IDENT_REQ || id == MAVLINK_MSG_ID_MABUR_UPDATE ||
         id == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL;
}

std::optional<OtaMsg> decode(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_MABUR_IDENT_REQ:
      return OtaMsg{IdentReq{}};
    case MAVLINK_MSG_ID_MABUR_IDENT: {
      mavlink_mabur_ident_t d;
      mavlink_msg_mabur_ident_decode(&msg, &d);
      return OtaMsg{Ident{unpack_str(d.version, sizeof d.version),
                          unpack_str(d.board, sizeof d.board), unpack_str(d.chip, sizeof d.chip)}};
    }
    case MAVLINK_MSG_ID_MABUR_UPDATE: {
      mavlink_mabur_update_t d;
      mavlink_msg_mabur_update_decode(&msg, &d);
      UpdateCmd u;
      u.cmd = static_cast<Cmd>(d.cmd);
      std::copy(d.sha256, d.sha256 + 32, u.sha256.begin());
      return OtaMsg{u};
    }
    case MAVLINK_MSG_ID_MABUR_UPDATE_STATUS: {
      mavlink_mabur_update_status_t d;
      mavlink_msg_mabur_update_status_decode(&msg, &d);
      return OtaMsg{Status{static_cast<State>(d.state), static_cast<Reason>(d.reason), d.trial_s}};
    }
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {
      mavlink_file_transfer_protocol_t d;
      mavlink_msg_file_transfer_protocol_decode(&msg, &d);
      const uint8_t* p = d.payload;
      Ftp f;
      f.seq = static_cast<uint16_t>(p[0] | (p[1] << 8));
      f.session = p[2];
      f.opcode = p[3];
      f.req_opcode = p[5];
      f.burst_complete = p[6];
      f.offset = p[8] | (p[9] << 8) | (p[10] << 16) | (static_cast<uint32_t>(p[11]) << 24);
      const size_t n = std::min<size_t>(p[4], kFtpDataMax);
      f.size = static_cast<uint8_t>(n);
      f.data.assign(p + 12, p + 12 + n);
      return OtaMsg{std::move(f)};
    }
    default:
      return std::nullopt;
  }
}

bool all_packets_uplink_lane(const uint8_t* bytes, size_t n) {
  if (n == 0 || bytes == nullptr) return false;
  // Fresh local parse state, never a channel index (see mav.h).
  mavlink_message_t rx{};
  mavlink_status_t st{};
  size_t packets = 0;
  for (size_t i = 0; i < n; ++i) {
    mavlink_message_t out{};
    if (mavlink_frame_char_buffer(&rx, &st, bytes[i], &out, nullptr) == MAVLINK_FRAMING_OK) {
      ++packets;
      if (!is_uplink_lane_msgid(out.msgid)) return false;
    }
  }
  // Mid-frame at the end means trailing garbage or a truncated packet.
  return packets > 0 && st.parse_state == MAVLINK_PARSE_STATE_IDLE;
}

std::vector<OtaMsg> decode_all(const uint8_t* bytes, size_t n) {
  std::vector<OtaMsg> out;
  if (bytes == nullptr) return out;
  mavlink_message_t rx{};
  mavlink_status_t st{};
  for (size_t i = 0; i < n; ++i) {
    mavlink_message_t msg{};
    if (mavlink_frame_char_buffer(&rx, &st, bytes[i], &msg, nullptr) != MAVLINK_FRAMING_OK) continue;
    if (auto m = decode(msg)) out.push_back(std::move(*m));
  }
  return out;
}

const char* reason_name(Reason r) {
  const auto i = static_cast<size_t>(r);
  return i < std::size(kReasonNames) ? kReasonNames[i] : "";
}

std::optional<Reason> reason_from_name(std::string_view s) {
  for (size_t i = 1; i < std::size(kReasonNames); ++i)
    if (s == kReasonNames[i]) return static_cast<Reason>(i);
  return std::nullopt;
}

}  // namespace mabur::ota
