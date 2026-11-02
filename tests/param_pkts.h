#pragma once
// MAVLink extended-parameter packet builders shared by test_param_client.cpp
// (ParamClient itself) and test_param_host.cpp (ParamHost, which wraps one):
// everything the drone's component (1/191) would send, pre-packed.
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "mabur/config_params.h"
#include "mabur/mav.h"
#include "mabur/param_value.h"

using Bytes = std::vector<uint8_t>;

inline Bytes pkt(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  return Bytes(buf, buf + n);
}

inline Bytes value_pkt(const char* id, uint8_t type, const char wire[128], uint16_t count,
                        uint16_t index) {
  char id16[16] = {};
  std::strncpy(id16, id, 16);
  mavlink_message_t msg;
  mavlink_msg_param_ext_value_pack(1, 191, &msg, id16, wire, type, count, index);
  return pkt(msg);
}

inline Bytes hb_pkt() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 191, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0,
                             0, MAV_STATE_ACTIVE);
  return pkt(msg);
}

inline Bytes ack_pkt(const char* id, uint8_t type, const char wire[128], uint8_t result) {
  char id16[16] = {};
  std::strncpy(id16, id, 16);
  mavlink_message_t msg;
  mavlink_msg_param_ext_ack_pack(1, 191, &msg, id16, wire, type, result);
  return pkt(msg);
}

inline Bytes status_pkt(const std::string& text) {
  char buf[50] = {};
  std::memcpy(buf, text.data(), std::min(text.size(), static_cast<size_t>(50)));
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(1, 191, &msg, MAV_SEVERITY_ERROR, buf, 0, 0);
  return pkt(msg);
}

// A plausible value for row `idx`, matching its own type -- VENC_MODE gets
// a shape decode_wire's loader-shape check accepts, every other Str row
// gets a plain printable string.
inline mabur::paramval::Value sample_value(size_t idx) {
  const auto& row = mabur::cfgparam::kTable[idx];
  mabur::paramval::Value v{row.type};
  switch (row.type) {
    case mabur::cfgparam::Type::Bool:
      v.b = true;
      break;
    case mabur::cfgparam::Type::Int32:
      v.i = 42;
      break;
    case mabur::cfgparam::Type::Real32:
      v.f = 1.5f;
      break;
    case mabur::cfgparam::Type::Str:
      v.s = (idx == mabur::cfgparam::kVencModeRow) ? "60@1920x1080/binned" : "0x0";
      break;
  }
  return v;
}
