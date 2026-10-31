#include <cstring>
#include "mtest.h"
#include "mabur/mav.h"
TEST(heartbeat_packs_and_parses) {
  mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(mabur::mav::kSysId, mabur::mav::kCompId, &msg,
      MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  CHECK(buf[0] == 0xFD && n == 21);
  mavlink_message_t out; mavlink_status_t st{};
  bool got = false;
  for (uint16_t i = 0; i < n; ++i)
    if (mavlink_parse_char(mabur::mav::kChanGsRx, buf[i], &out, &st)) got = true;
  CHECK(got && out.msgid == MAVLINK_MSG_ID_HEARTBEAT && out.sysid == 1 && out.compid == 191);
}
TEST(param_ext_value_is_161_bytes_untruncated) {
  mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  char value[128]; std::memset(value, 'x', sizeof value);
  mavlink_msg_param_ext_value_pack(1, 191, &msg, "ANY_PARAM_ID", value, MAV_PARAM_EXT_TYPE_INT32, 13, 2);
  CHECK(mavlink_msg_to_send_buffer(buf, &msg) == 161);
}
TEST(mabur_dialect_packs_and_decodes_cal_sweep) {
  mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mabur_cal_sweep_pack_chan(mabur::mav::kSysId, mabur::mav::kCompId,
      mabur::mav::kChanDroneRx, &msg,
      /*nonce=*/0xA5A5, /*seq=*/0, /*kind=*/3, /*bw=*/40,
      /*frames_per_cell=*/20, /*settle_ms=*/5, /*gap_us=*/1000,
      /*dwell_ms=*/2000, /*idx_lo=*/-41, /*idx_hi=*/63, /*mcs_mask=*/0xFF);
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);

  mavlink_message_t out; mavlink_status_t st{};
  bool got = false;
  for (uint16_t i = 0; i < n; ++i)
    if (mavlink_frame_char(mabur::mav::kChanGsRx, buf[i], &out, &st) == MAVLINK_FRAMING_OK) got = true;
  CHECK(got);
  CHECK(out.msgid == MAVLINK_MSG_ID_MABUR_CAL_SWEEP);

  mavlink_mabur_cal_sweep_t decoded;
  mavlink_msg_mabur_cal_sweep_decode(&out, &decoded);
  CHECK(decoded.nonce == 0xA5A5);
  CHECK(decoded.seq == 0);
  CHECK(decoded.kind == 3);
  CHECK(decoded.bw == 40);
  CHECK(decoded.frames_per_cell == 20);
  CHECK(decoded.settle_ms == 5);
  CHECK(decoded.gap_us == 1000);
  CHECK(decoded.dwell_ms == 2000);
  CHECK(decoded.idx_lo == -41);
  CHECK(decoded.idx_hi == 63);
  CHECK(decoded.mcs_mask == 0xFF);
}
TEST(common_messages_still_decode) {
  mavlink_message_t msg; uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(mabur::mav::kSysId, mabur::mav::kCompId, &msg,
      MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
  mavlink_message_t out; mavlink_status_t st{};
  bool got = false;
  for (uint16_t i = 0; i < n; ++i)
    if (mavlink_frame_char(mabur::mav::kChanGsLocal, buf[i], &out, &st) == MAVLINK_FRAMING_OK) got = true;
  CHECK(got && out.msgid == MAVLINK_MSG_ID_HEARTBEAT && out.sysid == 1 && out.compid == 191);
}
MTEST_MAIN
