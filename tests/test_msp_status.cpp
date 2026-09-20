#include "mtest.h"
#include "mabur/msp_dp.h"
#include "mabur/msp_status.h"
#include <vector>
using namespace mabur;

// One MSP v1 REPLY frame ($M> size cmd payload xor-checksum) -- the FC's
// direction byte, which the parser accepts alongside '<'.
static std::vector<uint8_t> msp_reply(uint8_t cmd, std::vector<uint8_t> payload) {
  std::vector<uint8_t> f = {'$', 'M', '>', static_cast<uint8_t>(payload.size()), cmd};
  uint8_t cks = static_cast<uint8_t>(payload.size()) ^ cmd;
  for (uint8_t b : payload) { f.push_back(b); cks ^= b; }
  f.push_back(cks);
  return f;
}

// The 27-byte MSP_STATUS payload the bench FC answered on 2026-09-19:
// cycle 123, i2c 0, sensors 0x002b, flightModeFlags, profile 0, then
// whatever the FC pads to 27. Only bytes 6..9 (the flags word) matter here.
static std::vector<uint8_t> status_payload(uint32_t flight_mode_flags) {
  std::vector<uint8_t> p(27, 0);
  p[0] = 123; p[1] = 0;      // cycleTime
  p[4] = 0x2b; p[5] = 0x00;  // sensors
  p[6] = static_cast<uint8_t>(flight_mode_flags);
  p[7] = static_cast<uint8_t>(flight_mode_flags >> 8);
  p[8] = static_cast<uint8_t>(flight_mode_flags >> 16);
  p[9] = static_cast<uint8_t>(flight_mode_flags >> 24);
  return p;
}

TEST(status_request_is_the_six_byte_msp_v1_frame) {
  std::vector<uint8_t> out;
  msp_append_status_request(out);
  const std::vector<uint8_t> want = {0x24, 0x4D, 0x3C, 0x00, 0x65, 0x65};
  CHECK(out == want);
}

TEST(status_reply_decodes_boxarm_bit0) {
  MspParser parser;
  auto disarmed = msp_reply(101, status_payload(0x00000002));  // ANGLE only
  auto armed = msp_reply(101, status_payload(0x00000003));     // ARM | ANGLE
  auto m1 = parser.feed(disarmed.data(), disarmed.size());
  REQUIRE(m1.size() == 1);
  auto a1 = msp_status_armed(m1[0]);
  REQUIRE(a1.has_value());
  CHECK(*a1 == false);
  auto m2 = parser.feed(armed.data(), armed.size());
  REQUIRE(m2.size() == 1);
  auto a2 = msp_status_armed(m2[0]);
  REQUIRE(a2.has_value());
  CHECK(*a2 == true);
}

TEST(status_decoder_ignores_other_commands_and_short_payloads) {
  MspMessage dp;
  dp.cmd = 182;
  dp.payload = {4};  // DRAW_SCREEN
  CHECK(!msp_status_armed(dp).has_value());
  MspMessage shrt;
  shrt.cmd = 101;
  shrt.payload = std::vector<uint8_t>(10, 0xFF);  // one byte short of the flags word
  CHECK(!msp_status_armed(shrt).has_value());
  MspMessage exact;
  exact.cmd = 101;
  exact.payload = std::vector<uint8_t>(11, 0);
  exact.payload[6] = 1;
  auto a = msp_status_armed(exact);
  REQUIRE(a.has_value());
  CHECK(*a == true);
}

TEST(status_reply_survives_interleaved_displayport_stream) {
  // A DisplayPort DRAW_SCREEN push on each side of the reply, fed byte-wise.
  std::vector<uint8_t> stream;
  msp_append_message(stream, 182, nullptr, 0);
  auto reply = msp_reply(101, status_payload(0x00000001));
  stream.insert(stream.end(), reply.begin(), reply.end());
  msp_append_message(stream, 182, nullptr, 0);
  MspParser parser;
  int armed_reports = 0, other = 0;
  for (uint8_t b : stream)
    for (auto& m : parser.feed(&b, 1)) {
      if (auto a = msp_status_armed(m)) { ++armed_reports; CHECK(*a == true); }
      else ++other;
    }
  CHECK(armed_reports == 1);
  CHECK(other == 2);
}

MTEST_MAIN
