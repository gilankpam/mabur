#include <cstring>

#include "mabur/cal_mav.h"
#include "mtest.h"

using namespace mabur::cal;
using mabur::mav::kChanGsLocal;
using mabur::mav::kCompId;
using mabur::mav::kSysId;

namespace {

// Stateless, fresh-locals parse of one whole encode()d packet -- the same
// shape cal_mav.cpp itself uses for all_packets_are_cal, and what a caller
// outside the tunnel's own channel would do with encode()'s output.
std::optional<mavlink_message_t> parse_one(const std::vector<uint8_t>& bytes) {
  mavlink_message_t rx{};
  mavlink_status_t st{};
  mavlink_message_t out{};
  bool got = false;
  for (uint8_t b : bytes) {
    if (mavlink_frame_char_buffer(&rx, &st, b, &out, nullptr) == MAVLINK_FRAMING_OK) got = true;
  }
  if (!got) return std::nullopt;
  return out;
}

}  // namespace

TEST(every_message_round_trips) {
  {
    SweepCmd c;
    c.nonce = 0xDEADBEEF;
    c.seq = 7;
    c.kind = Kind::Capacity;
    c.bw = 40;
    c.frames_per_cell = 20;
    c.settle_ms = 5;
    c.gap_us = 1000;
    c.dwell_ms = 2000;
    c.idx_lo = -41;
    c.idx_hi = 63;
    c.mcs_mask = 0x3C;
    auto bytes = encode(CalMsg{c}, kSysId, kCompId, kChanGsLocal);
    auto out = parse_one(bytes);
    REQUIRE(out.has_value());
    auto decoded = decode(*out);
    REQUIRE(decoded.has_value());
    auto* got = std::get_if<SweepCmd>(&*decoded);
    REQUIRE(got != nullptr);
    CHECK(got->nonce == c.nonce);
    CHECK(got->seq == c.seq);
    CHECK(got->kind == c.kind);
    CHECK(got->bw == c.bw);
    CHECK(got->frames_per_cell == c.frames_per_cell);
    CHECK(got->settle_ms == c.settle_ms);
    CHECK(got->gap_us == c.gap_us);
    CHECK(got->dwell_ms == c.dwell_ms);
    CHECK(got->idx_lo == c.idx_lo);
    CHECK(got->idx_hi == c.idx_hi);
    CHECK(got->mcs_mask == c.mcs_mask);
  }
  {
    SweepAck a;
    a.nonce = 0x1234;
    a.seq = 3;
    a.result = AckResult::Busy;
    a.start_in_ms = 500;
    a.flags = kFlagLdpc | kFlagStbc;
    auto bytes = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);
    auto out = parse_one(bytes);
    REQUIRE(out.has_value());
    auto decoded = decode(*out);
    REQUIRE(decoded.has_value());
    auto* got = std::get_if<SweepAck>(&*decoded);
    REQUIRE(got != nullptr);
    CHECK(got->nonce == a.nonce);
    CHECK(got->seq == a.seq);
    CHECK(got->result == a.result);
    CHECK(got->start_in_ms == a.start_in_ms);
    CHECK(got->flags == a.flags);
  }
  {
    SweepDone d;
    d.nonce = 0x55;
    d.seq = 9;
    for (size_t i = 0; i < 8; ++i) {
      d.offered[i] = static_cast<uint32_t>(1000 + i);
      d.blocked_pct[i] = static_cast<uint8_t>(i * 10);
    }
    auto bytes = encode(CalMsg{d}, kSysId, kCompId, kChanGsLocal);
    auto out = parse_one(bytes);
    REQUIRE(out.has_value());
    auto decoded = decode(*out);
    REQUIRE(decoded.has_value());
    auto* got = std::get_if<SweepDone>(&*decoded);
    REQUIRE(got != nullptr);
    CHECK(got->nonce == d.nonce);
    CHECK(got->seq == d.seq);
    CHECK(got->offered == d.offered);
    CHECK(got->blocked_pct == d.blocked_pct);
  }
  {
    ApplyCmd a;
    a.nonce = 0x99;
    a.seq = 3;
    a.bw = 40;
    a.has_walls = true;
    a.has_eff = true;
    for (size_t i = 0; i < 8; ++i) {
      a.walls[i] = static_cast<int8_t>(-20 + i);
      a.eff_x1000[i] = static_cast<uint16_t>(700 + i);
    }
    a.legacy = 17;
    auto bytes = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);
    auto out = parse_one(bytes);
    REQUIRE(out.has_value());
    auto decoded = decode(*out);
    REQUIRE(decoded.has_value());
    auto* got = std::get_if<ApplyCmd>(&*decoded);
    REQUIRE(got != nullptr);
    CHECK(got->nonce == a.nonce);
    CHECK(got->seq == a.seq);
    CHECK(got->bw == a.bw);
    CHECK(got->has_walls == a.has_walls);
    CHECK(got->has_eff == a.has_eff);
    CHECK(got->walls == a.walls);
    CHECK(got->eff_x1000 == a.eff_x1000);
    CHECK(got->legacy == a.legacy);
  }
  {
    Applied a;
    a.nonce = 0x77;
    a.seq = 2;
    a.bw = 20;
    a.ok = true;
    a.reason = "ok";
    auto bytes = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);
    auto out = parse_one(bytes);
    REQUIRE(out.has_value());
    auto decoded = decode(*out);
    REQUIRE(decoded.has_value());
    auto* got = std::get_if<Applied>(&*decoded);
    REQUIRE(got != nullptr);
    CHECK(got->nonce == a.nonce);
    CHECK(got->seq == a.seq);
    CHECK(got->bw == a.bw);
    CHECK(got->ok == a.ok);
    CHECK(got->reason == a.reason);
  }
  {
    Abort a;
    a.nonce = 0xABCDEF01;
    auto bytes = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);
    auto out = parse_one(bytes);
    REQUIRE(out.has_value());
    auto decoded = decode(*out);
    REQUIRE(decoded.has_value());
    auto* got = std::get_if<Abort>(&*decoded);
    REQUIRE(got != nullptr);
    CHECK(got->nonce == a.nonce);
  }
}

TEST(applied_reason_is_cut_at_49) {
  Applied a;
  a.nonce = 1;
  a.bw = 20;
  a.ok = false;
  a.reason = std::string(60, 'x');  // longer than the 49-char cut
  auto bytes = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);
  auto out = parse_one(bytes);
  REQUIRE(out.has_value());
  auto decoded = decode(*out);
  REQUIRE(decoded.has_value());
  auto* got = std::get_if<Applied>(&*decoded);
  REQUIRE(got != nullptr);
  CHECK(got->reason.size() == 49);
  CHECK(got->reason == std::string(49, 'x'));
}

TEST(decode_returns_nullopt_for_a_heartbeat) {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(kSysId, kCompId, &msg, MAV_TYPE_ONBOARD_CONTROLLER,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  CHECK(!decode(msg).has_value());
}

TEST(all_packets_are_cal_rejects_a_mixed_frame) {
  Abort a;
  a.nonce = 1;
  auto sweep_bytes = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);

  mavlink_message_t hb;
  mavlink_msg_heartbeat_pack(kSysId, kCompId, &hb, MAV_TYPE_ONBOARD_CONTROLLER,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  uint8_t hb_buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t hb_n = mavlink_msg_to_send_buffer(hb_buf, &hb);

  std::vector<uint8_t> mixed = sweep_bytes;
  mixed.insert(mixed.end(), hb_buf, hb_buf + hb_n);
  CHECK(!all_packets_are_cal(mixed.data(), mixed.size()));

  std::vector<uint8_t> two_sweeps = sweep_bytes;
  two_sweeps.insert(two_sweeps.end(), sweep_bytes.begin(), sweep_bytes.end());
  CHECK(all_packets_are_cal(two_sweeps.data(), two_sweeps.size()));

  CHECK(!all_packets_are_cal(nullptr, 0));
}

TEST(all_packets_are_cal_rejects_a_truncated_trailing_packet) {
  // Two whole cal packets and a third cut short: the tail is not a cal
  // packet, so the frame is not all cal.
  Abort a;
  a.nonce = 1;
  const auto one = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);
  std::vector<uint8_t> frame = one;
  frame.insert(frame.end(), one.begin(), one.end());
  REQUIRE(all_packets_are_cal(frame.data(), frame.size()));
  frame.insert(frame.end(), one.begin(), one.begin() + static_cast<long>(one.size() / 2));
  CHECK(!all_packets_are_cal(frame.data(), frame.size()));
}

TEST(decode_all_returns_the_cal_packets_in_order) {
  SweepCmd c;
  c.nonce = 9;
  c.seq = 2;
  Abort a;
  a.nonce = 9;
  const auto cb = encode(CalMsg{c}, kSysId, kCompId, kChanGsLocal);
  const auto ab = encode(CalMsg{a}, kSysId, kCompId, kChanGsLocal);

  mavlink_message_t hb;
  mavlink_msg_heartbeat_pack(kSysId, kCompId, &hb, MAV_TYPE_ONBOARD_CONTROLLER,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  uint8_t hb_buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t hb_n = mavlink_msg_to_send_buffer(hb_buf, &hb);

  // sweep, heartbeat (skipped), abort, then a truncated sweep (skipped).
  std::vector<uint8_t> bytes = cb;
  bytes.insert(bytes.end(), hb_buf, hb_buf + hb_n);
  bytes.insert(bytes.end(), ab.begin(), ab.end());
  bytes.insert(bytes.end(), cb.begin(), cb.begin() + 5);
  const auto msgs = decode_all(bytes.data(), bytes.size());
  REQUIRE(msgs.size() == 2);
  REQUIRE(std::holds_alternative<SweepCmd>(msgs[0]));
  CHECK(std::get<SweepCmd>(msgs[0]).nonce == 9 && std::get<SweepCmd>(msgs[0]).seq == 2);
  REQUIRE(std::holds_alternative<Abort>(msgs[1]));
  CHECK(std::get<Abort>(msgs[1]).nonce == 9);

  CHECK(decode_all(nullptr, 0).empty());
}

MTEST_MAIN
