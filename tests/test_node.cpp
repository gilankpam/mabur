#include "mtest.h"
#include "vectors.h"
#include "mabur/node.h"
using namespace mabur;

TEST(rx_body_pack_matches_python_and_roundtrips) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/node.json");
  for (auto& c : j["rx_body"]) {
    node::RxBody m;
    m.card_id = c["card_id"];
    m.mono_us = c["mono_us"].get<uint64_t>();
    m.rssi[0] = c["rssi"][0]; m.rssi[1] = c["rssi"][1];
    m.snr[0] = static_cast<int8_t>(c["snr"][0].get<int>());
    m.snr[1] = static_cast<int8_t>(c["snr"][1].get<int>());
    m.crc_ok = c["crc_ok"];
    m.mac_seq = c["mac_seq"];
    m.body = mtest::unhex(c["body"].get<std::string>());
    auto wire = node::pack_rx_body(m);
    CHECK(mtest::hex(wire) == c["wire"].get<std::string>());
    auto back = node::parse_rx_body(wire.data(), wire.size());
    REQUIRE(back.has_value());
    CHECK(back->card_id == m.card_id);
    CHECK(back->mono_us == m.mono_us);
    CHECK(back->rssi[1] == m.rssi[1]);
    CHECK(back->snr[0] == m.snr[0]);
    CHECK(back->crc_ok == m.crc_ok);
    CHECK(back->mac_seq == m.mac_seq);
    CHECK(back->body == m.body);
  }
}

TEST(rx_body_phy_valid_roundtrips) {
  node::RxBody m;
  m.card_id = 2; m.mono_us = 42; m.mac_seq = 7; m.crc_ok = true;
  m.phy_valid = true;
  auto wire_true = node::pack_rx_body(m);
  auto back_true = node::parse_rx_body(wire_true.data(), wire_true.size());
  REQUIRE(back_true.has_value());
  CHECK(back_true->phy_valid == true);

  m.phy_valid = false;
  auto wire_false = node::pack_rx_body(m);
  auto back_false = node::parse_rx_body(wire_false.data(), wire_false.size());
  REQUIRE(back_false.has_value());
  CHECK(back_false->phy_valid == false);
}

TEST(rx_body_rejects_corruption) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/node.json");
  auto wire = mtest::unhex(j["rx_body"][1]["wire"].get<std::string>());
  auto flip = wire; flip[10] ^= 0xFF;                       // payload corrupt
  CHECK(!node::parse_rx_body(flip.data(), flip.size()).has_value());
  CHECK(!node::parse_rx_body(wire.data(), wire.size() - 1).has_value());  // truncated
  auto magic = wire; magic[0] ^= 0xFF;
  CHECK(!node::parse_rx_body(magic.data(), magic.size()).has_value());
  // body_len larger than the buffer must not read out of bounds
  auto lenbad = wire; lenbad[19] = 0xFF; lenbad[20] = 0xFF;
  CHECK(!node::parse_rx_body(lenbad.data(), lenbad.size()).has_value());
}

TEST(card_status_roundtrip) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/node.json");
  for (auto& c : j["card_status"]) {
    node::CardStatus s;
    s.card_id = c["card_id"];
    s.mono_us = c["mono_us"].get<uint64_t>();
    s.frames_seen = c["frames_seen"];
    s.crc_fail = c["crc_fail"];
    s.uptime_s = c["uptime_s"];
    auto wire = node::pack_card_status(s);
    CHECK(mtest::hex(wire) == c["wire"].get<std::string>());
    auto back = node::parse_card_status(wire.data(), wire.size());
    REQUIRE(back.has_value());
    CHECK(back->frames_seen == s.frames_seen);
    CHECK(back->uptime_s == s.uptime_s);
  }
}
MTEST_MAIN
