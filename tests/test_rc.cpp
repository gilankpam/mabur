#include "mtest.h"
#include "vectors.h"
#include "mabur/rc_proto.h"
using namespace mabur;
using namespace mabur::rc;

static Rcf rcf_from_json(const nlohmann::json& f) {
  Rcf r;
  r.vtx_id = f["vtx_id"].get<uint32_t>();
  r.seq = f["seq"].get<uint16_t>();
  r.ack_seq = f["ack_seq"].get<uint16_t>();
  r.profile = f["profile"].get<uint8_t>();
  r.score = f["score"].get<uint16_t>();
  r.pwr_offset_biased = f["pwr_idx"].get<uint8_t>();
  r.fec_overhead_16ths = f["fec_overhead_16ths"].get<uint8_t>();
  r.flags = f["flags"].get<uint8_t>();
  for (auto& v : f["layer_delivery"]) r.layer_delivery.push_back(v.get<uint8_t>());
  return r;
}

static Disc disc_from_json(const nlohmann::json& f) {
  Disc d;
  d.vtx_id = f["vtx_id"].get<uint32_t>();
  d.vrx_nonce = f["vrx_nonce"].get<uint32_t>();
  d.op_channel = f["op_channel"].get<uint8_t>();
  d.op_width = f["op_width"].get<uint8_t>();
  d.table_ver = f["table_ver"].get<uint8_t>();
  d.init_profile = f["init_profile"].get<uint8_t>();
  d.cap_bits = f["cap_bits"].get<uint16_t>();
  d.seq = f["seq"].get<uint16_t>();
  return d;
}

static DiscAck disc_ack_from_json(const nlohmann::json& f) {
  DiscAck a;
  a.vtx_id = f["vtx_id"].get<uint32_t>();
  a.vrx_nonce = f["vrx_nonce"].get<uint32_t>();
  a.chip_caps = f["chip_caps"].get<uint16_t>();
  a.agreed_channel = f["agreed_channel"].get<uint8_t>();
  a.agreed_width = f["agreed_width"].get<uint8_t>();
  a.seq = f["seq"].get<uint16_t>();
  return a;
}

TEST(rcf_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  for (auto& c : j["rcf"]) {
    auto r = rcf_from_json(c["fields"]);
    auto wire = pack_rcf(r);
    CHECK(mtest::hex(wire) == c["wire"].get<std::string>());

    auto raw = mtest::unhex(c["wire"].get<std::string>());
    auto parsed = parse_rcf(raw.data(), raw.size());
    REQUIRE(parsed.has_value());
    CHECK(parsed->vtx_id == r.vtx_id);
    CHECK(parsed->seq == r.seq);
    CHECK(parsed->ack_seq == r.ack_seq);
    CHECK(parsed->profile == r.profile);
    CHECK(parsed->score == r.score);
    CHECK(parsed->pwr_offset_biased == r.pwr_offset_biased);
    CHECK(parsed->fec_overhead_16ths == r.fec_overhead_16ths);
    CHECK(parsed->flags == r.flags);
    CHECK(parsed->layer_delivery == r.layer_delivery);

    CHECK(frame_type(raw.data(), raw.size()) == T_RCF);
  }
}

TEST(disc_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  for (auto& c : j["disc"]) {
    auto d = disc_from_json(c["fields"]);
    auto wire = pack_disc(d);
    CHECK(mtest::hex(wire) == c["wire"].get<std::string>());

    auto raw = mtest::unhex(c["wire"].get<std::string>());
    auto parsed = parse_disc(raw.data(), raw.size());
    REQUIRE(parsed.has_value());
    CHECK(parsed->vtx_id == d.vtx_id);
    CHECK(parsed->vrx_nonce == d.vrx_nonce);
    CHECK(parsed->op_channel == d.op_channel);
    CHECK(parsed->op_width == d.op_width);
    CHECK(parsed->table_ver == d.table_ver);
    CHECK(parsed->init_profile == d.init_profile);
    CHECK(parsed->cap_bits == d.cap_bits);
    CHECK(parsed->seq == d.seq);

    CHECK(frame_type(raw.data(), raw.size()) == T_DISC);
  }
}

TEST(disc_ack_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  for (auto& c : j["disc_ack"]) {
    auto a = disc_ack_from_json(c["fields"]);
    auto wire = pack_disc_ack(a);
    CHECK(mtest::hex(wire) == c["wire"].get<std::string>());

    auto raw = mtest::unhex(c["wire"].get<std::string>());
    auto parsed = parse_disc_ack(raw.data(), raw.size());
    REQUIRE(parsed.has_value());
    CHECK(parsed->vtx_id == a.vtx_id);
    CHECK(parsed->vrx_nonce == a.vrx_nonce);
    CHECK(parsed->chip_caps == a.chip_caps);
    CHECK(parsed->agreed_channel == a.agreed_channel);
    CHECK(parsed->agreed_width == a.agreed_width);
    CHECK(parsed->seq == a.seq);

    CHECK(frame_type(raw.data(), raw.size()) == T_DISC_ACK);
  }
}

TEST(rcf_truncation_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto raw = mtest::unhex(j["rcf"][0]["wire"].get<std::string>());
  for (size_t len = 0; len < raw.size(); ++len) {
    auto parsed = parse_rcf(raw.data(), len);
    CHECK(!parsed.has_value());
  }
}

TEST(disc_truncation_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto raw = mtest::unhex(j["disc"][0]["wire"].get<std::string>());
  for (size_t len = 0; len < raw.size(); ++len) {
    auto parsed = parse_disc(raw.data(), len);
    CHECK(!parsed.has_value());
  }
}

TEST(disc_ack_truncation_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto raw = mtest::unhex(j["disc_ack"][0]["wire"].get<std::string>());
  for (size_t len = 0; len < raw.size(); ++len) {
    auto parsed = parse_disc_ack(raw.data(), len);
    CHECK(!parsed.has_value());
  }
}

// Single-byte flips across one full RCF wire: every flip must either yield
// nullopt, or (if by freak chance a flip still passes CRC and header checks)
// an unchanged struct. For this frame, no flip should pass — every byte is
// covered either by header validation or by the CRC.
TEST(rcf_single_byte_flip_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto orig_hex = j["rcf"][0]["wire"].get<std::string>();
  auto orig = mtest::unhex(orig_hex);
  auto orig_parsed = parse_rcf(orig.data(), orig.size());
  REQUIRE(orig_parsed.has_value());

  for (size_t byte_idx = 0; byte_idx < orig.size(); ++byte_idx) {
    for (int bit = 0; bit < 8; ++bit) {
      auto flipped = orig;
      flipped[byte_idx] ^= static_cast<uint8_t>(1u << bit);
      auto parsed = parse_rcf(flipped.data(), flipped.size());
      if (parsed.has_value()) {
        // Only acceptable if the struct is bit-for-bit identical to the
        // original (would mean a flip landed somewhere inert, which for
        // this frame layout shouldn't happen since the CRC covers
        // everything before it).
        CHECK(parsed->vtx_id == orig_parsed->vtx_id);
        CHECK(parsed->seq == orig_parsed->seq);
        CHECK(parsed->ack_seq == orig_parsed->ack_seq);
        CHECK(parsed->profile == orig_parsed->profile);
        CHECK(parsed->score == orig_parsed->score);
        CHECK(parsed->pwr_offset_biased == orig_parsed->pwr_offset_biased);
        CHECK(parsed->fec_overhead_16ths == orig_parsed->fec_overhead_16ths);
        CHECK(parsed->flags == orig_parsed->flags);
        CHECK(parsed->layer_delivery == orig_parsed->layer_delivery);
      }
    }
  }
}

TEST(layer_delivery_clamps_on_pack) {
  Rcf r;
  r.vtx_id = 1;
  r.layer_delivery = {0, 100, 255, 150, 1};
  auto wire = pack_rcf(r);
  auto parsed = parse_rcf(wire.data(), wire.size());
  REQUIRE(parsed.has_value());
  std::vector<uint8_t> expect = {0, 100, 100, 100, 1};
  CHECK(parsed->layer_delivery == expect);
}

TEST(frame_type_peek) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto rcf_wire = mtest::unhex(j["rcf"][0]["wire"].get<std::string>());
  auto disc_wire = mtest::unhex(j["disc"][0]["wire"].get<std::string>());
  auto ack_wire = mtest::unhex(j["disc_ack"][0]["wire"].get<std::string>());
  CHECK(frame_type(rcf_wire.data(), rcf_wire.size()) == T_RCF);
  CHECK(frame_type(disc_wire.data(), disc_wire.size()) == T_DISC);
  CHECK(frame_type(ack_wire.data(), ack_wire.size()) == T_DISC_ACK);

  std::vector<uint8_t> too_short = {0x43};
  CHECK(frame_type(too_short.data(), too_short.size()) == -1);

  std::vector<uint8_t> not_rc = {0x00, 0x00, 0x00, 0x00};
  CHECK(frame_type(not_rc.data(), not_rc.size()) == -1);

  auto bad_ver = rcf_wire;
  bad_ver[2] = 0xFF;  // version byte
  CHECK(frame_type(bad_ver.data(), bad_ver.size()) == -1);
}

TEST(overhead_to_16ths_cases) {
  CHECK(overhead_to_16ths(0.25) == 4);
  CHECK(overhead_to_16ths(1.0) == 16);
  CHECK(overhead_to_16ths(0.0) == 1);
  CHECK(overhead_to_16ths(2.0) == 16);  // clamp high
  CHECK(overhead_to_16ths(-1.0) == 1);  // clamp low
  CHECK(overhead_to_16ths(0.10) == 2);  // round(1.6) == 2
}

TEST(pwr_offset_bias_encoding) {
  CHECK(encode_pwr_offset_qdb(0) == 64);
  CHECK(encode_pwr_offset_qdb(-12) == 52);
  CHECK(encode_pwr_offset_qdb(16) == 80);
  CHECK(encode_pwr_offset_qdb(-100) == 0);    // clamped
  CHECK(encode_pwr_offset_qdb(100) == 127);   // clamped, never 0xFF
  CHECK(decode_pwr_offset_qdb(52) == -12);
  CHECK(decode_pwr_offset_qdb(64) == 0);
}

TEST(rcf_carries_biased_offset) {
  Rcf r;
  r.pwr_offset_biased = encode_pwr_offset_qdb(-8);
  auto b = pack_rcf(r);
  auto p = parse_rcf(b.data(), b.size());
  CHECK(p.has_value());
  CHECK(decode_pwr_offset_qdb(p->pwr_offset_biased) == -8);
}

MTEST_MAIN
