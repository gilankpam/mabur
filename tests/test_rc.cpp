#include <cmath>
#include "mtest.h"
#include "vectors.h"
#include "mabur/rc_proto.h"
#include "mabur/profile.h"
using namespace mabur;
using namespace mabur::rc;

static Rcf rcf_from_json(const nlohmann::json& f) {
  Rcf r;
  r.vtx_id = f["vtx_id"].get<uint32_t>();
  r.seq = f["seq"].get<uint16_t>();
  r.profile = f["profile"].get<uint8_t>();
  r.fec_overhead_base = f["fec_overhead_base"].get<double>();
  r.fec_overhead_enh = f["fec_overhead_enh"].get<double>();
  if (f.contains("probe_profile")) { r.probe3 = true; r.probe_profile = f["probe_profile"].get<uint8_t>(); }
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

TEST(rcf_matches_golden_wire) {
  // Golden pin: mabur owns these bytes (devourer's frozen Python is stuck
  // at RC_VERSION 1). Print-once, then hardcode:
  //   std::fprintf(stderr, "%s\n", mtest::hex(wire).c_str());
  // Reverting any pack_rcf() layout change without updating these fails
  // here, which is the point -- the format cannot drift silently.
  const std::vector<std::string> GOLDEN = {
      "4352050100efbeadde070024323295a6",
      "435205010001000000ffff006464df0e",
      // Asym pair (base 1.0 / enh 0.5): ENH actually rides a different
      // literal overhead than BASE here, not a duplicated equal-pair scalar.
      "4352050100443322112a00086432d635",
  };
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  REQUIRE(j["rcf"].size() == GOLDEN.size());
  size_t i = 0;
  for (auto& c : j["rcf"]) {
    auto r = rcf_from_json(c["fields"]);
    auto wire = pack_rcf(r);
    CHECK(mtest::hex(wire) == GOLDEN[i]);

    auto raw = mtest::unhex(GOLDEN[i]);
    auto parsed = parse_rcf(raw.data(), raw.size());
    REQUIRE(parsed.has_value());
    CHECK(parsed->vtx_id == r.vtx_id);
    CHECK(parsed->seq == r.seq);
    CHECK(parsed->profile == r.profile);
    CHECK(std::abs(parsed->fec_overhead_base - r.fec_overhead_base) < 1e-9);
    CHECK(std::abs(parsed->fec_overhead_enh - r.fec_overhead_enh) < 1e-9);
    CHECK(frame_type(raw.data(), raw.size()) == T_RCF);
    ++i;
  }
}

TEST(disc_matches_golden_wire) {
  // Golden pin: mabur owns these bytes (devourer's frozen Python is stuck
  // at RC_VERSION 1). Print-once, then hardcode:
  //   std::fprintf(stderr, "%s\n", mtest::hex(wire).c_str());
  // Reverting any pack_disc() layout change without updating this fails
  // here, which is the point -- the format cannot drift silently.
  const std::vector<std::string> GOLDEN = {
      "4352050204010000000100feca95140100000002001021",
  };
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  REQUIRE(j["disc"].size() == GOLDEN.size());
  size_t i = 0;
  for (auto& c : j["disc"]) {
    auto d = disc_from_json(c["fields"]);
    auto wire = pack_disc(d);
    CHECK(mtest::hex(wire) == GOLDEN[i]);

    auto raw = mtest::unhex(GOLDEN[i]);
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
    ++i;
  }
}

TEST(disc_ack_matches_golden_wire) {
  // Golden pin: mabur owns these bytes (devourer's frozen Python is stuck
  // at RC_VERSION 1). Print-once, then hardcode:
  //   std::fprintf(stderr, "%s\n", mtest::hex(wire).c_str());
  // Reverting any pack_disc_ack() layout change without updating this
  // fails here, which is the point -- the format cannot drift silently.
  const std::vector<std::string> GOLDEN = {
      "4352050304010000000100feca030095140100e883",
  };
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  REQUIRE(j["disc_ack"].size() == GOLDEN.size());
  size_t i = 0;
  for (auto& c : j["disc_ack"]) {
    auto a = disc_ack_from_json(c["fields"]);
    auto wire = pack_disc_ack(a);
    CHECK(mtest::hex(wire) == GOLDEN[i]);

    auto raw = mtest::unhex(GOLDEN[i]);
    auto parsed = parse_disc_ack(raw.data(), raw.size());
    REQUIRE(parsed.has_value());
    CHECK(parsed->vtx_id == a.vtx_id);
    CHECK(parsed->vrx_nonce == a.vrx_nonce);
    CHECK(parsed->chip_caps == a.chip_caps);
    CHECK(parsed->agreed_channel == a.agreed_channel);
    CHECK(parsed->agreed_width == a.agreed_width);
    CHECK(parsed->seq == a.seq);
    CHECK(frame_type(raw.data(), raw.size()) == T_DISC_ACK);
    ++i;
  }
}

TEST(rcf_truncation_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto raw = pack_rcf(rcf_from_json(j["rcf"][0]["fields"]));
  for (size_t len = 0; len < raw.size(); ++len) {
    auto parsed = parse_rcf(raw.data(), len);
    CHECK(!parsed.has_value());
  }
}

TEST(disc_truncation_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto raw = pack_disc(disc_from_json(j["disc"][0]["fields"]));
  for (size_t len = 0; len < raw.size(); ++len) {
    auto parsed = parse_disc(raw.data(), len);
    CHECK(!parsed.has_value());
  }
}

TEST(disc_ack_truncation_fails) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto raw = pack_disc_ack(disc_ack_from_json(j["disc_ack"][0]["fields"]));
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
  auto orig = pack_rcf(rcf_from_json(j["rcf"][0]["fields"]));
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
        CHECK(parsed->profile == orig_parsed->profile);
        CHECK(std::abs(parsed->fec_overhead_base - orig_parsed->fec_overhead_base) < 1e-9);
        CHECK(std::abs(parsed->fec_overhead_enh - orig_parsed->fec_overhead_enh) < 1e-9);
      }
    }
  }
}

TEST(frame_type_peek) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rc.json");
  auto rcf_wire = pack_rcf(rcf_from_json(j["rcf"][0]["fields"]));
  auto disc_wire = pack_disc(disc_from_json(j["disc"][0]["fields"]));
  auto ack_wire = pack_disc_ack(disc_ack_from_json(j["disc_ack"][0]["fields"]));
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

TEST(overhead_to_x100_clamps) {
  CHECK(rc::overhead_to_x100(0.5) == 50);
  CHECK(rc::overhead_to_x100(0.15) == 15);   // exact now (was 0.125 in 16ths)
  CHECK(rc::overhead_to_x100(0.10) == 10);   // distinct from 0.15 now
  CHECK(rc::overhead_to_x100(0.0) == 5);     // clamp floor 0.05
  CHECK(rc::overhead_to_x100(9.9) == 200);   // clamp ceiling 2.0
}

TEST(rcf_fec_overhead_is_literal_x100) {
  rc::Rcf r;
  r.fec_overhead_base = 0.5;
  r.fec_overhead_enh = 1.0;
  auto w = rc::pack_rcf(r);
  auto p = rc::parse_rcf(w.data(), w.size());
  CHECK(p.has_value());
  CHECK(std::abs(p->fec_overhead_base - 0.5) < 1e-9);
  CHECK(std::abs(p->fec_overhead_enh - 1.0) < 1e-9);
}

TEST(telem_applied_ov_split_round_trip) {
  rc::Telem t;
  t.applied_ov_base = 0.28;
  t.applied_ov_enh = 0.80;
  auto w = rc::pack_telem(t);
  auto p = rc::parse_telem(w.data(), w.size());
  CHECK(p.has_value());
  CHECK(std::abs(p->applied_ov_base - 0.28) < 0.005);
  CHECK(std::abs(p->applied_ov_enh - 0.80) < 0.005);
}

TEST(telem_round_trip_and_golden) {
  mabur::rc::Telem t;
  t.tlm_seq = 0x0102; t.state = 2; t.flags = 0x03; t.generation = 0x04050607;
  t.applied_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 5, 20);
  t.applied_ov_base = 0.25; t.applied_ov_enh = 0.30;
  t.rcf_age_ms = 45; t.rcf_seq_echo = 0x1234;
  t.pts_at_build = 0x0011223344556677ull;
  t.rcf_rx = 100000; t.enc_frames = 200000;
  t.enc_kbytes = 300000; t.cmd_kbps = 9000; t.qp = 8; t.roi_qp = -24;
  t.ring_drops = 1;
  t.txq_depth = 3; t.txq_cap = 64; t.txq_drops = 7; t.txq_wait_max_ms = 1234;
  t.radio_sent = 400000;
  t.radio_drops = 9; t.usb_fail = 2;
  t.up_rssi[0] = 51; t.up_rssi[1] = 52; t.up_snr[0] = 21; t.up_snr[1] = 22;
  t.soc_temp_c = 61; t.thermal_delta = 3; t.load_x100 = 72;
  t.idr_disagree = 4; t.enhance_disagree = 5;
  t.vanished_base = 7; t.vanished_enh = 8; t.self_idr_refused = 9;
  t.venc_full_drops = 10; t.venc_ring_fill_pct = 62;
  auto wire = mabur::rc::pack_telem(t);
  CHECK(mabur::rc::frame_type(wire.data(), wire.size()) == mabur::rc::T_TELEM);
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK(back->tlm_seq == t.tlm_seq);
  CHECK(back->generation == t.generation);
  CHECK(back->applied_profile == t.applied_profile);
  CHECK(std::abs(back->applied_ov_base - 0.25) < 0.005);
  CHECK(std::abs(back->applied_ov_enh - 0.30) < 0.005);
  CHECK(back->rcf_seq_echo == 0x1234);
  CHECK(back->pts_at_build == 0x0011223344556677ull);
  CHECK(back->rcf_rx == t.rcf_rx);
  CHECK(back->enc_kbytes == t.enc_kbytes);
  CHECK(back->qp == 8);
  CHECK(back->roi_qp == -24);
  CHECK(back->txq_wait_max_ms == 1234);
  CHECK(back->radio_sent == t.radio_sent);
  CHECK(back->up_snr[1] == 22);
  CHECK(back->soc_temp_c == 61);
  CHECK(back->load_x100 == 72);
  CHECK(back->idr_disagree == 4);
  CHECK(back->enhance_disagree == 5);
  CHECK(back->vanished_base == 7);
  CHECK(back->vanished_enh == 8);
  CHECK(back->self_idr_refused == 9);
  CHECK(back->venc_full_drops == 10);
  CHECK(back->venc_ring_fill_pct == 62);
  // Golden pin: byte-exact wire so the format can never drift silently.
  // Print-once, then hardcode: std::fprintf(stderr, "%s\n", mtest::hex(wire).c_str());
  // (fill GOLDEN with the printed hex in the same commit — the test must
  // not pass with an empty golden)
  const std::string GOLDEN =
      "43520504030201020706050405191e2d0034127766554433221100a0860100400d0"
      "300e0930400282308e80100034007000000d204801a0600090000000200333415"
      "163d034800040005000700080009000a003e796c";
  CHECK(mtest::hex(wire) == GOLDEN);
  // Corrupt/truncate rejection, mirroring the disc_ack tests:
  auto trunc = wire; trunc.pop_back();
  CHECK(!mabur::rc::parse_telem(trunc.data(), trunc.size()).has_value());
  auto flip = wire; flip[wire.size() / 2] ^= 0xFF;
  CHECK(!mabur::rc::parse_telem(flip.data(), flip.size()).has_value());
}

TEST(telem_rtt_sync_fields_round_trip) {
  // link-rtt (2026-09-02): the drone echoes WHICH RCF rcf_age_ms is aging
  // against (seq identity for the GS send-time match) and its pts-domain
  // clock at telem build (the t3 of the NTP-style offset estimate). Both
  // must survive the wire at full width — pts_at_build is a 64-bit µs
  // value in the MI timebase and must not truncate.
  mabur::rc::Telem t;
  t.rcf_age_ms = 12;
  t.rcf_seq_echo = 0xBEEF;
  t.pts_at_build = 0x0123456789ABCDEFull;
  auto w = mabur::rc::pack_telem(t);
  auto p = mabur::rc::parse_telem(w.data(), w.size());
  REQUIRE(p.has_value());
  CHECK(p->rcf_seq_echo == 0xBEEF);
  CHECK(p->pts_at_build == 0x0123456789ABCDEFull);
}

TEST(rcf_probe_roundtrip) {
  mabur::rc::Rcf r;
  r.vtx_id = 7; r.seq = 42; r.profile = 5;
  r.probe3 = true;
  r.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 6, 20);
  auto buf = mabur::rc::pack_rcf(r);
  auto p = mabur::rc::parse_rcf(buf.data(), buf.size());
  REQUIRE(p.has_value());
  CHECK(p->probe3);
  CHECK(p->probe_profile == r.probe_profile);
  // probe3 is the ONLY thing the flags byte carries now, so pin the byte
  // itself: dropping the flag from pack_rcf() would still round-trip through
  // the struct if parse read the probe byte unconditionally.
  CHECK(buf[4] == mabur::rc::RCF_F_PROBE_ENH);
}

TEST(rcf_no_probe_unchanged_length) {
  mabur::rc::Rcf r;
  auto plain = mabur::rc::pack_rcf(r);
  CHECK(plain[4] == 0);  // flags byte is zero when not probing
  r.probe3 = true;
  auto probed = mabur::rc::pack_rcf(r);
  CHECK(probed.size() == plain.size() + 1);  // exactly one extra byte
  auto p = mabur::rc::parse_rcf(plain.data(), plain.size());
  REQUIRE(p.has_value());
  CHECK(!p->probe3);
}

TEST(rcf_probe_truncated_rejected) {
  mabur::rc::Rcf r;
  r.probe3 = true;
  auto buf = mabur::rc::pack_rcf(r);
  // Drop the last byte (CRC tail) — must not parse.
  CHECK(!mabur::rc::parse_rcf(buf.data(), buf.size() - 1).has_value());
}

TEST(version_mismatch_rejected_both_directions) {
  // Reverting the RC_VERSION bump in rc_proto.h makes the doctored v4 frame
  // become current-version, so it parses and the first CHECK fails.
  mabur::rc::Rcf r;
  r.vtx_id = 7;
  r.seq = 1;
  r.profile = 0;
  r.fec_overhead_base = 0.25;
  r.fec_overhead_enh = 0.25;
  auto body = mabur::rc::pack_rcf(r);

  // Sanity: as packed, it parses.
  CHECK(mabur::rc::parse_rcf(body.data(), body.size()).has_value());

  // Byte 2 is the version. Any other version must be refused outright —
  // including 4, the previous RCF wire this build bumped away from.
  auto v4 = body;
  v4[2] = 4;
  CHECK(!mabur::rc::parse_rcf(v4.data(), v4.size()).has_value());

  auto v6 = body;
  v6[2] = 6;
  CHECK(!mabur::rc::parse_rcf(v6.data(), v6.size()).has_value());

  // The same guard must hold for telemetry, which travels the opposite
  // direction (drone -> GS). A half-deployed pair must fail BOTH ways.
  mabur::rc::Telem t;
  t.tlm_seq = 9;
  auto tb = mabur::rc::pack_telem(t);
  CHECK(mabur::rc::parse_telem(tb.data(), tb.size()).has_value());
  auto tv1 = tb;
  tv1[2] = 1;
  CHECK(!mabur::rc::parse_telem(tv1.data(), tv1.size()).has_value());
}

TEST(rcf_head_is_fourteen_bytes) {
  // Pins the removed power byte AND the removed ack_seq/score/layer_delivery
  // fields, plus the RC_VERSION 5 overhead split: the RCF head is
  // fixed-length, so restoring any dropped field or losing the second
  // overhead byte makes the packed body a different length and this fails.
  mabur::rc::Rcf r;
  r.vtx_id = 1;
  r.fec_overhead_base = 0.42;
  r.fec_overhead_enh = 0.37;
  auto body = mabur::rc::pack_rcf(r);
  // 14-byte head + 2 CRC bytes, with no variable-length tail at all.
  CHECK(body.size() == 14 + 2);
  // The two literal x100 overhead bytes are the last two head bytes.
  CHECK(body[12] == 42);
  CHECK(body[13] == 37);
}

// The version check drops a foreign frame with no trace anywhere -- on a
// half-deployed pair that presents as no-video, which sends the operator to
// `restart maburd`, which cannot help. Both ingest points now log on this
// predicate, so it must be exact: RC magic + a version that is not ours, and
// nothing else. Deleting the buf[2] != RC_VERSION term (making it "is this an
// RC frame at all") makes the own-version case fail; deleting the magic term
// makes the non-RC case fail.
TEST(foreign_rc_version_predicate) {
  mabur::rc::Rcf r;
  r.vtx_id = 7;
  r.seq = 1;
  r.fec_overhead_base = 0.25;
  r.fec_overhead_enh = 0.25;
  const auto body = mabur::rc::pack_rcf(r);

  // Our own version: not foreign, and still a normal RC frame.
  CHECK(!mabur::rc::is_foreign_rc_version(body.data(), body.size()));
  CHECK(mabur::rc::frame_type(body.data(), body.size()) == mabur::rc::T_RCF);

  // Byte 2 doctored to another version: foreign. frame_type() must NOT change
  // its answer -- gs/src/main.cpp routes video on that -1.
  for (uint8_t ver : {uint8_t{1}, uint8_t{2}, uint8_t{255}}) {
    auto foreign = body;
    foreign[2] = ver;
    CHECK(mabur::rc::is_foreign_rc_version(foreign.data(), foreign.size()));
    CHECK(mabur::rc::frame_type(foreign.data(), foreign.size()) == -1);
  }

  // A non-RC body (wrong magic) is not a version mismatch, whatever byte 2
  // happens to hold -- this is the ~1-in-65536 false-positive class the RX
  // paths additionally gate on crc_ok for.
  const std::vector<uint8_t> video = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
  CHECK(!mabur::rc::is_foreign_rc_version(video.data(), video.size()));

  // Too short to hold magic+version+type: never reported as a mismatch, at
  // any length, including empty.
  auto truncated = body;
  truncated[2] = 1;  // would be foreign if it were long enough
  for (size_t n = 0; n < 4; ++n)
    CHECK(!mabur::rc::is_foreign_rc_version(truncated.data(), n));
  CHECK(!mabur::rc::is_foreign_rc_version(nullptr, 0));
}

MTEST_MAIN
