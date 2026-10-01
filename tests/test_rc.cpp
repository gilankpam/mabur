#include <cmath>
#include "mtest.h"
#include "vectors.h"
#include "mabur/rc_proto.h"
#include "mabur/profile.h"
#include "mabur/cal_wire.h"
#include "mabur/crc16.h"
#include "mabur/link_key.h"
using namespace mabur;
using namespace mabur::rc;

static LinkKey test_key() { return *parse_key_hex("3f9a1c77e04b5d2290ab6ef1c8d34e5a"); }

static Rcf rcf_from_json(const nlohmann::json& f) {
  Rcf r;
  r.seq = f["seq"].get<uint16_t>();
  r.profile = f["profile"].get<uint8_t>();
  r.fec_overhead_base = f["fec_overhead_base"].get<double>();
  r.fec_overhead_enh = f["fec_overhead_enh"].get<double>();
  r.probe_profile = f.contains("probe_profile") ? f["probe_profile"].get<uint8_t>()
                                                : kNoProbeProfile;
  r.hop_ch = f.contains("hop_ch") ? f["hop_ch"].get<uint8_t>() : 0;
  r.hop_epoch = f.contains("hop_epoch") ? f["hop_epoch"].get<uint8_t>() : 0;
  r.rec = f.contains("rec") ? f["rec"].get<uint8_t>() : 0;
  r.idr_epoch = f.contains("idr_epoch") ? f["idr_epoch"].get<uint8_t>() : 0;
  return r;
}

static Disc disc_from_json(const nlohmann::json& f) {
  Disc d;
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
  a.vrx_nonce = f["vrx_nonce"].get<uint32_t>();
  a.vtx_nonce = f["vtx_nonce"].get<uint32_t>();
  a.chip_caps = f["chip_caps"].get<uint16_t>();
  a.agreed_channel = f["agreed_channel"].get<uint8_t>();
  a.agreed_width = f["agreed_width"].get<uint8_t>();
  a.flags = f["flags"].get<uint8_t>();
  a.seq = f["seq"].get<uint16_t>();
  return a;
}

TEST(rcf_tag_verifies_with_key_and_ctx_only) {
  Rcf r; r.seq = 7; r.profile = 0x24; r.fec_overhead_base = 0.5; r.fec_overhead_enh = 0.25;
  const TagCtx ctx{0xCAFEF00D, 0x12345678, 7};
  auto wire = pack_rcf(r, test_key(), ctx);
  CHECK(wire.size() == 15 + kTagLen + 2);
  REQUIRE(parse_rcf(wire.data(), wire.size()).has_value());   // structural parse ignores the tag
  CHECK(verify_control(wire.data(), wire.size(), test_key(), ctx));
  CHECK(!verify_control(wire.data(), wire.size(), kDefaultLinkKey, ctx));
  CHECK(!verify_control(wire.data(), wire.size(), test_key(), TagCtx{0xCAFEF00E, 0x12345678, 7}));
  CHECK(!verify_control(wire.data(), wire.size(), test_key(), TagCtx{0xCAFEF00D, 0x12345679, 7}));
  CHECK(!verify_control(wire.data(), wire.size(), test_key(), TagCtx{0xCAFEF00D, 0x12345678, 7 + 65536}));
  // A flipped payload byte breaks the tag even if the CRC were re-signed.
  auto bad = wire; bad[7] ^= 1;
  const uint16_t crc = crc16_ccitt(bad.data(), bad.size() - 2);
  bad[bad.size() - 2] = crc & 0xFF; bad[bad.size() - 1] = crc >> 8;
  REQUIRE(parse_rcf(bad.data(), bad.size()).has_value());
  CHECK(!verify_control(bad.data(), bad.size(), test_key(), ctx));
  CHECK(!verify_control(wire.data(), 9, test_key(), ctx));
}

TEST(disc_tag_uses_key_alone) {
  Disc d; d.vrx_nonce = 0xCAFE0001; d.op_channel = 149; d.seq = 3;
  auto wire = pack_disc(d, test_key());
  CHECK(wire.size() == 17 + kTagLen + 2);
  CHECK(verify_control(wire.data(), wire.size(), test_key(), TagCtx{}));
  CHECK(!verify_control(wire.data(), wire.size(), kDefaultLinkKey, TagCtx{}));
  auto def = pack_disc(d);   // default key when none given
  CHECK(verify_control(def.data(), def.size(), kDefaultLinkKey, TagCtx{}));
}

TEST(cal_frames_are_tagged_without_seq) {
  CalCmd c; c.nonce = 5; c.windows.push_back(CalWindow{3, -4, 4, 1});
  const TagCtx ctx{1, 2, 0};
  auto w = pack_cal_cmd(c, test_key(), ctx);
  REQUIRE(parse_cal_cmd(w.data(), w.size()).has_value());
  CHECK(verify_control(w.data(), w.size(), test_key(), ctx));
  CHECK(!verify_control(w.data(), w.size(), test_key(), TagCtx{1, 2, 1}));
  CalResult r; r.nonce = 5;
  auto rw = pack_cal_result(r, test_key(), ctx);
  CHECK(rw.size() == 27 + kTagLen + 2);
  REQUIRE(parse_cal_result(rw.data(), rw.size()).has_value());
  CHECK(verify_control(rw.data(), rw.size(), test_key(), ctx));
}

TEST(disc_ack_carries_vtx_nonce_and_flags) {
  DiscAck a; a.vrx_nonce = 0xCAFE0001; a.vtx_nonce = 0xBEEF0002; a.chip_caps = 3;
  a.agreed_channel = 149; a.agreed_width = 20; a.flags = kAckKeyMismatch; a.seq = 9;
  auto w = pack_disc_ack(a);
  CHECK(w.size() == 20 + 2);
  auto p = parse_disc_ack(w.data(), w.size());
  REQUIRE(p.has_value());
  CHECK(p->vtx_nonce == 0xBEEF0002);
  CHECK(p->flags == kAckKeyMismatch);
  CHECK(p->seq == 9);
}

TEST(rcf_matches_golden_wire) {
  // Golden pin: mabur owns these bytes (devourer's frozen Python is stuck
  // at RC_VERSION 1). Print-once, then hardcode:
  //   std::fprintf(stderr, "%s\n", mtest::hex(wire).c_str());
  // Reverting any pack_rcf() layout change without updating these fails
  // here, which is the point -- the format cannot drift silently.
  const std::vector<std::string> GOLDEN = {
      "43520e01000700243232ff000000003664ea56cbbade5c4e75",
      "43520e0100ffff006464ff00000300738f96419a1ccbd2dea3",
      // Asym pair (base 1.0 / enh 0.5): ENH actually rides a different
      // literal overhead than BASE here, not a duplicated equal-pair scalar.
      "43520e01002a00086432060000025a4caf94dbea454ba981bf",
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
    CHECK(parsed->seq == r.seq);
    CHECK(parsed->profile == r.profile);
    CHECK(std::abs(parsed->fec_overhead_base - r.fec_overhead_base) < 1e-9);
    CHECK(std::abs(parsed->fec_overhead_enh - r.fec_overhead_enh) < 1e-9);
    CHECK(parsed->rec == r.rec);
    CHECK(parsed->idr_epoch == r.idr_epoch);
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
      "43520e02040100feca9514010000000200088800cd1879d75c788b",
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
      "43520e03040100feca0200efbe030095140001008ff9",
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
    CHECK(parsed->vrx_nonce == a.vrx_nonce);
    CHECK(parsed->vtx_nonce == a.vtx_nonce);
    CHECK(parsed->chip_caps == a.chip_caps);
    CHECK(parsed->agreed_channel == a.agreed_channel);
    CHECK(parsed->agreed_width == a.agreed_width);
    CHECK(parsed->flags == a.flags);
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

TEST(telem_round_trip_and_golden) {
  mabur::rc::Telem t;
  t.tlm_seq = 0x0102; t.state = 2; t.flags = 0x09;  // failsafe_shed | echo valid
  t.rcf_age_ms = 45; t.rcf_seq_echo = 0x1234;
  t.pts_at_build = 0x0011223344556677ull;
  t.rcf_rx = 100000; t.cmd_kbps = 9000;
  t.txq_drops = 7; t.txq_wait_max_ms = 1234; t.usb_fail = 2;
  t.up_rssi[0] = 51; t.up_rssi[1] = 52; t.up_snr[0] = 21; t.up_snr[1] = 22;
  t.soc_temp_c = 61; t.cpu_busy_x100 = 72;
  // RX-side channel view per telemetry period (cca-on 2026-09-23): the
  // drone's own frame split -- ours (RC from the GS), foreign CRC-clean,
  // and CRC-failed (preamble heard, payload not decodable).
  t.rx_own = 13; t.rx_foreign = 14; t.rx_crcfail = 15;
  // VTX recorder status (spec 2026-09-26): state 2 (Error) | err 5 (LowSpace) << 2.
  t.rec_status = 0x16;
  auto wire = mabur::rc::pack_telem(t);
  CHECK(wire.size() == 48 + 2);
  CHECK(mabur::rc::frame_type(wire.data(), wire.size()) == mabur::rc::T_TELEM);
  auto back = mabur::rc::parse_telem(wire.data(), wire.size());
  REQUIRE(back.has_value());
  CHECK(back->tlm_seq == t.tlm_seq);
  CHECK(back->state == 2);
  CHECK(back->flags == 0x09);
  CHECK(back->rcf_age_ms == 45);
  CHECK(back->rcf_seq_echo == 0x1234);
  CHECK(back->pts_at_build == 0x0011223344556677ull);
  CHECK(back->rcf_rx == t.rcf_rx);
  CHECK(back->cmd_kbps == 9000);
  CHECK(back->txq_drops == 7);
  CHECK(back->txq_wait_max_ms == 1234);
  CHECK(back->usb_fail == 2);
  CHECK(back->up_rssi[0] == 51); CHECK(back->up_rssi[1] == 52);
  CHECK(back->up_snr[0] == 21); CHECK(back->up_snr[1] == 22);
  CHECK(back->soc_temp_c == 61);
  CHECK(back->cpu_busy_x100 == 72);
  CHECK(back->rx_own == 13);
  CHECK(back->rx_foreign == 14);
  CHECK(back->rx_crcfail == 15);
  CHECK(back->rec_status == 0x16);
  // Golden pin: byte-exact wire so the format can never drift silently.
  // Computed independently of pack_telem (2026-09-30: python struct.pack of
  // the documented layout + CRC16-CCITT init 0xFFFF), not printed from it.
  // Re-pinned 2026-10-01 for the RC_VERSION 13 -> 14 bump (version byte 2
  // moved, re-deriving the CRC over the changed body) -- Telem's own
  // layout is untouched.
  const std::string GOLDEN =
      "43520e04090201022d0034127766554433221100a0860100282307000000d2040200"
      "333415163d48000d000e000f0016cf9d";
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

TEST(rcf_probe_profile_is_a_fixed_head_byte) {
  Rcf r;
  r.seq = 2; r.profile = 0x04;
  r.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 6, 20);
  auto wire = mabur::rc::pack_rcf(r);
  Rcf none = r;
  none.probe_profile = kNoProbeProfile;
  auto wire_none = mabur::rc::pack_rcf(none);
  CHECK(wire.size() == wire_none.size());   // fixed byte, no optional tail
  CHECK(wire.size() == 15 + kTagLen + 2);    // head 15 + tag + crc
  CHECK(wire[4] == 0);                       // flags byte carries nothing
  CHECK(wire[10] == r.probe_profile);
  CHECK(wire_none[10] == 0xFF);
  auto p = mabur::rc::parse_rcf(wire.data(), wire.size());
  REQUIRE(p.has_value());
  CHECK(p->probe_profile == r.probe_profile);
  auto pn = mabur::rc::parse_rcf(wire_none.data(), wire_none.size());
  REQUIRE(pn.has_value());
  CHECK(pn->probe_profile == kNoProbeProfile);
}

TEST(rcf_v5_wire_is_rejected) {
  Rcf r; r.seq = 1; r.profile = 0;
  auto wire = mabur::rc::pack_rcf(r);
  wire[2] = 5;  // old version byte; CRC no longer matches either, but the
                // version check fires first and is the point of this test
  CHECK(!mabur::rc::parse_rcf(wire.data(), wire.size()).has_value());
}

TEST(version_mismatch_rejected_both_directions) {
  // Reverting the RC_VERSION bump in rc_proto.h makes the doctored frame
  // become current-version, so it parses and the first CHECK fails.
  mabur::rc::Rcf r;
  r.seq = 1;
  r.profile = 0;
  r.fec_overhead_base = 0.25;
  r.fec_overhead_enh = 0.25;
  auto body = mabur::rc::pack_rcf(r);

  // Sanity: as packed, it parses.
  CHECK(mabur::rc::parse_rcf(body.data(), body.size()).has_value());

  // Byte 2 is the version. Any other version must be refused outright —
  // including 13, the version before the 2026-10-01 bump to 14.
  auto v_old = body;
  v_old[2] = 13;
  CHECK(!mabur::rc::parse_rcf(v_old.data(), v_old.size()).has_value());

  auto v_future = body;
  v_future[2] = 15;
  CHECK(!mabur::rc::parse_rcf(v_future.data(), v_future.size()).has_value());

  // The same guard must hold for telemetry, which travels the opposite
  // direction (drone -> GS). A half-deployed pair must fail BOTH ways.
  mabur::rc::Telem t;
  t.tlm_seq = 9;
  auto tb = mabur::rc::pack_telem(t);
  CHECK(mabur::rc::parse_telem(tb.data(), tb.size()).has_value());
  auto tv1 = tb;
  tv1[2] = 1;
  CHECK(!mabur::rc::parse_telem(tv1.data(), tv1.size()).has_value());
  auto tv12 = tb;
  tv12[2] = 12;
  CHECK(!mabur::rc::parse_telem(tv12.data(), tv12.size()).has_value());
}

TEST(rcf_head_is_fifteen_bytes) {
  mabur::rc::Rcf r; r.seq = 7; r.profile = 0x24;
  r.fec_overhead_base = 0.42; r.fec_overhead_enh = 0.37; r.hop_ch = 149; r.hop_epoch = 3;
  r.rec = 0x05;
  r.idr_epoch = 0xA7;
  auto body = mabur::rc::pack_rcf(r);
  CHECK(body.size() == 15 + mabur::rc::kTagLen + 2);
  CHECK(body[8] == 42); CHECK(body[9] == 37); CHECK(body[10] == mabur::rc::kNoProbeProfile);
  CHECK(body[11] == 149); CHECK(body[12] == 3); CHECK(body[13] == 5);
  CHECK(body[14] == 0xA7);
  auto back = mabur::rc::parse_rcf(body.data(), body.size());
  REQUIRE(back.has_value());
  CHECK(back->hop_ch == 149); CHECK(back->hop_epoch == 3); CHECK(back->rec == 5);
  CHECK(back->idr_epoch == 0xA7);
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

TEST(cal_cmd_round_trip) {
  mabur::rc::CalCmd c;
  c.nonce = 0x12345678;
  c.phase = mabur::cal::kPhaseCoarse;
  c.frames_per_cell = 20;
  c.settle_ms = 100;
  c.gap_us = 2000;
  c.windows = {{0, -40, 60, 4}, {7, -8, 8, 1}};
  auto b = mabur::rc::pack_cal_cmd(c);
  CHECK(mabur::rc::frame_type(b.data(), b.size()) == mabur::rc::T_CAL_CMD);
  auto got = mabur::rc::parse_cal_cmd(b.data(), b.size());
  REQUIRE(got.has_value());
  CHECK(got->nonce == 0x12345678);
  CHECK(got->phase == mabur::cal::kPhaseCoarse);
  CHECK(got->frames_per_cell == 20);
  CHECK(got->settle_ms == 100);
  CHECK(got->gap_us == 2000);
  REQUIRE(got->windows.size() == 2);
  CHECK(got->windows[0].rate == 0);
  CHECK(got->windows[0].idx_lo == -40);
  CHECK(got->windows[0].idx_hi == 60);
  CHECK(got->windows[0].idx_step == 4);
  CHECK(got->windows[1].rate == 7);
  CHECK(got->windows[1].idx_lo == -8);
}

TEST(cal_cmd_rejects_corrupt_crc) {
  mabur::rc::CalCmd c;
  c.windows = {{0, 0, 124, 4}};
  auto b = mabur::rc::pack_cal_cmd(c);
  b[b.size() - 1] ^= 0xFF;
  CHECK(!mabur::rc::parse_cal_cmd(b.data(), b.size()).has_value());
}

TEST(cal_cmd_rejects_bad_window_count) {
  mabur::rc::CalCmd c;
  c.windows = {{0, 0, 124, 4}};
  auto b = mabur::rc::pack_cal_cmd(c);
  // n_windows sits after hdr(5) + nonce(4) + phase(1) + three
  // u16s(6) = offset 16. Claim 9 windows; the max is 8.
  const size_t n_off = 5 + 4 + 1 + 2 + 2 + 2;  // 16
  b[n_off] = 9;
  CHECK(!mabur::rc::parse_cal_cmd(b.data(), b.size()).has_value());
}

TEST(cal_cmd_rejects_window_outside_relative_range) {
  mabur::rc::CalCmd c;
  c.windows = {{0, -40, 60, 4}};
  auto b = mabur::rc::pack_cal_cmd(c);
  // Window bytes start at kCalCmdFixedLen (17): rate, idx_lo, idx_hi, step.
  b[18] = static_cast<uint8_t>(-70);  // idx_lo below -64
  // A bad CRC also rejects, so re-sign the body: mabur::crc16_ccitt from
  // common/include/mabur/crc16.h, little-endian, exactly as put_crc() in
  // rc_proto.cpp writes it (check put_crc's byte order and match it).
  const size_t plen = b.size() - 2;
  const uint16_t crc = mabur::crc16_ccitt(b.data(), plen);
  b[plen] = static_cast<uint8_t>(crc & 0xFF);
  b[plen + 1] = static_cast<uint8_t>(crc >> 8);
  CHECK(!mabur::rc::parse_cal_cmd(b.data(), b.size()).has_value());
}

TEST(cal_result_round_trip) {
  mabur::rc::CalResult r;
  r.nonce = 99;
  r.walls = {91, 91, 91, 95, 73, 54, 51, 49};
  r.legacy_wall = 91;
  auto b = mabur::rc::pack_cal_result(r);
  CHECK(mabur::rc::frame_type(b.data(), b.size()) == mabur::rc::T_CAL_RESULT);
  auto got = mabur::rc::parse_cal_result(b.data(), b.size());
  REQUIRE(got.has_value());
  CHECK(got->walls[3] == 95);
  CHECK(got->walls[7] == 49);
  CHECK(got->legacy_wall == 91);
}

TEST(cal_result_sentinel_is_minus_128_and_minus_1_is_a_real_wall) {
  // Relative walls make -1 a legal value (one index below the anchor), so
  // "no wall could be derived" is kWallUndetermined (-128), never -1.
  mabur::rc::CalResult r;
  r.walls = {63, 63, 63, mabur::rc::kWallUndetermined, 20, 1, -2, -1};
  r.legacy_wall = 63;
  auto b = mabur::rc::pack_cal_result(r);
  auto got = mabur::rc::parse_cal_result(b.data(), b.size());
  REQUIRE(got.has_value());
  CHECK(got->walls[3] == mabur::rc::kWallUndetermined);
  CHECK(got->walls[6] == -2);
  CHECK(got->walls[7] == -1);
}

TEST(telem_ack_is_the_cal_active_bit_alone) {
  // The anchor never leaves the drone (spec 2026-09-13): the calibration
  // ack is flags bit6 and nothing else. TELEM_LEN shrank 88 -> 87 (95 since 2026-09-23, +rx_*), now 96 since 2026-09-26 (+rec_status), 98 since 2026-09-28 (+idr_gs), 48 since 2026-09-30 (fields no GS consumer needs dropped).
  mabur::rc::Telem t;
  t.flags = 0x40;
  auto b = mabur::rc::pack_telem(t);
  CHECK(b.size() == 48 + 2);  // body + crc16
  auto got = mabur::rc::parse_telem(b.data(), b.size());
  REQUIRE(got.has_value());
  CHECK((got->flags & 0x40) != 0);
}

TEST(rc_version_is_fourteen) { CHECK(mabur::rc::RC_VERSION == 14); }

MTEST_MAIN
