#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "mtest.h"
#include "vectors.h"
#include "mabur/profile.h"
using namespace mabur;
using namespace mabur::rc;

namespace {
PhyMode mode_from_json(const std::string& s) {
  return s == "vht" ? PhyMode::VHT : PhyMode::HT;
}

// Independent (test-side) parser for one "CRIT=...;T0=...;T1=...;T2=..."
// ladder-spec string, used only to build an oracle to compare
// ladder_for_row() against — deliberately NOT sharing code with the
// production parser in profile.cpp.
struct ParsedRung { uint8_t mcs = 0; uint8_t bw = 20; bool ldpc = false, stbc = false, sgi = false; };

ParsedRung parse_rung_token(const std::string& tok) {
  // tok looks like "MCS0/20/LDPC" or "VHT1SS_MCS7/40/SGI" (mcs/bw then zero
  // or more of /LDPC /STBC /SGI in any order).
  std::vector<std::string> parts;
  std::stringstream ss(tok);
  std::string part;
  while (std::getline(ss, part, '/')) parts.push_back(part);
  ParsedRung r;
  // parts[0] = "MCSn" or "VHT1SS_MCSn"
  size_t pos = parts[0].find("MCS");
  r.mcs = static_cast<uint8_t>(std::stoi(parts[0].substr(pos + 3)));
  r.bw = static_cast<uint8_t>(std::stoi(parts[1]));
  for (size_t i = 2; i < parts.size(); ++i) {
    if (parts[i] == "LDPC") r.ldpc = true;
    else if (parts[i] == "STBC") r.stbc = true;
    else if (parts[i] == "SGI") r.sgi = true;
  }
  return r;
}

std::array<ParsedRung, 4> parse_ladder_str(const std::string& s) {
  // "CRIT=X;T0=Y;T1=Z;T2=W"
  std::array<ParsedRung, 4> out;
  std::stringstream ss(s);
  std::string clause;
  int i = 0;
  while (std::getline(ss, clause, ';')) {
    size_t eq = clause.find('=');
    out[static_cast<size_t>(i)] = parse_rung_token(clause.substr(eq + 1));
    ++i;
  }
  return out;
}
}  // namespace

TEST(profile_byte_encode_decode_ladder_matches_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/profile.json");
  for (auto& c : j["profiles"]) {
    PhyMode mode = mode_from_json(c["mode"].get<std::string>());
    uint8_t mcs = c["mcs"].get<uint8_t>();
    uint8_t bw = c["bw"].get<uint8_t>();
    uint8_t expect_byte = c["byte"].get<uint8_t>();

    uint8_t byte = encode_profile(mode, mcs, bw);
    CHECK(byte == expect_byte);

    PhyMode dmode;
    uint8_t dmcs, dbw;
    decode_profile(byte, dmode, dmcs, dbw);
    CHECK(dmode == mode);
    CHECK(dmcs == mcs);
    CHECK(dbw == bw);

    std::string ladder = ladder_spec_str(mode, mcs, bw);
    CHECK(ladder == c["ladder"].get<std::string>());
  }
}

TEST(decode_profile_clamps_mcs) {
  // HT: mcs bits go up to 15 (bits[3:0]), clamp to 7.
  PhyMode mode;
  uint8_t mcs, bw;
  decode_profile(0x0F, mode, mcs, bw);  // ht, mcs=15, bw code 0 (20)
  CHECK(mode == PhyMode::HT);
  CHECK(mcs == 7);
  CHECK(bw == 20);

  // VHT: bit6 set, mcs=15 clamp to 8.
  decode_profile(0x4F, mode, mcs, bw);
  CHECK(mode == PhyMode::VHT);
  CHECK(mcs == 8);
  CHECK(bw == 20);
}

TEST(probe_bw_matches_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/profile.json");
  for (auto& c : j["probe"]) {
    std::vector<uint8_t> bw_set;
    for (auto& v : c["bw_set"]) bw_set.push_back(v.get<uint8_t>());

    auto& probes = c["probe"];
    for (size_t seq = 0; seq < probes.size(); ++seq) {
      int expect = probes[seq].is_null() ? -1 : probes[seq].get<int>();
      int got = probe_bw(static_cast<uint16_t>(seq), bw_set);
      CHECK(got == expect);
    }
  }
}

TEST(phy_rate_mbps_matches_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/profile.json");
  for (auto& c : j["rates"]) {
    LayerTxSpec s;
    s.mode = mode_from_json(c["mode"].get<std::string>());
    s.mcs = c["mcs"].get<uint8_t>();
    s.bw = c["bw"].get<uint8_t>();
    s.sgi = c["sgi"].get<bool>();
    double expect = c["mbps"].get<double>();
    double got = phy_rate_mbps(s);
    CHECK(got > expect - 1e-9 && got < expect + 1e-9);
  }
}

TEST(profile_table_matches_vectors_verbatim) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/profile.json");
  auto& table = profile_table();
  auto& expect_rows = j["table"];
  REQUIRE(table.size() == expect_rows.size());
  for (size_t i = 0; i < table.size(); ++i) {
    CHECK(std::string(table[i].svc_ladder) == expect_rows[i]["ladder"].get<std::string>());
    CHECK(table[i].pwr_offset_qdb == expect_rows[i]["pwr_offset_qdb"].get<int>());
    double ov = table[i].fec_overhead;
    double expect_ov = expect_rows[i]["ov"].get<double>();
    CHECK(ov > expect_ov - 1e-9 && ov < expect_ov + 1e-9);
    CHECK(table[i].bw == expect_rows[i]["bw"].get<uint8_t>());
  }
  CHECK(MAX_RANGE_PROFILE == 0);
}

TEST(ladder_from_applies_default_flag_policy) {
  FlagPolicy fp;  // defaults: all true
  auto ladder = ladder_from(PhyMode::HT, 2, 20, fp);
  // CRIT
  CHECK(ladder[0].mode == PhyMode::HT);
  CHECK(ladder[0].mcs == 2);
  CHECK(ladder[0].bw == 20);
  CHECK(ladder[0].ldpc == true);
  CHECK(ladder[0].stbc == true);
  // T0
  CHECK(ladder[1].mode == PhyMode::HT);
  CHECK(ladder[1].mcs == 2);
  CHECK(ladder[1].bw == 20);
  CHECK(ladder[1].ldpc == true);
  CHECK(ladder[1].stbc == true);
  // T1 — identical PHY to T0 (hw 2026-07-26 follow-up: all rungs ride the
  // scored base rate AND T0's ldpc/stbc; FEC overhead is the only
  // per-layer differentiator).
  CHECK(ladder[2].mode == PhyMode::HT);
  CHECK(ladder[2].mcs == 2);
  CHECK(ladder[2].bw == 20);
  CHECK(ladder[2].ldpc == true);
  CHECK(ladder[2].stbc == true);
  // T2
  CHECK(ladder[3].mode == PhyMode::HT);
  CHECK(ladder[3].mcs == 2);
  CHECK(ladder[3].bw == 20);
  CHECK(ladder[3].ldpc == true);
  CHECK(ladder[3].stbc == true);
}

TEST(ladder_from_respects_disabled_flags) {
  FlagPolicy fp;
  fp.crit_ldpc = false;
  fp.crit_stbc = false;
  fp.t0_ldpc = false;
  fp.t0_stbc = false;
  auto ladder = ladder_from(PhyMode::HT, 2, 20, fp);
  CHECK(ladder[0].ldpc == false);
  CHECK(ladder[0].stbc == false);
  CHECK(ladder[1].ldpc == false);
  CHECK(ladder[1].stbc == false);
  CHECK(ladder[2].ldpc == false);
  CHECK(ladder[2].stbc == false);
  CHECK(ladder[3].ldpc == false);
  CHECK(ladder[3].stbc == false);
}

TEST(ladder_from_clamps_at_top) {
  // HT top = 7: an out-of-range base mcs clamps to 7, and every rung rides
  // that same clamped value (no more per-rung +1/+2 offset to clamp
  // separately).
  FlagPolicy fp;
  auto ladder = ladder_from(PhyMode::HT, 9, 20, fp);
  CHECK(ladder[0].mcs == 7);
  CHECK(ladder[1].mcs == 7);
  CHECK(ladder[2].mcs == 7);
  CHECK(ladder[3].mcs == 7);

  // VHT top = 8: mcs=10 clamps to 8, all rungs match.
  auto vladder = ladder_from(PhyMode::VHT, 10, 40, fp);
  CHECK(vladder[0].mcs == 8);
  CHECK(vladder[1].mcs == 8);
  CHECK(vladder[2].mcs == 8);
  CHECK(vladder[3].mcs == 8);
}

TEST(ladder_for_row_matches_table_strings_verbatim) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/profile.json");
  auto& expect_rows = j["table"];
  FlagPolicy fp;  // defaults: all true
  for (size_t idx = 0; idx < 5; ++idx) {
    auto oracle = parse_ladder_str(expect_rows[idx]["ladder"].get<std::string>());
    auto ladder = ladder_for_row(static_cast<int>(idx), fp);
    // CRIT and T0 come straight from the vendored row string (plus flag
    // policy) — the table itself stays a byte-exact devourer port.
    for (int i = 0; i < 2; ++i) {
      CHECK(ladder[static_cast<size_t>(i)].mode == PhyMode::HT);
      CHECK(ladder[static_cast<size_t>(i)].mcs == oracle[static_cast<size_t>(i)].mcs);
      CHECK(ladder[static_cast<size_t>(i)].bw == oracle[static_cast<size_t>(i)].bw);
      CHECK(ladder[static_cast<size_t>(i)].sgi == oracle[static_cast<size_t>(i)].sgi);
    }
    // FlagPolicy only ADDS ldpc/stbc to CRIT/T0 — never removes what the
    // committed table string already sets.
    CHECK(ladder[0].ldpc == (oracle[0].ldpc || fp.crit_ldpc));
    CHECK(ladder[0].stbc == (oracle[0].stbc || fp.crit_stbc));
    CHECK(ladder[1].ldpc == (oracle[1].ldpc || fp.t0_ldpc));
    CHECK(ladder[1].stbc == (oracle[1].stbc || fp.t0_stbc));
    // hw 2026-07-26 ruling (+ same-day follow-up): T1/T2 ride T0's ENTIRE
    // spec — rate fields and policy-applied ldpc/stbc alike — regardless
    // of what the vendored row string carries for T1/T2 (some rows still
    // spell out a devourer-style spread there; it's ignored by the applied
    // ladder). Uniform PHY across all video streams; only FEC overhead
    // differentiates layers. (Flags-off T1/T2 was tried first and measured
    // 2-3 dB weaker on air — single-chain, no coding gain — with the
    // weaker GS card losing 3-5x more of stream 3's frames.)
    for (int i = 2; i < 4; ++i) {
      CHECK(ladder[static_cast<size_t>(i)].mode == ladder[1].mode);
      CHECK(ladder[static_cast<size_t>(i)].mcs == ladder[1].mcs);
      CHECK(ladder[static_cast<size_t>(i)].bw == ladder[1].bw);
      CHECK(ladder[static_cast<size_t>(i)].sgi == ladder[1].sgi);
      CHECK(ladder[static_cast<size_t>(i)].ldpc == ladder[1].ldpc);
      CHECK(ladder[static_cast<size_t>(i)].stbc == ladder[1].stbc);
    }
  }
}

TEST(ladder_for_row_flattens_t1_t2_to_t0_rate) {
  // Row 3's vendored string is "CRIT=MCS2/20;T0=MCS4/20;T1=MCS5/20;
  // T2=MCS7/20/SGI" — a real per-rung spread straight from the
  // byte-exact devourer port. hw 2026-07-26 ruling: streams above CRIT
  // ride T0's scored rate; only FEC overhead differentiates them. Pin
  // that the *applied* ladder enforces this even though the committed
  // table row keeps devourer's own spread. FlagPolicy defaults all true,
  // and T1/T2 take T0's WHOLE spec — rate fields and policy-applied
  // ldpc/stbc alike (hw 2026-07-26 follow-up: uniform PHY; flags-off T1/T2
  // measured 2-3 dB weaker on air at the same MCS).
  FlagPolicy fp;
  auto ladder = ladder_for_row(3, fp);

  // T0/T1/T2 identical in rate fields (mode/mcs/bw/sgi).
  CHECK(ladder[1].mode == ladder[2].mode);
  CHECK(ladder[1].mcs == ladder[2].mcs);
  CHECK(ladder[1].bw == ladder[2].bw);
  CHECK(ladder[1].sgi == ladder[2].sgi);
  CHECK(ladder[1].mode == ladder[3].mode);
  CHECK(ladder[1].mcs == ladder[3].mcs);
  CHECK(ladder[1].bw == ladder[3].bw);
  CHECK(ladder[1].sgi == ladder[3].sgi);

  // T0's policy-applied flags (defaults all true) carry onto T1/T2 —
  // identical PHY, matching ladder_from.
  CHECK(ladder[1].ldpc == true);
  CHECK(ladder[1].stbc == true);
  CHECK(ladder[2].ldpc == true);
  CHECK(ladder[2].stbc == true);
  CHECK(ladder[3].ldpc == true);
  CHECK(ladder[3].stbc == true);

  // Concretely: T0 in row 3 is MCS4/20, no SGI. T1/T2 must match that —
  // NOT the row string's own MCS5 / MCS7-SGI.
  CHECK(ladder[1].mcs == 4);
  CHECK(ladder[2].mcs == 4);
  CHECK(ladder[3].mcs == 4);
  CHECK(ladder[2].sgi == false);
  CHECK(ladder[3].sgi == false);

  // CRIT keeps its own, more-protected row spec — independent of T0/T1/T2.
  CHECK(ladder[0].mcs == 2);
  CHECK(ladder[0].mcs != ladder[1].mcs);
}

TEST(ladder_for_row_disabled_flags_keep_string_flags) {
  // Row 0's CRIT/T0 already set LDPC in the string; disabling the policy
  // must NOT strip that (policy only adds, never removes).
  FlagPolicy fp;
  fp.crit_ldpc = false;
  fp.crit_stbc = false;
  fp.t0_ldpc = false;
  fp.t0_stbc = false;
  auto ladder = ladder_for_row(0, fp);
  CHECK(ladder[0].ldpc == true);   // string sets it
  CHECK(ladder[0].stbc == false);  // string never sets stbc; policy off -> off
  CHECK(ladder[1].ldpc == true);   // string sets it (T0 also /LDPC in row 0)
  CHECK(ladder[1].stbc == false);
}

MTEST_MAIN
