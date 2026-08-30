#include <cstdint>
#include <string>

#include "mtest.h"
#include "vectors.h"
#include "mabur/profile.h"
using namespace mabur;
using namespace mabur::rc;

namespace {
PhyMode mode_from_json(const std::string& s) {
  return s == "vht" ? PhyMode::VHT : PhyMode::HT;
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
    CHECK(table[i].mcs == expect_rows[i]["mcs"].get<uint8_t>());
    double expect_ov = expect_rows[i]["ov"].get<double>();
    // Both slots duplicate the same overhead value (same-rate ruling,
    // 2026-08-30): no rate split, ov_base == ov_enh for every rung.
    double ov_base = table[i].ov_base;
    double ov_enh = table[i].ov_enh;
    CHECK(ov_base > expect_ov - 1e-9 && ov_base < expect_ov + 1e-9);
    CHECK(ov_enh > expect_ov - 1e-9 && ov_enh < expect_ov + 1e-9);
    CHECK(table[i].bw == expect_rows[i]["bw"].get<uint8_t>());
  }
  CHECK(MAX_RANGE_PROFILE == 0);
}

TEST(ladder_from_applies_same_rate) {
  // 2026-08-30 ruling: both slots ride the scored mcs (no rate split).
  auto l5 = rc::ladder_from(rc::PhyMode::HT, 5, 20);
  CHECK(l5[0].mcs == 5);   // base
  CHECK(l5[1].mcs == 5);   // enh
  CHECK(l5[0].ldpc && l5[0].stbc && l5[1].ldpc && l5[1].stbc);
  auto l0 = rc::ladder_from(rc::PhyMode::HT, 0, 20);
  CHECK(l0[0].mcs == 0);   // clamp: streams equal at the floor
  CHECK(l0[1].mcs == 0);
}

TEST(ladder_from_sets_ldpc_stbc_on_all_rungs) {
  // Config policy removed 2026-07-26: LDPC+STBC are unconditionally on for
  // every rung (the all-true policy was the only shape ever flown).
  auto ladder = ladder_from(PhyMode::HT, 2, 20);
  // BASE
  CHECK(ladder[0].mode == PhyMode::HT);
  CHECK(ladder[0].mcs == 2);  // same-rate ruling, 2026-08-30
  CHECK(ladder[0].bw == 20);
  CHECK(ladder[0].ldpc == true);
  CHECK(ladder[0].stbc == true);
  // ENH
  CHECK(ladder[1].mode == PhyMode::HT);
  CHECK(ladder[1].mcs == 2);
  CHECK(ladder[1].bw == 20);
  CHECK(ladder[1].ldpc == true);
  CHECK(ladder[1].stbc == true);
}

TEST(ladder_from_clamps_at_top) {
  // HT top = 7: an out-of-range mcs clamps to 7; both slots ride it
  // (same-rate ruling, 2026-08-30).
  auto ladder = ladder_from(PhyMode::HT, 9, 20);
  CHECK(ladder[0].mcs == 7);
  CHECK(ladder[1].mcs == 7);

  // VHT top = 8: mcs=10 clamps to 8; both slots ride it.
  auto vladder = ladder_from(PhyMode::VHT, 10, 40);
  CHECK(vladder[0].mcs == 8);
  CHECK(vladder[1].mcs == 8);
}

TEST(ladder_for_row_matches_table_ladder_from) {
  // ladder_for_row(idx) must be exactly ladder_from(mode, row.mcs, bw) — the
  // table now carries a bare mcs, not a spec string, so there is nothing
  // left to parse per-row.
  auto& table = profile_table();
  for (size_t idx = 0; idx < table.size(); ++idx) {
    const auto& row = table[idx];
    auto expect = ladder_from(PhyMode::HT, row.mcs, row.bw);
    auto ladder = ladder_for_row(static_cast<int>(idx));
    CHECK(ladder[0].mode == expect[0].mode);
    CHECK(ladder[0].mcs == expect[0].mcs);
    CHECK(ladder[0].bw == expect[0].bw);
    CHECK(ladder[0].ldpc == expect[0].ldpc);
    CHECK(ladder[0].stbc == expect[0].stbc);
    CHECK(ladder[1].mode == expect[1].mode);
    CHECK(ladder[1].mcs == expect[1].mcs);
    CHECK(ladder[1].bw == expect[1].bw);
    CHECK(ladder[1].ldpc == expect[1].ldpc);
    CHECK(ladder[1].stbc == expect[1].stbc);
  }
}

TEST(ladder_for_row_applies_same_rate_rule) {
  // Row 3 carries T0 mcs 4 (see profile_table): both slots ride 4
  // (same-rate ruling, 2026-08-30).
  auto ladder = ladder_for_row(3);
  CHECK(ladder[0].mcs == 4);
  CHECK(ladder[1].mcs == 4);
  CHECK(ladder[0].ldpc == true);
  CHECK(ladder[0].stbc == true);
  CHECK(ladder[1].ldpc == true);
  CHECK(ladder[1].stbc == true);
}

MTEST_MAIN
