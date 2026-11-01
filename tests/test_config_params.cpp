#include "mtest.h"
#include "mabur/config_params.h"
#include <cstring>
#include <set>
#include <string>
using namespace mabur::cfgparam;
TEST(fourteen_rows_in_spec_order) {
  REQUIRE(kCount == 14);
  CHECK(!std::strcmp(kTable[0].id, "RADIO_WALL_MARGN") && !std::strcmp(kTable[0].section, "radio") && !std::strcmp(kTable[0].key, "wall_margin_db") && kTable[0].type == Type::Real32);
  CHECK(!std::strcmp(kTable[1].id, "RADIO_LDPC")     && kTable[1].type == Type::Bool);
  CHECK(!std::strcmp(kTable[2].id, "VENC_MODE")      && kTable[2].type == Type::Str);
  CHECK(!std::strcmp(kTable[3].id, "VENC_SLICES"));
  CHECK(!std::strcmp(kTable[4].id, "ENC_BITRATE_MAX") && !std::strcmp(kTable[4].section, "encoder") && !std::strcmp(kTable[4].key, "bitrate_max_kbps"));
  CHECK(!std::strcmp(kTable[5].id, "ENC_AIRTIME")    && kTable[5].type == Type::Real32);
  CHECK(!std::strcmp(kTable[6].id, "REC_ENABLE")     && !std::strcmp(kTable[6].section, "record"));
  CHECK(!std::strcmp(kTable[7].id, "REC_BITRATE"));
  CHECK(!std::strcmp(kTable[8].id, "REC_FPS")        && !std::strcmp(kTable[8].key, "fps"));
  CHECK(!std::strcmp(kTable[9].id, "REC_SIZE")       && kTable[9].type == Type::Str);
  CHECK(!std::strcmp(kTable[10].id, "LOWPWR_ENABLE") && !std::strcmp(kTable[10].section, "low_power"));
  CHECK(!std::strcmp(kTable[11].id, "MSP_ENABLE")    && !std::strcmp(kTable[11].section, "msp"));
  CHECK(!std::strcmp(kTable[12].id, "VENC_MIRROR")   && !std::strcmp(kTable[12].section, "venc") && !std::strcmp(kTable[12].key, "mirror") && kTable[12].type == Type::Bool);
  CHECK(!std::strcmp(kTable[13].id, "VENC_FLIP")     && !std::strcmp(kTable[13].section, "venc") && !std::strcmp(kTable[13].key, "flip") && kTable[13].type == Type::Bool);
  CHECK(kVencModeRow == 2);
  CHECK(find("VENC_MODE") == 2);
  CHECK(find("VENC_FPS") == -1);
}
TEST(parse_mode_text_shape) {
  int f, w, h;
  CHECK(parse_mode_text("120@1728x816", &f, &w, &h) && f == 120 && w == 1728 && h == 816);
  CHECK(parse_mode_text("1000@1x1", &f, &w, &h));          // shape only; the loader refuses the range
  CHECK(!parse_mode_text("100", &f, &w, &h));  CHECK(!parse_mode_text("100@", &f, &w, &h));
  CHECK(!parse_mode_text("100@1728", &f, &w, &h));  CHECK(!parse_mode_text("abc@1x1", &f, &w, &h));
  CHECK(!parse_mode_text("100@1728x972 ", &f, &w, &h));  CHECK(!parse_mode_text("@1x1", &f, &w, &h));
  // Six digits per number at most: a longer run is a shape refusal, never
  // a value wrapped back into range.
  CHECK(parse_mode_text("999999@999999x999999", &f, &w, &h) && f == 999999 && h == 999999);
  CHECK(!parse_mode_text("1000000@1x1", &f, &w, &h));
  CHECK(!parse_mode_text("60@1x1000000", &f, &w, &h));
  CHECK(!parse_mode_text("60@1000000x1", &f, &w, &h));
  CHECK(!parse_mode_text("4294967356@1x1", &f, &w, &h));
}
TEST(ids_unique_and_at_most_16_chars_with_help) {
  std::set<std::string> ids;
  for (const Row& r : kTable) {
    CHECK(std::strlen(r.id) <= 16 && std::strlen(r.id) > 0);
    CHECK(ids.insert(r.id).second);
    CHECK(std::strlen(r.help) > 10);
  }
}
TEST(find_compares_at_most_16_bytes) {
  char wire[16]; std::memcpy(wire, "RADIO_WALL_MARGN", 16);     // exactly 16, no NUL
  CHECK(find(wire) == 0);
  CHECK(find("VENC_MODE") == 2);
  CHECK(find("NOPE") == -1);
}
TEST(mav_type_values) {
  CHECK(mav_type(Type::Bool) == 1 && mav_type(Type::Int32) == 6 && mav_type(Type::Real32) == 9 && mav_type(Type::Str) == 11);
}
TEST(apply_column_matches_the_mavlink_doc) {
  CHECK(kTable[0].apply == Apply::Reprogram && kTable[1].apply == Apply::Reprogram);   // RADIO_*
  CHECK(kTable[2].apply == Apply::Rebuild && kTable[3].apply == Apply::Rebuild);        // VENC_MODE, VENC_SLICES
  CHECK(kTable[4].apply == Apply::Knob && kTable[5].apply == Apply::Knob);
  for (size_t i = 6; i <= 9; ++i) CHECK(kTable[i].apply == Apply::Rebuild);             // REC_*
  CHECK(kTable[10].apply == Apply::Knob && kTable[11].apply == Apply::Reprogram);
  CHECK(kTable[12].apply == Apply::Rebuild && kTable[13].apply == Apply::Rebuild);      // VENC_MIRROR, VENC_FLIP
  CHECK(!std::strcmp(apply_name(Apply::Rebuild), "rebuild"));
}
MTEST_MAIN
