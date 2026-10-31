#include "mtest.h"
#include "cal_apply.h"
#include "config.h"
#include "scratch.h"
#include "mabur/rc_proto.h"

#include <fstream>
#include <sstream>
#include <string>

using namespace mabur;

namespace {

// Both per-width tables present, same shape as bundle/mabur.default.toml's
// [radio]/[air_clock] (trimmed to what cal_apply touches, plus a couple of
// unrelated keys/comments to pin that they survive untouched).
const char* kSample = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use the calibrated walls

# Wall-equalization measured on my vtx not yours.
rate_walls_rel_20 = [63, 63, 63, 63, 45, 23, 22, 21]
rate_walls_rel_40 = [63, 63, 63, 63, 45, 28, 28, 23]
cal_flags_20      = "ldpc+stbc"
cal_flags_40      = "stbc"
legacy_wall_rel   = 63
wall_margin_db    = 1.0

[fec]
symbol_size = 332

[air_clock]
shed_ms       = 25
efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]
efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]
body_us       = 0
)";

// Same shape, but the unit has only ever been calibrated at 20 MHz: no
// rate_walls_rel_40, cal_flags_40 or efficiency_40 line at all. Stands in
// for the first-ever calibration of the other width.
const char* kSampleOnly20 = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use the calibrated walls

rate_walls_rel_20 = [63, 63, 63, 63, 45, 23, 22, 21]
cal_flags_20      = "ldpc+stbc"
legacy_wall_rel   = 63
wall_margin_db    = 1.0

[fec]
symbol_size = 332

[air_clock]
shed_ms       = 25
efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]
body_us       = 0
)";

// Same as kSample but with rate_walls_rel_20 removed entirely -- the unit
// was calibrated at 40 MHz only, so legacy_wall_rel has nowhere else to
// come from and a 40 MHz write must repatch it.
const char* kSampleOnly40 = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use the calibrated walls

rate_walls_rel_40 = [63, 63, 63, 63, 45, 28, 28, 23]
cal_flags_40      = "stbc"
legacy_wall_rel   = 63
wall_margin_db    = 1.0

[fec]
symbol_size = 332

[air_clock]
shed_ms       = 25
efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]
body_us       = 0
)";

// [air_clock] has shed_ms but neither efficiency_20 nor efficiency_40 --
// stands in for the shed_ms fallback anchor when there is no other-width
// efficiency line to sit next to either.
const char* kSampleNoEff = R"([air_clock]
shed_ms = 25
body_us = 0
)";

// Same as kSample but with the rate_walls_rel_20 line removed and nothing
// else changed -- stands in for "unexpected file structure" (missing,
// hand-edited, reformatted): patch_toml can't find the line to rewrite, and
// with no anchor to insert against either (rate_walls_rel_40 is present, so
// this fixture is for a 20 MHz write, whose own anchor -- the other
// width's line -- is absent) it must be refused loudly rather than
// silently produce a config that never actually picked up the new walls.
// (Used only for the bw==20 KeyNotFound case below; power_mode is removed
// too so there is no fallback anchor at all.)
const char* kSampleNoAnchorAtAll = R"(# a comment that must survive
[radio]
usb_vid    = 3034

legacy_wall_rel   = 63
wall_margin_db    = 1.0

[fec]
symbol_size = 332
)";

CalWrite walls_write(uint8_t bw) {
  CalWrite w;
  w.bw = bw;
  w.has_walls = true;
  w.walls = {60, 60, 60, 45, 22, 6, 9, 5};
  w.legacy_wall = 60;
  w.cal_flags = "ldpc+stbc";
  return w;
}

CalWrite eff_write(uint8_t bw) {
  CalWrite w;
  w.bw = bw;
  w.has_eff = true;
  w.eff = {0.9, 0.0, 0.85, 0.0, 0.0, 0.0, 0.0, 0.0};
  return w;
}

}  // namespace

TEST(patch_rewrites_only_the_width_asked) {
  // A 40 MHz walls apply must leave every 20 MHz key -- including
  // legacy_wall_rel, which a 20 MHz line still owns -- untouched.
  const auto out = patch_toml(kSample, walls_write(40));
  CHECK(out.find("rate_walls_rel_40 = [60, 60, 60, 45, 22, 6, 9, 5]") != std::string::npos);
  CHECK(out.find("cal_flags_40      = \"ldpc+stbc\"") != std::string::npos);
  CHECK(out.find("rate_walls_rel_20 = [63, 63, 63, 63, 45, 23, 22, 21]") != std::string::npos);
  CHECK(out.find("cal_flags_20      = \"ldpc+stbc\"") != std::string::npos);
  CHECK(out.find("legacy_wall_rel   = 63") != std::string::npos);  // NOT repatched to 60
  CHECK(out.find("power_mode = \"offset\"") != std::string::npos);
  CHECK(out.find("# a comment that must survive") != std::string::npos);
  CHECK(out.find("wall_margin_db    = 1.0") != std::string::npos);  // not ours
}

TEST(patch_inserts_a_missing_40_table_after_the_20_one) {
  PatchStatus status;
  const auto out = patch_toml(kSampleOnly20, walls_write(40), &status);
  CHECK(status.walls);
  CHECK(status.inserted_walls);
  CHECK(status.cal_flags);
  CHECK(status.inserted_cal_flags);

  const size_t p20 = out.find("rate_walls_rel_20 = [63, 63, 63, 63, 45, 23, 22, 21]");
  const size_t p40 = out.find("rate_walls_rel_40 = [60, 60, 60, 45, 22, 6, 9, 5]");
  const size_t c20 = out.find("cal_flags_20      = \"ldpc+stbc\"");
  const size_t c40 = out.find("cal_flags_40 = \"ldpc+stbc\"");
  REQUIRE(p20 != std::string::npos);
  REQUIRE(p40 != std::string::npos);
  REQUIRE(c20 != std::string::npos);
  REQUIRE(c40 != std::string::npos);
  CHECK(p20 < p40);   // the new _40 line sits right after the _20 one
  CHECK(p40 < c20);   // ... and before the pre-existing cal_flags_20
  CHECK(c20 < c40);   // the new cal_flags_40 sits right after cal_flags_20

  // legacy_wall_rel: a 20 MHz line is present, so the 40 MHz write above
  // must not repatch it even though it set CalWrite::legacy_wall.
  CHECK(out.find("legacy_wall_rel   = 63") != std::string::npos);

  CHECK(out.find("# a comment that must survive") != std::string::npos);
  bool threw = false;
  try {
    (void)toml::parse_toml_string(out, "patched");
  } catch (...) {
    threw = true;
  }
  CHECK(!threw);
}

TEST(patch_inserts_a_missing_efficiency_table_after_efficiency_other) {
  // efficiency_20 is present, efficiency_40 is not: the new line must sit
  // right after efficiency_20 (the same other-width anchoring rule as
  // walls/cal_flags), not merely somewhere after shed_ms.
  PatchStatus status;
  const auto out = patch_toml(kSampleOnly20, eff_write(40), &status);
  CHECK(status.eff);
  const size_t eff20 = out.find("efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]");
  const size_t eff40 = out.find("efficiency_40 = [0.9, 1.0, 0.85, 1.0, 1.0, 1.0, 1.0, 1.0]");
  REQUIRE(eff20 != std::string::npos);
  REQUIRE(eff40 != std::string::npos);
  CHECK(eff20 < eff40);
  CHECK(eff40 - eff20 < 80);  // directly adjacent, not merely "somewhere after"
  // An eff-only apply never touches the radio section.
  CHECK(out.find("power_mode = \"none\"") != std::string::npos);
}

TEST(patch_inserts_efficiency_after_shed_ms_when_no_other_width_eff) {
  // Neither efficiency_20 nor efficiency_40 exists yet: the fallback
  // anchor is shed_ms, the table's own first key.
  PatchStatus status;
  const auto out = patch_toml(kSampleNoEff, eff_write(20), &status);
  CHECK(status.eff);
  const size_t shed = out.find("shed_ms = 25");
  const size_t eff20 = out.find("efficiency_20 = [0.9, 1.0, 0.85, 1.0, 1.0, 1.0, 1.0, 1.0]");
  REQUIRE(shed != std::string::npos);
  REQUIRE(eff20 != std::string::npos);
  CHECK(shed < eff20);
}

TEST(patch_writes_efficiency_and_leaves_zero_entries) {
  const auto out = patch_toml(kSample, eff_write(20));
  CHECK(out.find("efficiency_20 = [0.9, 0.88, 0.85, 0.8, 0.76, 0.76, 0.78, 0.76]") !=
        std::string::npos);
  // efficiency_40 and every radio key are untouched by an eff-only apply.
  CHECK(out.find("efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]") !=
        std::string::npos);
  CHECK(out.find("power_mode = \"none\"") != std::string::npos);
  CHECK(out.find("cal_flags_20      = \"ldpc+stbc\"") != std::string::npos);
}

TEST(patch_leaves_undetermined_wall_entries_untouched) {
  // The sentinel slot's original value is itself negative -- this pins
  // that the sentinel check compares against kWallUndetermined (-128), not
  // against "< 0", and that a kept negative original round-trips intact.
  CalWrite w = walls_write(40);
  w.walls[6] = mabur::rc::kWallUndetermined;
  const auto out = patch_toml(kSample, w);
  CHECK(out.find("rate_walls_rel_40 = [60, 60, 60, 45, 22, 6, 28, 5]") != std::string::npos);
}

TEST(patch_leaves_undetermined_legacy_untouched) {
  CalWrite w = walls_write(20);
  w.legacy_wall = mabur::rc::kWallUndetermined;
  const auto out = patch_toml(kSample, w);
  CHECK(out.find("legacy_wall_rel   = 63") != std::string::npos);
}

TEST(legacy_follows_mcs0_only_from_20) {
  // Both tables present: a 40 MHz write never repatches legacy_wall_rel,
  // even with a determined legacy value.
  CHECK(patch_toml(kSample, walls_write(40)).find("legacy_wall_rel   = 63") !=
        std::string::npos);
  // Only a 40 MHz table on file (no rate_walls_rel_20 line at all): a 40
  // MHz write is the only owner left, so it DOES repatch it.
  CHECK(patch_toml(kSampleOnly40, walls_write(40)).find("legacy_wall_rel   = 60") !=
        std::string::npos);
  // A 20 MHz write always repatches it, regardless of what 40 MHz holds.
  CHECK(patch_toml(kSample, walls_write(20)).find("legacy_wall_rel   = 60") !=
        std::string::npos);
}

TEST(patch_output_still_parses) {
  const auto out = patch_toml(kSample, walls_write(20));
  bool threw = false;
  try {
    (void)toml::parse_toml_string(out, "patched");
  } catch (...) {
    threw = true;
  }
  CHECK(!threw);
}

TEST(range_check_matches_the_config_loader) {
  CalWrite w = walls_write(40);
  w.walls[7] = -62;
  std::string why;
  CHECK(!walls_in_range(w, 1.0, &why));   // -62 - 4 = -66 < -64
  CHECK(why.find("radio.rate_walls_rel_40") != std::string::npos);

  CalWrite hi = walls_write(20);
  hi.walls[0] = 64;                       // outside the field outright
  CHECK(!walls_in_range(hi, 1.0, &why));
  CHECK(why.find("radio.rate_walls_rel_20") != std::string::npos);

  CHECK(walls_in_range(walls_write(20), 1.0, &why));
  CHECK(walls_in_range(walls_write(40), 1.0, &why));

  CalWrite lo_eff = eff_write(40);
  lo_eff.eff[0] = 0.04;                   // below the floor
  CHECK(!walls_in_range(lo_eff, 1.0, &why));
  CHECK(why.find("air_clock.efficiency_40") != std::string::npos);

  CalWrite hi_eff = eff_write(20);
  hi_eff.eff[2] = 1.5;                    // above 1.0
  CHECK(!walls_in_range(hi_eff, 1.0, &why));
  CHECK(why.find("air_clock.efficiency_20") != std::string::npos);

  CHECK(walls_in_range(eff_write(20), 1.0, &why));
}

TEST(apply_writes_backup_and_new_file) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal1.toml";
  { std::ofstream f(path); f << kSample; }
  std::string err;
  CHECK(apply_calibration(path, walls_write(40), 1.0, /*backup=*/true, &err) ==
        ApplyResult::Ok);

  std::ifstream backup(path + ".pre-cal");
  CHECK(backup.good());
  std::stringstream b; b << backup.rdbuf();
  CHECK(b.str() == kSample);   // backup is the ORIGINAL, byte for byte

  std::ifstream now(path);
  std::stringstream n; n << now.rdbuf();
  CHECK(n.str().find("rate_walls_rel_40 = [60, 60, 60, 45, 22, 6, 9, 5]") != std::string::npos);
}

TEST(backup_is_written_only_when_asked) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_backup.toml";
  { std::ofstream f(path); f << kSample; }
  std::remove((path + ".pre-cal").c_str());
  std::string err;

  CHECK(apply_calibration(path, walls_write(40), 1.0, /*backup=*/false, &err) ==
        ApplyResult::Ok);
  CHECK(!std::ifstream(path + ".pre-cal").good());

  // The session's first call asks for the backup: it must appear, holding
  // whatever the file held right before THIS call (not the original
  // kSample text, since the previous call already changed it).
  const std::string before_second = [&] {
    std::ifstream f(path); std::stringstream s; s << f.rdbuf(); return s.str();
  }();
  CHECK(apply_calibration(path, eff_write(20), 1.0, /*backup=*/true, &err) ==
        ApplyResult::Ok);
  std::ifstream backup(path + ".pre-cal");
  CHECK(backup.good());
  std::stringstream b; b << backup.rdbuf();
  CHECK(b.str() == before_second);

  // A later call in the same session with backup=false must not disturb
  // that backup at all: publish_config's own backup step is skipped
  // entirely for this call, not run-then-reverted.
  CHECK(apply_calibration(path, eff_write(40), 1.0, /*backup=*/false, &err) ==
        ApplyResult::Ok);
  std::ifstream backup2(path + ".pre-cal");
  CHECK(backup2.good());
  std::stringstream b2; b2 << backup2.rdbuf();
  CHECK(b2.str() == before_second);   // unchanged by the backup=false call
}

TEST(backup_false_never_touches_an_existing_pre_cal_file) {
  // Pins the fix directly: an existing ".pre-cal" is byte-identical before
  // and after a backup=false apply, with no window where publish_config's
  // unconditional backup step could have overwritten it first.
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_backup2.toml";
  { std::ofstream f(path); f << kSample; }
  const std::string kUnrelatedBackup = "# this is an earlier session's real backup\n";
  { std::ofstream f(path + ".pre-cal"); f << kUnrelatedBackup; }

  std::string err;
  CHECK(apply_calibration(path, walls_write(40), 1.0, /*backup=*/false, &err) ==
        ApplyResult::Ok);

  std::ifstream backup(path + ".pre-cal");
  REQUIRE(backup.good());
  std::stringstream b; b << backup.rdbuf();
  CHECK(b.str() == kUnrelatedBackup);
}

TEST(apply_refuses_out_of_range_without_touching_the_file) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal2.toml";
  { std::ofstream f(path); f << kSample; }
  CalWrite bad = walls_write(20);
  bad.walls = {64, 64, 64, 64, 64, 64, 64, 64};
  bad.legacy_wall = 64;
  std::string err;
  CHECK(apply_calibration(path, bad, 1.0, true, &err) == ApplyResult::OutOfRange);
  std::ifstream f(path);
  std::stringstream s; s << f.rdbuf();
  CHECK(s.str() == kSample);   // untouched
}

TEST(apply_refuses_when_a_required_key_is_missing) {
  // patch_toml has no error channel: a key it can neither find nor insert
  // (no anchor at all here) is silently left alone, and load_config()
  // alone can't tell that apart from "found and rewritten" -- an
  // unmodified original value is still a valid one. apply_calibration
  // must catch this itself (KeyNotFound) rather than report Ok on a
  // calibration that never actually applied.
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal6.toml";
  { std::ofstream f(path); f << kSampleNoAnchorAtAll; }
  std::string err;
  CHECK(apply_calibration(path, walls_write(20), 1.0, true, &err) == ApplyResult::KeyNotFound);
  CHECK(err.find("rate_walls_rel_20") != std::string::npos);
  std::ifstream f(path);
  std::stringstream s; s << f.rdbuf();
  CHECK(s.str() == kSampleNoAnchorAtAll);   // untouched
  std::ifstream tmp(path + ".new");
  CHECK(!tmp.good());   // never even got as far as writing a scratch file
}

TEST(apply_never_publishes_a_file_that_will_not_parse) {
  // The respawn-loop guard: verification runs on "<path>.new" BEFORE it is
  // ever renamed onto `path`, so a candidate that will not load is never
  // published in the first place -- the original is never replaced, not
  // restored after the fact.
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal3.toml";
  { std::ofstream f(path); f << kSample; }
  std::string err;
  CHECK(apply_calibration_for_test_with_corrupt_writer(path, walls_write(20), 1.0, &err) ==
        ApplyResult::ReparseFailed);
  std::ifstream f(path);
  std::stringstream s; s << f.rdbuf();
  CHECK(s.str() == kSample);   // never replaced

  std::ifstream tmp(path + ".new");
  CHECK(!tmp.good());   // a failed run must not leave a stale scratch file
}

TEST(apply_writes_once) {
  // Flash wear: an earlier bug had the drone saving config per bitrate
  // change. One session, one write.
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal4.toml";
  { std::ofstream f(path); f << kSample; }
  std::string err;
  CHECK(apply_calibration(path, walls_write(20), 1.0, true, &err) == ApplyResult::Ok);
  CHECK(write_count_for_test() == 1);
}

namespace {

// A [radio] section with no wall table of either width: nothing for an
// undetermined row of a first table to be taken from.
const char* kSampleNoWalls = R"([radio]
usb_vid    = 3034
power_mode = "none"
legacy_wall_rel   = 63
wall_margin_db    = 1.0
)";

std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

bool exists(const std::string& p) { return std::ifstream(p).good(); }

}  // namespace

// Revert (the rail fill: vals[s] = kRailRel for an undetermined slot): the
// inserted line carries 63 at mcs5/mcs6 and borrowed_walls stays 0.
TEST(an_inserted_40_table_takes_undetermined_rows_from_the_20_table) {
  CalWrite w = walls_write(40);
  w.walls[5] = mabur::rc::kWallUndetermined;
  w.walls[6] = mabur::rc::kWallUndetermined;
  PatchStatus status;
  const auto out = patch_toml(kSampleOnly20, w, &status);
  CHECK(status.inserted_walls);
  CHECK(status.borrowed_walls == ((1u << 5) | (1u << 6)));
  CHECK(status.unfilled_walls == 0);
  // kSampleOnly20's 20 MHz mcs5/mcs6 are 23 and 22.
  CHECK(out.find("rate_walls_rel_40 = [60, 60, 60, 45, 22, 23, 22, 5]") != std::string::npos);
}

TEST(apply_of_a_borrowed_row_says_so_in_its_note) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_borrow.toml";
  { std::ofstream f(path); f << kSampleOnly20; }
  CalWrite w = walls_write(40);
  w.walls[5] = mabur::rc::kWallUndetermined;
  w.walls[6] = mabur::rc::kWallUndetermined;
  std::string err, note = "stale";
  CHECK(apply_calibration(path, w, 1.0, /*backup=*/false, &err, &note) == ApplyResult::Ok);
  CHECK(note == "ok; mcs5,6 kept from the 20 MHz table");
  CHECK(note.size() <= 49);
  const Config c = load_config(path, nullptr);
  REQUIRE(c.radio.rate_walls_rel_40.has_value());
  CHECK((*c.radio.rate_walls_rel_40)[5] == 23);
  CHECK((*c.radio.rate_walls_rel_40)[6] == 22);

  // A write that kept nothing clears the note.
  CHECK(apply_calibration(path, walls_write(40), 1.0, false, &err, &note) == ApplyResult::Ok);
  CHECK(note.empty());
}

// Revert (insert anyway with the rail): ApplyResult::Ok and a 63 on file.
TEST(an_inserted_table_with_nothing_to_take_from_is_refused) {
  CalWrite w40 = walls_write(40);
  w40.walls[5] = mabur::rc::kWallUndetermined;
  w40.walls[6] = mabur::rc::kWallUndetermined;
  PatchStatus status;
  const auto out = patch_toml(kSampleNoWalls, w40, &status);
  CHECK(status.unfilled_walls == ((1u << 5) | (1u << 6)));
  CHECK(!status.walls);
  CHECK(out.find("rate_walls_rel_40") == std::string::npos);

  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_nokeep.toml";
  std::remove((path + ".pre-cal").c_str());
  { std::ofstream f(path); f << kSampleNoWalls; }
  std::string err, note;
  CHECK(apply_calibration(path, w40, 1.0, /*backup=*/true, &err, &note) ==
        ApplyResult::NoTableToKeep);
  CHECK(err == "undetermined mcs5,6; no table to keep");
  CHECK(err.size() <= 49);
  CHECK(slurp(path) == kSampleNoWalls);
  CHECK(!exists(path + ".pre-cal"));

  // A first 20 MHz table never takes a 40 MHz wall: 40 MHz walls sit at or
  // above the 20 MHz ones, the overdriving direction at 20.
  CalWrite w20 = walls_write(20);
  w20.walls[7] = mabur::rc::kWallUndetermined;
  const std::string path2 = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_nokeep20.toml";
  { std::ofstream f(path2); f << kSampleOnly40; }
  CHECK(apply_calibration(path2, w20, 1.0, true, &err, &note) == ApplyResult::NoTableToKeep);
  CHECK(err == "undetermined mcs7; no table to keep");
  CHECK(slurp(path2) == kSampleOnly40);

  // Seven undetermined rows (the most an apply carries) still fit the wire.
  CalWrite many = walls_write(40);
  for (int r = 1; r < 8; ++r) many.walls[static_cast<size_t>(r)] = mabur::rc::kWallUndetermined;
  CHECK(apply_calibration(path, many, 1.0, false, &err, &note) == ApplyResult::NoTableToKeep);
  CHECK(err.size() <= 49);
}

// Revert (fill with a literal instead of AirClockCfg's default): breaks as
// soon as the loader's absent-key default is anything else. Revert
// fmt_double's ".0": the filled 1 renders as a TOML integer, the array is
// mixed-type and the apply is refused (ReparseFailed).
TEST(an_inserted_efficiency_line_fills_left_entries_with_the_absent_key_default) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_eff_ins.toml";
  { std::ofstream f(path); f << kSampleOnly20; }
  // The value the loader flies at 40 MHz with no efficiency_40 line.
  const Config before = load_config(path, nullptr);
  std::string err;
  CHECK(apply_calibration(path, eff_write(40), 1.0, false, &err) == ApplyResult::Ok);
  const Config after = load_config(path, nullptr);
  const AirClockCfg absent{};
  CHECK(after.air_clock.efficiency_40[0] == 0.9);
  CHECK(after.air_clock.efficiency_40[2] == 0.85);
  for (int r : {1, 3, 4, 5, 6, 7}) {
    CHECK(after.air_clock.efficiency_40[static_cast<size_t>(r)] ==
          before.air_clock.efficiency_40[static_cast<size_t>(r)]);
    CHECK(after.air_clock.efficiency_40[static_cast<size_t>(r)] ==
          absent.efficiency_40[static_cast<size_t>(r)]);
  }
}

// Revert fmt_double's ".0": "1" lands in a float array and the loader
// refuses the mixed-type candidate (ReparseFailed).
TEST(an_efficiency_of_exactly_one_still_loads) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_eff_one.toml";
  { std::ofstream f(path); f << kSample; }
  CalWrite w = eff_write(20);
  w.eff[0] = 1.0;
  std::string err;
  CHECK(apply_calibration(path, w, 1.0, false, &err) == ApplyResult::Ok);
  CHECK(slurp(path).find("efficiency_20 = [1.0, 0.88, 0.85,") != std::string::npos);
  CHECK(load_config(path, nullptr).air_clock.efficiency_20[0] == 1.0);
}

MTEST_MAIN
