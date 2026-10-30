#include "mtest.h"
#include "cal_apply.h"
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

TEST(patch_inserts_a_missing_efficiency_table_after_shed_ms) {
  PatchStatus status;
  const auto out = patch_toml(kSampleOnly20, eff_write(40), &status);
  CHECK(status.eff);
  const size_t shed = out.find("shed_ms       = 25");
  const size_t eff40 = out.find("efficiency_40 = [0.9, 1, 0.85, 1, 1, 1, 1, 1]");
  REQUIRE(shed != std::string::npos);
  REQUIRE(eff40 != std::string::npos);
  CHECK(shed < eff40);
  // An eff-only apply never touches the radio section.
  CHECK(out.find("power_mode = \"none\"") != std::string::npos);
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
  // that backup, even though publish_config's own backup step runs on
  // every call underneath.
  CHECK(apply_calibration(path, eff_write(40), 1.0, /*backup=*/false, &err) ==
        ApplyResult::Ok);
  std::ifstream backup2(path + ".pre-cal");
  CHECK(backup2.good());
  std::stringstream b2; b2 << backup2.rdbuf();
  CHECK(b2.str() == before_second);   // unchanged by the backup=false call
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

MTEST_MAIN
