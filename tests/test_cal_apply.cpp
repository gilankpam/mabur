#include "mtest.h"
#include "cal_apply.h"
#include "config.h"
#include "scratch.h"

#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include <string>

using namespace mabur;

namespace {

// Both per-width efficiency tables present, same shape as
// bundle/mabur.default.toml's [air_clock], plus unrelated keys and comments
// to pin that they survive untouched.
const char* kSample = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # a radio key an efficiency apply never touches
wall_margin_db    = 1.0

[fec]
symbol_size = 332

[air_clock]
shed_ms       = 25
efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]   # keep this comment
efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]
body_us       = 0
)";

// Only efficiency_20 on file: stands in for the first capacity run at 40.
const char* kSampleOnly20 = R"(# a comment that must survive
[radio]
usb_vid    = 3034

[air_clock]
shed_ms       = 25
efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]
body_us       = 0
)";

// [air_clock] has shed_ms but no efficiency line of either width.
const char* kSampleNoEff = R"([air_clock]
shed_ms = 25
body_us = 0
)";

// No [air_clock] section at all: the line goes in through set_toml_key.
const char* kSampleNoAirClock = R"(# a comment that must survive
[radio]
usb_vid    = 3034

[fec]
symbol_size = 332
)";

CalWrite eff_write(uint8_t bw) {
  CalWrite w;
  w.bw = bw;
  w.eff = {0.9, 0.0, 0.85, 0.0, 0.0, 0.0, 0.0, 0.0};
  return w;
}

std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

std::string scratch_file(const char* name, const char* text) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name;
  std::remove((path + ".pre-cal").c_str());
  std::ofstream f(path);
  f << text;
  return path;
}

}  // namespace

// An efficiency apply rewrites its width's line and nothing else: every
// other byte of the file is the original's. Revert (a radio key or the
// other width's line repatched alongside): the byte comparison fails.
TEST(an_efficiency_apply_patches_only_its_widths_key) {
  std::string expected = kSample;
  const std::string old_line = "efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]";
  expected.replace(expected.find(old_line), old_line.size(),
                   "efficiency_20 = [0.9, 0.88, 0.85, 0.8, 0.76, 0.76, 0.78, 0.76]");
  CHECK(patch_toml(kSample, eff_write(20)) == expected);

  const std::string path = scratch_file("cal_eff_only.toml", kSample);
  std::string err;
  CHECK(apply_calibration(path, eff_write(20), /*backup=*/false, &err) == ApplyResult::Ok);
  CHECK(slurp(path) == expected);
}

TEST(patch_inserts_a_missing_efficiency_table_after_efficiency_other) {
  PatchStatus status;
  const auto out = patch_toml(kSampleOnly20, eff_write(40), &status);
  CHECK(status.eff);
  const size_t eff20 = out.find("efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]");
  const size_t eff40 = out.find("efficiency_40 = [0.9, 0.75, 0.85, 0.75, 0.74, 0.77, 0.76, 0.75]");
  REQUIRE(eff20 != std::string::npos);
  REQUIRE(eff40 != std::string::npos);
  CHECK(eff20 < eff40);
  CHECK(eff40 - eff20 < 80);  // directly adjacent, not merely "somewhere after"
}

TEST(patch_inserts_efficiency_after_shed_ms_when_no_other_width_eff) {
  PatchStatus status;
  const auto out = patch_toml(kSampleNoEff, eff_write(20), &status);
  CHECK(status.eff);
  const size_t shed = out.find("shed_ms = 25");
  const size_t eff20 = out.find("efficiency_20 = [0.9, 0.88, 0.85, 0.8, 0.76, 0.76, 0.78, 0.76]");
  REQUIRE(shed != std::string::npos);
  REQUIRE(eff20 != std::string::npos);
  CHECK(shed < eff20);
}

TEST(apply_inserts_efficiency_with_no_air_clock_section) {
  const std::string path = scratch_file("cal_no_ac.toml", kSampleNoAirClock);
  std::string err;
  CHECK(apply_calibration(path, eff_write(20), false, &err) == ApplyResult::Ok);
  const std::string out = slurp(path);
  CHECK(out.rfind("# a comment that must survive\n", 0) == 0);
  CHECK(out.find("symbol_size = 332") != std::string::npos);
  const Config c = load_config(path);
  CHECK(c.air_clock.efficiency_20[0] == 0.9 && c.air_clock.efficiency_20[2] == 0.85);
}

TEST(apply_writes_backup_and_new_file) {
  const std::string path = scratch_file("cal1.toml", kSample);
  std::string err;
  CHECK(apply_calibration(path, eff_write(40), /*backup=*/true, &err) == ApplyResult::Ok);
  CHECK(slurp(path + ".pre-cal") == kSample);  // the ORIGINAL, byte for byte
  CHECK(slurp(path).find("efficiency_40 = [0.9, 0.75, 0.85,") != std::string::npos);
}

TEST(backup_is_written_only_when_asked) {
  const std::string path = scratch_file("cal_backup.toml", kSample);
  std::string err;
  CHECK(apply_calibration(path, eff_write(40), /*backup=*/false, &err) == ApplyResult::Ok);
  CHECK(!std::ifstream(path + ".pre-cal").good());

  // A call that asks for the backup gets the file as it was right before it.
  const std::string before_second = slurp(path);
  CHECK(apply_calibration(path, eff_write(20), /*backup=*/true, &err) == ApplyResult::Ok);
  CHECK(slurp(path + ".pre-cal") == before_second);

  // A later backup=false call never disturbs it.
  CalWrite w = eff_write(40);
  w.eff[0] = 0.6;
  CHECK(apply_calibration(path, w, /*backup=*/false, &err) == ApplyResult::Ok);
  CHECK(slurp(path + ".pre-cal") == before_second);
}

TEST(backup_false_never_touches_an_existing_pre_cal_file) {
  const std::string path = scratch_file("cal_backup2.toml", kSample);
  const std::string kUnrelatedBackup = "# this is an earlier session's real backup\n";
  { std::ofstream f(path + ".pre-cal"); f << kUnrelatedBackup; }
  std::string err;
  CHECK(apply_calibration(path, eff_write(40), /*backup=*/false, &err) == ApplyResult::Ok);
  CHECK(slurp(path + ".pre-cal") == kUnrelatedBackup);
}

TEST(apply_refuses_out_of_range_without_touching_the_file) {
  const std::string path = scratch_file("cal2.toml", kSample);
  std::string err;
  CalWrite lo = eff_write(40);
  lo.eff[0] = 0.04;  // below the floor
  CHECK(apply_calibration(path, lo, true, &err) == ApplyResult::OutOfRange);
  CHECK(err.find("air_clock.efficiency_40[0]") != std::string::npos);
  CalWrite hi = eff_write(20);
  hi.eff[2] = 1.5;  // above 1.0
  CHECK(apply_calibration(path, hi, true, &err) == ApplyResult::OutOfRange);
  CHECK(err.find("air_clock.efficiency_20[2]") != std::string::npos);
  CHECK(slurp(path) == kSample);  // untouched
  CHECK(!std::ifstream(path + ".pre-cal").good());
}

TEST(apply_never_publishes_a_file_that_will_not_parse) {
  // The respawn-loop guard: verification runs on "<path>.new" BEFORE it is
  // ever renamed onto `path`, so a candidate that will not load is never
  // published in the first place.
  const std::string path = scratch_file("cal3.toml", kSample);
  std::string err;
  CHECK(apply_calibration_for_test_with_corrupt_writer(path, eff_write(20), &err) ==
        ApplyResult::ReparseFailed);
  CHECK(slurp(path) == kSample);              // never replaced
  CHECK(!std::ifstream(path + ".new").good());  // no stale scratch file
}

TEST(apply_writes_once) {
  // Flash wear: one apply, one write.
  const std::string path = scratch_file("cal4.toml", kSample);
  std::string err;
  CHECK(apply_calibration(path, eff_write(20), true, &err) == ApplyResult::Ok);
  CHECK(write_count_for_test() == 1);
}

// Revert (fill with a literal instead of AirClockCfg's default): breaks as
// soon as the loader's absent-key default is anything else. Revert
// fmt_double's ".0": the filled 1 renders as a TOML integer, the array is
// mixed-type and the apply is refused (ReparseFailed).
TEST(an_inserted_efficiency_line_fills_left_entries_with_the_absent_key_default) {
  const std::string path = scratch_file("cal_eff_ins.toml", kSampleOnly20);
  const Config before = load_config(path, nullptr);
  std::string err;
  CHECK(apply_calibration(path, eff_write(40), false, &err) == ApplyResult::Ok);
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
  const std::string path = scratch_file("cal_eff_one.toml", kSample);
  CalWrite w = eff_write(20);
  w.eff[0] = 1.0;
  std::string err;
  CHECK(apply_calibration(path, w, false, &err) == ApplyResult::Ok);
  CHECK(slurp(path).find("efficiency_20 = [1.0, 0.88, 0.85,") != std::string::npos);
  CHECK(load_config(path, nullptr).air_clock.efficiency_20[0] == 1.0);
}

MTEST_MAIN
