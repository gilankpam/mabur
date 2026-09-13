#include "mtest.h"
#include "cal_apply.h"
#include "scratch.h"
#include "mabur/rc_proto.h"

#include <fstream>
#include <sstream>
#include <string>

using namespace mabur;

namespace {

const char* kSample = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use rate_walls_rel

# Wall-equalization measured on my vtx not yours.
rate_walls_rel  = [63, 63, 63, 42, 20, 1, -2, -4]
legacy_wall_rel = 63
wall_margin_db  = 1.0

[fec]
symbol_size = 332
)";

CalWrite sample_write() {
  CalWrite w;
  w.walls = {60, 60, 60, 45, 22, 6, 9, 5};
  w.legacy_wall = 60;
  return w;
}

// Same as kSample but with the rate_walls_rel line removed entirely --
// stands in for "unexpected file structure" (missing, hand-edited,
// reformatted): patch_toml can't find the line to rewrite, and that must
// be refused loudly rather than silently produce a config that never
// actually picked up the new walls.
const char* kSampleMissingRateWalls = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use rate_walls_rel

legacy_wall_rel = 63
wall_margin_db  = 1.0

[fec]
symbol_size = 332
)";

}  // namespace

TEST(patch_rewrites_only_the_three_keys) {
  const auto out = patch_toml(kSample, sample_write());
  CHECK(out.find("rate_walls_rel  = [60, 60, 60, 45, 22, 6, 9, 5]") !=
        std::string::npos);
  CHECK(out.find("legacy_wall_rel = 60") != std::string::npos);
  CHECK(out.find("power_mode = \"offset\"") != std::string::npos);
  CHECK(out.find("base_ref_idx") == std::string::npos);
}

TEST(patch_preserves_every_comment_and_unrelated_key) {
  const auto out = patch_toml(kSample, sample_write());
  CHECK(out.find("# a comment that must survive") != std::string::npos);
  CHECK(out.find("# Wall-equalization measured on my vtx not yours.") !=
        std::string::npos);
  CHECK(out.find("usb_vid    = 3034") != std::string::npos);
  CHECK(out.find("symbol_size = 332") != std::string::npos);
  CHECK(out.find("wall_margin_db  = 1.0") != std::string::npos);  // not ours
}

TEST(patch_leaves_undetermined_entries_untouched) {
  // The sentinel slot's original value is itself negative -- this pins
  // that the sentinel check compares against kWallUndetermined (-128), not
  // against "< 0", and that a kept negative original round-trips intact.
  CalWrite w = sample_write();
  w.walls[6] = mabur::rc::kWallUndetermined;  // undetermined: keep the existing -2
  const auto out = patch_toml(kSample, w);
  CHECK(out.find("rate_walls_rel  = [60, 60, 60, 45, 22, 6, -2, 5]") !=
        std::string::npos);
}

TEST(patch_leaves_undetermined_legacy_untouched) {
  // Mirrors the walls[] sentinel test above, for legacy_wall_rel: MCS0
  // came back undetermined this session, so the line must be left exactly
  // as the original had it, not rewritten to a fabricated number.
  CalWrite w = sample_write();
  w.legacy_wall = mabur::rc::kWallUndetermined;
  const auto out = patch_toml(kSample, w);
  CHECK(out.find("legacy_wall_rel = 63") != std::string::npos);
}

TEST(patch_output_still_parses) {
  const auto out = patch_toml(kSample, sample_write());
  bool threw = false;
  try {
    (void)toml::parse_toml_string(out, "patched");
  } catch (...) {
    threw = true;
  }
  CHECK(!threw);
}

TEST(range_check_matches_the_config_loader) {
  CalWrite w;
  w.walls = {63, 63, 63, 63, 63, 63, 63, -62};
  w.legacy_wall = 63;
  std::string why;
  CHECK(!walls_in_range(w, 1.0, &why));   // -62 - 4 = -66 < -64
  CHECK(why.find("rate_walls_rel") != std::string::npos);

  CalWrite hi = sample_write();
  hi.walls[0] = 64;                       // outside the field outright
  CHECK(!walls_in_range(hi, 1.0, &why));

  CHECK(walls_in_range(sample_write(), 1.0, &why));
}

TEST(apply_writes_backup_and_new_file) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal1.toml";
  { std::ofstream f(path); f << kSample; }
  std::string err;
  CHECK(apply_calibration(path, sample_write(), 1.0, &err) == ApplyResult::Ok);

  std::ifstream backup(path + ".pre-cal");
  CHECK(backup.good());
  std::stringstream b; b << backup.rdbuf();
  CHECK(b.str() == kSample);   // backup is the ORIGINAL, byte for byte

  std::ifstream now(path);
  std::stringstream n; n << now.rdbuf();
  CHECK(n.str().find("legacy_wall_rel = 60") != std::string::npos);
}

TEST(apply_refuses_out_of_range_without_touching_the_file) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal2.toml";
  { std::ofstream f(path); f << kSample; }
  CalWrite bad;
  bad.walls = {64, 64, 64, 64, 64, 64, 64, 64};
  bad.legacy_wall = 64;
  std::string err;
  CHECK(apply_calibration(path, bad, 1.0, &err) == ApplyResult::OutOfRange);
  std::ifstream f(path);
  std::stringstream s; s << f.rdbuf();
  CHECK(s.str() == kSample);   // untouched
}

TEST(apply_refuses_when_a_required_key_is_missing) {
  // patch_toml has no error channel: a key it can't find is silently left
  // alone, and load_config() alone can't tell "found and rewritten" apart
  // from "never touched because it wasn't there" -- an unmodified original
  // value is still a valid one. apply_calibration must catch this itself
  // (KeyNotFound) rather than report Ok on a calibration that never
  // actually applied.
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal6.toml";
  { std::ofstream f(path); f << kSampleMissingRateWalls; }
  std::string err;
  CHECK(apply_calibration(path, sample_write(), 1.0, &err) ==
        ApplyResult::KeyNotFound);
  CHECK(err.find("rate_walls_rel") != std::string::npos);
  std::ifstream f(path);
  std::stringstream s; s << f.rdbuf();
  CHECK(s.str() == kSampleMissingRateWalls);   // untouched
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
  CHECK(apply_calibration_for_test_with_corrupt_writer(path, sample_write(),
                                                       1.0, &err) ==
        ApplyResult::ReparseFailed);
  std::ifstream f(path);
  std::stringstream s; s << f.rdbuf();
  CHECK(s.str() == kSample);   // never replaced

  // A failed run must not leave a stale scratch file next to a live config.
  std::ifstream tmp(path + ".new");
  CHECK(!tmp.good());
}

TEST(apply_writes_once) {
  // Flash wear: an earlier bug had the drone saving config per bitrate
  // change. One session, one write.
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal4.toml";
  { std::ofstream f(path); f << kSample; }
  std::string err;
  CHECK(apply_calibration(path, sample_write(), 1.0, &err) == ApplyResult::Ok);
  CHECK(write_count_for_test() == 1);
}

MTEST_MAIN
