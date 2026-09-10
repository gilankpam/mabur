#pragma once
// Writes a measured wall-equalization table into a running unit's
// /etc/mabur.toml.
//
// The failure mode this file exists to prevent: CLAUDE.md and
// docs/deploy.md both record that a daemon which sees a config it cannot
// load exits, and its wrapper respawns it forever at 2 s. The drone has no
// console -- a calibration run that leaves an unloadable config behind
// turns a 75-second convenience into a device that needs a laptop and a
// serial cable to recover. Every step below exists to make that
// unreachable:
//
//   1. validate  -- run the candidate walls through the same derivation
//      config.cpp's loader applies (drone/src/config.cpp:133-155) and
//      refuse to touch anything if a derived diff would leave the 8822E's
//      7-bit rate-diff field range [-64,63].
//   2. back up, then write atomically -- copy the original to
//      "<path>.pre-cal" (the repo's ".pre-*" rollback convention), then
//      write "<path>.new" + fsync + rename so a crash mid-write never
//      leaves a half-written file at the real path.
//   3. re-parse what was just written, with the REAL loader
//      (mabur::toml::parse_toml_file, not a hand-rolled check) -- this is
//      the step that makes the respawn loop unreachable, and it costs
//      milliseconds. If it does not load, the backup is restored before
//      anyone can restart the daemon against the bad file.
//
// There is no TOML *writer* in this tree (mabur/toml.h declares only
// parse_toml_file/parse_toml_string), and /etc/mabur.toml is
// bundle/mabur.default.toml verbatim -- a heavily commented file a
// re-serialize would flatten. patch_toml is therefore line-surgical: it
// rewrites only the four keys this task owns, in place, preserving every
// other line -- comments included -- byte for byte.

#include <array>
#include <string>

#include "mabur/toml.h"

namespace mabur {

// Re-exported so callers (this header's own tests included) can write
// mabur::parse_toml_string instead of mabur::toml::parse_toml_string --
// the re-parse step below is the reason this header pulls in mabur/toml.h
// at all.
using toml::parse_toml_string;
using toml::parse_toml_file;

// -1 in walls[r] (or legacy_wall) means "undetermined": that rate wasn't
// swept this session, so the existing config's value for it is kept.
// Never write a fabricated number over a real measurement.
struct CalWrite {
  std::array<int, 8> walls{-1, -1, -1, -1, -1, -1, -1, -1};
  int legacy_wall = -1;  // derived from the MCS0 sweep result, not swept itself
  int base_ref_idx = 0;
};

enum class ApplyResult {
  Ok,
  OutOfRange,     // a derived diff would leave the hardware field's [-64,63]
  BackupFailed,   // couldn't write "<path>.pre-cal" -- nothing else touched
  WriteFailed,    // couldn't write/fsync/rename "<path>.new"
  ReparseFailed,  // wrote it, but the real loader rejects it -- restored
};

// Rewrites only rate_walls_idx, legacy_wall_idx, base_ref_idx and
// power_mode (set to "offset") in `text`, preserving every other line,
// every comment, and each rewritten line's original spacing before '=' and
// any trailing comment. An entry left at -1 keeps whatever value the
// original text had for it.
std::string patch_toml(const std::string& text, const CalWrite& w);

// Mirrors the range check drone/src/config.cpp:133-155 applies at load
// time: wall - lround(margin_db * 4.0) - base_ref_idx must land in
// [-64,63] for every swept rate (walls[r] != -1) and for legacy_wall (if
// determined). On failure, *why names the offending key, matching
// config.cpp's message so this refusal reads the same as a boot failure
// would.
bool walls_in_range(const CalWrite& w, double margin_db, std::string* why);

// Validate -> back up -> atomic write -> re-parse with the real loader ->
// restore on failure. See the file header for why each step exists. On
// anything but Ok, `path` is left exactly as it was found: OutOfRange never
// touches it; ReparseFailed restores it from the backup taken this call,
// except in the vanishingly rare case where even that restore rename
// fails, in which case *err says so explicitly rather than pretending it
// worked -- see the failure-ordering analysis in cal_apply.cpp.
ApplyResult apply_calibration(const std::string& path, const CalWrite& w,
                               double margin_db, std::string* err);

// Test-only seam: same as apply_calibration, but writes deliberate garbage
// to `path` instead of the patched config, so the re-parse-and-restore
// path can be exercised without needing a real writer bug to trigger it.
ApplyResult apply_calibration_for_test_with_corrupt_writer(
    const std::string& path, const CalWrite& w, double margin_db,
    std::string* err);

// Counts how many times this process has actually written to the most
// recently written config path (i.e. renamed a candidate into place),
// successful or not -- keyed per path, so it reflects repeated calls
// against the SAME file (in production, always /etc/mabur.toml) rather
// than accumulating across unrelated paths. A past bug had the drone
// saving config on every bitrate change and chewing through flash; a
// calibration session must write at most once.
int write_count_for_test();

}  // namespace mabur
