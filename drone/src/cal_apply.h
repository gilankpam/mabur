#pragma once
// Writes one width's measured air efficiency into a running unit's
// /etc/mabur.toml.
//
// The failure mode this file exists to prevent: CLAUDE.md and
// docs/deploy.md both record that a daemon which sees a config it cannot
// load exits, and its wrapper respawns it forever at 2 s. The drone has no
// console -- a calibration run that leaves an unloadable config behind
// turns a two-minute convenience into a device that needs a laptop and a
// serial cable to recover. Every step below exists to make that
// unreachable:
//
//   1. validate -- refuse to touch anything if an efficiency lies outside
//      [0.05, 1.0].
//   2. patch -- rewrite (or insert) this width's efficiency key in memory
//      and check that it was actually found, rewritten or inserted
//      (PatchStatus). patch_toml has no TOML-writer fallback: a line it
//      cannot rewrite (reformatted, a multi-line array) silently left alone
//      would still load, and apply_calibration would report success on a
//      calibration that never applied -- so it is refused loudly
//      (KeyNotFound) instead.
//   3-6. publish_config (config_write.h): write "<path>.new" + fsync,
//      verify it with mabur::load_config -- the same function maburd boots
//      with -- back the original up to "<path>.pre-cal", then rename the
//      verified candidate into place. config_write.h's header explains why
//      verification happens before publishing and why the backup goes
//      through write-then-rename.
//
// patch_toml is line-surgical for the same reason patch_toml_key is (see
// config_write.h): /etc/mabur.toml is a commented file a re-serialize would
// flatten, so it rewrites only the one key, in place, preserving every
// other line -- comments included -- byte for byte. A width calibrated for
// the first time has no efficiency_<bw> line to rewrite; patch_toml inserts
// one instead, next to the other width's line so the file keeps reading as
// one per-width pair.

#include <array>
#include <cstdint>
#include <string>

#include "mabur/config_write.h"
#include "mabur/toml.h"

namespace mabur {

// air_clock.efficiency_<bw>: eff[r] == 0 means "leave the config's value"
// (a capacity point the sweep never resolved, or a rate it didn't measure).
struct CalWrite {
  uint8_t bw = 20;
  std::array<double, 8> eff{};
};

// Whether patch_toml found-and-rewrote or inserted efficiency_<bw>. False
// only when the key's line was matched but could not be rewritten
// (reformatted, a multi-line array patch_toml doesn't parse).
struct PatchStatus {
  bool eff = false;
};

enum class ApplyResult {
  Ok,
  OutOfRange,     // an efficiency would leave [0.05, 1.0]; nothing touched
  KeyNotFound,    // patch_toml couldn't rewrite the key -- the file's
                  // structure wasn't what this line-surgical patcher
                  // expects; nothing touched
  BackupFailed,   // couldn't write "<path>.pre-cal" -- nothing else touched
  WriteFailed,    // couldn't write/fsync "<path>.new", or the final publish
                  // rename failed after verification already succeeded
  ReparseFailed,  // candidate failed mabur::load_config -- never published
};

// Rewrites (or inserts) efficiency_<bw> in `text`, preserving every other
// line, every comment, and the rewritten line's original spacing before
// '=' and any trailing comment. An entry left at 0 keeps whatever value the
// original line had there -- or, when the line is being inserted, takes
// what the loader flies for that width with the key absent (AirClockCfg's
// default). If `status` is non-null it says whether the key was touched.
// Never throws.
std::string patch_toml(const std::string& text, const CalWrite& w,
                       PatchStatus* status = nullptr);

// Validate -> patch (+ key check) -> write "<path>.new" + fsync -> verify
// with mabur::load_config -> back up -> rename into place. See the file
// header for why each step exists. On anything but Ok, `path` is left
// exactly as it was found. `err`, if non-null, is always set on a non-Ok
// result.
//
// `backup` is the caller's call on whether THIS apply should write
// "<path>.pre-cal": a calibration run applying several widths one call at
// a time wants exactly one backup for the whole run (the state before any
// of them ran). Passing false never destroys an existing backup; it only
// suppresses writing a new one.
ApplyResult apply_calibration(const std::string& path, const CalWrite& w, bool backup,
                              std::string* err);

// Test-only seam: same as apply_calibration (with backup requested), but
// writes deliberate garbage to "<path>.new" instead of the patched config,
// so the verify-then-refuse path can be exercised without needing a real
// writer bug to trigger it.
ApplyResult apply_calibration_for_test_with_corrupt_writer(const std::string& path,
                                                           const CalWrite& w, std::string* err);

}  // namespace mabur
