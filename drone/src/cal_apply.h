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
//   2. patch -- rewrite the four keys in memory and check that every
//      REQUIRED one was actually found and rewritten (see PatchStatus).
//      patch_toml has no TOML-writer fallback: if a key's line can't be
//      matched -- missing, reformatted, a multi-line array -- silently
//      leaving it alone would mean load_config() below still accepts the
//      file (an unmodified original value is still a valid one) and
//      apply_calibration reports success on a calibration that never
//      actually applied. That is the silent-miscalibration failure this
//      whole kit exists to prevent, arriving through the back door, so it
//      is checked explicitly and refused loudly (KeyNotFound) rather than
//      left to slip through.
//   3. write "<path>.new" + fsync -- a scratch file, not a publish. `path`
//      itself is not touched yet.
//   4. verify "<path>.new" with mabur::load_config (drone/src/config.h),
//      the SAME function maburd boots with -- not mabur::toml::parse_toml_file.
//      parse_toml_file only proves the bytes are syntactically valid TOML;
//      everything that actually fails a boot -- unknown-key rejection,
//      every per-section range/type check -- lives inside load_config,
//      past that parse. Verifying with anything weaker would let a file
//      through here that maburd then refuses, which is exactly the
//      scenario this whole file exists to prevent. On failure, the
//      scratch file is discarded and `path` was never touched.
//   5. back up -- copy the original to "<path>.pre-cal" (the repo's
//      ".pre-*" rollback convention), now purely an operator's undo copy,
//      since nothing downstream of step 4 can produce a `path` that fails
//      to load. Written through the same write-then-rename primitive as
//      everything else, so a mid-write failure can't destroy the PREVIOUS
//      session's rollback copy while producing nothing usable in its place.
//   6. rename "<path>.new" onto `path` -- publishing bytes already proven
//      loadable by the function that will load them for real.
//
// Verifying before publishing (rather than publishing, then verifying,
// then restoring on failure) is deliberate: publish-then-verify leaves a
// crash window between the rename that makes a candidate live and the
// check that would have caught it, and no restore logic run after the
// fact can close a window that already passed. Verify-then-publish
// removes the window instead of narrowing it -- every crash point leaves
// either the untouched original or bytes already proven loadable, and
// there is no operation left afterward that could still fail with a bad
// file live.
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

// -1 in walls[r] (or legacy_wall) means "undetermined": that rate wasn't
// swept this session, so the existing config's value for it is kept.
// Never write a fabricated number over a real measurement.
struct CalWrite {
  std::array<int, 8> walls{-1, -1, -1, -1, -1, -1, -1, -1};
  int legacy_wall = -1;  // derived from the MCS0 sweep result, not swept itself
  int base_ref_idx = 0;
};

// Which of the four keys patch_toml actually found and rewrote. A flag
// stays false either because the key's line was never matched at all
// (missing, reformatted, a multi-line array patch_toml doesn't parse) or,
// for legacy_wall_idx only, because CalWrite::legacy_wall was -1 and the
// line was deliberately left untouched. apply_calibration_impl uses this
// to tell those two cases apart from each other and from "found and
// rewritten", since only the first is an error.
struct PatchStatus {
  bool rate_walls_idx = false;
  bool legacy_wall_idx = false;
  bool base_ref_idx = false;
  bool power_mode = false;
};

enum class ApplyResult {
  Ok,
  OutOfRange,     // a derived diff would leave the hardware field's [-64,63]
  KeyNotFound,    // patch_toml couldn't find/rewrite a required key -- the
                  // file's structure wasn't what this line-surgical patcher
                  // expects; nothing touched
  BackupFailed,   // couldn't write "<path>.pre-cal" -- nothing else touched
  WriteFailed,    // couldn't write/fsync "<path>.new", or the final publish
                  // rename failed after verification already succeeded
  ReparseFailed,  // candidate failed mabur::load_config -- never published
};

// Rewrites only rate_walls_idx, legacy_wall_idx, base_ref_idx and
// power_mode (set to "offset") in `text`, preserving every other line,
// every comment, and each rewritten line's original spacing before '=' and
// any trailing comment. An entry left at -1 keeps whatever value the
// original text had for it. If `status` is non-null, it is filled in with
// which keys were actually found and rewritten -- see PatchStatus. Never
// throws; a key it can't find is simply left unmatched in `*status` for
// the caller to act on.
std::string patch_toml(const std::string& text, const CalWrite& w,
                        PatchStatus* status = nullptr);

// Mirrors the range check drone/src/config.cpp:133-155 applies at load
// time: wall - lround(margin_db * 4.0) - base_ref_idx must land in
// [-64,63] for every swept rate (walls[r] != -1) and for legacy_wall (if
// determined). On failure, *why names the offending key, matching
// config.cpp's message so this refusal reads the same as a boot failure
// would.
//
// This stays even though apply_calibration()'s load_config() verification
// re-derives the same numbers: this check refuses a bad table before the
// disk is touched at all, with a specific OutOfRange result and a message
// naming the exact derivation that failed. load_config() is the backstop
// that catches anything this model does not -- a mistyped key, a value
// outside the four keys this file patches, a config field this function
// knows nothing about. Belt and braces, not redundancy. Note that this
// check is necessarily narrower than load_config()'s: it only checks the
// DERIVED diff, not (for example) base_ref_idx's own [0,127] range, so a
// candidate can pass this and still be refused at the load_config() step.
bool walls_in_range(const CalWrite& w, double margin_db, std::string* why);

// Validate -> patch (+ required-key check) -> write "<path>.new" + fsync
// -> verify with mabur::load_config -> back up -> rename into place. See
// the file header for why each step exists and why verification happens
// before publishing rather than after. On anything but Ok, `path` is left
// exactly as it was found: it is never replaced by anything that has not
// already been proven loadable by the same function maburd boots with.
// `err`, if non-null, is always set on a non-Ok result.
ApplyResult apply_calibration(const std::string& path, const CalWrite& w,
                               double margin_db, std::string* err);

// Test-only seam: same as apply_calibration, but writes deliberate garbage
// to "<path>.new" instead of the patched config (bypassing patch_toml and
// its required-key check entirely), so the verify-then-refuse path can be
// exercised without needing a real writer bug to trigger it.
ApplyResult apply_calibration_for_test_with_corrupt_writer(
    const std::string& path, const CalWrite& w, double margin_db,
    std::string* err);

// Counts how many times this process has attempted to PUBLISH (rename a
// verified candidate onto) the most recently published config path,
// successful or not -- keyed per path, so it reflects repeated calls
// against the SAME file (in production, always /etc/mabur.toml) rather
// than accumulating across unrelated paths. A past bug had the drone
// saving config on every bitrate change and chewing through flash; a
// calibration session must write at most once.
int write_count_for_test();

}  // namespace mabur
