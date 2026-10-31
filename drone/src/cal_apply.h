#pragma once
// Writes one width's measured walls and/or air efficiency into a running
// unit's /etc/mabur.toml.
//
// The failure mode this file exists to prevent: CLAUDE.md and
// docs/deploy.md both record that a daemon which sees a config it cannot
// load exits, and its wrapper respawns it forever at 2 s. The drone has no
// console -- a calibration run that leaves an unloadable config behind
// turns a two-minute convenience into a device that needs a laptop and a
// serial cable to recover. Every step below exists to make that
// unreachable:
//
//   1. validate -- run the candidate walls/efficiency through the same
//      derivation config.cpp's loader applies and refuse to touch anything
//      if a derived diff would leave the 8822E's 7-bit rate-diff field
//      range [-64,63], or an efficiency outside [0.05,1.0].
//   2. patch -- rewrite (or insert) this width's keys in memory and check
//      that every REQUIRED one was actually found, rewritten or inserted
//      (see PatchStatus). patch_toml has no TOML-writer fallback: if a
//      key's line can't be matched or inserted -- missing, reformatted, a
//      multi-line array -- silently leaving it alone would mean
//      load_config() below still accepts the file (an unmodified original
//      value is still a valid one) and apply_calibration reports success
//      on a calibration that never actually applied. That is the
//      silent-miscalibration failure this whole kit exists to prevent,
//      arriving through the back door, so it is checked explicitly and
//      refused loudly (KeyNotFound) rather than left to slip through.
//   3-6. publish_config (config_write.h): write "<path>.new" + fsync,
//      verify it with mabur::load_config -- the same function maburd boots
//      with -- back the original up to "<path>.pre-cal", then rename the
//      verified candidate into place. config_write.h's header explains why
//      verification happens before publishing and why the backup goes
//      through write-then-rename.
//
// patch_toml is line-surgical for the same reason patch_toml_key is (see
// config_write.h): /etc/mabur.toml is bundle/mabur.default.toml verbatim,
// a heavily commented file a re-serialize would flatten, so it rewrites
// only the keys this file owns, in place, preserving every other line --
// comments included -- byte for byte. A width calibrated for the first
// time has no existing rate_walls_rel_<bw>/cal_flags_<bw>/efficiency_<bw>
// line to rewrite; patch_toml inserts one instead (see insert_toml_key_after
// in the .cpp), anchored next to the other width's line of the same key so
// the file keeps reading as one per-width pair, not two unrelated halves.

#include <array>
#include <cstdint>
#include <string>

#include "config_write.h"
#include "mabur/rc_proto.h"
#include "mabur/toml.h"

namespace mabur {

// kWallUndetermined in walls[r] (or legacy_wall) means "undetermined: keep
// the config's value": that rate wasn't swept this session, so the
// existing config's value for it is kept. Never write a fabricated number
// over a real measurement. eff[r] == 0 is the same convention for
// air_clock.efficiency_<bw>: a capacity point the sweep never resolved (a
// tx_bound point, or a rate this session didn't measure) leaves the
// existing value alone.
//
// bw selects which per-width keys this write touches (rate_walls_rel_<bw>,
// cal_flags_<bw>, efficiency_<bw>); has_walls/has_eff say which of the two
// halves of a CalWrite are actually meant to be applied -- a session that
// only re-measured capacity (efficiency) must not touch power_mode,
// cal_flags_<bw> or legacy_wall_rel, and vice versa.
struct CalWrite {
  uint8_t bw = 20;
  bool has_walls = false;
  bool has_eff = false;
  std::array<int, 8> walls{
      mabur::rc::kWallUndetermined, mabur::rc::kWallUndetermined,
      mabur::rc::kWallUndetermined, mabur::rc::kWallUndetermined,
      mabur::rc::kWallUndetermined, mabur::rc::kWallUndetermined,
      mabur::rc::kWallUndetermined, mabur::rc::kWallUndetermined};
  int legacy_wall =
      mabur::rc::kWallUndetermined;  // derived from the MCS0 sweep result,
                                      // not swept itself
  std::array<double, 8> eff{};       // 0 = leave
  std::string cal_flags = "ldpc+stbc";
};

// Which keys patch_toml actually found-and-rewrote or created. A flag
// stays false either because the key's line was never matched and could
// not be inserted either (missing anchor, reformatted, a multi-line array
// patch_toml doesn't parse) or, for legacy_wall_rel only, because it was
// deliberately left untouched (CalWrite::legacy_wall undetermined, or this
// width/file combination doesn't own that line -- see patch_toml's
// comment). apply_calibration uses this to tell those cases apart from
// "found, rewritten or inserted", since only the first is an error.
// inserted_walls/inserted_cal_flags additionally say which of those two
// took the insert path rather than rewriting an existing line; there is no
// equivalent inserted_eff -- nothing downstream needs to tell an inserted
// efficiency_<bw> line apart from a rewritten one.
struct PatchStatus {
  bool walls = false, cal_flags = false, legacy_wall_rel = false,
       power_mode = false, eff = false, inserted_walls = false,
       inserted_cal_flags = false;
};

enum class ApplyResult {
  Ok,
  OutOfRange,     // a derived diff/efficiency would leave its valid range
  KeyNotFound,    // patch_toml couldn't find/insert a required key -- the
                  // file's structure wasn't what this line-surgical patcher
                  // expects; nothing touched
  BackupFailed,   // couldn't write "<path>.pre-cal" -- nothing else touched
  WriteFailed,    // couldn't write/fsync "<path>.new", or the final publish
                  // rename failed after verification already succeeded
  ReparseFailed,  // candidate failed mabur::load_config -- never published
};

// Rewrites (or inserts) only the keys `w` touches in `text`, preserving
// every other line, every comment, and each rewritten line's original
// spacing before '=' and any trailing comment. An array entry left at its
// "leave" sentinel (kWallUndetermined for walls[], 0 for eff[]) keeps
// whatever value the original text had for it there -- unless the whole
// line is being inserted (first calibration of this width), in which case
// there is no original to inherit and a left-at-sentinel entry takes a
// fixed fallback instead (kRailRel for walls, 1.0/nominal for eff -- see
// the .cpp). cal_flags_<bw> is rewritten to w.cal_flags whenever has_walls
// is set, unconditionally (it records what the walls above were measured
// with, not a per-rate value with its own sentinel). legacy_wall_rel is
// shared by both widths: it is only ever repatched from a 20 MHz write, or
// from a 40 MHz write on a file with no rate_walls_rel_20 line of its own.
// power_mode is set to "offset" whenever has_walls is set. If `status` is
// non-null, it is filled in with which keys were actually touched -- see
// PatchStatus. Never throws; a key it can neither find nor insert is
// simply left unmatched in `*status` for the caller to act on.
std::string patch_toml(const std::string& text, const CalWrite& w,
                        PatchStatus* status = nullptr);

// Mirrors config.cpp: every determined rel in [-64,63] and
// rel - lround(margin_db*4) >= -64, for every swept rate
// (walls[r] != kWallUndetermined) and for legacy_wall (if determined);
// every determined eff (eff[r] != 0) in [0.05, 1.0]. On failure, *why
// names the offending key, matching config.cpp's message so a walls
// refusal reads the same as a boot failure would (the eff floor is this
// file's own sanity bound -- the loader itself only requires (0,1]).
//
// This stays even though apply_calibration()'s load_config() verification
// re-derives the same numbers: this check refuses a bad table before the
// disk is touched at all, with a specific OutOfRange result and a message
// naming the exact derivation that failed. load_config() is the backstop
// that catches anything this model does not -- a mistyped key, a value
// outside the keys this file patches, a config field this function knows
// nothing about. Belt and braces, not redundancy.
bool walls_in_range(const CalWrite& w, double margin_db, std::string* why);

// Validate -> patch (+ required-key check) -> write "<path>.new" + fsync
// -> verify with mabur::load_config -> back up -> rename into place. See
// the file header for why each step exists and why verification happens
// before publishing rather than after. On anything but Ok, `path` is left
// exactly as it was found: it is never replaced by anything that has not
// already been proven loadable by the same function maburd boots with.
// `err`, if non-null, is always set on a non-Ok result.
//
// `backup` is the caller's call on whether THIS apply should write
// "<path>.pre-cal": a calibration session applying several widths/kinds
// one call at a time wants exactly one backup for the whole session (the
// state before any of them ran), not one per call that would overwrite it
// with an already-mid-session state. Passing false never destroys an
// existing backup from an earlier call in the same session; it only
// suppresses writing a NEW one.
ApplyResult apply_calibration(const std::string& path, const CalWrite& w,
                               double margin_db, bool backup, std::string* err);

// Test-only seam: same as apply_calibration (with backup requested), but
// writes deliberate garbage to "<path>.new" instead of the patched config
// (bypassing patch_toml and its required-key check entirely), so the
// verify-then-refuse path can be exercised without needing a real writer
// bug to trigger it.
ApplyResult apply_calibration_for_test_with_corrupt_writer(
    const std::string& path, const CalWrite& w, double margin_db,
    std::string* err);

}  // namespace mabur
