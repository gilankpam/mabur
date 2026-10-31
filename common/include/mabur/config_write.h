#pragma once
// Rewrites a running unit's TOML config file in place without ever leaving
// it in a state its own loader cannot boot from.
//
// The failure mode this file exists to prevent: a daemon that sees a
// config it cannot load exits, and a supervisor that respawns it hits the
// same bad file on every retry. A device with no console needs a laptop
// and a serial cable to recover from that. publish_config's steps exist to
// make that unreachable:
//
//   1. write "<path>.new" + fsync -- a scratch file, not a publish. `path`
//      itself is not touched yet.
//   2. verify "<path>.new" with the caller's `verify` function -- the SAME
//      loader the daemon boots with, not a bare TOML syntax check.
//      Syntax-only parsing only proves the bytes are valid TOML; everything
//      that actually fails a boot -- unknown-key rejection, every
//      per-section range/type check -- lives inside the real loader, past
//      that parse. Verifying with anything weaker would let a file through
//      here that the daemon then refuses, which is exactly the scenario
//      this whole file exists to prevent. On failure, the scratch file is
//      discarded and `path` was never touched.
//   3. back up -- copy the original to "<path>.<backup_suffix>" (the
//      repo's ".pre-*" rollback convention), purely an operator's undo
//      copy, since nothing downstream of step 2 can produce a `path` that
//      fails to load. Written through the same write-then-rename primitive
//      as everything else, so a mid-write failure can't destroy the
//      PREVIOUS rollback copy while producing nothing usable in its place.
//   4. rename "<path>.new" onto `path` -- publishing bytes already proven
//      loadable by the function that will load them for real.
//
// `path` is resolved with realpath(3) before any of this runs, so a config
// that is itself a symlink -- onto a vfat partition mounted elsewhere, say
// -- has every step happen beside the symlink's TARGET: the symlink itself
// is never unlinked or replaced, only the file it points at.
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
// parse_toml_file/parse_toml_string), and the configs this patches are
// their default bundle file verbatim -- heavily commented files a
// re-serialize would flatten. patch_toml_key is therefore line-surgical:
// it rewrites one key's value in place, preserving every other line --
// comments included -- byte for byte. replace_toml_array_tables is the
// same idea for an array of tables ([[name]]): a whole run of repeated
// blocks is replaced verbatim rather than parsed and re-rendered.

#include <functional>
#include <mutex>
#include <string>

namespace mabur {

// Leading-whitespace-tolerant match for a bare key on one line: the key
// text followed (after optional whitespace) by '='. Rejects a key that's
// really a longer identifier's prefix (e.g. this must not fire for
// "legacy_wall_rel" while scanning for "legacy"), and a commented-out key.
// On success, *key_begin is where the key starts (after the line's leading
// whitespace) and *eq_pos is the index of '='.
bool match_toml_key(const std::string& line, const std::string& key,
                    size_t* key_begin, size_t* eq_pos);

// Rewrites the value of `key` on its own line inside [section], first
// occurrence, keeping leading whitespace, the spacing around '=' and any
// trailing comment (with the whitespace run before it). [section] is
// matched against a header's trimmed bracket contents exactly, and ends at
// the next line starting with '[' -- so [venc] ends where [venc.roi]
// begins, and a key above every header belongs to no section. A commented
// out key, or a longer key the requested one is a prefix of, does not
// match. `rendered_value` is inserted verbatim (a string value carries its
// own quotes). Returns false, leaving `*text` unchanged, when the key's
// line is not found in that section.
bool patch_toml_key(std::string* text, const std::string& section,
                    const std::string& key, const std::string& rendered_value);

enum class PublishResult {
  Ok,
  BackupFailed,   // couldn't read `path` or write its backup -- `path`
                  // untouched
  WriteFailed,    // couldn't write/fsync "<path>.new", or the final publish
                  // rename failed after verification already succeeded
  ReparseFailed,  // candidate failed `verify` -- never published
};

// True when `candidate_path` loads with the caller's own boot loader; on
// false, `*err` carries the loader's message (verbatim -- publish_config
// does not reshape it). The one thing every caller must supply: a verifier
// that is any weaker than the real loader defeats the whole point of this
// file (see the header comment above).
using VerifyFn = std::function<bool(const std::string& candidate_path, std::string* err)>;

// write "<path>.new" + fsync -> verify("<path>.new") -> back up `path` to
// "<path>.<backup_suffix>" -> rename. `path` is resolved with realpath(3)
// first, so a symlinked `path` has the scratch file, the backup and the
// rename all happen beside the symlink's target -- the symlink itself is
// never replaced -- and the real path's parent directory is what gets
// fsynced after the rename. See the file header for why each step exists
// and why verification happens before publishing. On anything but Ok,
// `path` is left exactly as it was found and no "<path>.new" is left
// behind. `err`, if non-null, is always set on a non-Ok result; on
// ReparseFailed it carries `verify`'s message. Serialised by one
// process-wide mutex: apply_calibration and the parameter component may
// call this from different threads, and both use the same scratch and
// backup names next to the same live config.
//
// `write_backup` (default true, so every existing caller keeps today's
// behaviour) skips step 3 entirely when false: "<path>.<backup_suffix>" is
// then never read, written or even looked at by this call. A caller that
// wants exactly one backup across several publish_config() calls against
// the same path -- a multi-apply calibration session owning its backup on
// only the first call -- passes false on every call after the first,
// rather than letting each one clobber the previous call's backup with
// its own pre-state and trying to patch that up afterwards.
PublishResult publish_config(const std::string& path, const std::string& content,
                             const std::string& backup_suffix, std::string* err,
                             const VerifyFn& verify, bool write_backup = true);

// Replaces the whole run of `[[<name>]]` array-of-tables blocks with
// `rendered_blocks` verbatim: from the first line that is exactly
// `[[<name>]]` (after trimming leading/trailing whitespace) through the
// line before the next table header -- `[other]` or `[[other]]`, including
// another `[[<name>]]` run that isn't contiguous -- that is NOT exactly
// `[[<name>]]`, or through end of file if there is none. A header is any
// line table_header() recognises (first non-blank character '['); "ours"
// is decided by comparing the trimmed line text, not by table_header()'s
// bracket-content parse, which only strips one pair of brackets. Returns
// false, `*text` unchanged, when no `[[<name>]]` header exists at all.
bool replace_toml_array_tables(std::string* text, const std::string& name,
                               const std::string& rendered_blocks);

// Takes the lock publish_config serialises on. A writer that reads the
// config, patches it and publishes the result holds this across all three:
// publish_config alone only serialises the publish, so two such writers on
// different threads could each patch the text they read before the other's
// publish and the later one would silently put the earlier one's key back.
// The lock is recursive, so publish_config can be called while holding it.
std::unique_lock<std::recursive_mutex> lock_config_writes();

// Reads the whole file at `path` into *out. False if it cannot be opened.
bool read_text_file(const std::string& path, std::string* out);

// Writes `content` to `path` (create or truncate), fsyncs it and its
// parent directory. The step-1 scratch write publish_config uses; a caller
// that wants a load_config verdict on a candidate before applying it writes
// the same "<path>.new" with this. False on any open/write/fsync failure,
// leaving whatever was written behind for the caller to remove.
bool write_file_fsync(const std::string& path, const std::string& content);

// load_config's message is "config: <file>:<line>: <field>: <why>". The
// file is a scratch candidate, meaningless to the operator, and a MAVLink
// STATUSTEXT holds 50 chars: returns "<field>: <why>". A message without
// that shape comes back with only the "config: " prefix removed.
std::string loader_reason(std::string s);

// Counts how many times this process has attempted to PUBLISH (rename a
// verified candidate onto) the most recently published config path,
// successful or not -- keyed per path, so it reflects repeated calls
// against the SAME file (in production, always /etc/mabur.toml) rather
// than accumulating across unrelated paths. A past bug had the drone
// saving config on every bitrate change and chewing through flash; a
// calibration session must write at most once.
int write_count_for_test();

}  // namespace mabur
