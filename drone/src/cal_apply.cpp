#include "cal_apply.h"

#include <fcntl.h>
#include <unistd.h>

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// The real loader, not just the TOML syntax check: see the "Verification"
// section of the file header comment for why config.cpp's load_config --
// unknown-key rejection, every per-section range/type check -- is the
// thing that must run here, not toml::parse_toml_file alone.
#include "config.h"

namespace mabur {

namespace {

// --- line-level key matching ------------------------------------------

// Leading-whitespace-tolerant match for a bare top-level key: the key text
// followed (after optional whitespace) by '='. Rejects a key that's really
// a longer identifier's prefix (e.g. this must not fire for "legacy_wall_idx"
// while scanning for "legacy"). On success, *key_begin is where the key's
// own leading whitespace starts and *eq_pos is the index of '='.
bool match_key(const std::string& line, const std::string& key,
               size_t* key_begin, size_t* eq_pos) {
  size_t i = 0;
  while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
  if (line.compare(i, key.size(), key) != 0) return false;
  size_t j = i + key.size();
  if (j < line.size()) {
    const unsigned char c = static_cast<unsigned char>(line[j]);
    if (std::isalnum(c) || c == '_') return false;  // longer identifier
  }
  size_t k = j;
  while (k < line.size() && (line[k] == ' ' || line[k] == '\t')) ++k;
  if (k >= line.size() || line[k] != '=') return false;
  *key_begin = i;
  *eq_pos = k;
  return true;
}

std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

// Parses the 8 ints out of a "[91, 91, ...]" array's inner text. A
// malformed original (should never happen -- it's bundle/mabur.default.toml
// verbatim) falls back to 0 for the unparseable slot rather than throwing;
// walls_in_range() and the load_config() verification step are the layers
// that actually guard correctness, not this cosmetic fallback.
std::array<int, 8> parse_int_array(const std::string& inner) {
  std::array<int, 8> vals{};
  size_t pos = 0;
  for (int idx = 0; idx < 8; ++idx) {
    size_t comma = inner.find(',', pos);
    const std::string tok =
        trim(comma == std::string::npos ? inner.substr(pos)
                                         : inner.substr(pos, comma - pos));
    try {
      vals[static_cast<size_t>(idx)] = std::stoi(tok);
    } catch (...) {
      vals[static_cast<size_t>(idx)] = 0;
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  return vals;
}

// Rewrites one matched line's value, preserving the leading whitespace
// before the key, the exact original spacing between the key and '=', and
// every byte from the end of the old value onward (trailing whitespace and
// any "# comment"). `value_end` is where the new value text goes; the
// caller supplies where the old value ended in `old_value_end`.
std::string rewrite_line(const std::string& line, size_t key_begin,
                          size_t key_len, size_t eq_pos,
                          const std::string& new_value,
                          size_t old_value_end) {
  const std::string leading = line.substr(0, key_begin);
  const std::string key = line.substr(key_begin, key_len);
  const std::string sep = line.substr(key_begin + key_len, eq_pos - (key_begin + key_len));
  const std::string trailing = line.substr(old_value_end);
  return leading + key + sep + "= " + new_value + trailing;
}

}  // namespace

std::string patch_toml(const std::string& text, const CalWrite& w,
                        PatchStatus* status) {
  std::vector<std::string> lines;
  {
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) lines.push_back(line);
  }
  const bool trailing_newline = !text.empty() && text.back() == '\n';

  for (std::string& line : lines) {
    size_t key_begin, eq_pos;

    if (match_key(line, "rate_walls_idx", &key_begin, &eq_pos)) {
      size_t v = eq_pos + 1;
      while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
      if (v < line.size() && line[v] == '[') {
        const size_t close = line.find(']', v);
        if (close != std::string::npos) {
          const auto orig = parse_int_array(line.substr(v + 1, close - v - 1));
          std::string arr = "[";
          for (int i = 0; i < 8; ++i) {
            const int val = w.walls[static_cast<size_t>(i)] == -1
                                ? orig[static_cast<size_t>(i)]
                                : w.walls[static_cast<size_t>(i)];
            if (i) arr += ", ";
            arr += std::to_string(val);
          }
          arr += "]";
          line = rewrite_line(line, key_begin, std::string("rate_walls_idx").size(),
                               eq_pos, arr, close + 1);
          if (status) status->rate_walls_idx = true;
        }
      }
      continue;
    }

    if (match_key(line, "legacy_wall_idx", &key_begin, &eq_pos)) {
      // -1 = not swept this session (derived from MCS0, per the header
      // comment) -- leave the line untouched rather than write a
      // fabricated number.
      if (w.legacy_wall == -1) continue;
      size_t v = eq_pos + 1;
      while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
      size_t e = v;
      if (e < line.size() && line[e] == '-') ++e;
      while (e < line.size() && std::isdigit(static_cast<unsigned char>(line[e]))) ++e;
      line = rewrite_line(line, key_begin, std::string("legacy_wall_idx").size(),
                           eq_pos, std::to_string(w.legacy_wall), e);
      if (status) status->legacy_wall_idx = true;
      continue;
    }

    if (match_key(line, "base_ref_idx", &key_begin, &eq_pos)) {
      size_t v = eq_pos + 1;
      while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
      size_t e = v;
      if (e < line.size() && line[e] == '-') ++e;
      while (e < line.size() && std::isdigit(static_cast<unsigned char>(line[e]))) ++e;
      line = rewrite_line(line, key_begin, std::string("base_ref_idx").size(),
                           eq_pos, std::to_string(w.base_ref_idx), e);
      if (status) status->base_ref_idx = true;
      continue;
    }

    if (match_key(line, "power_mode", &key_begin, &eq_pos)) {
      size_t v = eq_pos + 1;
      while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
      if (v < line.size() && line[v] == '"') {
        const size_t close = line.find('"', v + 1);
        if (close != std::string::npos) {
          line = rewrite_line(line, key_begin, std::string("power_mode").size(),
                               eq_pos, "\"offset\"", close + 1);
          if (status) status->power_mode = true;
        }
      }
      continue;
    }
  }

  std::string out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out += lines[i];
    if (i + 1 < lines.size()) out += "\n";
  }
  if (trailing_newline) out += "\n";
  return out;
}

bool walls_in_range(const CalWrite& w, double margin_db, std::string* why) {
  // Mirrors drone/src/config.cpp:133-155 exactly, including its message
  // wording, so a refusal here reads the same way a boot-time refusal
  // would. This pre-check and the load_config() verification step later in
  // apply_calibration() serve different purposes and both stay: this one
  // refuses a bad table before the disk is touched at all, with a specific
  // OutOfRange result and a message naming the exact derivation that
  // failed; load_config() is the backstop that catches anything this
  // model does not -- a mistyped key, a value outside the four keys we
  // patch, a config field this function knows nothing about.
  const int m = static_cast<int>(std::lround(margin_db * 4.0));
  for (size_t i = 0; i < w.walls.size(); ++i) {
    if (w.walls[i] == -1) continue;  // undetermined this session, not written
    const int diff = w.walls[i] - m - w.base_ref_idx;
    if (diff < -64 || diff > 63) {
      if (why)
        *why = "radio.rate_walls_idx[" + std::to_string(i) +
               "]: derived diff (wall - wall_margin_db*4 - base_ref_idx) = " +
               std::to_string(diff) +
               " is out of the 7-bit hardware field range [-64,63] -- "
               "check base_ref_idx/wall_margin_db calibration";
      return false;
    }
  }
  if (w.legacy_wall != -1) {
    const int diff = w.legacy_wall - m - w.base_ref_idx;
    if (diff < -64 || diff > 63) {
      if (why)
        *why = "radio.legacy_wall_idx: derived diff (legacy_wall_idx - "
               "wall_margin_db*4 - base_ref_idx) = " +
               std::to_string(diff) +
               " is out of the 7-bit hardware field range [-64,63] -- "
               "check base_ref_idx/wall_margin_db calibration";
      return false;
    }
  }
  return true;
}

// Checks that patch_toml() actually found and rewrote every key this file
// is required to have touched. Required-ness mirrors CalWrite's own
// sentinel rule: rate_walls_idx, base_ref_idx and power_mode are always
// rewritten, so their PatchStatus flag must always be set; legacy_wall_idx
// is only required when CalWrite::legacy_wall is determined (not -1) --
// when it's -1 the line is deliberately left untouched and its flag is
// expected to stay false.
bool patch_covered_required_keys(const PatchStatus& s, const CalWrite& w,
                                  std::string* why) {
  if (!s.rate_walls_idx) {
    if (why) *why = "radio.rate_walls_idx: line not found in config -- unexpected file structure, nothing written";
    return false;
  }
  if (!s.base_ref_idx) {
    if (why) *why = "radio.base_ref_idx: line not found in config -- unexpected file structure, nothing written";
    return false;
  }
  if (!s.power_mode) {
    if (why) *why = "radio.power_mode: line not found in config -- unexpected file structure, nothing written";
    return false;
  }
  if (w.legacy_wall != -1 && !s.legacy_wall_idx) {
    if (why) *why = "radio.legacy_wall_idx: line not found in config -- unexpected file structure, nothing written";
    return false;
  }
  return true;
}

namespace {

// Keyed by real config path, not accumulated globally: production only
// ever publishes to one path (/etc/mabur.toml) across a whole daemon
// session, so this still catches the flash-wear bug (repeated publishes
// against that one path) while a test's own scratch path -- untouched by
// any other test -- starts fresh regardless of how many other paths
// earlier tests in the same binary published to. Incremented once per
// *publish attempt* (the final rename onto `path`), not per byte written
// to a scratch file -- writing "<path>.new" and then discarding it
// because verification failed never touches the real path's flash cell
// at all.
std::map<std::string, int> g_write_counts;
std::string g_last_write_path;

bool read_file(const std::string& path, std::string* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

// fsync's the directory holding `path`. A rename's durability depends on
// its parent directory entry being flushed, not just the file's own data
// -- without this, a crash right after a "successful" rename could still
// lose it on some filesystems.
void fsync_parent_dir(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  const std::string dir = (slash == std::string::npos) ? "." : path.substr(0, slash);
  const int dfd = ::open(dir.c_str(), O_RDONLY);
  if (dfd >= 0) {
    ::fsync(dfd);
    ::close(dfd);
  }
}

bool write_file_fsync(const std::string& path, const std::string& content) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return false;
  bool ok = true;
  size_t off = 0;
  while (ok && off < content.size()) {
    const ssize_t n = ::write(fd, content.data() + off, content.size() - off);
    if (n < 0) {
      ok = false;
      break;
    }
    off += static_cast<size_t>(n);
  }
  if (ok && ::fsync(fd) != 0) ok = false;
  if (::close(fd) != 0) ok = false;
  if (ok) fsync_parent_dir(path);
  return ok;
}

// Best-effort cleanup of the scratch file: never let a failed run leave a
// stale "<path>.new" sitting next to a live config (confusing at best, and
// a later successful run's O_TRUNC would clobber it anyway -- this just
// makes a failed run leave no trace instead of a trap for whoever looks
// at the directory next).
void remove_scratch(const std::string& tmp) { ::unlink(tmp.c_str()); }

// Writes `content` to `path` via a private temp file + fsync + rename,
// same pattern as the main "<path>.new" publish, so a failure partway
// through (disk full mid-write) never destroys whatever `path` already
// held. Used for the backup copy: "<path>.pre-cal" is the operator's
// actual rollback artifact, and losing the PREVIOUS session's backup to a
// failed O_TRUNC write -- while producing nothing usable in its place --
// would be a silent loss even though the live config stays unaffected.
bool write_file_atomic(const std::string& path, const std::string& content) {
  const std::string tmp = path + ".tmp";
  if (!write_file_fsync(tmp, content)) {
    remove_scratch(tmp);
    return false;
  }
  if (::rename(tmp.c_str(), path.c_str()) != 0) {
    remove_scratch(tmp);
    return false;
  }
  fsync_parent_dir(path);
  return true;
}

// Shared body of apply_calibration() and its corrupt-writer test seam.
// `content_override`, when set, replaces the real patch_toml() output --
// the seam tests need a way to get deliberate garbage onto disk without a
// real writer bug, to prove verification actually catches it.
//
// Order matters and is deliberately NOT "publish, then verify": that
// ordering (this file's first draft) leaves a crash window between the
// rename that publishes a candidate and the check that would have caught
// it, during which a bad file can already be live at `path`. No restore
// logic can close that window -- it's a property of doing the checkable
// operation on the file only after that file is already the one `maburd`
// will read. Verifying "<path>.new" BEFORE it is ever renamed onto `path`
// removes the window instead of narrowing it: every crash point below
// leaves either the untouched original or bytes already proven loadable.
ApplyResult apply_calibration_impl(const std::string& path, const CalWrite& w,
                                    double margin_db, std::string* err,
                                    const std::string* content_override) {
  // Step 1: validate the derived diffs. Nothing on disk is touched if
  // this fails -- see the walls_in_range() comment for why this pre-check
  // stays even though load_config() below re-derives the same numbers.
  std::string why;
  if (!walls_in_range(w, margin_db, &why)) {
    if (err) *err = why;
    return ApplyResult::OutOfRange;
  }

  std::string original;
  if (!read_file(path, &original)) {
    if (err) *err = "failed to read " + path;
    return ApplyResult::WriteFailed;
  }

  // Step 2: patch in memory and check every REQUIRED key was actually
  // found and rewritten. patch_toml has no error channel of its own -- a
  // key it can't match (missing, reformatted, a multi-line array) is
  // silently left as the original's value, which load_config() below
  // would still accept (an unmodified original value is still a valid
  // one). Without this check, apply_calibration could report Ok on a
  // calibration that never actually applied -- the exact
  // silent-miscalibration failure this file exists to prevent, arriving
  // through the back door. Skipped for content_override: that seam
  // bypasses patch_toml on purpose to inject raw garbage.
  std::string content;
  if (content_override) {
    content = *content_override;
  } else {
    PatchStatus status;
    content = patch_toml(original, w, &status);
    std::string key_why;
    if (!patch_covered_required_keys(status, w, &key_why)) {
      if (err) *err = key_why;
      return ApplyResult::KeyNotFound;
    }
  }

  // Step 3: write the candidate to a scratch file. Nothing at `path` has
  // moved yet -- this is not a publish, it's a place to point the real
  // loader at.
  const std::string tmp = path + ".new";
  if (!write_file_fsync(tmp, content)) {
    remove_scratch(tmp);
    if (err) *err = "failed to write/fsync " + tmp;
    return ApplyResult::WriteFailed;
  }

  // Step 4: verify with the SAME function maburd boots with --
  // mabur::load_config, not toml::parse_toml_file. parse_toml_file only
  // proves the bytes are syntactically valid TOML; everything that
  // actually fails a boot -- check_known_keys' unknown-key rejection, and
  // every per-section range/type check config.cpp applies, including the
  // very rate_walls_idx/base_ref_idx diff check walls_in_range() mirrors
  // above -- lives inside load_config(), past the parse. A verification
  // step that cannot detect the failure it exists to guard against is
  // worse than none: it would make maburd's later refusal look
  // impossible right up until it happens on the device.
  //
  // load_config() is a pure parse-and-validate with no hardware side
  // effects (it builds and returns a Config value; it does not open any
  // device or touch global state beyond a scoped defaulted-keys collector
  // that is cleared on every return path), which is what makes it safe to
  // call speculatively on a file that is not the live config. That is an
  // assumption about a function this file does not own -- if a future
  // load_config() gained a side effect, this call would need revisiting.
  try {
    (void)load_config(tmp, nullptr);
  } catch (const std::exception& e) {
    // Never published: `path` was never touched, so there is nothing to
    // restore, only something to discard.
    remove_scratch(tmp);
    if (err) *err = std::string("candidate failed load_config, not published: ") + e.what();
    return ApplyResult::ReparseFailed;
  }

  // Step 5: back up the original. Now that the candidate is proven
  // loadable, this is purely the operator's rollback artifact (the "undo
  // my last calibration" copy), not a crash-recovery mechanism -- nothing
  // downstream of this point can produce a `path` that fails to load.
  // Written via write_file_atomic so a failure partway through can't
  // destroy the PREVIOUS session's backup while leaving nothing usable.
  const std::string backup_path = path + ".pre-cal";
  if (!write_file_atomic(backup_path, original)) {
    remove_scratch(tmp);
    if (err) *err = "failed to write backup " + backup_path;
    return ApplyResult::BackupFailed;
  }

  // Step 6: publish. `tmp` has already been proven loadable by the same
  // function maburd boots with, so this rename can only ever replace
  // `path` with something good.
  ++g_write_counts[path];  // flash-wear guard: counts publish attempts to this path
  g_last_write_path = path;
  if (::rename(tmp.c_str(), path.c_str()) != 0) {
    // The live config was never touched -- `path` still holds exactly
    // what it did before this call, which is a safe outcome. But it is a
    // failure, not a success: report it as one, and don't leave the
    // proven-good scratch file sitting next to the (unchanged) live
    // config for someone to find later.
    if (err)
      *err = "verified candidate but failed to rename " + tmp + " onto " + path;
    remove_scratch(tmp);
    return ApplyResult::WriteFailed;
  }
  fsync_parent_dir(path);
  return ApplyResult::Ok;
}

}  // namespace

ApplyResult apply_calibration(const std::string& path, const CalWrite& w,
                               double margin_db, std::string* err) {
  return apply_calibration_impl(path, w, margin_db, err, nullptr);
}

ApplyResult apply_calibration_for_test_with_corrupt_writer(
    const std::string& path, const CalWrite& w, double margin_db,
    std::string* err) {
  static const std::string kGarbage = "this is not valid toml [[[ ===\n";
  return apply_calibration_impl(path, w, margin_db, err, &kGarbage);
}

int write_count_for_test() {
  const auto it = g_write_counts.find(g_last_write_path);
  return it == g_write_counts.end() ? 0 : it->second;
}

}  // namespace mabur
