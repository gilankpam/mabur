#include "config_write.h"

#include <fcntl.h>
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <exception>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

// The real loader, not just the TOML syntax check: see the file header of
// config_write.h for why config.cpp's load_config -- unknown-key
// rejection, every per-section range/type check -- is the thing that must
// run here, not toml::parse_toml_file alone.
#include "config.h"

namespace mabur {

bool match_toml_key(const std::string& line, const std::string& key,
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

namespace {

bool is_blank(char c) { return c == ' ' || c == '\t'; }

// If `line` is a table header ("[name]", optionally indented and followed
// by a comment), stores its trimmed bracket contents in *name. A header is
// any line whose first non-blank character is '[': TOML has no other line
// that can start that way outside a multi-line value, which this patcher
// does not edit.
bool table_header(const std::string& line, std::string* name) {
  size_t i = 0;
  while (i < line.size() && is_blank(line[i])) ++i;
  if (i >= line.size() || line[i] != '[') return false;
  const size_t close = line.find(']', i + 1);
  size_t a = i + 1;
  size_t b = close == std::string::npos ? line.size() : close;
  while (a < b && is_blank(line[a])) ++a;
  while (b > a && is_blank(line[b - 1])) --b;
  *name = line.substr(a, b - a);
  return true;
}

// Where the value that starts at `v` ends: the start of the whitespace run
// before a trailing '#' comment, or before the end of the line. A '#'
// inside a quoted string is part of the value, not a comment.
size_t value_end(const std::string& line, size_t v) {
  size_t e = line.size();
  char quote = 0;
  for (size_t i = v; i < line.size(); ++i) {
    const char c = line[i];
    if (quote) {
      if (c == '\\' && quote == '"') {
        ++i;  // basic strings escape; literal strings do not
      } else if (c == quote) {
        quote = 0;
      }
    } else if (c == '"' || c == '\'') {
      quote = c;
    } else if (c == '#') {
      e = i;
      break;
    }
  }
  while (e > v && is_blank(line[e - 1])) --e;
  return e;
}

// One mutex for every publish in the process, and for the write-count
// hook it updates. Recursive because lock_config_writes() hands it to a
// read-patch-publish caller who then calls publish_config.
std::recursive_mutex g_publish_mu;

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

// Best-effort cleanup of the scratch file: never let a failed run leave a
// stale "<path>.new" sitting next to a live config (confusing at best, and
// a later successful run's O_TRUNC would clobber it anyway -- this just
// makes a failed run leave no trace instead of a trap for whoever looks
// at the directory next).
void remove_scratch(const std::string& tmp) { ::unlink(tmp.c_str()); }

// Writes `content` to `path` via a private temp file + fsync + rename,
// same pattern as the main "<path>.new" publish, so a failure partway
// through (disk full mid-write) never destroys whatever `path` already
// held. Used for the backup copy: "<path>.pre-*" is the operator's actual
// rollback artifact, and losing the PREVIOUS backup to a failed O_TRUNC
// write -- while producing nothing usable in its place -- would be a
// silent loss even though the live config stays unaffected.
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

}  // namespace

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

bool patch_toml_key(std::string* text, const std::string& section,
                    const std::string& key, const std::string& rendered_value) {
  std::string current;  // "" until the first header: no section
  bool in_any = false;
  size_t start = 0;
  while (start < text->size()) {
    size_t nl = text->find('\n', start);
    if (nl == std::string::npos) nl = text->size();
    const std::string line = text->substr(start, nl - start);
    std::string name;
    if (table_header(line, &name)) {
      current = name;
      in_any = true;
    } else if (in_any && current == section) {
      size_t key_begin, eq_pos;
      if (match_toml_key(line, key, &key_begin, &eq_pos)) {
        size_t v = eq_pos + 1;
        while (v < line.size() && is_blank(line[v])) ++v;
        const size_t e = value_end(line, v);
        text->replace(start + v, e - v, rendered_value);
        return true;
      }
    }
    start = nl + 1;
  }
  return false;
}

bool read_text_file(const std::string& path, std::string* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

std::string loader_reason(std::string s) {
  static const std::string kPrefix = "config: ";
  if (s.compare(0, kPrefix.size(), kPrefix) == 0) s.erase(0, kPrefix.size());
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] != ':') continue;
    size_t j = i + 1;
    while (j < s.size() && std::isdigit(static_cast<unsigned char>(s[j]))) ++j;
    if (j > i + 1 && s.compare(j, 2, ": ") == 0) return s.substr(j + 2);
  }
  return s;
}

PublishResult publish_config(const std::string& path, const std::string& content,
                             const std::string& backup_suffix, std::string* err,
                             bool write_backup) {
  // Every caller shares "<path>.new" and the backup's temp name, so two
  // concurrent publishes would truncate each other's candidate mid-verify
  // or rename it out from under each other.
  std::lock_guard<std::recursive_mutex> lock(g_publish_mu);

  // Read the original first: when write_backup, it is the backup's
  // content, and a path that cannot be read has nothing to back up --
  // refuse before writing anything. Read unconditionally even when
  // !write_backup: a path this call can't read is just as wrong a thing
  // to publish over, and callers (apply_calibration) already rely on this
  // failing the same way regardless of the backup flag.
  std::string original;
  if (!read_text_file(path, &original)) {
    if (err) *err = "failed to read " + path;
    return PublishResult::BackupFailed;
  }

  // Step 1: write the candidate to a scratch file. Nothing at `path` has
  // moved yet -- this is not a publish, it's a place to point the real
  // loader at.
  const std::string tmp = path + ".new";
  if (!write_file_fsync(tmp, content)) {
    remove_scratch(tmp);
    if (err) *err = "failed to write/fsync " + tmp;
    return PublishResult::WriteFailed;
  }

  // Step 2: verify with the SAME function maburd boots with --
  // mabur::load_config, not toml::parse_toml_file. parse_toml_file only
  // proves the bytes are syntactically valid TOML; everything that
  // actually fails a boot -- check_known_keys' unknown-key rejection, and
  // every per-section range/type check config.cpp applies -- lives inside
  // load_config(), past the parse. A verification step that cannot detect
  // the failure it exists to guard against is worse than none: it would
  // make maburd's later refusal look impossible right up until it happens
  // on the device.
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
    if (err) *err = e.what();
    return PublishResult::ReparseFailed;
  }

  // Step 3: back up the original. Now that the candidate is proven
  // loadable, this is purely the operator's rollback artifact, not a
  // crash-recovery mechanism -- nothing downstream of this point can
  // produce a `path` that fails to load. Skipped entirely when the caller
  // doesn't want this call to own the backup (see the write_backup
  // comment in config_write.h) -- "<path>.<backup_suffix>" is left
  // completely untouched, not written and then reverted.
  if (write_backup) {
    const std::string backup_path = path + "." + backup_suffix;
    if (!write_file_atomic(backup_path, original)) {
      remove_scratch(tmp);
      if (err) *err = "failed to write backup " + backup_path;
      return PublishResult::BackupFailed;
    }
  }

  // Step 4: publish. `tmp` has already been proven loadable by the same
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
    return PublishResult::WriteFailed;
  }
  fsync_parent_dir(path);
  return PublishResult::Ok;
}

std::unique_lock<std::recursive_mutex> lock_config_writes() {
  return std::unique_lock<std::recursive_mutex>(g_publish_mu);
}

int write_count_for_test() {
  std::lock_guard<std::recursive_mutex> lock(g_publish_mu);
  const auto it = g_write_counts.find(g_last_write_path);
  return it == g_write_counts.end() ? 0 : it->second;
}

}  // namespace mabur
