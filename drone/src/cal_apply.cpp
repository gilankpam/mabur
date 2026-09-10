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
// walls_in_range() and the re-parse step are the layers that actually
// guard correctness, not this cosmetic fallback.
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

std::string patch_toml(const std::string& text, const CalWrite& w) {
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
  // would -- this task's whole point is to never let a config reach that
  // boot-time check unvalidated.
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

namespace {

// Keyed by real config path, not accumulated globally: production only
// ever touches one path (/etc/mabur.toml) across a whole daemon session,
// so this still catches the flash-wear bug (repeated calls against that
// one path) while a test's own scratch path -- untouched by any other
// test -- starts fresh regardless of how many other paths earlier tests
// in the same binary wrote to.
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

// The only place anything is written to a real config path. write ->
// fsync -> rename: a crash before the rename leaves the old file at
// `path` untouched (the new content only ever exists at "<path>.new"),
// and a crash after it is a completed, durable write. There is no
// in-between state where `path` itself is half-written.
ApplyResult write_new_and_rename(const std::string& path,
                                  const std::string& content,
                                  std::string* err) {
  ++g_write_counts[path];  // flash-wear guard: counts every write attempt to this path
  g_last_write_path = path;
  const std::string tmp = path + ".new";
  if (!write_file_fsync(tmp, content)) {
    if (err) *err = "failed to write/fsync " + tmp;
    return ApplyResult::WriteFailed;
  }
  if (::rename(tmp.c_str(), path.c_str()) != 0) {
    if (err) *err = "failed to rename " + tmp + " onto " + path;
    return ApplyResult::WriteFailed;
  }
  fsync_parent_dir(path);
  return ApplyResult::Ok;
}

// Shared body of apply_calibration() and its corrupt-writer test seam.
// `content_override`, when set, replaces the real patch_toml() output --
// the seam tests need a way to get deliberate garbage onto disk without a
// real writer bug, to prove the restore path actually runs.
ApplyResult apply_calibration_impl(const std::string& path, const CalWrite& w,
                                    double margin_db, std::string* err,
                                    const std::string* content_override) {
  // Step 1: validate. Nothing is touched if this fails.
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

  // Step 2a: back up, before anything at `path` is touched.
  const std::string backup_path = path + ".pre-cal";
  if (!write_file_fsync(backup_path, original)) {
    if (err) *err = "failed to write backup " + backup_path;
    return ApplyResult::BackupFailed;
  }

  // Step 2b: atomic write of the candidate content.
  const std::string content = content_override ? *content_override : patch_toml(original, w);
  const ApplyResult wr = write_new_and_rename(path, content, err);
  if (wr != ApplyResult::Ok) return wr;

  // Step 3: re-parse with the REAL loader. This is the check that makes
  // the respawn loop unreachable -- everything above is defense in depth
  // for a bug this step would still catch.
  try {
    (void)toml::parse_toml_file(path);
  } catch (const std::exception& e) {
    // Restore via rename, not a content copy: one more atomic swap, same
    // guarantee as the write we're undoing -- no window where `path` is
    // half-restored. This does consume the ".pre-cal" file (it becomes
    // `path` again), which is fine: a session that got this far already
    // failed and the caller is being told so.
    //
    // The restore rename itself can fail too (rare, but this is exactly
    // the corner the whole design exists to be paranoid about): if it
    // does, `path` is left holding the content that just failed to parse
    // -- the one outcome every earlier step was meant to prevent. Report
    // that distinctly rather than claiming a restore that didn't happen,
    // so an operator (or a caller logging this) knows the device needs a
    // manual fix, not just a "try again."
    if (::rename(backup_path.c_str(), path.c_str()) != 0) {
      if (err)
        *err = std::string("re-parse failed AND restore failed -- ") + path +
               " is left holding unparseable content, backup is at " +
               backup_path + ": " + e.what();
      return ApplyResult::ReparseFailed;
    }
    fsync_parent_dir(path);
    if (err) *err = std::string("re-parse failed, restored backup: ") + e.what();
    return ApplyResult::ReparseFailed;
  }
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
