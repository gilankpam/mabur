#include "config_migrate.h"

#include <unistd.h>

#include <cstdio>
#include <exception>
#include <utility>

#include "config.h"
#include "mabur/config_write.h"

namespace mabur {
namespace {

// Per-drone keys: the channel set, the files, and the calibration results
// (walls, what they were measured with, the legacy wall). A value equal to
// today's default is still this drone's, and must not follow a later
// default change.
const char* const kNeverPrune[] = {"radio.channels",    "radio.rate_walls_rel_20",
                                   "radio.rate_walls_rel_40", "radio.cal_flags_20",
                                   "radio.cal_flags_40", "radio.legacy_wall_rel",
                                   "link.key_file",     "venc.sensor_bin"};

std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r')) ++a;
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
  return s.substr(a, b - a);
}

struct KeyLine {
  std::string section, key, value;   // value as written, comment dropped
};

// Every `key = value` line in file order, with its table. Continuation lines
// of a multi-line value carry no '=' and are skipped.
std::vector<KeyLine> key_lines(const std::string& text) {
  std::vector<KeyLine> out;
  std::string section;
  size_t at = 0;
  while (at < text.size()) {
    size_t nl = text.find('\n', at);
    if (nl == std::string::npos) nl = text.size();
    const std::string line = trim(text.substr(at, nl - at));
    at = nl + 1;
    if (line.empty() || line[0] == '#') continue;
    if (line[0] == '[') {
      const size_t close = line.find(']');
      section = trim(line.substr(1, close == std::string::npos ? std::string::npos : close - 1));
      continue;
    }
    const size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string value = line.substr(eq + 1);
    const size_t hash = value.find(" #");
    if (hash != std::string::npos) value = value.substr(0, hash);
    out.push_back({section, trim(line.substr(0, eq)), trim(value)});
  }
  return out;
}

bool never_prune(const std::string& dotted) {
  for (const char* k : kNeverPrune)
    if (dotted == k) return true;
  return false;
}

// Loads `text` through the real loader. False with *err on any refusal.
bool load_text(const std::string& text, const std::string& scratch, Config* out,
               std::string* err) {
  if (!write_file_fsync(scratch, text)) {
    *err = "cannot write " + scratch;
    return false;
  }
  try {
    *out = load_config(scratch);
    return true;
  } catch (const std::exception& e) {
    *err = loader_reason(e.what());
    return false;
  }
}

MigrateResult migrate_impl(std::string text, const std::string& scratch) {
  MigrateResult r;
  Config base;
  std::string err;
  std::string last_removed;
  while (!load_text(text, scratch, &base, &err)) {
    const std::string suffix = ": unknown key";
    if (err.size() <= suffix.size() ||
        err.compare(err.size() - suffix.size(), suffix.size(), suffix) != 0) {
      r.error = err;
      return r;
    }
    const std::string field = err.substr(0, err.size() - suffix.size());
    const size_t dot = field.rfind('.');
    const bool removed =
        field != last_removed &&
        (remove_toml_table(&text, field) ||
         (dot != std::string::npos &&
          remove_toml_key(&text, field.substr(0, dot), field.substr(dot + 1))) ||
         (dot == std::string::npos && remove_toml_key(&text, "", field)));
    if (!removed) {
      r.error = "cannot remove " + field;
      return r;
    }
    last_removed = field;
    r.log.push_back("removed unknown " + field);
  }

  for (const KeyLine& k : key_lines(text)) {
    const std::string dotted = k.section.empty() ? k.key : k.section + "." + k.key;
    if (never_prune(dotted)) continue;
    std::string cand = text;
    if (!remove_toml_key(&cand, k.section, k.key)) continue;
    Config c;
    if (load_text(cand, scratch, &c, &err) && c == base) {
      text = std::move(cand);
      r.log.push_back("removed default " + dotted + " = " + k.value);
    }
  }

  Config check;
  if (!load_text(text, scratch, &check, &err) || !(check == base)) {
    r.error = err.empty() ? "migrated file does not load equal" : err;
    return r;
  }
  r.ok = true;
  r.text = std::move(text);
  return r;
}

}  // namespace

MigrateResult migrate_config_text(const std::string& text, const std::string& scratch_path) {
  MigrateResult r = migrate_impl(text, scratch_path);
  ::unlink(scratch_path.c_str());
  return r;
}

int migrate_config_cli(const std::string& in, const std::string& out) {
  std::string text;
  if (!read_text_file(in, &text)) {
    std::fprintf(stderr, "migrate: cannot read %s\n", in.c_str());
    return 1;
  }
  const MigrateResult r = migrate_config_text(text, out + ".scratch");
  for (const std::string& l : r.log) std::fprintf(stderr, "migrate: %s\n", l.c_str());
  if (!r.ok) {
    std::fprintf(stderr, "migrate: %s\n", r.error.c_str());
    return 1;
  }
  const std::string tmp = out + ".new";
  if (!write_file_fsync(tmp, r.text) || ::rename(tmp.c_str(), out.c_str()) != 0) {
    ::unlink(tmp.c_str());
    std::fprintf(stderr, "migrate: cannot write %s\n", out.c_str());
    return 1;
  }
  return 0;
}

}  // namespace mabur
