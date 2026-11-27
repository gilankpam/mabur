#include "cal_apply.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "config.h"

namespace mabur {

namespace {

std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

// Parses the 8 numbers out of a "[0.9, 0.88, ...]" array's inner text. A
// malformed original falls back to 0 for the unparseable slot rather than
// throwing; load_config() is the layer that guards correctness, not this
// cosmetic fallback.
std::array<double, 8> parse_double_array(const std::string& inner) {
  std::array<double, 8> vals{};
  size_t pos = 0;
  for (int idx = 0; idx < 8; ++idx) {
    size_t comma = inner.find(',', pos);
    const std::string tok =
        trim(comma == std::string::npos ? inner.substr(pos)
                                         : inner.substr(pos, comma - pos));
    try {
      vals[static_cast<size_t>(idx)] = std::stod(tok);
    } catch (...) {
      vals[static_cast<size_t>(idx)] = 0.0;
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  return vals;
}

// Always a TOML float: "1.0", never "1". A bare integer in an otherwise
// float array is a mixed-type array the loader refuses, so an efficiency of
// exactly 1 (a clamped measurement, the absent-key default, or an original
// "1.0" read back) would otherwise make the whole candidate unloadable.
std::string fmt_double(double v) {
  std::ostringstream ss;
  ss << v;
  std::string out = ss.str();
  if (out.find_first_of(".eEn") == std::string::npos) out += ".0";
  return out;
}

std::string render_double_array(const std::array<double, 8>& v) {
  std::string s = "[";
  for (int i = 0; i < 8; ++i) {
    if (i) s += ", ";
    s += fmt_double(v[static_cast<size_t>(i)]);
  }
  s += "]";
  return s;
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

// Index of the (first) line matching `key`, or -1. Not section-scoped:
// cal_apply only ever deals with the one [air_clock] table mabur.toml
// carries, and every key name it touches is unique across the whole file.
int find_key_line(const std::vector<std::string>& lines, const std::string& key) {
  size_t key_begin, eq_pos;
  for (size_t i = 0; i < lines.size(); ++i)
    if (match_toml_key(lines[i], key, &key_begin, &eq_pos)) return static_cast<int>(i);
  return -1;
}

// Rewrites lines[idx]'s bracketed 8-double array in place, keeping entry i
// where vals[i] == 0 ("leave": inherits the original's value at that slot).
// False (nothing rewritten) if the line isn't the bracketed-array shape this
// patcher understands.
bool rewrite_double_array(std::vector<std::string>& lines, int idx,
                          const std::string& key, const std::array<double, 8>& vals) {
  std::string& line = lines[static_cast<size_t>(idx)];
  size_t key_begin, eq_pos;
  if (!match_toml_key(line, key, &key_begin, &eq_pos)) return false;
  size_t v = eq_pos + 1;
  while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
  if (v >= line.size() || line[v] != '[') return false;
  const size_t close = line.find(']', v);
  if (close == std::string::npos) return false;
  const auto orig = parse_double_array(line.substr(v + 1, close - v - 1));
  std::array<double, 8> merged{};
  for (int i = 0; i < 8; ++i) {
    const size_t s = static_cast<size_t>(i);
    merged[s] = (vals[s] == 0.0) ? orig[s] : vals[s];
  }
  line = rewrite_line(line, key_begin, key.size(), eq_pos, render_double_array(merged), close + 1);
  return true;
}

// publish_config's verifier: the SAME function maburd boots with, not a
// bare TOML syntax check (see mabur/config_write.h for why anything weaker
// defeats the point of publishing through it at all).
bool verify_with_load_config(const std::string& candidate_path, std::string* err) {
  try {
    (void)load_config(candidate_path);
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

// [0.05, 1.0]: this file's own sanity bound on a measured capacity
// fraction (the loader itself only requires (0,1]), checked before the disk
// is touched at all.
bool eff_in_range(const CalWrite& w, std::string* why) {
  const std::string eff_field = "air_clock.efficiency_" + std::to_string(w.bw);
  for (size_t i = 0; i < w.eff.size(); ++i) {
    const double e = w.eff[i];
    if (e == 0.0) continue;  // 0 = leave: not a measurement to range-check
    if (e < 0.05 || e > 1.0) {
      if (why)
        *why = eff_field + "[" + std::to_string(i) + "]: " + fmt_double(e) +
               " is outside [0.05,1.0]";
      return false;
    }
  }
  return true;
}

}  // namespace

std::string patch_toml(const std::string& text, const CalWrite& w, PatchStatus* status) {
  std::vector<std::string> lines;
  {
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) lines.push_back(line);
  }
  const bool trailing_newline = !text.empty() && text.back() == '\n';

  const std::string eff_key = "efficiency_" + std::to_string(w.bw);
  const std::string eff_other_key = "efficiency_" + std::to_string(w.bw == 40 ? 20 : 40);
  const int idx_eff_bw = find_key_line(lines, eff_key);
  bool fallback = false;
  std::string fallback_value;

  if (idx_eff_bw >= 0) {
    if (rewrite_double_array(lines, idx_eff_bw, eff_key, w.eff) && status) status->eff = true;
  } else {
    // A left-at-0 entry has nothing to inherit when the line doesn't exist
    // yet, so it takes what the loader flies for that width with the key
    // absent (AirClockCfg's own default): the inserted line changes nothing
    // the drone flew for that rate.
    const AirClockCfg absent{};
    const std::array<double, 8>& dflt = (w.bw == 40) ? absent.efficiency_40 : absent.efficiency_20;
    std::array<double, 8> vals{};
    for (size_t s = 0; s < 8; ++s) vals[s] = (w.eff[s] == 0.0) ? dflt[s] : w.eff[s];
    // Next to the other width's line, so the file keeps reading as one
    // per-width pair; after shed_ms (the table's own first key) when there
    // is none; through set_toml_key when there is no [air_clock] line at all.
    const int idx_other = find_key_line(lines, eff_other_key);
    const int anchor = idx_other >= 0 ? idx_other : find_key_line(lines, "shed_ms");
    if (anchor >= 0) {
      lines.insert(lines.begin() + anchor + 1, eff_key + " = " + render_double_array(vals));
    } else {
      fallback = true;
      fallback_value = render_double_array(vals);
    }
    if (status) status->eff = true;
  }

  std::string out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out += lines[i];
    if (i + 1 < lines.size()) out += "\n";
  }
  if (trailing_newline) out += "\n";
  if (fallback) set_toml_key(&out, "air_clock", eff_key, fallback_value);
  return out;
}

namespace {

// Shared body of apply_calibration() and its corrupt-writer test seam.
// `content_override`, when set, replaces the real patch_toml() output --
// the seam tests need a way to get deliberate garbage onto disk without a
// real writer bug, to prove verification actually catches it. The
// publisher's verify-before-publish order is what makes that refusal
// leave `path` untouched; see config_write.h.
ApplyResult apply_calibration_impl(const std::string& path, const CalWrite& w, bool backup,
                                   std::string* err, const std::string* content_override) {
  // Step 1: validate. Nothing on disk is touched if this fails.
  std::string why;
  if (!eff_in_range(w, &why)) {
    if (err) *err = why;
    return ApplyResult::OutOfRange;
  }

  // Held from the read through the publish, so a parameter write on
  // another thread can't land between them and be overwritten by a
  // candidate patched from the text read here.
  const auto lock = lock_config_writes();
  std::string original;
  if (!read_text_file(path, &original)) {
    if (err) *err = "failed to read " + path;
    return ApplyResult::WriteFailed;
  }

  // Step 2: patch in memory and check the key was actually rewritten or
  // inserted -- see the file header. Skipped for content_override: that
  // seam bypasses patch_toml on purpose to inject raw garbage.
  std::string content;
  if (content_override) {
    content = *content_override;
  } else {
    PatchStatus status;
    content = patch_toml(original, w, &status);
    if (!status.eff) {
      if (err)
        *err = "air_clock.efficiency_" + std::to_string(w.bw) +
               ": line not found in config -- unexpected file structure, nothing written";
      return ApplyResult::KeyNotFound;
    }
  }

  // Steps 3-6: write "<path>.new", verify it with load_config, conditionally
  // back up, rename -- the shared publisher, see config_write.h. A call that
  // doesn't own the run's backup never touches "<path>.pre-cal" at all.
  std::string pub_err;
  const PublishResult pr = publish_config(path, content, "pre-cal", &pub_err,
                                          verify_with_load_config, /*write_backup=*/backup);
  switch (pr) {
    case PublishResult::Ok:
      return ApplyResult::Ok;
    case PublishResult::BackupFailed:
      if (err) *err = pub_err;
      return ApplyResult::BackupFailed;
    case PublishResult::WriteFailed:
      if (err) *err = pub_err;
      return ApplyResult::WriteFailed;
    case PublishResult::ReparseFailed:
      if (err) *err = "candidate failed load_config, not published: " + pub_err;
      return ApplyResult::ReparseFailed;
  }
  if (err) *err = "unknown publish result";
  return ApplyResult::WriteFailed;
}

}  // namespace

ApplyResult apply_calibration(const std::string& path, const CalWrite& w, bool backup,
                              std::string* err) {
  return apply_calibration_impl(path, w, backup, err, nullptr);
}

ApplyResult apply_calibration_for_test_with_corrupt_writer(const std::string& path,
                                                           const CalWrite& w, std::string* err) {
  static const std::string kGarbage = "this is not valid toml [[[ ===\n";
  return apply_calibration_impl(path, w, /*backup=*/true, err, &kGarbage);
}

}  // namespace mabur
