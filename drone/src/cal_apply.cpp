#include "cal_apply.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
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

// Parses the 8 numbers out of a "[91, 91, ...]" array's inner text. A
// malformed original (should never happen -- it's bundle/mabur.default.toml
// verbatim) falls back to 0 for the unparseable slot rather than throwing;
// walls_in_range()/load_config() are the layers that actually guard
// correctness, not this cosmetic fallback.
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

std::string fmt_double(double v) {
  std::ostringstream ss;
  ss << v;
  return ss.str();
}

std::string render_int_array(const std::array<int, 8>& v) {
  std::string s = "[";
  for (int i = 0; i < 8; ++i) {
    if (i) s += ", ";
    s += std::to_string(v[static_cast<size_t>(i)]);
  }
  s += "]";
  return s;
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
// cal_apply only ever deals with the one [radio] and one [air_clock] table
// mabur.toml carries, and every key name it touches is unique across the
// whole file.
int find_key_line(const std::vector<std::string>& lines, const std::string& key) {
  size_t key_begin, eq_pos;
  for (size_t i = 0; i < lines.size(); ++i)
    if (match_toml_key(lines[i], key, &key_begin, &eq_pos)) return static_cast<int>(i);
  return -1;
}

bool has_key_line(const std::string& text, const std::string& key) {
  size_t key_begin, eq_pos;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line))
    if (match_toml_key(line, key, &key_begin, &eq_pos)) return true;
  return false;
}

// Rewrites lines[idx]'s bracketed 8-int array in place, keeping entry i
// unless vals[i] == kWallUndetermined (kept = inherits the original's
// value at that slot). False (nothing rewritten) if the line isn't the
// bracketed-array shape this patcher understands.
bool rewrite_int_array(std::vector<std::string>& lines, int idx,
                       const std::string& key, const std::array<int, 8>& vals) {
  std::string& line = lines[static_cast<size_t>(idx)];
  size_t key_begin, eq_pos;
  if (!match_toml_key(line, key, &key_begin, &eq_pos)) return false;
  size_t v = eq_pos + 1;
  while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
  if (v >= line.size() || line[v] != '[') return false;
  const size_t close = line.find(']', v);
  if (close == std::string::npos) return false;
  const auto orig = parse_int_array(line.substr(v + 1, close - v - 1));
  std::array<int, 8> merged{};
  for (int i = 0; i < 8; ++i) {
    const size_t s = static_cast<size_t>(i);
    merged[s] = (vals[s] == mabur::rc::kWallUndetermined) ? orig[s] : vals[s];
  }
  line = rewrite_line(line, key_begin, key.size(), eq_pos, render_int_array(merged), close + 1);
  return true;
}

// Same as rewrite_int_array, for an 8-double array where 0 means "leave".
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

bool rewrite_quoted_string(std::vector<std::string>& lines, int idx,
                           const std::string& key, const std::string& new_value) {
  std::string& line = lines[static_cast<size_t>(idx)];
  size_t key_begin, eq_pos;
  if (!match_toml_key(line, key, &key_begin, &eq_pos)) return false;
  size_t v = eq_pos + 1;
  while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
  if (v >= line.size() || line[v] != '"') return false;
  const size_t close = line.find('"', v + 1);
  if (close == std::string::npos) return false;
  line = rewrite_line(line, key_begin, key.size(), eq_pos, "\"" + new_value + "\"", close + 1);
  return true;
}

bool rewrite_int(std::vector<std::string>& lines, int idx, const std::string& key,
                 int new_value) {
  std::string& line = lines[static_cast<size_t>(idx)];
  size_t key_begin, eq_pos;
  if (!match_toml_key(line, key, &key_begin, &eq_pos)) return false;
  size_t v = eq_pos + 1;
  while (v < line.size() && (line[v] == ' ' || line[v] == '\t')) ++v;
  size_t e = v;
  if (e < line.size() && line[e] == '-') ++e;
  while (e < line.size() && std::isdigit(static_cast<unsigned char>(line[e]))) ++e;
  line = rewrite_line(line, key_begin, key.size(), eq_pos, std::to_string(new_value), e);
  return true;
}

// Inserts `new_line` right after lines[after_idx], shifting every later
// line down by one. Callers that queue several insertions in one
// patch_toml() call sort them by anchor and apply low-to-high, adding the
// running shift to each anchor themselves -- otherwise an insertion whose
// anchor sits after an earlier one would land one line short.
void insert_toml_key_after(std::vector<std::string>& lines, int after_idx,
                           const std::string& new_line) {
  lines.insert(lines.begin() + after_idx + 1, new_line);
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

  const std::string bw_s = std::to_string(w.bw);
  const std::string other_s = std::to_string(w.bw == 40 ? 20 : 40);
  const std::string rate_key = "rate_walls_rel_" + bw_s;
  const std::string rate_other_key = "rate_walls_rel_" + other_s;
  const std::string cflag_key = "cal_flags_" + bw_s;
  const std::string cflag_other_key = "cal_flags_" + other_s;
  const std::string eff_key = "efficiency_" + bw_s;
  const std::string eff_other_key = "efficiency_" + other_s;

  const int idx_rate_bw = find_key_line(lines, rate_key);
  const int idx_rate_other = find_key_line(lines, rate_other_key);
  const int idx_cflag_bw = find_key_line(lines, cflag_key);
  const int idx_cflag_other = find_key_line(lines, cflag_other_key);
  const int idx_legacy = find_key_line(lines, "legacy_wall_rel");
  const int idx_power_mode = find_key_line(lines, "power_mode");
  const int idx_eff_bw = find_key_line(lines, eff_key);
  const int idx_eff_other = find_key_line(lines, eff_other_key);
  const int idx_shed_ms = find_key_line(lines, "shed_ms");
  // Which line owns rate_walls_rel_20 specifically, independent of which
  // width this call is writing -- legacy_wall_rel's patch condition reads
  // this, not idx_rate_bw/idx_rate_other.
  const int idx_rate_20 = (w.bw == 20) ? idx_rate_bw : idx_rate_other;

  struct Insertion {
    int anchor;
    std::string text;
  };
  std::vector<Insertion> pending;

  if (w.has_walls) {
    if (idx_rate_bw >= 0) {
      if (rewrite_int_array(lines, idx_rate_bw, rate_key, w.walls))
        if (status) status->walls = true;
    } else {
      // First calibration of this width: there is no original line to
      // inherit an undetermined slot from, so it rails (kRailRel) -- the
      // documented meaning of a no-dip row's wall -- rather than
      // inheriting nothing.
      std::array<int, 8> vals{};
      for (int i = 0; i < 8; ++i) {
        const size_t s = static_cast<size_t>(i);
        vals[s] = (w.walls[s] == mabur::rc::kWallUndetermined) ? rc::kRailRel : w.walls[s];
      }
      const int anchor = (idx_rate_other >= 0) ? idx_rate_other : idx_power_mode;
      if (anchor >= 0) {
        pending.push_back({anchor, rate_key + " = " + render_int_array(vals)});
        if (status) {
          status->walls = true;
          status->inserted_walls = true;
        }
      }
    }

    // cal_flags_<bw> records what the walls above were measured with, not
    // a per-rate value with its own sentinel: it is always overwritten
    // whenever walls are written.
    if (idx_cflag_bw >= 0) {
      if (rewrite_quoted_string(lines, idx_cflag_bw, cflag_key, w.cal_flags))
        if (status) status->cal_flags = true;
    } else {
      const int anchor = (idx_cflag_other >= 0) ? idx_cflag_other : idx_power_mode;
      if (anchor >= 0) {
        pending.push_back({anchor, cflag_key + " = \"" + w.cal_flags + "\""});
        if (status) {
          status->cal_flags = true;
          status->inserted_cal_flags = true;
        }
      }
    }

    // legacy_wall_rel is shared by both widths (legacy/CCK rides the one
    // legacy wall regardless of width): only a 20 MHz write repatches it,
    // unless the file has no rate_walls_rel_20 line at all -- a unit
    // calibrated on only 40 MHz still needs legacy derived from somewhere.
    // No insertion path: this key always already exists in a loadable
    // config (RadioCfg::legacy_wall_rel's struct default is never absent
    // from the bundle), so a missing line here is a KeyNotFound, not an
    // insert.
    const bool patch_legacy = (w.bw == 20) || (idx_rate_20 < 0);
    if (patch_legacy && w.legacy_wall != mabur::rc::kWallUndetermined && idx_legacy >= 0) {
      if (rewrite_int(lines, idx_legacy, "legacy_wall_rel", w.legacy_wall))
        if (status) status->legacy_wall_rel = true;
    }

    // Any walls write means the unit now has at least one measured table,
    // so bring-up should stop running the card's as-shipped power.
    if (idx_power_mode >= 0) {
      if (rewrite_quoted_string(lines, idx_power_mode, "power_mode", "offset"))
        if (status) status->power_mode = true;
    }
  }

  if (w.has_eff) {
    if (idx_eff_bw >= 0) {
      if (rewrite_double_array(lines, idx_eff_bw, eff_key, w.eff))
        if (status) status->eff = true;
    } else {
      // Same anchoring rule as the walls/cal_flags insertions above: sit
      // next to the other width's line of the same key so the file keeps
      // reading as one per-width pair, falling back to shed_ms (the
      // table's own first key) only when there is no other-width line
      // either.
      const int anchor = (idx_eff_other >= 0) ? idx_eff_other : idx_shed_ms;
      if (anchor >= 0) {
        // Same reasoning as the walls insertion above: a left-at-0
        // ("leave") entry has nothing to inherit when the line doesn't
        // exist yet, so it takes AirClockCfg::efficiency_20/_40's own
        // struct default (1.0, nominal) instead.
        std::array<double, 8> vals{};
        for (int i = 0; i < 8; ++i) {
          const size_t s = static_cast<size_t>(i);
          vals[s] = (w.eff[s] == 0.0) ? 1.0 : w.eff[s];
        }
        pending.push_back({anchor, eff_key + " = " + render_double_array(vals)});
        if (status) status->eff = true;
      }
    }
  }

  // Apply insertions anchor-ascending, so each insertion's own shift only
  // ever pushes an anchor that comes AFTER it in the file -- never one
  // already used. stable_sort keeps two insertions sharing an anchor (the
  // degenerate case where both this width's wall and flag lines fall back
  // to the same power_mode anchor) in the order they were queued above:
  // walls, then flags.
  std::stable_sort(pending.begin(), pending.end(),
                   [](const Insertion& a, const Insertion& b) { return a.anchor < b.anchor; });
  int shift = 0;
  for (const auto& ins : pending) {
    insert_toml_key_after(lines, ins.anchor + shift, ins.text);
    ++shift;
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
  // Mirrors drone/src/config.cpp exactly for walls, including its message
  // wording, so a refusal here reads the same way a boot-time refusal
  // would. This pre-check and the load_config() verification step later in
  // apply_calibration() serve different purposes and both stay: this one
  // refuses a bad table before the disk is touched at all, with a
  // specific OutOfRange result and a message naming the exact derivation
  // that failed; load_config() is the backstop that catches anything this
  // model does not.
  const int m = static_cast<int>(std::lround(margin_db * 4.0));
  auto check_wall = [&](int v, const std::string& key) {
    if (v == mabur::rc::kWallUndetermined) return true;
    if (v < rc::kRelMin || v > rc::kRelMax) {
      if (why) *why = key + ": " + std::to_string(v) +
                      " is outside the 7-bit hardware field range [-64,63]";
      return false;
    }
    if (v - m < rc::kRelMin) {
      if (why) *why = key + ": " + std::to_string(v) +
                      " - wall_margin_db*4 = " + std::to_string(v - m) +
                      " is below the hardware field floor -64";
      return false;
    }
    return true;
  };
  const std::string rate_field = "radio.rate_walls_rel_" + std::to_string(w.bw);
  for (size_t i = 0; i < w.walls.size(); ++i)
    if (!check_wall(w.walls[i], rate_field + "[" + std::to_string(i) + "]"))
      return false;
  if (!check_wall(w.legacy_wall, "radio.legacy_wall_rel")) return false;

  // eff has no hardware field to mirror: [0.05, 1.0] is this file's own
  // sanity bound on a measured capacity fraction (the loader itself only
  // enforces (0,1]).
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

namespace {

// Checks that patch_toml() actually found-and-rewrote or inserted every
// key this write is required to have touched. `legacy_required` is passed
// in rather than recomputed from `w` alone because it also depends on
// whether the file had its own rate_walls_rel_20 line -- information
// patch_toml had but PatchStatus doesn't carry back out (apply_calibration
// re-derives it with has_key_line() against the same original text).
bool patch_covered_required_keys(const PatchStatus& s, const CalWrite& w,
                                 bool legacy_required, std::string* why) {
  if (w.has_walls) {
    if (!s.walls) {
      if (why)
        *why = "radio.rate_walls_rel_" + std::to_string(w.bw) +
               ": line not found in config -- unexpected file structure, nothing written";
      return false;
    }
    if (!s.cal_flags) {
      if (why)
        *why = "radio.cal_flags_" + std::to_string(w.bw) +
               ": line not found in config -- unexpected file structure, nothing written";
      return false;
    }
    if (!s.power_mode) {
      if (why)
        *why = "radio.power_mode: line not found in config -- unexpected file structure, nothing written";
      return false;
    }
    if (legacy_required && !s.legacy_wall_rel) {
      if (why)
        *why = "radio.legacy_wall_rel: line not found in config -- unexpected file structure, nothing written";
      return false;
    }
  }
  if (w.has_eff && !s.eff) {
    if (why)
      *why = "air_clock.efficiency_" + std::to_string(w.bw) +
             ": line not found in config -- unexpected file structure, nothing written";
    return false;
  }
  return true;
}

// Shared body of apply_calibration() and its corrupt-writer test seam.
// `content_override`, when set, replaces the real patch_toml() output --
// the seam tests need a way to get deliberate garbage onto disk without a
// real writer bug, to prove verification actually catches it. The
// publisher's verify-before-publish order is what makes that refusal
// leave `path` untouched; see config_write.h.
ApplyResult apply_calibration_impl(const std::string& path, const CalWrite& w,
                                   double margin_db, bool backup, std::string* err,
                                   const std::string* content_override) {
  // Step 1: validate the derived diffs/efficiencies. Nothing on disk is
  // touched if this fails -- see walls_in_range()'s comment for why this
  // pre-check stays even though load_config() below re-derives the same
  // numbers.
  std::string why;
  if (!walls_in_range(w, margin_db, &why)) {
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

  // Step 2: patch in memory and check every REQUIRED key was actually
  // found/rewritten or inserted. patch_toml has no error channel of its
  // own -- a key it can't match or insert is silently left as the
  // original's value (or absent), which load_config() below would still
  // accept. Without this check, apply_calibration could report Ok on a
  // calibration that never actually applied -- the exact
  // silent-miscalibration failure this file exists to prevent, arriving
  // through the back door, so it is checked explicitly and refused
  // loudly. Skipped for content_override: that seam bypasses patch_toml on
  // purpose to inject raw garbage.
  std::string content;
  if (content_override) {
    content = *content_override;
  } else {
    PatchStatus status;
    content = patch_toml(original, w, &status);
    const bool legacy_required =
        w.has_walls && w.legacy_wall != mabur::rc::kWallUndetermined &&
        (w.bw == 20 || !has_key_line(original, "rate_walls_rel_20"));
    std::string key_why;
    if (!patch_covered_required_keys(status, w, legacy_required, &key_why)) {
      if (err) *err = key_why;
      return ApplyResult::KeyNotFound;
    }
  }

  // Steps 3-6: write "<path>.new", verify it with load_config, conditionally
  // back up, rename -- the shared publisher, see config_write.h. Passing
  // `backup` straight through as publish_config's write_backup means a
  // call that doesn't own the session's backup never touches
  // "<path>.pre-cal" at all -- it is not written and then reverted, so an
  // earlier call's real backup in the same session is never at risk.
  std::string pub_err;
  const PublishResult pr =
      publish_config(path, content, "pre-cal", &pub_err, /*write_backup=*/backup);
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

ApplyResult apply_calibration(const std::string& path, const CalWrite& w,
                              double margin_db, bool backup, std::string* err) {
  return apply_calibration_impl(path, w, margin_db, backup, err, nullptr);
}

ApplyResult apply_calibration_for_test_with_corrupt_writer(
    const std::string& path, const CalWrite& w, double margin_db, std::string* err) {
  static const std::string kGarbage = "this is not valid toml [[[ ===\n";
  return apply_calibration_impl(path, w, margin_db, /*backup=*/true, err, &kGarbage);
}

}  // namespace mabur
