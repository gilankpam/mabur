#include "wall_cache.h"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace mabur {
namespace {

constexpr const char* kHeader = "maburwalls 1";
// Shared by load and save: save never writes what load would reject.
constexpr int kChMin = 1, kChMax = 177, kAnchorMax = 127, kEvmMin = -600, kEvmMax = 0;
constexpr long kMaxBytes = 64 * 1024;  // 354 entries x ~140 B is ~50 KB
bool width_ok(long w) { return w == 20 || w == 40; }

// Whole token must be an optionally negative decimal integer ("5x", "-3.5", "+1", "" are out).
bool parse_int(const std::string& s, long* out) {
  size_t i = (!s.empty() && s[0] == '-') ? 1 : 0;
  if (i == s.size() || s.size() - i > 6) return false;
  for (size_t j = i; j < s.size(); ++j)
    if (s[j] < '0' || s[j] > '9') return false;
  *out = std::strtol(s.c_str(), nullptr, 10);
  return true;
}

// Empty string on success, else the reason.
std::string parse(const std::string& text, WallCache* out) {
  // A valid file always ends in '\n'; one that doesn't was cut mid-write or hand-edited.
  if (text.empty() || text.back() != '\n') return "truncated (no trailing newline)";
  std::vector<std::string> lines;
  std::stringstream ss(text);
  for (std::string l; std::getline(ss, l);) lines.push_back(l);
  if (lines[0] != kHeader) return "unknown header";
  for (size_t n = 1; n < lines.size(); ++n) {
    const std::string at = "line " + std::to_string(n + 1) + ": ";
    std::vector<std::string> tok;
    std::string cur;
    for (char ch : lines[n] + " ") {
      if (ch == ' ') { tok.push_back(cur); cur.clear(); } else cur += ch;
    }
    if (tok.size() != 3 + kWallPoints) return at + "wrong token count";
    long ch, w, anchor;
    if (!parse_int(tok[0], &ch) || ch < kChMin || ch > kChMax) return at + "bad channel";
    if (!parse_int(tok[1], &w) || !width_ok(w)) return at + "bad width";
    if (!parse_int(tok[2], &anchor) || anchor < 0 || anchor > kAnchorMax) return at + "bad anchor";
    WallCurve c;
    c.anchor = static_cast<int>(anchor);
    for (int k = 0; k < kWallPoints; ++k) {
      const std::string& t = tok[3 + k];
      if (t == "x") continue;
      long v;
      if (!parse_int(t, &v) || v < kEvmMin || v > kEvmMax) return at + "bad value";
      c.evm_db[k] = v / 10.0f;
    }
    if (!out->entries.emplace(std::make_pair(uint8_t(ch), uint8_t(w)), c).second)
      return at + "duplicate channel and width";
  }
  return "";
}

}  // namespace

WallCache load_wall_cache(const std::string& path, std::string* note) {
  WallCache c;
  std::string why;
  try {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
      why = "cannot open " + path;
    } else if (f.tellg() > kMaxBytes) {
      why = "file too large";
    } else {
      f.seekg(0);
      std::stringstream buf;
      buf << f.rdbuf();
      why = parse(buf.str(), &c);
    }
  } catch (...) {
    why = "read failed";
  }
  if (!why.empty()) c.entries.clear();
  if (note) *note = why;
  return c;
}

bool save_wall_cache(const std::string& path, const WallCache& c) {
  for (const auto& [key, cv] : c.entries) {
    bool ok = key.first >= kChMin && key.first <= kChMax && width_ok(key.second) &&
              cv.anchor >= 0 && cv.anchor <= kAnchorMax;
    for (const auto& v : cv.evm_db)  // NaN fails both comparisons' negation: !(x>=lo) catches it
      if (v && !(std::lround(*v * 10.0f) >= kEvmMin && std::lround(*v * 10.0f) <= kEvmMax && *v == *v)) ok = false;
    if (!ok) return false;
  }
  const std::string tmp = path + ".tmp";
  std::FILE* f = std::fopen(tmp.c_str(), "w");
  if (!f) return false;
  bool ok = std::fprintf(f, "%s\n", kHeader) > 0;
  for (const auto& [key, cv] : c.entries) {  // std::map order = channel, then width
    ok = ok && std::fprintf(f, "%u %u %d", unsigned(key.first), unsigned(key.second), cv.anchor) > 0;
    for (const auto& v : cv.evm_db) {
      if (v) ok = ok && std::fprintf(f, " %ld", std::lround(*v * 10.0f)) > 0;
      else ok = ok && std::fputs(" x", f) >= 0;
    }
    ok = ok && std::fputc('\n', f) != EOF;
  }
  // fsync before the rename, as write_channel_file does: else a power cut can leave an empty file.
  ok = ok && std::fflush(f) == 0 && ::fsync(fileno(f)) == 0;
  std::fclose(f);
  if (!ok || std::rename(tmp.c_str(), path.c_str()) != 0) { std::remove(tmp.c_str()); return false; }
  return true;
}

}  // namespace mabur
