#include "mabur/link_key.h"
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include "mabur/siphash.h"

namespace mabur {
namespace {
int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}
}  // namespace

std::optional<LinkKey> parse_key_hex(const std::string& hex) {
  if (hex.size() != 32) return std::nullopt;
  LinkKey k{};
  for (size_t i = 0; i < 16; ++i) {
    const int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return std::nullopt;
    k[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return k;
}

LinkKey parse_key_text(const std::string& text) {
  std::istringstream in(text);
  std::string line, token;
  int tokens = 0;
  while (std::getline(in, line)) {
    const std::string t = trim(line);
    if (t.empty() || t[0] == '#') continue;
    ++tokens;
    token = t;
  }
  if (tokens == 0) throw std::runtime_error("no key");
  if (tokens > 1) throw std::runtime_error("more than one key");
  auto k = parse_key_hex(token);
  if (!k) throw std::runtime_error("not 32 hex characters");
  return *k;
}

KeyLoad load_key_file(const std::string& path) {
  // Only ENOENT means "no key file -> default key". Anything else (EACCES,
  // a directory, ENOTDIR, a read error) is a boot failure naming the path.
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) {
    const int err = errno;
    if (err == ENOENT) return KeyLoad{kDefaultLinkKey, true, "default"};
    throw std::runtime_error(path + ": " + std::strerror(err));
  }
  std::string text;
  char buf[256];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
  const int read_err = std::ferror(f) ? errno : 0;
  std::fclose(f);
  if (read_err) throw std::runtime_error(path + ": " + std::strerror(read_err));
  try {
    return KeyLoad{parse_key_text(text), false, path};
  } catch (const std::runtime_error& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

std::string key_to_hex(const LinkKey& k) {
  static const char* d = "0123456789abcdef";
  std::string s;
  s.reserve(32);
  for (uint8_t b : k) { s.push_back(d[b >> 4]); s.push_back(d[b & 15]); }
  return s;
}

std::string key_fingerprint(const LinkKey& k) {
  if (k == kDefaultLinkKey) return "default";
  static const uint8_t tag[] = {'m', 'a', 'b', 'u', 'r', '.', 'k', 'e', 'y'};
  const uint64_t h = siphash24(k, tag, sizeof tag);
  char b[5];
  std::snprintf(b, sizeof b, "%02x%02x", static_cast<unsigned>(h & 0xff),
                static_cast<unsigned>((h >> 8) & 0xff));
  return b;
}
}  // namespace mabur
