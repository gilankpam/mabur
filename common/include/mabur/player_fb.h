#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

// maburplay -> maburgs feedback datagram (spec
// docs/superpowers/specs/2026-08-12-decoder-idr-backchannel-design.md).
// One ASCII line of key=value tokens, UDP, loopback only. `idr` is a LEVEL
// and the only field that drives behaviour; the rest is observability.
// UNKNOWN KEYS ARE IGNORED, matching the sideport's additive-only rule, so a
// newer player can add fields without breaking an older maburgs.
namespace mabur {
namespace playerfb {

constexpr int kDefaultPort = 8303;
constexpr char kMagic[] = "mabur-fb";
constexpr int kVersion = 1;

// 0 none, 1 flush, 2 join, 3 watchdog -- matches PlayerIdrLatch::Reason.
inline const char* reason_name(uint8_t r) {
  switch (r) {
    case 1: return "flush";
    case 2: return "join";
    case 3: return "watchdog";
    default: return "none";
  }
}

struct Msg {
  bool idr = false;
  uint8_t reason = 0;
  uint64_t flushes = 0, joins = 0, watchdogs = 0, episodes = 0;
};

// Returns bytes written, or 0 if `cap` is too small -- a truncated datagram
// would not parse, so emitting nothing is the honest failure.
inline size_t format(const Msg& m, char* buf, size_t cap) {
  const int n = std::snprintf(
      buf, cap,
      "%s v=%d idr=%d reason=%s flushes=%llu joins=%llu watchdogs=%llu "
      "episodes=%llu",
      kMagic, kVersion, m.idr ? 1 : 0, reason_name(m.reason),
      static_cast<unsigned long long>(m.flushes),
      static_cast<unsigned long long>(m.joins),
      static_cast<unsigned long long>(m.watchdogs),
      static_cast<unsigned long long>(m.episodes));
  if (n <= 0 || static_cast<size_t>(n) >= cap) return 0;
  return static_cast<size_t>(n);
}

namespace detail {
// Parses an unsigned decimal spanning [p, end). Returns false on empty or
// any non-digit -- callers must not accept a partially-parsed number.
inline bool to_u64(const char* p, const char* end, uint64_t* out) {
  if (p >= end) return false;
  uint64_t v = 0;
  for (; p < end; ++p) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10 + static_cast<uint64_t>(*p - '0');
  }
  *out = v;
  return true;
}
}  // namespace detail

// True only for a well-formed v=1 datagram carrying `idr`. Never reads past
// p[n): the socket hands us a length, not a NUL-terminated string.
inline bool parse(const char* p, size_t n, Msg* out) {
  const char* const end = p + n;
  const size_t maglen = sizeof(kMagic) - 1;
  if (n < maglen || std::memcmp(p, kMagic, maglen) != 0) return false;
  const char* cur = p + maglen;

  Msg m;
  bool have_ver = false, have_idr = false;
  while (cur < end) {
    while (cur < end && *cur == ' ') ++cur;
    const char* tok = cur;
    while (cur < end && *cur != ' ') ++cur;
    if (tok == cur) break;
    const char* eq = tok;
    while (eq < cur && *eq != '=') ++eq;
    if (eq == cur) continue;  // no '=' -- not a key=value token, ignore
    const size_t klen = static_cast<size_t>(eq - tok);
    const char* val = eq + 1;
    uint64_t num = 0;
    const bool numeric = detail::to_u64(val, cur, &num);

    auto key_is = [&](const char* k) {
      return klen == std::strlen(k) && std::memcmp(tok, k, klen) == 0;
    };
    if (key_is("v")) {
      if (!numeric || num != kVersion) return false;
      have_ver = true;
    } else if (key_is("idr")) {
      if (!numeric || num > 1) return false;
      m.idr = num != 0;
      have_idr = true;
    } else if (key_is("reason")) {
      for (uint8_t r = 0; r <= 3; ++r) {
        const char* rn = reason_name(r);
        if (static_cast<size_t>(cur - val) == std::strlen(rn) &&
            std::memcmp(val, rn, static_cast<size_t>(cur - val)) == 0) {
          m.reason = r;
          break;
        }
      }
    } else if (key_is("flushes")) {
      if (numeric) m.flushes = num;
    } else if (key_is("joins")) {
      if (numeric) m.joins = num;
    } else if (key_is("watchdogs")) {
      if (numeric) m.watchdogs = num;
    } else if (key_is("episodes")) {
      if (numeric) m.episodes = num;
    }
    // else: unknown key -- ignored on purpose (forward compatibility)
  }
  if (!have_ver || !have_idr) return false;
  *out = m;
  return true;
}

}  // namespace playerfb
}  // namespace mabur
