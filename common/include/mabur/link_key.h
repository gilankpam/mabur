#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
namespace mabur {
// The pairing key (spec 2026-10-01 link-pairing §2): 16 bytes shared by the
// drone, the GS and the web page. Lives in a key FILE, never in the TOML.
using LinkKey = std::array<uint8_t, 16>;
// Compiled-in default used when the key file is absent so a fresh install
// links out of the box. Offers no security: every default install pairs
// with every other.
constexpr LinkKey kDefaultLinkKey = {'m', 'a', 'b', 'u', 'r', '-', 'd', 'e',
                                     'f', 'a', 'u', 'l', 't', '-', '0', '0'};
struct KeyLoad {
  LinkKey key = kDefaultLinkKey;
  bool is_default = true;
  std::string source;  // the path, or "default"
};
// 32 hex chars, any case; nullopt otherwise.
std::optional<LinkKey> parse_key_hex(const std::string& hex);
// Key-file text: blank lines, surrounding whitespace (incl. CR) and lines
// starting with '#' ignored; exactly one 32-hex token. Throws
// std::runtime_error("no key" | "more than one key" | "not 32 hex characters").
LinkKey parse_key_text(const std::string& text);
// Missing file -> default key (is_default). Present but malformed -> throws
// std::runtime_error("<path>: <why>"). Never a silent fallback for a bad file.
KeyLoad load_key_file(const std::string& path);
std::string key_to_hex(const LinkKey& k);
// 4 lowercase hex chars from SipHash-2-4(key, "mabur.key"), or "default".
std::string key_fingerprint(const LinkKey& k);
}  // namespace mabur
