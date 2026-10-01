#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include "mtest.h"
#include "mabur/link_key.h"
using namespace mabur;

static std::string what_of(const std::function<void()>& f) {
  try { f(); } catch (const std::exception& e) { return e.what(); }
  return "";
}
static std::string scratch(const char* name) {
  return std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name;
}
static void write(const std::string& path, const std::string& text) {
  std::ofstream o(path, std::ios::binary);
  o << text;
}

static const std::string kHex = "3f9a1c77e04b5d2290ab6ef1c8d34e5a";

TEST(parse_key_hex_accepts_32_hex_any_case) {
  auto a = parse_key_hex(kHex);
  auto b = parse_key_hex("3F9A1C77E04B5D2290AB6EF1C8D34E5A");
  REQUIRE(a.has_value());
  REQUIRE(b.has_value());
  CHECK(*a == *b);
  CHECK((*a)[0] == 0x3f && (*a)[15] == 0x5a);
  CHECK(key_to_hex(*a) == kHex);
  CHECK(!parse_key_hex(kHex.substr(0, 31)).has_value());
  CHECK(!parse_key_hex(kHex + "0").has_value());
  CHECK(!parse_key_hex("3g9a1c77e04b5d2290ab6ef1c8d34e5a").has_value());
}

TEST(parse_key_text_ignores_comments_blank_lines_and_crlf) {
  CHECK(parse_key_text("# mabur link key\n\n  " + kHex + "  \r\n") == *parse_key_hex(kHex));
  CHECK(parse_key_text(kHex) == *parse_key_hex(kHex));
  CHECK(what_of([] { parse_key_text(""); }).find("no key") != std::string::npos);
  CHECK(what_of([] { parse_key_text(kHex + "\n" + kHex + "\n"); }).find("more than one") != std::string::npos);
  CHECK(what_of([] { parse_key_text(kHex.substr(0, 31)); }).find("32 hex") != std::string::npos);
}

TEST(load_key_file_missing_is_default_bad_throws) {
  const auto missing = load_key_file(scratch("does-not-exist.key"));
  CHECK(missing.is_default);
  CHECK(missing.key == kDefaultLinkKey);
  CHECK(missing.source == "default");

  const auto p = scratch("ok.key");
  write(p, "# generated 2026-10-01\n" + kHex + "\n");
  const auto ok = load_key_file(p);
  CHECK(!ok.is_default);
  CHECK(key_to_hex(ok.key) == kHex);
  CHECK(ok.source == p);

  const auto bad = scratch("bad.key");
  write(bad, "not a key\n");
  const std::string msg = what_of([&] { load_key_file(bad); });
  CHECK(msg.find(bad) != std::string::npos);
  CHECK(msg.find("32 hex") != std::string::npos);
}

TEST(fingerprint_is_default_or_four_hex) {
  CHECK(key_fingerprint(kDefaultLinkKey) == "default");
  const auto fp = key_fingerprint(*parse_key_hex(kHex));
  CHECK(fp.size() == 4);
  CHECK(fp != key_fingerprint(*parse_key_hex("3f9a1c77e04b5d2290ab6ef1c8d34e5b")));
  // Print-once golden (same convention as tests/test_rc.cpp): pin the
  // value after the first run so the fingerprint can never drift silently.
  CHECK(fp == "55db");
}
MTEST_MAIN
