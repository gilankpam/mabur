#include "mtest.h"
#include "mabur/sha256.h"
#include "scratch.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace mabur;

namespace {
std::string hash_of(const std::string& s) {
  Sha256Ctx c;
  c.update(s.data(), s.size());
  return sha256_hex(c.final());
}
}  // namespace

TEST(empty_string) {
  CHECK(hash_of("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(abc) {
  CHECK(hash_of("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(two_block_message) {
  CHECK(hash_of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST(update_in_pieces_equals_one_update) {
  std::string s(1000, 0);
  for (size_t i = 0; i < s.size(); i++) s[i] = static_cast<char>(i * 7 + 3);
  Sha256Ctx c;
  size_t off = 0;
  for (size_t n : {1, 63, 64, 872}) {
    c.update(s.data() + off, n);
    off += n;
  }
  CHECK(sha256_hex(c.final()) == hash_of(s));
}

TEST(file_matches_buffer) {
  std::string s(100000, 0);
  for (size_t i = 0; i < s.size(); i++) s[i] = static_cast<char>(i * 13 + 1);
  ScratchFile f("test_sha256", ".bin");
  std::FILE* fp = std::fopen(f.c_str(), "wb");
  CHECK(fp != nullptr);
  std::fwrite(s.data(), 1, s.size(), fp);
  std::fclose(fp);
  auto h = sha256_file(f.path);
  CHECK(h.has_value());
  CHECK(sha256_hex(*h) == hash_of(s));
  CHECK(!sha256_file(f.path + ".missing").has_value());
}

TEST(hex_round_trip) {
  const std::string hex = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
  auto h = sha256_from_hex(hex);
  CHECK(h.has_value());
  CHECK(sha256_hex(*h) == hex);
  CHECK(!sha256_from_hex(hex.substr(1)).has_value());
  std::string bad = hex;
  bad[10] = 'g';
  CHECK(!sha256_from_hex(bad).has_value());
  std::string up = hex;
  for (char& c : up) c = static_cast<char>(std::toupper(c));
  auto hu = sha256_from_hex(up);
  CHECK(hu.has_value() && *hu == *h);
}

MTEST_MAIN
