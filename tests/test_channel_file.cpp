#include <cstdio>
#include <fstream>
#include <string>
#include "mtest.h"
#include "mabur/channel_file.h"
using namespace mabur;

static std::string tmp_path(const char* name) {
  return std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/mabur_chfile_" + name;
}
static void put(const std::string& p, const std::string& text) { std::ofstream f(p); f << text; }

TEST(write_then_read_round_trips) {
  const std::string p = tmp_path("rt");
  std::remove(p.c_str());
  CHECK(write_channel_file(p, 112));
  auto r = read_channel_file(p);
  REQUIRE(r.has_value());
  CHECK(*r == 112);
  std::ifstream f(p); std::string s; std::getline(f, s);
  CHECK(s == "112");                       // decimal text, one line
  CHECK(write_channel_file(p, 64));        // overwrite via temp+rename
  CHECK(*read_channel_file(p) == 64);
  std::remove(p.c_str());
}

TEST(missing_or_garbage_reads_as_nullopt) {
  const std::string p = tmp_path("bad");
  std::remove(p.c_str());
  CHECK(!read_channel_file(p).has_value());
  put(p, "");       CHECK(!read_channel_file(p).has_value());
  put(p, "abc\n");  CHECK(!read_channel_file(p).has_value());
  put(p, "0\n");    CHECK(!read_channel_file(p).has_value());
  put(p, "300\n");  CHECK(!read_channel_file(p).has_value());
  put(p, " 144 \n"); CHECK(read_channel_file(p).value_or(0) == 144);
  std::remove(p.c_str());
}

TEST(write_to_unwritable_dir_fails_cleanly) {
  CHECK(!write_channel_file("/nonexistent-dir-xyz/mabur.channel", 40));
}

MTEST_MAIN
