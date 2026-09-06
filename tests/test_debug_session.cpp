#include "debug_session.h"
#include "mtest.h"

#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace {
std::string make_root(const char* tag) {
  std::string dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/dbgses-" + tag;
  (void)std::system(("rm -rf " + dir).c_str());
  ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
  ::mkdir(dir.c_str(), 0755);
  return dir;
}
std::string marker_for(const char* tag) {
  return std::string(MABUR_TEST_SCRATCH_DIR) + "/dbgses-marker-" + tag;
}
std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
bool is_dir(const std::string& p) {
  struct stat st{};
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}
}  // namespace

TEST(fresh_root_allocates_index_zero_and_writes_marker) {
  const std::string root = make_root("fresh");
  const std::string mk = marker_for("fresh");
  std::remove(mk.c_str());
  maburgs::DebugSession s(root, true, mk.c_str());
  REQUIRE(s.ok());
  CHECK(s.index() == 0);
  CHECK(s.dir() == root + "/0000");
  CHECK(is_dir(s.dir()));
  CHECK(!s.rejoined());
  CHECK(slurp(mk) == root + "/0000");
}

TEST(existing_dirs_pick_max_plus_one_and_ignore_junk) {
  const std::string root = make_root("maxplus");
  const std::string mk = marker_for("maxplus");
  std::remove(mk.c_str());
  ::mkdir((root + "/0000").c_str(), 0755);
  ::mkdir((root + "/0003").c_str(), 0755);
  ::mkdir((root + "/notanumber").c_str(), 0755);
  { std::ofstream(root + "/0009.txt") << "not a dir\n"; }
  maburgs::DebugSession s(root, true, mk.c_str());
  REQUIRE(s.ok());
  CHECK(s.index() == 4);
  CHECK(s.dir() == root + "/0004");
}

TEST(marker_pointing_at_live_dir_rejoins_without_allocating) {
  const std::string root = make_root("rejoin");
  const std::string mk = marker_for("rejoin");
  std::remove(mk.c_str());
  maburgs::DebugSession first(root, true, mk.c_str());
  REQUIRE(first.ok());
  maburgs::DebugSession second(root, true, mk.c_str());
  REQUIRE(second.ok());
  CHECK(second.dir() == first.dir());
  CHECK(second.index() == first.index());
  CHECK(second.rejoined());
  CHECK(!is_dir(root + "/0001"));  // nothing new allocated
}

TEST(stale_marker_allocates_fresh_index) {
  const std::string root = make_root("stale");
  const std::string mk = marker_for("stale");
  { std::ofstream(mk) << root + "/0007"; }  // names a dir that does not exist
  maburgs::DebugSession s(root, true, mk.c_str());
  REQUIRE(s.ok());
  CHECK(s.index() == 0);
  CHECK(s.dir() == root + "/0000");
  CHECK(!s.rejoined());
}

TEST(disabled_removes_marker_and_is_not_ok) {
  const std::string root = make_root("disabled");
  const std::string mk = marker_for("disabled");
  { std::ofstream(mk) << root + "/0000"; }
  maburgs::DebugSession s(root, false, mk.c_str());
  CHECK(!s.ok());
  CHECK(s.dir().empty());
  std::ifstream f(mk);
  CHECK(!f.good());  // marker gone
}

TEST(unwritable_root_is_nonfatal) {
  const std::string mk = marker_for("unwritable");
  std::remove(mk.c_str());
  maburgs::DebugSession s("/nonexistent-root-xyz", true, mk.c_str());
  CHECK(!s.ok());
  CHECK(s.dir().empty());
}

MTEST_MAIN
