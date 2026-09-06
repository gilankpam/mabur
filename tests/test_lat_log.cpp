#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "lat_log.h"
#include "mtest.h"

namespace {
std::string make_dir(const char* tag) {
  std::string dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/latlog-" + tag;
  (void)std::system(("rm -rf " + dir).c_str());
  ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
  ::mkdir(dir.c_str(), 0755);
  return dir;
}
std::string marker_for(const char* tag) {
  return std::string(MABUR_TEST_SCRATCH_DIR) + "/latlog-marker-" + tag;
}
void set_marker(const std::string& mk, const std::string& dir) {
  std::ofstream f(mk, std::ios::trunc);
  f << dir;
}
std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
}  // namespace

TEST(writes_into_the_session_named_by_the_marker) {
  const std::string dir = make_dir("basic");
  const std::string mk = marker_for("basic");
  set_marker(mk, dir);
  maburplay::LatLog log(mk.c_str());
  log.write(1'000'000, "lat: n=12 e2e=9/17");
  log.write(2'000'000, "lat: n=12 e2e=8/16");
  CHECK(log.path() == dir + "/lat.log");
  const std::string s = slurp(log.path());
  CHECK(s.rfind("# latlog 2\n", 0) == 0);
  CHECK(s.find("# sync") == std::string::npos);  // one clock, no bridge
  CHECK(s.find("1000000 lat: n=12 e2e=9/17\n") != std::string::npos);
  CHECK(s.find("2000000 lat: n=12 e2e=8/16\n") != std::string::npos);
}

TEST(no_marker_writes_nothing) {
  const std::string mk = marker_for("absent");
  std::remove(mk.c_str());
  maburplay::LatLog log(mk.c_str());
  log.write(1'000'000, "lat: x");
  CHECK(log.path().empty());
}

TEST(missing_marker_backs_off_for_the_full_recheck_window) {
  // no_marker_writes_nothing above only calls write() once, so it can't
  // tell "no marker" from "checked and backing off" -- this pins the
  // throttle itself: several closely-spaced calls with no marker must not
  // hammer read_marker()/fopen() on every tick, and a marker that appears
  // mid-window must not be picked up until the window elapses.
  const std::string dir = make_dir("backoff");
  const std::string mk = marker_for("backoff");
  std::remove(mk.c_str());
  maburplay::LatLog log(mk.c_str());
  log.write(0, "lat: a");
  log.write(1'000, "lat: b");
  log.write(2'000, "lat: c");
  CHECK(log.path().empty());
  set_marker(mk, dir);
  log.write(29'000'000, "lat: still-throttled");  // inside the 30 s window
  CHECK(log.path().empty());
  log.write(31'000'000, "lat: opens-now");  // past the window
  CHECK(log.path() == dir + "/lat.log");
  const std::string s = slurp(log.path());
  CHECK(s.find("lat: still-throttled") == std::string::npos);
  CHECK(s.find("lat: opens-now") != std::string::npos);
}

TEST(marker_change_reopens_in_the_new_session) {
  const std::string a = make_dir("switch-a");
  const std::string b = make_dir("switch-b");
  const std::string mk = marker_for("switch");
  set_marker(mk, a);
  maburplay::LatLog log(mk.c_str());
  log.write(1'000'000, "lat: first");
  CHECK(log.path() == a + "/lat.log");
  set_marker(mk, b);
  log.write(32'000'000, "lat: second");  // past the 30 s recheck
  CHECK(log.path() == b + "/lat.log");
  CHECK(slurp(a + "/lat.log").find("lat: first") != std::string::npos);
  CHECK(slurp(b + "/lat.log").find("lat: second") != std::string::npos);
}

TEST(reopening_the_same_session_appends) {
  const std::string dir = make_dir("append");
  const std::string mk = marker_for("append");
  set_marker(mk, dir);
  { maburplay::LatLog a(mk.c_str()); a.write(1'000'000, "lat: run1"); }
  { maburplay::LatLog b(mk.c_str()); b.write(2'000'000, "lat: run2"); }
  const std::string s = slurp(dir + "/lat.log");
  CHECK(s.find("lat: run1") != std::string::npos);
  CHECK(s.find("lat: run2") != std::string::npos);
  CHECK(s.find("# latlog 2") != s.rfind("# latlog 2"));  // header twice
}

TEST(unwritable_session_dir_is_nonfatal) {
  const std::string mk = marker_for("unwritable");
  set_marker(mk, "/nonexistent-dir-xyz");
  maburplay::LatLog log(mk.c_str());
  log.write(1'000'000, "lat: x");  // must not crash
  CHECK(log.path().empty());
}

MTEST_MAIN
