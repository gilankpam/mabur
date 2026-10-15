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
  log.write(4'000'000, "lat: still-throttled");  // inside the 5 s window
  CHECK(log.path().empty());
  log.write(6'000'000, "lat: opens-now");  // past the window
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
  log.write(7'000'000, "lat: second");  // past the 5 s recheck
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

TEST(a_second_instance_owns_its_own_file_and_marker_line) {
  // frame.log is the same follow-the-session writer under another name:
  // same marker, its own file and format marker, the two never mix.
  const std::string dir = make_dir("frame");
  const std::string mk = marker_for("frame");
  set_marker(mk, dir);
  maburplay::LatLog lat(mk.c_str());
  maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 2");
  lat.write(1'000'000, "lat: n=1");
  frame.write(1'000'500, "1000 7 0 61234 129 4");
  CHECK(lat.path() == dir + "/lat.log");
  CHECK(frame.path() == dir + "/frame.log");
  const std::string f = slurp(frame.path());
  CHECK(f.rfind("# framelog 2\n", 0) == 0);
  CHECK(f.find("1000500 1000 7 0 61234 129 4\n") != std::string::npos);
  CHECK(f.find("lat:") == std::string::npos);
  const std::string l = slurp(lat.path());
  CHECK(l.find("framelog") == std::string::npos);
  CHECK(l.find("1000000 lat: n=1\n") != std::string::npos);
}

TEST(a_batched_instance_holds_rows_until_flush) {
  // frame.log writes ~60 rows in one burst on the render thread: they
  // must leave as one write at flush(), not one syscall per row.
  const std::string dir = make_dir("batched");
  const std::string mk = marker_for("batched");
  set_marker(mk, dir);
  maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 2",
                          /*batched=*/true);
  frame.write(1'000'000, "row-a");
  frame.write(1'016'000, "row-b");
  CHECK(frame.path() == dir + "/frame.log");
  CHECK(slurp(frame.path()).find("row-a") == std::string::npos);
  frame.flush();
  const std::string f = slurp(frame.path());
  CHECK(f.rfind("# framelog 2\n", 0) == 0);
  CHECK(f.find("1000000 row-a\n1016000 row-b\n") != std::string::npos);
}

TEST(a_batched_rotation_lands_held_rows_in_the_old_session) {
  // Rows still buffered when the session rotates belong to the old file,
  // and only rows written after the rotation go to the new one.
  const std::string a = make_dir("batched-rot-a");
  const std::string b = make_dir("batched-rot-b");
  const std::string mk = marker_for("batched-rot");
  set_marker(mk, a);
  maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 2", true);
  frame.write(1'000'000, "row-old");
  set_marker(mk, b);
  frame.write(7'000'000, "row-new");  // past the 5 s recheck: rotates
  frame.flush();
  const std::string fa = slurp(a + "/frame.log");
  const std::string fb = slurp(b + "/frame.log");
  CHECK(fa.find("1000000 row-old\n") != std::string::npos);
  CHECK(fa.find("row-new") == std::string::npos);
  CHECK(fb.rfind("# framelog 2\n", 0) == 0);
  CHECK(fb.find("7000000 row-new\n") != std::string::npos);
  CHECK(fb.find("row-old") == std::string::npos);
}

TEST(an_earlier_row_stamp_does_not_force_a_recheck) {
  // frame.log stamps each row with its flip time, up to ~1 s before the
  // write. A stamp behind the last check is not "5 s elapsed": the
  // marker is not re-read, so a session switch is picked up only once the
  // stamps have really moved 5 s on.
  const std::string a = make_dir("backstamp-a");
  const std::string b = make_dir("backstamp-b");
  const std::string mk = marker_for("backstamp");
  set_marker(mk, a);
  maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 2", true);
  frame.write(6'000'000, "row-1");
  set_marker(mk, b);
  frame.write(5'990'000, "row-0");  // earlier stamp: no recheck
  CHECK(frame.path() == a + "/frame.log");
  frame.write(11'000'000, "row-2");  // 5 s on: the switch lands
  CHECK(frame.path() == b + "/frame.log");
  frame.flush();
  CHECK(slurp(a + "/frame.log").find("5990000 row-0\n") != std::string::npos);
}

TEST(flush_with_no_session_is_nonfatal) {
  const std::string mk = marker_for("flush-absent");
  std::remove(mk.c_str());
  maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 2", true);
  frame.flush();  // never opened: must not crash
  frame.write(1'000'000, "row");
  frame.flush();
  CHECK(frame.path().empty());
}

TEST(unwritable_session_dir_is_nonfatal) {
  const std::string mk = marker_for("unwritable");
  set_marker(mk, "/nonexistent-dir-xyz");
  maburplay::LatLog log(mk.c_str());
  log.write(1'000'000, "lat: x");  // must not crash
  CHECK(log.path().empty());
}

MTEST_MAIN
