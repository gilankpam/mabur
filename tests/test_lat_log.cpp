#include <sys/stat.h>
#include <chrono>
#include <thread>
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
  log.sync();
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
  log.sync();
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
  log.sync();
  CHECK(log.path().empty());
  set_marker(mk, dir);
  log.write(4'000'000, "lat: still-throttled");  // inside the 5 s window
  log.sync();
  CHECK(log.path().empty());
  log.write(6'000'000, "lat: opens-now");  // past the window
  log.sync();
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
  log.sync();
  CHECK(log.path() == a + "/lat.log");
  set_marker(mk, b);
  log.write(7'000'000, "lat: second");  // past the 5 s recheck
  log.sync();
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
  lat.sync();
  frame.sync();
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
  frame.sync();
  CHECK(frame.path() == dir + "/frame.log");
  CHECK(slurp(frame.path()).find("row-a") == std::string::npos);
  frame.flush();
  frame.sync();
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
  frame.sync();  // the marker is read when the writer handles the row
  set_marker(mk, b);
  frame.write(7'000'000, "row-new");  // past the 5 s recheck: rotates
  frame.flush();
  frame.sync();
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
  frame.sync();  // the marker is read when the writer handles the row
  set_marker(mk, b);
  frame.write(5'990'000, "row-0");  // earlier stamp: no recheck
  frame.sync();
  CHECK(frame.path() == a + "/frame.log");
  frame.write(11'000'000, "row-2");  // 5 s on: the switch lands
  frame.sync();
  CHECK(frame.path() == b + "/frame.log");
  frame.flush();
  frame.sync();
  CHECK(slurp(a + "/frame.log").find("5990000 row-0\n") != std::string::npos);
}

TEST(flush_with_no_session_is_nonfatal) {
  const std::string mk = marker_for("flush-absent");
  std::remove(mk.c_str());
  maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 2", true);
  frame.flush();  // never opened: must not crash
  frame.write(1'000'000, "row");
  frame.flush();
  frame.sync();
  CHECK(frame.path().empty());
}

TEST(unwritable_session_dir_is_nonfatal) {
  const std::string mk = marker_for("unwritable");
  set_marker(mk, "/nonexistent-dir-xyz");
  maburplay::LatLog log(mk.c_str());
  log.write(1'000'000, "lat: x");  // must not crash
  log.sync();
  CHECK(log.path().empty());
}

// The player's main loop drives display and decode, so a log write must
// never wait on the disk. A FIFO with no reader blocks open() exactly like
// a stalled SD card: write() and flush() must still return at once, the
// rows that don't fit the queue are dropped and counted, and the file
// carries a "# dropped N" line once the disk is back.
TEST(a_stalled_disk_never_blocks_the_caller) {
  const std::string dir = make_dir("stall");
  const std::string mk = marker_for("stall");
  set_marker(mk, dir);
  const std::string fifo = dir + "/frame.log";
  REQUIRE(::mkfifo(fifo.c_str(), 0644) == 0);
  std::string got;
  std::thread reader;
  {
    maburplay::LatLog frame(mk.c_str(), "frame.log", "# framelog 3", true);
    const auto t0 = std::chrono::steady_clock::now();
    const std::string row(200, 'r');
    for (int i = 0; i < 5000; ++i) {   // ~1 MB: more than the queue holds
      frame.write(1'000'000 + static_cast<uint64_t>(i), row.c_str());
      if (i % 60 == 59) frame.flush();
    }
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    CHECK(ms < 500);                   // returned at once, never on the disk
    CHECK(frame.dropped() > 0);        // the overflow was dropped, not queued
    // The disk comes back: a reader drains the FIFO until the writer closes
    // it, which the destructor does after landing every queued line.
    reader = std::thread([&] {
      std::ifstream f(fifo);
      std::stringstream ss;
      ss << f.rdbuf();
      got = ss.str();
    });
    frame.write(9'000'000, "after-the-stall");
    frame.flush();
  }
  reader.join();
  CHECK(got.rfind("# framelog 3\n", 0) == 0);
  CHECK(got.find("# dropped ") != std::string::npos);
  CHECK(got.find("9000000 after-the-stall\n") != std::string::npos);
}

TEST(destruction_lands_every_queued_line) {
  const std::string dir = make_dir("dtor");
  const std::string mk = marker_for("dtor");
  set_marker(mk, dir);
  {
    maburplay::LatLog log(mk.c_str());
    for (int i = 0; i < 100; ++i) log.write(1'000'000 + static_cast<uint64_t>(i), "lat: q");
  }
  const std::string s = slurp(dir + "/lat.log");
  size_t n = 0;
  for (size_t at = s.find("lat: q"); at != std::string::npos; at = s.find("lat: q", at + 1)) ++n;
  CHECK(n == 100);
}

MTEST_MAIN
