#include "log_writer.h"
#include "mtest.h"

#include <sys/stat.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace {
std::string make_dir(const char* tag) {
  std::string dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/logw-" + tag;
  (void)std::system(("rm -rf " + dir).c_str());
  ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
  ::mkdir(dir.c_str(), 0755);
  return dir;
}
std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
void put(maburgs::LogWriter& w, maburgs::LogWriter::Stream s, const std::string& t) {
  w.line(s, t.data(), t.size());
}
}  // namespace

TEST(two_streams_get_headers_and_ordered_lines) {
  const std::string dir = make_dir("two");
  maburgs::LogWriter w;
  auto a = w.open(dir, "ctl.log", "ctllog 11 ladder=x");
  auto b = w.open(dir, "au.log", "# aulog 4");
  REQUIRE(a != maburgs::LogWriter::kBadStream);
  REQUIRE(b != maburgs::LogWriter::kBadStream);
  CHECK(w.path(a) == dir + "/ctl.log");
  put(w, a, "S 1000 2");
  put(w, b, "111 222");
  put(w, a, "S 2000 3");
  w.flush_now();
  CHECK(slurp(dir + "/ctl.log") == "ctllog 11 ladder=x\nS 1000 2\nS 2000 3\n");
  CHECK(slurp(dir + "/au.log") == "# aulog 4\n111 222\n");
}

TEST(reopen_appends_and_repeats_the_header) {
  const std::string dir = make_dir("append");
  {
    maburgs::LogWriter w;
    auto a = w.open(dir, "ctl.log", "ctllog 11 first");
    put(w, a, "S 1");
    w.flush_now();
  }
  {
    maburgs::LogWriter w;  // a respawn rejoining the same session
    auto a = w.open(dir, "ctl.log", "ctllog 11 second");
    put(w, a, "S 2");
    w.flush_now();
  }
  CHECK(slurp(dir + "/ctl.log") ==
        "ctllog 11 first\nS 1\nctllog 11 second\nS 2\n");
}

TEST(bad_dir_returns_bad_stream_and_line_is_a_noop) {
  maburgs::LogWriter w;
  auto s = w.open("/nonexistent-dir-xyz", "ctl.log", "h");
  CHECK(s == maburgs::LogWriter::kBadStream);
  put(w, s, "must not crash");
  w.flush_now();
}

TEST(oversize_line_is_dropped_not_truncated) {
  const std::string dir = make_dir("oversize");
  maburgs::LogWriter w;
  auto a = w.open(dir, "au.log", "# aulog 4");
  const std::string huge(maburgs::LogWriter::kMaxLine + 1, 'x');
  put(w, a, huge);
  put(w, a, "kept");
  w.flush_now();
  const std::string text = slurp(dir + "/au.log");
  CHECK(text.find('x') == std::string::npos);
  CHECK(text.find("\nkept\n") != std::string::npos);
  CHECK(w.dropped(a) == 1);
}

TEST(ring_overflow_drops_and_reports) {
  const std::string dir = make_dir("overflow");
  maburgs::LogWriter w;
  auto a = w.open(dir, "au.log", "# aulog 4");
  // Fill the ring faster than the writer thread can possibly drain it. The
  // line body is large so a bounded loop is enough to overrun 1 MiB.
  const std::string big(8192, 'y');
  for (int i = 0; i < 4096; ++i) put(w, a, big);
  w.flush_now();
  REQUIRE(w.dropped(a) > 0);
  const std::string text = slurp(dir + "/au.log");
  CHECK(text.find("# dropped ") != std::string::npos);
}

TEST(empty_header_writes_nothing_and_is_not_a_drop) {
  const std::string dir = make_dir("nohdr");
  maburgs::LogWriter w;
  auto a = w.open(dir, "flight.jsonl", "", /*mark_drops=*/false);
  REQUIRE(a != maburgs::LogWriter::kBadStream);
  put(w, a, "{\"v\":1}");
  w.flush_now();
  CHECK(slurp(dir + "/flight.jsonl") == "{\"v\":1}\n");
  CHECK(w.dropped(a) == 0);
}

TEST(mark_drops_false_keeps_the_file_free_of_comment_lines) {
  const std::string dir = make_dir("nomark");
  maburgs::LogWriter w;
  auto a = w.open(dir, "flight.jsonl", "", /*mark_drops=*/false);
  const std::string big(8192, 'y');
  for (int i = 0; i < 4096; ++i) put(w, a, big);
  w.flush_now();
  REQUIRE(w.dropped(a) > 0);          // drops really happened
  const std::string text = slurp(dir + "/flight.jsonl");
  CHECK(text.find('#') == std::string::npos);  // ...and left no comment line
}

MTEST_MAIN
