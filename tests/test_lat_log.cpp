#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>

#include "lat_log.h"
#include "mtest.h"

namespace {
std::string make_tmpdir() {
  char tmpl[] = "/tmp/latlog_test_XXXXXX";
  return std::string(mkdtemp(tmpl));
}
std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
}  // namespace

TEST(writes_header_and_lines_with_next_free_index) {
  const std::string dir = make_tmpdir();
  // Pre-existing index 3 -> new file must be lat-0004.log.
  { std::ofstream(dir + "/lat-0003.log") << "x\n"; }
  maburplay::LatLog log(dir);
  log.write(1'000'000, 1'700'000'000'000'000ull, "lat: n=12 e2e=9/17");
  log.write(2'000'000, 1'700'000'001'000'000ull, "lat: n=12 e2e=8/16");
  CHECK(log.path() == dir + "/lat-0004.log");
  const std::string s = slurp(log.path());
  CHECK(s.find("# latlog 1\n") == 0);
  CHECK(s.find("# sync 1000000 1700000000000000\n") != std::string::npos);
  CHECK(s.find("1000000 lat: n=12 e2e=9/17\n") != std::string::npos);
  CHECK(s.find("2000000 lat: n=12 e2e=8/16\n") != std::string::npos);
}

TEST(empty_dir_disables) {
  maburplay::LatLog log("");
  log.write(1, 2, "lat: x");  // must not crash, must not create anything
  CHECK(log.path().empty());
}

TEST(unwritable_dir_retries_every_30s) {
  maburplay::LatLog log("/nonexistent/mabur-latlog");
  log.write(1'000'000, 1, "a");            // open fails silently
  CHECK(log.path().empty());
  log.write(10'000'000, 2, "b");           // 9 s later: no retry yet
  CHECK(log.path().empty());
  // (Retry-side success can't be simulated on a fixed bad path; the
  // 30 s gate is pinned from the other side: create the dir late.)
  const std::string dir = make_tmpdir();
  maburplay::LatLog log2(dir + "/sub");     // missing subdir: open fails
  log2.write(1'000'000, 1, "a");
  CHECK(log2.path().empty());
  CHECK(mkdir((dir + "/sub").c_str(), 0755) == 0);
  log2.write(20'000'000, 2, "b");           // 19 s: still inside backoff
  CHECK(log2.path().empty());
  log2.write(32'000'000, 3, "c");           // 31 s: retry fires, succeeds
  CHECK(!log2.path().empty());
  const std::string s = slurp(log2.path());
  CHECK(s.find("# sync 32000000 3\n") != std::string::npos);  // sync = first success
  CHECK(s.find("32000000 c\n") != std::string::npos);         // no backfill
}

MTEST_MAIN
