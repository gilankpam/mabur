#include "probe_log.h"
#include "log_writer.h"
#include "mtest.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
static std::string read_all(const std::string& p) {
  std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}
TEST(probe_log_header_row_and_name) {
  std::string dir = "build_probe_log_test";
  (void)std::system(("rm -rf " + dir).c_str()); mkdir(dir.c_str(), 0755);
  maburgs::LogWriter w;
  maburgs::ProbeLog log(w, dir, /*bpb=*/4);
  REQUIRE(log.ok());
  CHECK(log.path() == dir + "/probe.log");
  log.row(1234, 99, 6, 17, 3, 0b11, 30.5, std::nan(""), -24.0, -22.5, 1130.4375);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.rfind("probelog 2 bpb=4\n", 0) == 0);
  // first_ms is the radio's µs-resolution arrival stamp: printed to 3
  // decimals so the completion->probe offset (a 1-10 ms quantity) survives.
  CHECK(text.find("\n1234 99 6 17 3 3 30.5 nan -24.0 -22.5 1130.438\n") != std::string::npos);
}
TEST(probe_log_bad_dir_is_nonfatal) {
  maburgs::LogWriter w;
  maburgs::ProbeLog log(w, "/nonexistent-dir-xyz", 4);
  CHECK(!log.ok());
  log.row(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}
MTEST_MAIN
