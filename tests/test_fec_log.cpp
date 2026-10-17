#include "fec_log.h"
#include "log_writer.h"
#include "mtest.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>
static std::string read_all(const std::string& p) {
  std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}
TEST(fec_log_header_row_and_name) {
  std::string dir = "build_fec_log_test";
  (void)std::system(("rm -rf " + dir).c_str()); mkdir(dir.c_str(), 0755);
  maburgs::LogWriter w;
  maburgs::FecLog log(w, dir);
  REQUIRE(log.ok());
  CHECK(log.path() == dir + "/fec.log");
  mabur::LossEpisode e;
  e.first_seq = (2ull << 32) + 3;  // one wrap past the anchor: wire seq 3
  e.span = 7; e.missing = 12; e.recovered = 11; e.retx = 4; e.abandoned = 1;
  e.stale = 2; e.repairs = 32; e.window = 32;
  log.row(1234.6, 1, 5, 40, 0.5, e);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.rfind("feclog 4\n", 0) == 0);
  CHECK(text.find("\n1235 1 5 40 0.50 3 7 12 11 4 1 2 32 32\n") != std::string::npos);
}
TEST(fec_log_bad_dir_is_nonfatal) {
  maburgs::LogWriter w;
  maburgs::FecLog log(w, "/nonexistent-dir-xyz");
  CHECK(!log.ok());
  log.row(0, 0, 0, 20, 0.0, mabur::LossEpisode{});
}
TEST(fec_log_request_and_heal_rows) {
  std::string dir = "build_fec_log_test_rh";
  (void)std::system(("rm -rf " + dir).c_str()); mkdir(dir.c_str(), 0755);
  maburgs::LogWriter w;
  maburgs::FecLog log(w, dir);
  REQUIRE(log.ok());
  log.request(2000.4, "hole", 1);
  log.request(2300.0, "hole", 2);
  log.heal(2412.0, 412, 1);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.find("\nI 2000 hole 1\n") != std::string::npos);
  CHECK(text.find("\nI 2300 hole 2\n") != std::string::npos);
  CHECK(text.find("\nH 2412 412 1\n") != std::string::npos);
}
MTEST_MAIN
