#include "ctl_log.h"
#include "log_writer.h"
#include "mtest.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <sys/stat.h>

static std::string read_all(const std::string& p) {
  std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

static void reset_dir(const std::string& dir) {
  (void)std::system(("rm -rf " + dir).c_str());
  mkdir(dir.c_str(), 0755);
}

TEST(ctl_log_writes_header_and_records) {
  std::string dir = "build_ctl_log_test";
  reset_dir(dir);
  maburgs::LogWriter w;
  maburgs::CtlLog log(w, dir, "ladder=0/100,2/50 down_util=0.35 up_util=0.15");
  REQUIRE(log.ok());
  log.sample(1000, 2, 0.05, 31.5, 0.0, 0.10, 0.0, -24.5, 0.0, 9.5, 4.2, -63.4, 3, 0.12, 60);
  log.event(1500, 2, 1, "s3_util", 0.4, 30.0, -23.0);
  log.probe(2000, 3, "lossy", 24.0, 0.9, 600, -22.5);
  log.penalty(2000, 3, 1, 12000);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.rfind("ctllog 11 ladder=0/100,2/50 down_util=0.35 up_util=0.15\n", 0) == 0);
  CHECK(text.find("\nS 1000 2 0.0500 31.5 0.0000 0.1000 0.0000 -24.5 0.0000 9.5 4.2 -63.4 3 0.1200 60\n") != std::string::npos);
  CHECK(text.find("\nE 1500 2 1 s3_util 0.4000 30.0 -23.0\n") != std::string::npos);
  CHECK(text.find("\nP 2000 3 lossy 24.0 0.9000 600 -22.5\n") != std::string::npos);
  CHECK(text.find("\nN 2000 3 1 12000\n") != std::string::npos);
  CHECK(log.path() == dir + "/ctl.log");
}

TEST(ctl_log_bad_dir_is_nonfatal) {
  maburgs::LogWriter w;
  maburgs::CtlLog log(w, "/nonexistent-dir-xyz", "x");
  CHECK(!log.ok());
  log.sample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
             std::numeric_limits<double>::quiet_NaN(), 0);  // must not crash
}

TEST(ctl_log_nan_snr_prints_nan) {
  std::string dir = "build_ctl_log_test3";
  reset_dir(dir);
  maburgs::LogWriter w;
  maburgs::CtlLog log(w, dir, "x");
  REQUIRE(log.ok());
  log.sample(1, 0, 0, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0,
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(), -1,
             std::numeric_limits<double>::quiet_NaN(), 0);
  w.flush_now();
  CHECK(read_all(log.path()).find(" nan ") != std::string::npos);
  CHECK(read_all(log.path()).find("-1 nan 0\n") != std::string::npos);  // trailing probe fields
}

TEST(ctl_log_v2_sample_carries_resid_cur) {
  // resid_cur (2026-08-14 attribution) still sits at the same position in
  // the S line under ctllog 3 -- only drssi/dsnr were appended after it.
  std::string dir = "build_ctl_log_test_v2";
  reset_dir(dir);
  maburgs::LogWriter w;
  maburgs::CtlLog log(w, dir, "ladder=5/25 down_util=0.60 up_util=0.15");
  REQUIRE(log.ok());
  log.sample(1234, 3, 0.0123, 31.5, 0.0456, 0.0, 0.0, -21.0, 0.0011, 9.5,
             4.2, -63.4, -1, std::numeric_limits<double>::quiet_NaN(), 0);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.rfind("ctllog 11 ladder=5/25 down_util=0.60 up_util=0.15\n", 0) == 0);
  CHECK(text.find("\nS 1234 3 0.0123 31.5 0.0456 0.0000 0.0000 -21.0 0.0011 9.5 4.2 -63.4 -1 nan 0\n") != std::string::npos);
}

TEST(ctl_log_v3_sample_carries_fade_deltas) {
  std::string dir = "build_ctl_log_test_v3";
  reset_dir(dir);
  maburgs::LogWriter w;
  maburgs::CtlLog log(w, dir, "ladder=5/25 down_util=0.60 up_util=0.15");
  REQUIRE(log.ok());
  log.sample(1234, 3, 0.0123, 31.5, 0.0456, 0.0, 0.0, -21.0, 0.0011, 9.5,
             4.2, -63.4, -1, std::numeric_limits<double>::quiet_NaN(), 0);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.rfind("ctllog 11 ladder=5/25 down_util=0.60 up_util=0.15\n", 0) == 0);
  CHECK(text.find("\nS 1234 3 0.0123 31.5 0.0456 0.0000 0.0000 -21.0 0.0011 9.5 4.2 -63.4 -1 nan 0\n") != std::string::npos);
}

TEST(ctl_log_rung_record_layout) {
  std::string dir = "build_ctl_log_test_rung";
  reset_dir(dir);
  maburgs::LogWriter w;
  maburgs::CtlLog log(w, dir, "x");
  REQUIRE(log.ok());
  log.rung(5000, 3, 0.0625, 0.25, 0.125, 0.0, -24.5, 0.75, 1234, 12.5, 0.25, 7);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // Sentinel clamp on u/u3/probe_u; nan evm legal; never-sampled age -1.
  log.rung(6000, 4, 1e9, 0.0, 1e9, 0.0, nan, nan, 0, -1.0, 1e9, 2);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.find("\nR 5000 3 0.0625 0.2500 0.1250 0.0000 -24.5 0.75 1234 12.5 0.2500 7\n") != std::string::npos);
  CHECK(text.find("\nR 6000 4 1000.0000 0.0000 1000.0000 0.0000 nan nan 0 -1.0 1000.0000 2\n") != std::string::npos);
}

MTEST_MAIN
