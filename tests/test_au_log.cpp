#include "au_log.h"
#include "log_writer.h"
#include "mtest.h"

#include <sys/stat.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace {
std::string make_dir(const char* tag) {
  std::string dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/aulog-" + tag;
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
maburgs::AuRecordMeta meta() {
  maburgs::AuRecordMeta m;
  m.pts_us = 123456;
  m.sid = 1;
  m.frame_id64 = 70000;   // past the u16 wrap, so the column must be 64-bit
  m.len = 4321;
  m.flags = 0x81;         // kFlagIdr | kRecFlagComplete
  m.t_first_us = 900000;
  m.t_complete_us = 912000;
  m.enc_us = 7000;
  m.drone_q_ms = 3;
  m.drone_air_ms = 21;
  return m;
}
}  // namespace

TEST(header_and_row_bytes) {
  const std::string dir = make_dir("row");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  REQUIRE(au.ok());
  au.begin();
  const uint8_t payload[] = {0, 0, 0, 1, 0x40, 0x01};  // 4-byte start, nal 32
  au.payload(payload, sizeof(payload));
  au.row(555000, meta());
  w.flush_now();
  const std::string text = slurp(dir + "/au.log");
  CHECK(text.rfind("# aulog 4\n", 0) == 0);
  CHECK(text.find(
      "\n555000 123456 1 70000 4321 0x81 32 900000 912000 7000 3 21\n") !=
      std::string::npos);
}

TEST(three_byte_start_code_nal_type) {
  const std::string dir = make_dir("nal3");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t payload[] = {0, 0, 1, 0x02, 0x01};  // 3-byte start, nal 1
  au.payload(payload, sizeof(payload));
  au.row(1, meta());
  w.flush_now();
  CHECK(slurp(dir + "/au.log").find(" 0x81 1 ") != std::string::npos);
}

TEST(unrecognised_payload_reports_minus_one) {
  const std::string dir = make_dir("nalbad");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t payload[] = {9, 9, 9, 9, 9, 9};
  au.payload(payload, sizeof(payload));
  au.row(1, meta());
  w.flush_now();
  CHECK(slurp(dir + "/au.log").find(" 0x81 -1 ") != std::string::npos);
}

TEST(three_byte_start_code_nal_type_at_true_minimum_length) {
  const std::string dir = make_dir("nal3min");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t payload[] = {0, 0, 1, 0x02};  // exactly 4 bytes: n >= 4, not 5
  au.payload(payload, sizeof(payload));
  au.row(1, meta());
  w.flush_now();
  CHECK(slurp(dir + "/au.log").find(" 0x81 1 ") != std::string::npos);
}

TEST(three_byte_start_code_one_byte_short_reports_minus_one) {
  const std::string dir = make_dir("nal3short");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t payload[] = {0, 0, 1};  // one byte short of the true minimum
  au.payload(payload, sizeof(payload));
  au.row(1, meta());
  w.flush_now();
  CHECK(slurp(dir + "/au.log").find(" 0x81 -1 ") != std::string::npos);
}

TEST(four_byte_start_code_nal_type_at_true_minimum_length) {
  const std::string dir = make_dir("nal4min");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t payload[] = {0, 0, 0, 1, 0x40};  // exactly 5 bytes: n >= 5, not 6
  au.payload(payload, sizeof(payload));
  au.row(1, meta());
  w.flush_now();
  CHECK(slurp(dir + "/au.log").find(" 0x81 32 ") != std::string::npos);
}

TEST(four_byte_start_code_one_byte_short_reports_minus_one) {
  const std::string dir = make_dir("nal4short");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t payload[] = {0, 0, 0, 1};  // one byte short of the true minimum
  au.payload(payload, sizeof(payload));
  au.row(1, meta());
  w.flush_now();
  CHECK(slurp(dir + "/au.log").find(" 0x81 -1 ") != std::string::npos);
}

TEST(head_is_latched_across_fragmented_payload_and_reset_by_begin) {
  const std::string dir = make_dir("frag");
  maburgs::LogWriter w;
  maburgs::AuLog au(w, dir);
  au.begin();
  const uint8_t a[] = {0, 0};
  const uint8_t b[] = {1, 0x40, 0x01, 0xff};
  au.payload(a, sizeof(a));
  au.payload(b, sizeof(b));  // only the first 6 bytes overall are latched
  au.row(1, meta());
  au.begin();                // a new AU must not inherit the old head
  const uint8_t c[] = {0, 0, 1, 0x02, 0x01};
  au.payload(c, sizeof(c));
  au.row(2, meta());
  w.flush_now();
  const std::string text = slurp(dir + "/au.log");
  CHECK(text.find("\n1 123456 1 70000 4321 0x81 32 ") != std::string::npos);
  CHECK(text.find("\n2 123456 1 70000 4321 0x81 1 ") != std::string::npos);
}

MTEST_MAIN
