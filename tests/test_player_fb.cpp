#include <cstring>
#include <string>
#include "mabur/player_fb.h"
#include "mtest.h"

using namespace mabur::playerfb;

TEST(round_trip) {
  Msg m;
  m.idr = true; m.reason = 1;
  m.flushes = 3; m.joins = 4; m.watchdogs = 5; m.episodes = 6;
  char buf[256];
  const size_t n = format(m, buf, sizeof(buf));
  CHECK(n > 0 && n < sizeof(buf));
  Msg got;
  CHECK(parse(buf, n, &got));
  CHECK(got.idr == true);
  CHECK(got.reason == 1);
  CHECK(got.flushes == 3);
  CHECK(got.joins == 4);
  CHECK(got.watchdogs == 5);
  CHECK(got.episodes == 6);
}

TEST(idr_zero_round_trips) {
  Msg m; m.idr = false; m.reason = 0;
  char buf[256];
  const size_t n = format(m, buf, sizeof(buf));
  Msg got; got.idr = true;
  CHECK(parse(buf, n, &got));
  CHECK(got.idr == false);
}

TEST(unknown_keys_are_ignored) {
  const std::string s = "mabur-fb v=1 idr=1 reason=join newfield=99 joins=2";
  Msg got;
  CHECK(parse(s.data(), s.size(), &got));
  CHECK(got.idr == true);
  CHECK(got.reason == 2);
  CHECK(got.joins == 2);
}

TEST(rejects_wrong_magic_version_and_junk) {
  Msg got;
  const std::string bad_magic = "nope v=1 idr=1";
  CHECK(!parse(bad_magic.data(), bad_magic.size(), &got));
  const std::string bad_ver = "mabur-fb v=2 idr=1";
  CHECK(!parse(bad_ver.data(), bad_ver.size(), &got));
  const std::string no_ver = "mabur-fb idr=1";
  CHECK(!parse(no_ver.data(), no_ver.size(), &got));
  const std::string empty = "";
  CHECK(!parse(empty.data(), empty.size(), &got));
  const std::string truncated = "mabur-";
  CHECK(!parse(truncated.data(), truncated.size(), &got));
}

TEST(rejects_missing_or_non_numeric_idr) {
  Msg got;
  const std::string no_idr = "mabur-fb v=1 joins=1";
  CHECK(!parse(no_idr.data(), no_idr.size(), &got));
  const std::string junk_idr = "mabur-fb v=1 idr=yes";
  CHECK(!parse(junk_idr.data(), junk_idr.size(), &got));
}

TEST(parse_does_not_read_past_n) {
  // The socket hands us a length, not a NUL-terminated string.
  const char raw[] = "mabur-fb v=1 idr=1 joins=7GARBAGE";
  Msg got;
  CHECK(parse(raw, 26, &got));  // cut immediately after "joins=7"
  CHECK(got.joins == 7);
}

TEST(format_respects_capacity) {
  Msg m; m.idr = true;
  char small[8];
  const size_t n = format(m, small, sizeof(small));
  CHECK(n == 0);  // refuses to emit a truncated, unparseable datagram
}

TEST(reason_names) {
  CHECK(std::strcmp(reason_name(0), "none") == 0);
  CHECK(std::strcmp(reason_name(1), "flush") == 0);
  CHECK(std::strcmp(reason_name(2), "join") == 0);
  CHECK(std::strcmp(reason_name(3), "watchdog") == 0);
  CHECK(std::strcmp(reason_name(99), "none") == 0);
}

MTEST_MAIN
