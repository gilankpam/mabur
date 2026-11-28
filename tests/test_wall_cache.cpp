#include "mtest.h"
#include "wall_cache.h"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace mabur;

namespace {
struct Dir {
  std::string d;
  Dir() { char t[] = "/tmp/wallcacheXXXXXX"; REQUIRE(mkdtemp(t)); d = t; }
  ~Dir() { if (std::system(("rm -rf " + d).c_str())) {} }
  std::string p() const { return d + "/mabur.walls"; }
};
void put(const std::string& path, const std::string& s) { std::ofstream(path, std::ios::binary) << s; }
std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(f), {});
}
std::string vals(int n, const std::string& v = "-100") {
  std::string s;
  for (int i = 0; i < n; ++i) s += " " + v;
  return s;
}
std::string line(const std::string& ch, const std::string& w, const std::string& a, const std::string& rest) {
  return ch + " " + w + " " + a + " p" + rest + "\n";
}
const std::string H = "maburwalls 2\n";

WallCurve mk(int anchor, int base) {
  WallCurve c;
  c.anchor = anchor;
  for (int k = 0; k < 25; ++k) c.evm_db[k] = (base - k * 7) / 10.0f;
  return c;
}
bool same(const WallCurve& a, const WallCurve& b) {
  return a.anchor == b.anchor && a.evm_db == b.evm_db && a.weak_park == b.weak_park;
}

void expect_bad(const std::string& content) {
  Dir t;
  put(t.p(), content);
  std::string note;
  auto c = load_wall_cache(t.p(), &note);
  CHECK(c.entries.empty());
  CHECK(!note.empty() && note.find('\n') == std::string::npos);
}
}  // namespace

TEST(round_trip_keeps_values_and_gaps) {  // fails if save rounds by truncation, drops the x, or load scales differently
  Dir t;
  WallCache c;
  c.entries[{144, 20}] = mk(39, -306);
  c.entries[{144, 20}].evm_db[2].reset();
  c.entries[{40, 40}] = mk(75, -319);
  c.entries[{40, 40}].evm_db[3] = -31.1f;  // not exactly representable as k/10 via arithmetic
  REQUIRE(save_wall_cache(t.p(), c));
  std::string note = "stale";
  auto r = load_wall_cache(t.p(), &note);
  CHECK(note.empty());
  REQUIRE(r.entries.size() == 2);
  CHECK(same(r.entries.at({144, 20}), c.entries.at({144, 20})));
  CHECK(same(r.entries.at({40, 40}), c.entries.at({40, 40})));
  CHECK(!r.entries.at({144, 20}).evm_db[2]);
  CHECK(slurp(t.p()).rfind("maburwalls 2\n40 40 75 p ", 0) == 0);  // sorted: channel 40 first
}

TEST(version_2_round_trip_keeps_the_flag) {
  Dir t;
  WallCache c;
  c.entries[{40, 40}] = mk(75, -319);
  c.entries[{144, 20}] = mk(39, -306);
  c.entries[{144, 20}].weak_park = true;
  REQUIRE(save_wall_cache(t.p(), c));
  std::string note;
  auto r = load_wall_cache(t.p(), &note);
  CHECK(note.empty());
  REQUIRE(r.entries.size() == 2);
  CHECK(same(r.entries.at({40, 40}), c.entries.at({40, 40})));
  CHECK(same(r.entries.at({144, 20}), c.entries.at({144, 20})));
  CHECK(r.entries.at({144, 20}).weak_park && !r.entries.at({40, 40}).weak_park);
  const std::string f = slurp(t.p());
  CHECK(f.rfind("maburwalls 2\n40 40 75 p ", 0) == 0);
  CHECK(f.find("\n144 20 39 w ") != std::string::npos);
}
TEST(a_version_1_file_is_nothing_measured) {
  Dir t;
  put(t.p(), "maburwalls 1\n40 40 75" + vals(25) + "\n");
  std::string note;
  CHECK(load_wall_cache(t.p(), &note).entries.empty());
  CHECK(!note.empty());
}
TEST(a_bad_flag_token_discards_the_file) {
  Dir t;
  put(t.p(), "maburwalls 2\n40 40 75 q" + vals(25) + "\n");
  std::string note;
  CHECK(load_wall_cache(t.p(), &note).entries.empty());
  CHECK(note.find("line 2") != std::string::npos);
}

TEST(saving_replaces_the_whole_file) {  // fails if save appends/merges with the old file
  Dir t;
  WallCache a;
  a.entries[{40, 40}] = mk(75, -319);
  a.entries[{144, 20}] = mk(39, -306);
  REQUIRE(save_wall_cache(t.p(), a));
  WallCache b = a;
  b.entries[{144, 20}] = mk(41, -300);
  REQUIRE(save_wall_cache(t.p(), b));
  auto r = load_wall_cache(t.p(), nullptr);
  REQUIRE(r.entries.size() == 2);
  CHECK(same(r.entries.at({40, 40}), a.entries.at({40, 40})));
  CHECK(r.entries.at({144, 20}).anchor == 41);
}

TEST(missing_file_is_empty_with_a_note) {  // fails if load returns no note for a missing file
  Dir t;
  std::string note;
  CHECK(load_wall_cache(t.p(), &note).entries.empty());
  CHECK(!note.empty());
  load_wall_cache(t.p(), nullptr);  // null note is fine
}

TEST(wrong_header) { expect_bad("maburwalls 3\n" + line("40", "40", "75", vals(25))); }  // fails if the version isn't checked
TEST(short_line) { expect_bad(H + line("40", "40", "75", vals(24))); }                   // fails if the count is >= 25
TEST(long_line) { expect_bad(H + line("40", "40", "75", vals(26))); }                    // fails if the count is <= 25
TEST(value_below_range) { expect_bad(H + line("40", "40", "75", vals(24) + " -601")); }  // fails if lower bound is dropped
TEST(value_above_range) { expect_bad(H + line("40", "40", "75", vals(24) + " 5")); }     // fails if upper bound is dropped
TEST(bad_width) { expect_bad(H + line("40", "80", "75", vals(25))); }                    // fails if width isn't 20|40
TEST(bad_channel) { expect_bad(H + line("0", "40", "75", vals(25))); }                   // fails if channel lower bound is dropped
TEST(bad_channel_high) { expect_bad(H + line("178", "40", "75", vals(25))); }            // fails if channel upper bound is dropped
TEST(bad_anchor) { expect_bad(H + line("40", "40", "128", vals(25))); }                  // fails if anchor upper bound is dropped
TEST(duplicate_entry) {                                                                  // fails if duplicates are accepted
  expect_bad(H + line("40", "40", "75", vals(25)) + line("40", "40", "76", vals(25)));
}
TEST(partial_integer_tokens) {  // fails if strtol-style partial parsing is used
  expect_bad(H + line("40", "40", "75", vals(24) + " 5x"));
  expect_bad(H + line("40", "40", "75", vals(24) + " -3.5"));
  expect_bad(H + line("40x", "40", "75", vals(25)));
}
TEST(blank_line_is_malformed) { expect_bad(H + "\n" + line("40", "40", "75", vals(25))); }  // fails if blank lines are skipped
TEST(cut_mid_line) {  // fails if a file with no trailing newline / short last line is accepted
  expect_bad(H + line("40", "40", "75", vals(25)) + "144 20 39 -306 -302");
}
TEST(valid_last_line_without_newline_is_rejected) {  // fails if the newline requirement is dropped (a cut may land on a boundary)
  std::string s = H + line("40", "40", "75", vals(25));
  s.pop_back();
  expect_bad(s);
}
TEST(header_only_is_a_valid_empty_cache) {  // fails if an empty-but-valid file reports a note
  Dir t;
  put(t.p(), H);
  std::string note = "x";
  CHECK(load_wall_cache(t.p(), &note).entries.empty());
  CHECK(note.empty());
}

TEST(oversize_file_is_rejected) {  // fails if the size cap is removed
  expect_bad(H + std::string(70 * 1024, 'a') + "\n");
}
TEST(long_digit_token_is_rejected) {  // fails if parse_int's length cap is removed (7 digits of a value in range, e.g. 0000040, would load)
  expect_bad(H + line("0000040", "40", "75", vals(25)));
}
TEST(save_refuses_out_of_range_and_keeps_old_file) {  // fails if save stops validating: one bad entry would poison the next boot
  Dir t;
  WallCache good;
  good.entries[{40, 40}] = mk(75, -319);
  REQUIRE(save_wall_cache(t.p(), good));
  const std::string before = slurp(t.p());
  WallCache bad = good;
  bad.entries[{144, 20}] = mk(39, -306);
  bad.entries[{144, 20}].evm_db[1] = -61.0f;
  CHECK(!save_wall_cache(t.p(), bad));
  bad.entries[{144, 20}].evm_db[1] = std::nanf("");
  CHECK(!save_wall_cache(t.p(), bad));
  bad.entries[{144, 20}] = mk(39, -306); bad.entries[{144, 20}].evm_db[1] = 0.5f;
  CHECK(!save_wall_cache(t.p(), bad));
  bad.entries[{144, 20}] = mk(128, -306);
  CHECK(!save_wall_cache(t.p(), bad));
  bad.entries.erase({144, 20}); bad.entries[{0, 20}] = mk(1, -306);
  CHECK(!save_wall_cache(t.p(), bad));
  bad.entries.erase({0, 20}); bad.entries[{50, 80}] = mk(1, -306);
  CHECK(!save_wall_cache(t.p(), bad));
  CHECK(slurp(t.p()) == before);
  CHECK(access((t.p() + ".tmp").c_str(), F_OK) != 0);
}

TEST(leftover_tmp_is_ignored) {  // fails if load reads <path>.tmp
  Dir t;
  WallCache c;
  c.entries[{40, 40}] = mk(75, -319);
  REQUIRE(save_wall_cache(t.p(), c));
  put(t.p() + ".tmp", "garbage");
  std::string note;
  auto r = load_wall_cache(t.p(), &note);
  CHECK(note.empty());
  CHECK(r.entries.size() == 1);
}

TEST(save_into_missing_dir_fails_cleanly) {  // fails if save ignores the open failure
  Dir t;
  const std::string p = t.d + "/nodir/mabur.walls";
  CHECK(!save_wall_cache(p, WallCache{}));
  CHECK(access((t.d + "/nodir").c_str(), F_OK) != 0);
}
MTEST_MAIN
