#include "mtest.h"
#include "mabur/config_write.h"
#include "mabur/toml.h"
#include "scratch.h"

#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

using namespace mabur;

namespace {

std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

bool exists(const std::string& path) { return std::ifstream(path).good(); }

std::string bundle_text() {
  const std::string t = slurp(std::string(MABUR_BUNDLE_DIR) + "/mabur.default.toml");
  REQUIRE(!t.empty());
  return t;
}

void spit(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary);
  f << content;
}

// A real (if generic) stand-in for a caller's boot loader: proves
// publish_config's own plumbing -- scratch write, verify-before-publish,
// backup, rename, locking -- independently of what any one caller's
// verifier actually checks. The drone's own verifier (mabur::load_config,
// wired into drone/src/config_apply.cpp and cal_apply.cpp) is exercised by
// test_config_apply.cpp and test_cal_apply.cpp instead; this file has no
// drone dependency (LIBS mabur_common, not mabur_drone_core).
bool parses(const std::string& path, std::string* err) {
  try {
    (void)toml::parse_toml_file(path);
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

// ScratchFile unlinks only its own path; the publisher's siblings go too.
void remove_siblings(const std::string& path, const std::string& suffix) {
  std::remove((path + "." + suffix).c_str());
  std::remove((path + "." + suffix + ".tmp").c_str());
  std::remove((path + ".new").c_str());
}

}  // namespace

TEST(patches_fps_in_record_not_venc) {
  std::string t = bundle_text();
  REQUIRE(patch_toml_key(&t, "record", "fps", "30"));
  CHECK(t.find("fps        = 60            # 1..120") != std::string::npos);  // [venc] fps untouched
  CHECK(t.find("fps          = 30                 # at most venc.fps") != std::string::npos);
  CHECK(t.find("fps          = 60                 # at most venc.fps") == std::string::npos);
}

TEST(venc_section_ends_at_venc_roi) {
  std::string t = "[venc]\nslices = 4\n[venc.roi]\nenabled = false\nslices = 9\n";
  REQUIRE(patch_toml_key(&t, "venc", "slices", "2"));
  CHECK(t == "[venc]\nslices = 2\n[venc.roi]\nenabled = false\nslices = 9\n");
  CHECK(!patch_toml_key(&t, "venc", "enabled", "true"));  // lives in [venc.roi], not [venc]
  CHECK(t == "[venc]\nslices = 2\n[venc.roi]\nenabled = false\nslices = 9\n");
  REQUIRE(patch_toml_key(&t, "venc.roi", "slices", "7"));
  CHECK(t == "[venc]\nslices = 2\n[venc.roi]\nenabled = false\nslices = 7\n");
}

TEST(key_before_any_header_belongs_to_no_section) {
  std::string t = "fps = 1\n[record]\nfps = 2\n";
  REQUIRE(patch_toml_key(&t, "record", "fps", "3"));
  CHECK(t == "fps = 1\n[record]\nfps = 3\n");
}

TEST(empty_section_names_the_keys_above_every_header) {
  std::string t = "# lead\nfps = 1   # note\n[record]\nfps = 2\n";
  REQUIRE(patch_toml_key(&t, "", "fps", "3"));
  CHECK(t == "# lead\nfps = 3   # note\n[record]\nfps = 2\n");
  std::string u = "[record]\nfps = 2\n";
  CHECK(!patch_toml_key(&u, "", "fps", "3"));
  CHECK(u == "[record]\nfps = 2\n");
}

TEST(missing_section_is_not_found) {
  std::string t = "[venc]\nfps = 60\n";
  CHECK(!patch_toml_key(&t, "record", "fps", "30"));
  CHECK(t == "[venc]\nfps = 60\n");
}

TEST(preserves_spacing_and_trailing_comment) {
  std::string t = "[radio]\n  ldpc    =  true   # keep me\n";
  REQUIRE(patch_toml_key(&t, "radio", "ldpc", "false"));
  CHECK(t == "[radio]\n  ldpc    =  false   # keep me\n");
}

TEST(hash_inside_a_quoted_value_is_not_a_comment) {
  std::string t = "[record]\ndir = \"/mnt/a#b\"   # where\n";
  REQUIRE(patch_toml_key(&t, "record", "dir", "\"/mnt/c\""));
  CHECK(t == "[record]\ndir = \"/mnt/c\"   # where\n");
}

TEST(commented_out_key_is_not_found) {
  std::string t = "[msp]\n# enable = true\n";
  CHECK(!patch_toml_key(&t, "msp", "enable", "false") && t == "[msp]\n# enable = true\n");
}

TEST(prefix_key_does_not_match) {
  std::string t = "[radio]\nwall_margin_db_x = 1\nwall_margin_db = 1.0\n";
  REQUIRE(patch_toml_key(&t, "radio", "wall_margin_db", "2.0"));
  CHECK(t == "[radio]\nwall_margin_db_x = 1\nwall_margin_db = 2.0\n");
}

TEST(publish_verifies_then_backs_up_then_renames) {
  ScratchFile f("config_write", ".toml");
  const std::string p = f.path;
  const std::string original = bundle_text();
  spit(p, original);
  std::string t = original;
  REQUIRE(patch_toml_key(&t, "record", "fps", "30"));
  std::string err;
  CHECK(publish_config(p, t, "pre-param", &err, parses) == PublishResult::Ok);
  CHECK(slurp(p) == t);
  CHECK(slurp(p).find("fps          = 30") != std::string::npos);
  CHECK(slurp(p + ".pre-param") == original);
  CHECK(!exists(p + ".new"));
  CHECK(!exists(p + ".pre-param.tmp"));
  CHECK(parses(p, &err));
  CHECK(write_count_for_test() == 1);
  std::string back;
  CHECK(read_text_file(p, &back) && back == t);
  remove_siblings(p, "pre-param");
}

TEST(publish_refuses_a_candidate_the_verifier_rejects_and_leaves_the_file_untouched) {
  ScratchFile f("config_write", ".toml");
  const std::string p = f.path;
  const std::string original = bundle_text();
  spit(p, original);
  std::string t = original;
  // An unterminated string -- genuinely invalid TOML, not merely a value a
  // stricter loader happens to dislike. (That boundary is the drone's
  // load_config, verified against a real wrong-type value by
  // test_config_apply.cpp's loader_refusal_applies_nothing and
  // test_cal_apply.cpp's apply_never_publishes_a_file_that_will_not_parse.)
  REQUIRE(patch_toml_key(&t, "venc", "size", "\"1920x1080"));
  std::string err;
  CHECK(publish_config(p, t, "pre-param", &err, parses) == PublishResult::ReparseFailed);
  CHECK(!err.empty());
  CHECK(slurp(p) == original);
  CHECK(!exists(p + ".new"));
  CHECK(!exists(p + ".pre-param"));
  remove_siblings(p, "pre-param");
}

TEST(publish_of_a_missing_file_fails_without_creating_it) {
  ScratchFile f("config_write", ".toml");
  std::string err;
  const auto ok_verify = [](const std::string&, std::string*) { return true; };
  CHECK(publish_config(f.path, bundle_text(), "pre-param", &err, ok_verify) ==
        PublishResult::BackupFailed);
  CHECK(!err.empty());
  CHECK(!exists(f.path));
  CHECK(!exists(f.path + ".new"));
  remove_siblings(f.path, "pre-param");
}

TEST(publish_is_serialised_across_threads) {
  // Both threads publish onto the same path, so without the publisher's
  // own lock they share "<p>.new": one truncates the other's candidate
  // mid-verify, or renames it away so the other's rename fails. With it,
  // every publish succeeds and the file always holds a verified candidate.
  ScratchFile f("config_write", ".toml");
  const std::string p = f.path;
  const std::string original = bundle_text();
  spit(p, original);
  std::string fps30 = original, br20k = original;
  REQUIRE(patch_toml_key(&fps30, "record", "fps", "30"));
  REQUIRE(patch_toml_key(&br20k, "record", "bitrate_kbps", "20000"));
  int bad[2] = {0, 0};
  auto worker = [&](int idx, const std::string* content) {
    for (int i = 0; i < 20; ++i) {
      std::string err;
      if (publish_config(p, *content, "pre-param", &err, parses) != PublishResult::Ok) ++bad[idx];
    }
  };
  std::thread a(worker, 0, &fps30);
  std::thread b(worker, 1, &br20k);
  a.join();
  b.join();
  CHECK(bad[0] == 0);
  CHECK(bad[1] == 0);
  std::string err;
  CHECK(parses(p, &err));
  const std::string now = slurp(p);
  CHECK(now == fps30 || now == br20k);
  CHECK(!exists(p + ".new"));
  remove_siblings(p, "pre-param");
}

TEST(read_patch_publish_under_the_write_lock_loses_no_update) {
  // Each thread re-reads the file and patches its own key. Holding
  // lock_config_writes() across read -> patch -> publish keeps one thread
  // from publishing text it read before the other's publish, which would
  // put the other's key back to its old value.
  ScratchFile f("config_write", ".toml");
  const std::string p = f.path;
  spit(p, bundle_text());
  int bad[2] = {0, 0};
  auto worker = [&](int idx, const char* key, const char* value) {
    for (int i = 0; i < 20; ++i) {
      const auto lock = lock_config_writes();
      std::string t, err;
      if (!read_text_file(p, &t) || !patch_toml_key(&t, "record", key, value) ||
          publish_config(p, t, "pre-param", &err, parses) != PublishResult::Ok)
        ++bad[idx];
    }
  };
  std::thread a(worker, 0, "fps", "30");
  std::thread b(worker, 1, "bitrate_kbps", "20000");
  a.join();
  b.join();
  CHECK(bad[0] == 0);
  CHECK(bad[1] == 0);
  std::string err;
  CHECK(parses(p, &err));
  const std::string now = slurp(p);
  CHECK(now.find("fps          = 30") != std::string::npos);
  CHECK(now.find("bitrate_kbps = 20000") != std::string::npos);
  CHECK(!exists(p + ".new"));
  remove_siblings(p, "pre-param");
}

TEST(replace_toml_array_tables_replaces_the_whole_run_of_blocks) {
  std::string t = "[link]\nstatic_mcs = -1\n\n[[link.ladder]]\nmcs = 0\nbw = 40\n\n"
                  "[[link.ladder]]\nmcs = 2\nbw = 40\n\n[link.nack]\nenable = true\n";
  REQUIRE(replace_toml_array_tables(&t, "link.ladder", "[[link.ladder]]\nmcs = 5\nbw = 20\n\n"));
  CHECK(t == "[link]\nstatic_mcs = -1\n\n[[link.ladder]]\nmcs = 5\nbw = 20\n\n[link.nack]\nenable = true\n");
}
TEST(replace_toml_array_tables_false_without_a_header) {
  std::string t = "[link]\nstatic_mcs = -1\n";
  const std::string before = t;
  CHECK(!replace_toml_array_tables(&t, "link.ladder", "[[link.ladder]]\nmcs = 5\n"));
  CHECK(t == before);
}
TEST(publish_config_writes_through_a_symlink) {
  // target/live.toml + link -> target: after publish the link is still a
  // symlink, the target holds the content, the backup sits beside the target.
  ScratchFile target("test_config_write", ".toml");
  spit(target.path, "a = 1\n");
  const std::string link = target.path + ".link";
  REQUIRE(::symlink(target.path.c_str(), link.c_str()) == 0);
  std::string err;
  const auto ok_verify = [](const std::string&, std::string*) { return true; };
  CHECK(publish_config(link, "a = 2\n", "pre-live", &err, ok_verify) == PublishResult::Ok);
  struct stat st{};
  REQUIRE(::lstat(link.c_str(), &st) == 0);
  CHECK(S_ISLNK(st.st_mode));
  CHECK(slurp(target.path) == "a = 2\n");
  CHECK(slurp(target.path + ".pre-live") == "a = 1\n");
  ::unlink(link.c_str()); std::remove((target.path + ".pre-live").c_str());
}
TEST(publish_config_reparse_failed_carries_the_verifier_message) {
  ScratchFile f("test_config_write", ".toml");
  spit(f.path, "a = 1\n");
  std::string err;
  const auto bad = [](const std::string&, std::string* e) { *e = "config: x:1: a: nope"; return false; };
  CHECK(publish_config(f.path, "a = 2\n", "pre-live", &err, bad) == PublishResult::ReparseFailed);
  CHECK(err == "config: x:1: a: nope");
  CHECK(slurp(f.path) == "a = 1\n");
}

MTEST_MAIN
