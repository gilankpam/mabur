#include "mtest.h"
#include "ota.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <algorithm>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

using namespace mabur;

namespace {

std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
void spit(const std::string& p, const std::string& s) { std::ofstream(p, std::ios::binary) << s; }
bool exists(const std::string& p) { return ::access(p.c_str(), F_OK) == 0; }

std::string listing(const std::string& d) {
  std::vector<std::string> v;
  if (DIR* dp = ::opendir(d.c_str())) {
    while (auto* e = ::readdir(dp)) v.push_back(e->d_name);
    ::closedir(dp);
  }
  std::sort(v.begin(), v.end());
  std::string out;
  for (auto& n : v) out += n + ",";
  return out;
}

// root/ holds up/ (the lane dir) so "the parent is unchanged" is checkable.
struct Rig {
  std::string root, dir;
  Rig() {
    static int n = 0;
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    root = std::string(MABUR_TEST_SCRATCH_DIR) + "/ota-" + std::to_string(::getpid()) + "-" + std::to_string(n++);
    ::mkdir(root.c_str(), 0777);
    dir = root + "/up";
  }
  ~Rig() {
    std::string c = "rm -rf '" + root + "'";
    if (std::system(c.c_str())) {}
  }
  OtaEnv env(bool trial = false) const {
    OtaEnv e;
    e.dir = dir;
    e.trial = trial;
    return e;
  }
};

ota::Ftp req(uint8_t op, uint8_t session = 0, uint32_t off = 0, const std::string& data = "") {
  ota::Ftp f;
  f.seq = 7;
  f.session = session;
  f.opcode = op;
  f.offset = off;
  f.data.assign(data.begin(), data.end());
  f.size = uint8_t(data.size());
  return f;
}

ota::Ftp call(OtaServer& s, const ota::Ftp& r) {
  auto out = s.on_msg(r, 0);
  REQUIRE(out.size() == 1);
  const auto* f = std::get_if<ota::Ftp>(&out[0]);
  REQUIRE(f);
  CHECK(f->seq == 8);
  CHECK(f->req_opcode == r.opcode);
  return *f;
}
int nak_code(const ota::Ftp& f) { return f.opcode == 129 && f.size == 1 && f.data.size() == 1 ? f.data[0] : -1; }

}  // namespace

TEST(create_write_close_builds_the_file) {
  Rig rg;
  OtaServer s(rg.env());
  auto c = call(s, req(6, 0, 0, "maburd"));
  CHECK(c.opcode == 128 && c.size == 0);
  CHECK(call(s, req(7, c.session, 0, "ABC")).opcode == 128);
  CHECK(call(s, req(7, c.session, 3, "DEF")).opcode == 128);
  CHECK(call(s, req(1, c.session)).opcode == 128);
  CHECK(slurp(rg.dir + "/maburd") == "ABCDEF");
  CHECK(::access((rg.dir + "/maburd").c_str(), X_OK) == 0);
}

TEST(writes_out_of_order_fill_the_file) {
  Rig rg;
  OtaServer s(rg.env());
  std::string src(300, 'a');
  for (size_t i = 0; i < src.size(); ++i) src[i] = char('a' + i % 26);
  auto c = call(s, req(6, 0, 0, std::string("maburd\0", 7)));
  CHECK(call(s, req(7, c.session, 239, src.substr(239))).opcode == 128);
  CHECK(call(s, req(7, c.session, 0, src.substr(0, 239))).opcode == 128);
  call(s, req(1, c.session));
  CHECK(slurp(rg.dir + "/maburd") == src);
}

TEST(create_truncates_and_clears_result) {
  Rig rg;
  ::mkdir(rg.dir.c_str(), 0755);
  spit(rg.dir + "/maburd", "0123456789");
  spit(rg.dir + "/result", "committed");
  OtaServer s(rg.env());
  CHECK(call(s, req(6, 0, 0, "/maburd")).opcode == 128);
  CHECK(slurp(rg.dir + "/maburd").empty());
  CHECK(!exists(rg.dir + "/result"));
}

TEST(open_wo_keeps_existing_bytes) {
  Rig rg;
  ::mkdir(rg.dir.c_str(), 0755);
  spit(rg.dir + "/maburd", "0123456789");
  OtaServer s(rg.env());
  auto c = call(s, req(11, 0, 0, "maburd"));
  CHECK(c.opcode == 128);
  CHECK(call(s, req(7, c.session, 10, "XY")).opcode == 128);
  CHECK(slurp(rg.dir + "/maburd") == "0123456789XY");
}

TEST(truncate_cuts_to_offset) {
  Rig rg;
  ::mkdir(rg.dir.c_str(), 0755);
  spit(rg.dir + "/maburd", "0123456789");
  OtaServer s(rg.env());
  CHECK(call(s, req(12, 0, 4, "maburd")).opcode == 128);
  CHECK(slurp(rg.dir + "/maburd") == "0123");
}

TEST(write_with_wrong_session_naks_invalid_session) {
  Rig rg;
  OtaServer s(rg.env());
  CHECK(nak_code(call(s, req(7, 0, 0, "x"))) == 4);  // nothing open
  call(s, req(6, 0, 0, "maburd"));
  CHECK(nak_code(call(s, req(7, 9, 0, "x"))) == 4);
}

TEST(unknown_opcode_naks_unknown_command) {
  Rig rg;
  OtaServer s(rg.env());
  CHECK(nak_code(call(s, req(3, 0, 0, "/"))) == 7);
}

TEST(paths_outside_the_lane_are_refused) {
  Rig rg;
  ::mkdir(rg.dir.c_str(), 0755);
  OtaServer s(rg.env());
  const std::string bad[] = {"../etc/mabur.toml", "/etc/shadow", "maburd/../x", "go", "proven", ""};
  for (uint8_t op : {6, 11, 12})
    for (const auto& p : bad) CHECK(nak_code(call(s, req(op, 0, 0, p))) == 9);
  CHECK(listing(rg.dir) == ".,..,");
  CHECK(listing(rg.root) == ".,..,up,");
}

TEST(create_during_trial_naks_fail) {
  Rig rg;
  ::mkdir(rg.dir.c_str(), 0755);
  spit(rg.dir + "/maburd", "keep");
  OtaServer s(rg.env(true));
  CHECK(nak_code(call(s, req(6, 0, 0, "maburd"))) == 1);
  CHECK(slurp(rg.dir + "/maburd") == "keep");
  spit(rg.dir + "/result", "committed");
  CHECK(call(s, req(6, 0, 0, "maburd")).opcode == 128);
}

TEST(non_ftp_messages_get_no_reply) {
  Rig rg;
  OtaServer s(rg.env());
  CHECK(s.on_msg(ota::IdentReq{}, 0).empty());
  CHECK(!s.exit_requested());
}

MTEST_MAIN
