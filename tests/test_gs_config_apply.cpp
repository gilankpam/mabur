// GsConfigApplier + GsConfigControl: maburgs's live-key table, the loopback
// set/get parser and the apply pipeline, driven through fake GsApplyHooks
// against a real copy of the bundle config, the real loader and the real
// publisher.
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "config.h"
#include "gs_config_apply.h"
#include "mtest.h"
#include "scratch.h"

using namespace maburgs;

namespace {

const std::string kBundlePath = MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml";

std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void spit(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary);
  f << content;
}

bool exists(const std::string& path) { return std::ifstream(path).good(); }

std::string dirname_of(const std::string& path) { return path.substr(0, path.rfind('/')); }

std::string bundle_text() {
  const std::string t = slurp(kBundlePath);
  REQUIRE(!t.empty());
  return t;
}

Config bundle() { return load_config(kBundlePath); }

Config load_config_from_text(const std::string& text) {
  ScratchFile f("test_gs_config_apply", ".toml");
  spit(f.path, text);
  return load_config(f.path);
}

// A copy of the bundle config in its own directory, so a test can make the
// directory read-only without touching anything shared. Removes the
// publisher's siblings (backup, scratch candidate) with it.
struct ScratchBundle {
  std::string dir, file;
  ScratchBundle() {
    static int seq = 0;
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/test_gs_config_apply-" +
          std::to_string(::getpid()) + "-" + std::to_string(seq++);
    REQUIRE(::mkdir(dir.c_str(), 0755) == 0);
    file = dir + "/maburgs.toml";
    spit(file, bundle_text());
  }
  ~ScratchBundle() {
    ::chmod(dir.c_str(), 0755);
    for (const char* s : {"", ".new", ".pre-live", ".pre-live.tmp"})
      std::remove((file + s).c_str());
    ::rmdir(dir.c_str());
  }
  const std::string& path() const { return file; }
};

// Every hook records its call: `order` names the hooks in call order, and
// the per-hook vectors keep their arguments.
struct Fake {
  std::vector<std::string> order;
  std::vector<uint8_t> set_pin_calls;
  int set_auto_calls = 0;
  std::vector<int> set_static_mcs;
  std::vector<size_t> ladder_sizes;
  std::vector<bool> nack_calls, debug_calls, msp_calls;
  bool busy = false;                         // hop_or_cal_busy's answer
  std::optional<std::string> pin_refusal;    // set_pin's answer
  bool debug_ok = true;
  std::function<void()> on_nack;

  GsApplyHooks hooks() {
    GsApplyHooks h;
    h.set_pin = [this](uint8_t ch) {
      order.push_back("set_pin");
      set_pin_calls.push_back(ch);
      return pin_refusal;
    };
    h.set_auto = [this] {
      order.push_back("set_auto");
      ++set_auto_calls;
    };
    h.set_static = [this](const Config& c) {
      order.push_back("set_static");
      set_static_mcs.push_back(c.link.static_mcs);
    };
    h.replace_ladder = [this](const Config& c) {
      order.push_back("replace_ladder");
      ladder_sizes.push_back(c.link.ladder_cfg.ladder.size());
    };
    h.nack = [this](bool on) {
      order.push_back("nack");
      nack_calls.push_back(on);
      if (on_nack) on_nack();
    };
    h.debug_log = [this](bool on, std::string* err) {
      order.push_back("debug_log");
      debug_calls.push_back(on);
      if (!debug_ok) *err = "log root not writable";
      return debug_ok;
    };
    h.msp = [this](bool on, const Config&) {
      order.push_back("msp");
      msp_calls.push_back(on);
    };
    h.hop_or_cal_busy = [this] { return busy; };
    return h;
  }
};

// Polls the applier as the owning thread's tick would until it hands out a
// result; a 2 s deadline turns a hang into a failure.
GsApplyResult drive(GsConfigApplier& a) {
  GsApplyResult r{};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!a.poll(&r)) {
    REQUIRE(std::chrono::steady_clock::now() < deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  REQUIRE(!a.busy());
  return r;
}

}  // namespace

TEST(live_table_has_the_nine_keys_in_get_order) {
  REQUIRE(kGsLiveKeys.size() == 9);
  CHECK(std::string(kGsLiveKeys[0].key) == "radio.channel");
  CHECK(std::string(kGsLiveKeys[5].key) == "link.ladder");
  CHECK(gs_live_key("radio.width") == nullptr);
}

TEST(render_live_value_renders_each_type) {
  Config c = bundle();
  CHECK(render_live_value(c, *gs_live_key("link.static_mcs")) == "-1");
  CHECK(render_live_value(c, *gs_live_key("link.nack.enable")) == "true");
  CHECK(render_live_value(c, *gs_live_key("radio.channel")) == "\"auto\"");
  c.radio.pin = 112;
  CHECK(render_live_value(c, *gs_live_key("radio.channel")) == "112");
  const std::string l = render_live_value(c, *gs_live_key("link.ladder"));
  CHECK(l.rfind("[{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}", 0) == 0);
  CHECK(render_live_value(c, *gs_live_key("link.static_overhead_base")) == "0.5");
  c.link.static_overhead_base = 2.0;
  CHECK(render_live_value(c, *gs_live_key("link.static_overhead_base")) == "2.0");
}

TEST(patch_live_key_rewrites_the_ladder_region) {
  std::string t = bundle_text();
  std::string why;
  REQUIRE(patch_live_key(&t, *gs_live_key("link.ladder"),
                         "[{mcs = 0, bw = 20, overhead_base = 0.5, overhead_enh = 0.25}]", &why));
  const Config c = load_config_from_text(t);
  REQUIRE(c.link.ladder_cfg.ladder.size() == 1);
  CHECK(c.link.ladder_cfg.ladder[0].bw == 20);
  CHECK(c.link.nack.enable == bundle().link.nack.enable);   // the table after the region survived
  // The comment above [link.probe] belongs to that table and is kept.
  CHECK(t.find("# step happens only if they arrive clean.\n[link.probe]") != std::string::npos);
  CHECK(t.find("[[link.ladder]]\nmcs = 0\nbw  = 20\noverhead_base = 0.5\noverhead_enh  = 0.25\n") !=
        std::string::npos);
}

TEST(patch_live_key_refuses_a_ladder_with_a_foreign_field) {
  std::string t = bundle_text(); std::string why;
  CHECK(!patch_live_key(&t, *gs_live_key("link.ladder"), "[{mcs = 0, bw = 20, foo = 1}]", &why));
  CHECK(why.find("foo") != std::string::npos);
  CHECK(t == bundle_text());
}

TEST(patch_live_key_refuses_malformed_ladders) {
  const std::string orig = bundle_text();
  for (const char* v : {"[]", "{mcs = 0}", "[{mcs = 0, bw = 20, overhead_base = 0.5}]",
                        "[{mcs = 0, bw = 20, overhead_base = 0.5, overhead_enh = 0.25} x]",
                        "[{mcs = 0, bw = 20, overhead_base = 0.5, overhead_enh = 0.25, mcs = 1}]",
                        "[{mcs = 0, bw = 20, overhead_base = 0.5, overhead_enh = {a = 1}}]"}) {
    std::string t = orig, why;
    CHECK(!patch_live_key(&t, *gs_live_key("link.ladder"), v, &why));
    CHECK(!why.empty());
    CHECK(t == orig);
  }
}

TEST(applier_set_static_mcs_applies_then_publishes) {
  ScratchBundle sb; Fake f;
  std::vector<std::string> lines;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(),
                    [&](const std::string& l) { lines.push_back(l); });
  std::string why;
  REQUIRE(a.submit("link.static_mcs", "3", &why));
  GsApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(r.key == "link.static_mcs");
  CHECK(f.order == std::vector<std::string>{"set_static"});
  CHECK(f.set_static_mcs == std::vector<int>{3});
  CHECK(a.live().link.static_mcs == 3);
  CHECK(slurp(sb.path()).find("static_mcs             = 3 ") != std::string::npos);
  CHECK(exists(sb.path() + ".pre-live"));
  CHECK(!exists(sb.path() + ".new"));
  REQUIRE(lines.size() == 1);
  CHECK(lines[0].rfind("maburgs: cfg set link.static_mcs = 3 -> ok (", 0) == 0);
  CHECK(lines[0].size() > 4 && lines[0].compare(lines[0].size() - 4, 4, " ms)") == 0);
}

TEST(applier_writes_the_backup_once_per_run) {
  ScratchBundle sb; Fake f;
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("link.static_mcs", "3", &why));
  CHECK(drive(a).ok);
  REQUIRE(a.submit("link.static_mcs", "4", &why));
  CHECK(drive(a).ok);
  CHECK(slurp(sb.path() + ".pre-live") == before);
  CHECK(a.live().link.static_mcs == 4);
}

TEST(applier_refuses_wrong_type_and_leaves_file_untouched) {
  ScratchBundle sb; Fake f;
  const std::string before = slurp(sb.path());
  std::vector<std::string> lines;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(),
                    [&](const std::string& l) { lines.push_back(l); });
  std::string why;
  REQUIRE(a.submit("link.static_mcs", "\"3\"", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("static_mcs") != std::string::npos);
  CHECK(r.reason.find(sb.path()) == std::string::npos);
  CHECK(f.order.empty());
  CHECK(slurp(sb.path()) == before);
  CHECK(!exists(sb.path() + ".pre-live"));
  CHECK(!exists(sb.path() + ".new"));
  REQUIRE(lines.size() == 1);
  CHECK(lines[0].rfind("maburgs: cfg set link.static_mcs = \"3\" -> err " + r.reason + " (", 0) == 0);
}

TEST(applier_refuses_a_bool_given_an_int) {
  ScratchBundle sb; Fake f;
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("link.nack.enable", "1", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("enable") != std::string::npos);
  CHECK(f.order.empty());
  CHECK(slurp(sb.path()) == before);
}

TEST(applier_not_live_key_is_refused_at_submit) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  CHECK(!a.submit("radio.width", "20", &why));
  CHECK(why == "not live: radio.width");
  CHECK(!a.busy());
}

TEST(applier_refuses_a_value_with_a_line_break) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  CHECK(!a.submit("link.static_mcs", "3\nstatic_bw = 40", &why));
  CHECK(!why.empty());
  CHECK(!a.busy());
}

TEST(applier_same_value_is_ok_without_file_work) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("link.static_mcs", "-1", &why));
  GsApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(!exists(sb.path() + ".pre-live"));
  CHECK(f.order.empty());
}

TEST(applier_busy_while_one_is_in_flight) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("link.static_mcs", "3", &why));
  CHECK(a.busy());
  CHECK(!a.submit("link.static_mcs", "4", &why));
  CHECK(why == "busy");
  CHECK(drive(a).ok);
  CHECK(a.live().link.static_mcs == 3);
}

TEST(applier_publish_failure_reverts_the_process) {
  if (::geteuid() == 0) {
    std::printf("skip applier_publish_failure_reverts_the_process: root ignores a 0555 directory\n");
    return;
  }
  ScratchBundle sb; Fake f;
  const std::string before = slurp(sb.path());
  // The applier has read the file and loaded its candidate by the time it
  // applies; read-only from here, so only the publish fails.
  f.on_nack = [&] {
    if (f.nack_calls.size() == 1) ::chmod(dirname_of(sb.path()).c_str(), 0555);
  };
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("link.nack.enable", "false", &why));
  GsApplyResult r = drive(a);
  ::chmod(dirname_of(sb.path()).c_str(), 0755);
  CHECK(!r.ok);
  CHECK(r.reason.find("publish failed") != std::string::npos);
  CHECK(a.live().link.nack.enable == true);
  CHECK((f.nack_calls == std::vector<bool>{false, true}));   // applied, then re-applied
  CHECK(slurp(sb.path()) == before);
}

TEST(applier_hook_refusal_publishes_nothing) {
  ScratchBundle sb; Fake f; f.debug_ok = false;
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("debug_log.enable", "true", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("log root not writable") != std::string::npos);
  CHECK(f.debug_calls == std::vector<bool>{true});
  CHECK(a.live().debug_log.enable == false);
  CHECK(slurp(sb.path()) == before);
  CHECK(!exists(sb.path() + ".pre-live"));
}

TEST(applier_channel_pin_checks_preconditions_before_any_file_work) {
  ScratchBundle sb; Fake f; f.busy = true;
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("radio.channel", "112", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason == "hop or calibration in flight");
  CHECK(slurp(sb.path()) == before);
  CHECK(f.set_pin_calls.empty());
}

TEST(applier_channel_pin_publishes_then_orders) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("radio.channel", "112", &why));
  GsApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(f.set_pin_calls == std::vector<uint8_t>{112});
  CHECK(slurp(sb.path()).find("channel  = 112") != std::string::npos);
  CHECK((a.live().radio.pin == std::optional<uint8_t>(112)));
  REQUIRE(a.submit("radio.channel", "\"auto\"", &why));
  r = drive(a);
  CHECK(r.ok);
  CHECK(f.set_auto_calls == 1);
  CHECK(!a.live().radio.pin.has_value());
  CHECK(slurp(sb.path()).find("channel  = \"auto\"") != std::string::npos);
}

TEST(applier_channel_pin_refused_after_publish_restores_the_file) {
  ScratchBundle sb; Fake f; f.pin_refusal = "hop in flight";
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("radio.channel", "112", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason == "hop in flight");
  CHECK(f.set_pin_calls == std::vector<uint8_t>{112});
  CHECK(slurp(sb.path()) == before);
  CHECK(!a.live().radio.pin.has_value());
}

TEST(applier_channel_refuses_a_non_member) {
  ScratchBundle sb; Fake f;
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("radio.channel", "100", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("radio.channel") != std::string::npos);
  CHECK(f.set_pin_calls.empty());
  CHECK(slurp(sb.path()) == before);
}

TEST(applier_ladder_applies_then_publishes) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  const std::string v =
      "[{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, "
      "{mcs = 2, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}]";
  std::string why;
  REQUIRE(a.submit("link.ladder", v, &why));
  GsApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(f.ladder_sizes == std::vector<size_t>{2});
  CHECK(a.get("link.ladder", &why) == v);
  CHECK(load_config(sb.path()).link.ladder_cfg.ladder.size() == 2);
}

TEST(applier_ladder_refuses_a_rung_above_max_mcs) {
  ScratchBundle sb; Fake f;    // bundle max_mcs = 5
  const std::string before = slurp(sb.path());
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("link.ladder", "[{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, {mcs = 7, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}]", &why));
  GsApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason == "rung 1 exceeds link.max_mcs");
  CHECK(f.order.empty());
  CHECK(slurp(sb.path()) == before);
}

TEST(control_handle_get_set_and_errors) {
  ScratchBundle sb; Fake f;
  GsConfigApplier a(sb.path(), bundle(), f.hooks(), nullptr);
  bool pending = false;
  CHECK(GsConfigControl::handle("get link.static_mcs", a, &pending) == "ok -1");
  CHECK(GsConfigControl::handle("get radio.width", a, &pending) == "err not live: radio.width");
  CHECK(GsConfigControl::handle("set radio.width = 20", a, &pending) == "err not live: radio.width");
  CHECK(GsConfigControl::handle("bogus", a, &pending) == "err usage: set <key> = <value> | get [<key>]");
  CHECK(GsConfigControl::handle("set link.static_mcs", a, &pending) == "err usage: set <key> = <value> | get [<key>]");
  CHECK(GsConfigControl::handle("", a, &pending) == "err usage: set <key> = <value> | get [<key>]");
  CHECK(!pending);
  const std::string all = GsConfigControl::handle("get", a, &pending);
  CHECK(all.rfind("ok\nradio.channel = \"auto\"\n", 0) == 0);
  CHECK(all.find("\nmsp.enable = true\n") != std::string::npos);
  CHECK(GsConfigControl::handle("set link.static_mcs = 2\n", a, &pending).empty());
  CHECK(pending);
  pending = false;
  CHECK(GsConfigControl::handle("set link.static_mcs = 3", a, &pending) == "err busy");
  CHECK(!pending);
  drive(a);
  CHECK(GsConfigControl::handle("get link.static_mcs", a, &pending) == "ok 2");
}

MTEST_MAIN
