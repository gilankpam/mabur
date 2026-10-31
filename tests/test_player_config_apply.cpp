// PlayerConfigApplier + PlayerConfigControl: maburplay's live-key table,
// the loopback set/get parser and the apply pipeline, driven through fake
// PlayerApplyHooks against a real copy of the bundle config, the real
// loader and the real publisher.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mtest.h"
#include "player_config.h"
#include "player_config_apply.h"

using namespace maburplay;

namespace {

const std::string kBundlePath = MABUR_PLAY_BUNDLE_DIR "/maburplay.default.toml";

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

std::string bundle_text() {
  const std::string t = slurp(kBundlePath);
  REQUIRE(!t.empty());
  return t;
}

Config bundle() { return load_config(kBundlePath); }

// Replaces the first `from` in `text` with `to`; the edit must land.
std::string edited(std::string text, const std::string& from, const std::string& to) {
  const size_t at = text.find(from);
  REQUIRE(at != std::string::npos);
  return text.replace(at, from.size(), to);
}

// A copy of the bundle config (or of `text`) in its own directory, removed
// with the publisher's siblings (backup, scratch candidate).
struct ScratchPlayerBundle {
  std::string dir, file;
  explicit ScratchPlayerBundle(const std::string& text = bundle_text()) {
    static int seq = 0;
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/test_player_config_apply-" +
          std::to_string(::getpid()) + "-" + std::to_string(seq++);
    REQUIRE(::mkdir(dir.c_str(), 0755) == 0);
    file = dir + "/maburplay.toml";
    spit(file, text);
  }
  ~ScratchPlayerBundle() {
    for (const char* s : {"", ".new", ".pre-live", ".pre-live.tmp"})
      std::remove((file + s).c_str());
    ::rmdir(dir.c_str());
  }
  const std::string& path() const { return file; }
};

// Every hook records its call and the config it was handed.
struct PFake {
  std::vector<Config> reacquire_calls;
  std::vector<Config> osd_calls;
  int next_start_calls = 0;
  bool reacquire_ok = true;
  std::string reacquire_err;

  PlayerApplyHooks hooks() {
    PlayerApplyHooks h;
    h.reacquire_display = [this](const Config& c, std::string* err) {
      reacquire_calls.push_back(c);
      if (!reacquire_ok) *err = reacquire_err;
      return reacquire_ok;
    };
    h.osd_layers = [this](const Config& c, std::string*) {
      osd_calls.push_back(c);
      return true;
    };
    h.next_start = [this](const Config&) { ++next_start_calls; };
    return h;
  }
};

// Polls the applier as the housekeeping tick would until it hands out a
// result; a 2 s deadline turns a hang into a failure.
PlayerApplyResult drive(PlayerConfigApplier& a) {
  PlayerApplyResult r{};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!a.poll(&r)) {
    REQUIRE(std::chrono::steady_clock::now() < deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  REQUIRE(!a.busy());
  return r;
}

}  // namespace

TEST(player_live_table_has_nine_keys) {
  REQUIRE(kPlayerLiveKeys.size() == 9);
  CHECK(std::string(kPlayerLiveKeys[0].key) == "screen_mode");
  CHECK(std::string(kPlayerLiveKeys[8].key) == "dvr.burned.size");
  CHECK(player_live_key("backend") == nullptr);
}

TEST(render_player_live_value_renders_each_type) {
  Config c = bundle();
  CHECK(render_player_live_value(c, *player_live_key("screen_mode")) == "\"1920x1080@60\"");
  CHECK(render_player_live_value(c, *player_live_key("osd.gs.enable")) == "true");
  CHECK(render_player_live_value(c, *player_live_key("dvr.burned.bitrate_kbps")) == "8000");
  CHECK(render_player_live_value(c, *player_live_key("dvr.burned.size")) == "\"\"");
  c.dvr.burned.size_w = 1280;
  c.dvr.burned.size_h = 720;
  CHECK(render_player_live_value(c, *player_live_key("dvr.burned.size")) == "\"1280x720\"");
}

TEST(player_apply_rule_presenter_only_when_the_osd_plane_want_changes) {
  Config b = bundle();
  Config a = b;  // bundle: osd.enable true, osd.gs.enable true
  REQUIRE(b.osd.enable && b.osd.gs.enable);
  a.osd.enable = false;
  CHECK(player_apply_rule(*player_live_key("osd.enable"), b, a) == PlayerApplyRule::OsdLayers);
  a.osd.gs.enable = false;
  CHECK(player_apply_rule(*player_live_key("osd.gs.enable"), b, a) == PlayerApplyRule::Presenter);
  CHECK(player_apply_rule(*player_live_key("screen_mode"), b, a) == PlayerApplyRule::Presenter);
  CHECK(player_apply_rule(*player_live_key("colortrans.enable"), b, a) == PlayerApplyRule::Presenter);
  CHECK(player_apply_rule(*player_live_key("dvr.burned.every"), b, a) == PlayerApplyRule::NextStart);
}

TEST(player_applier_screen_mode_reacquires_then_publishes) {
  ScratchPlayerBundle sb;
  PFake f;
  std::vector<std::string> lines;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(),
                        [&](const std::string& l) { lines.push_back(l); });
  std::string why;
  REQUIRE(a.submit("screen_mode", "\"2560x1440@60\"", &why));
  PlayerApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(r.key == "screen_mode");
  REQUIRE(f.reacquire_calls.size() == 1);
  CHECK(f.reacquire_calls[0].screen_mode == "2560x1440@60");
  // Only the set key differs from the live config the hook is handed.
  Config want = bundle();
  want.screen_mode = "2560x1440@60";
  CHECK(f.reacquire_calls[0].dvr.mode == want.dvr.mode);
  CHECK(f.osd_calls.empty());
  CHECK(a.live().screen_mode == "2560x1440@60");
  CHECK(slurp(sb.path()).find("screen_mode = \"2560x1440@60\"") != std::string::npos);
  CHECK(exists(sb.path() + ".pre-live"));
  CHECK(!exists(sb.path() + ".new"));
  REQUIRE(lines.size() == 1);
  CHECK(lines[0].rfind("maburplay: cfg set screen_mode = \"2560x1440@60\" -> ok (", 0) == 0);
}

TEST(player_applier_reacquire_failure_publishes_nothing) {
  ScratchPlayerBundle sb;
  PFake f;
  f.reacquire_ok = false;
  f.reacquire_err = "mode not offered";
  const std::string before = slurp(sb.path());
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("screen_mode", "\"640x480@60\"", &why));
  PlayerApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason == "mode not offered");
  CHECK(f.reacquire_calls.size() == 1);
  CHECK(slurp(sb.path()) == before);
  CHECK(!exists(sb.path() + ".pre-live"));
  CHECK(a.live().screen_mode == bundle().screen_mode);
}

TEST(player_applier_burned_refused_without_mpp) {
  // The Orange Pi: v4l2req decode, no MPP encoder.
  const std::string text = edited(edited(bundle_text(), "backend     = \"mpp\"", "backend     = \"v4l2req\""),
                                  "mode        = \"burned\"", "mode        = \"raw\"");
  ScratchPlayerBundle sb(text);
  PFake f;
  const Config live = load_config(sb.path());
  REQUIRE(unsupported_on_build(live, BuildCaps{false, true}).empty());
  const std::string before = slurp(sb.path());
  PlayerConfigApplier a(sb.path(), live, BuildCaps{false, true}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("dvr.mode", "\"burned\"", &why));
  PlayerApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("needs the MPP encoder") != std::string::npos);
  CHECK(f.next_start_calls == 0);
  CHECK(slurp(sb.path()) == before);
  CHECK(a.live().dvr.mode == "raw");
  // The same build takes a set that leaves it runnable.
  REQUIRE(a.submit("dvr.target", "\"vtx\"", &why));
  CHECK(drive(a).ok);
}

TEST(player_applier_refuses_wrong_type) {
  ScratchPlayerBundle sb;
  PFake f;
  const std::string before = slurp(sb.path());
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("dvr.burned.every", "\"2\"", &why));
  PlayerApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("every") != std::string::npos);
  CHECK(r.reason.find(sb.path()) == std::string::npos);
  CHECK(f.next_start_calls == 0);
  CHECK(slurp(sb.path()) == before);
  CHECK(!exists(sb.path() + ".new"));
}

TEST(player_applier_refuses_out_of_range) {
  ScratchPlayerBundle sb;
  PFake f;
  const std::string before = slurp(sb.path());
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("dvr.burned.size", "\"1281x720\"", &why));
  PlayerApplyResult r = drive(a);
  CHECK(!r.ok);
  CHECK(r.reason.find("dvr.burned.size") != std::string::npos);
  CHECK(slurp(sb.path()) == before);
}

TEST(player_applier_next_start_keys_apply_without_a_park) {
  ScratchPlayerBundle sb;
  PFake f;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("dvr.target", "\"both\"", &why));
  PlayerApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(f.reacquire_calls.empty());
  CHECK(f.next_start_calls == 1);
  CHECK(a.live().dvr.target == "both");
  CHECK(load_config(sb.path()).dvr.target == "both");
}

TEST(player_applier_osd_switch_keeping_the_plane_opens_layers_in_place) {
  ScratchPlayerBundle sb;
  PFake f;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("osd.enable", "false", &why));
  CHECK(drive(a).ok);
  CHECK(f.reacquire_calls.empty());
  REQUIRE(f.osd_calls.size() == 1);
  CHECK(!f.osd_calls[0].osd.enable && f.osd_calls[0].osd.gs.enable);
  // The last overlay off drops the OSD plane: the presenter is rebuilt.
  REQUIRE(a.submit("osd.gs.enable", "false", &why));
  CHECK(drive(a).ok);
  REQUIRE(f.reacquire_calls.size() == 1);
  CHECK(!f.reacquire_calls[0].osd.enable && !f.reacquire_calls[0].osd.gs.enable);
  CHECK(f.osd_calls.size() == 1);
}

TEST(player_applier_refuses_an_unset_hook) {
  // An empty std::function throws bad_function_call on the housekeeping
  // thread at the first set that reaches it; the constructor says so first.
  ScratchPlayerBundle sb;
  PFake f;
  for (int missing = 0; missing < 3; ++missing) {
    PlayerApplyHooks h = f.hooks();
    if (missing == 0) h.reacquire_display = nullptr;
    if (missing == 1) h.osd_layers = nullptr;
    if (missing == 2) h.next_start = nullptr;
    bool threw = false;
    try {
      PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, h, nullptr);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  }
}

TEST(rec_plan_reads_target_and_mode) {
  DvrCfg d;
  d.target = "gs";
  d.mode = "raw";
  RecPlan p = rec_plan(d);
  CHECK(p.gs && !p.vtx && !p.burned);
  d.target = "vtx";
  p = rec_plan(d);
  CHECK(!p.gs && p.vtx);
  d.target = "both";
  d.mode = "burned";
  p = rec_plan(d);
  CHECK(p.gs && p.vtx && p.burned);
  CHECK(rec_plan(d) == p);
  d.target = "gs";
  CHECK(!(rec_plan(d) == p));
}

// The burned recorder's settings are latched with the plan: a restart
// inside a recording starts from these, not from a dvr.burned.* set since.
TEST(rec_plan_latches_the_burned_settings) {
  DvrCfg d;
  d.mode = "burned";
  d.burned.bitrate_kbps = 8000;
  d.burned.every = 2;
  d.burned.size_w = 1280;
  d.burned.size_h = 720;
  const RecPlan p = rec_plan(d);
  CHECK(p.burn.bitrate_kbps == 8000);
  CHECK(p.burn.every == 2);
  CHECK(p.burn.size_w == 1280 && p.burn.size_h == 720);
  d.burned.bitrate_kbps = 9000;
  CHECK(!(rec_plan(d) == p));
}

TEST(player_applier_same_value_spelled_differently_writes_nothing) {
  ScratchPlayerBundle sb;
  PFake f;
  const std::string before = slurp(sb.path());
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  std::string why;
  REQUIRE(a.submit("dvr.burned.every", "01", &why));   // live: 1
  PlayerApplyResult r = drive(a);
  CHECK(r.ok);
  CHECK(f.next_start_calls == 0);
  CHECK(slurp(sb.path()) == before);
  CHECK(!exists(sb.path() + ".pre-live"));
}

TEST(player_control_handle) {
  ScratchPlayerBundle sb;
  PFake f;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  const std::string usage = "err usage: set <key> = <value> | get [<key>]";
  bool pending = false;
  CHECK(PlayerConfigControl::handle("get screen_mode", a, &pending) == "ok \"1920x1080@60\"");
  CHECK(PlayerConfigControl::handle("get backend", a, &pending) == "err not live: backend");
  CHECK(PlayerConfigControl::handle("set backend = \"null\"", a, &pending) == "err not live: backend");
  CHECK(PlayerConfigControl::handle("bogus", a, &pending) == usage);
  CHECK(PlayerConfigControl::handle("set screen_mode", a, &pending) == usage);
  CHECK(PlayerConfigControl::handle("", a, &pending) == usage);
  CHECK(!pending);
  const std::string all = PlayerConfigControl::handle("get", a, &pending);
  CHECK(all.rfind("ok\nscreen_mode = \"1920x1080@60\"\ncolortrans.enable = true\n", 0) == 0);
  CHECK(all.find("\ndvr.burned.size = \"\"\n") != std::string::npos);
  CHECK(PlayerConfigControl::handle("set dvr.burned.every = 2\n", a, &pending).empty());
  CHECK(pending);
  pending = false;
  CHECK(PlayerConfigControl::handle("set dvr.burned.every = 3", a, &pending) == "err busy");
  CHECK(!pending);
  drive(a);
  CHECK(PlayerConfigControl::handle("get dvr.burned.every", a, &pending) == "ok 2");
}

TEST(player_control_answers_an_oversized_datagram_with_an_error) {
  constexpr int kPort = 18413;   // not 8403: a running maburplay may own it
  ScratchPlayerBundle sb;
  PFake f;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  PlayerConfigControl ctl;
  REQUIRE(ctl.open(kPort));
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(kPort);
  REQUIRE(inet_pton(AF_INET, "127.0.0.1", &to.sin_addr) == 1);
  auto ask = [&](const std::string& line) {
    REQUIRE(::sendto(fd, line.data(), line.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to)) ==
            static_cast<ssize_t>(line.size()));
    std::string reply;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (reply.empty()) {
      REQUIRE(std::chrono::steady_clock::now() < deadline);
      ctl.poll(a);
      pollfd p{fd, POLLIN, 0};
      if (::poll(&p, 1, 1) == 1) {
        char buf[256];
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        REQUIRE(n > 0);
        reply.assign(buf, static_cast<size_t>(n));
      }
    }
    return reply;
  };
  // A set whose tail lies past the receive buffer must not be applied as
  // the truncated head.
  std::string big = "set dvr.burned.every = 2 #";
  big.append(4000, 'x');
  const std::string reply = ask(big);
  CHECK(reply.rfind("err ", 0) == 0);
  CHECK(reply.find("too long") != std::string::npos);
  CHECK(!a.busy());
  CHECK(ask("get dvr.burned.every") == "ok 1");
  ::close(fd);
}

MTEST_MAIN
