// LiveUdpClient: one outstanding set/get over loopback against a real
// PlayerConfigControl + PlayerConfigApplier, the same shape maburgs'
// listener on 8402 answers with.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "live_udp_client.h"
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

Config bundle() { return load_config(kBundlePath); }

// A copy of the bundle config in its own directory, removed with the
// publisher's siblings (backup, scratch candidate).
struct ScratchPlayerBundle {
  std::string dir, file;
  ScratchPlayerBundle() {
    static int seq = 0;
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/test_live_udp_client-" +
          std::to_string(::getpid()) + "-" + std::to_string(seq++);
    REQUIRE(::mkdir(dir.c_str(), 0755) == 0);
    file = dir + "/maburplay.toml";
    const std::string t = slurp(kBundlePath);
    REQUIRE(!t.empty());
    spit(file, t);
  }
  ~ScratchPlayerBundle() {
    for (const char* s : {"", ".new", ".pre-live", ".pre-live.tmp"})
      std::remove((file + s).c_str());
    ::rmdir(dir.c_str());
  }
  const std::string& path() const { return file; }
};

// Every hook just records its call and answers ok.
struct PFake {
  PlayerApplyHooks hooks() {
    PlayerApplyHooks h;
    h.reacquire_display = [](const Config&, std::string*) { return true; };
    h.osd_layers = [](const Config&, std::string*) { return true; };
    h.next_start = [](const Config&) {};
    return h;
  }
};

// Drains the control's socket and lets its applier run, as the
// housekeeping loop would, until `done` is true or 2 s pass.
template <class Done>
void drive_until(PlayerConfigControl& ctl, PlayerConfigApplier& a, Done done) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!done()) {
    REQUIRE(std::chrono::steady_clock::now() < deadline);
    ctl.poll(a);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace

TEST(set_and_get_round_trip_over_loopback) {
  constexpr int kPort = 18414;
  ScratchPlayerBundle sb;
  PFake f;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  PlayerConfigControl ctl;
  REQUIRE(ctl.open(kPort));

  LiveUdpClient client;
  std::string err;
  REQUIRE(client.open(kPort, &err));

  REQUIRE(client.send_set("osd.gs.enable", "false", 0, &err));
  CHECK(client.in_flight());
  std::optional<LiveReply> reply;
  drive_until(ctl, a, [&] {
    reply = client.poll(1);
    return reply.has_value();
  });
  REQUIRE(reply);
  CHECK(reply->ok);
  CHECK(!client.in_flight());
  CHECK(a.live().osd.gs.enable == false);

  REQUIRE(client.send_get("osd.gs.enable", 2, &err));
  reply.reset();
  drive_until(ctl, a, [&] {
    reply = client.poll(3);
    return reply.has_value();
  });
  REQUIRE(reply);
  CHECK(reply->ok);
  CHECK(reply->text == "false");
}

TEST(a_second_send_while_one_is_in_flight_is_refused_as_busy) {
  constexpr int kPort = 18415;
  ScratchPlayerBundle sb;
  PFake f;
  PlayerConfigApplier a(sb.path(), bundle(), BuildCaps{true, false}, f.hooks(), nullptr);
  PlayerConfigControl ctl;
  REQUIRE(ctl.open(kPort));

  LiveUdpClient client;
  std::string err, why;
  REQUIRE(client.open(kPort, &err));
  REQUIRE(client.send_set("osd.gs.enable", "false", 0, &why));
  CHECK(!client.send_set("osd.enable", "false", 1, &why));
  CHECK(why == "busy");
  CHECK(!client.send_get("osd.gs.enable", 1, &why));
  CHECK(why == "busy");

  std::optional<LiveReply> reply;
  drive_until(ctl, a, [&] {
    reply = client.poll(2);
    return reply.has_value();
  });
  REQUIRE(reply);
  CHECK(reply->ok);
}

TEST(a_set_with_no_reply_times_out_and_frees_the_lock) {
  constexpr int kPort = 18417;  // nothing bound here
  LiveUdpClient client;
  std::string err, why;
  REQUIRE(client.open(kPort, &err));
  REQUIRE(client.send_set("osd.gs.enable", "false", 1000, &why));
  CHECK(client.in_flight());
  CHECK(!client.poll(1000 + LiveUdpClient::kReplyTimeoutMs - 1));
  CHECK(client.in_flight());
  std::optional<LiveReply> reply = client.poll(1000 + LiveUdpClient::kReplyTimeoutMs);
  REQUIRE(reply);
  CHECK(!reply->ok);
  CHECK(reply->text == "no answer");
  CHECK(!client.in_flight());
}

MTEST_MAIN
