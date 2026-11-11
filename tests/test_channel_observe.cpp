// ChannelObserve: holds maburgs' measuring lease open for as long as the
// Channel picker is, on a fake ObserveLink so the renewal cadence and the
// link-busy/reopen edge cases run without a socket.
#include <optional>
#include <string>
#include <vector>

#include "channel_observe.h"
#include "live_udp_client.h"
#include "mtest.h"

using maburplay::ChannelObserve;
using maburplay::LiveReply;
using maburplay::ObserveLink;

namespace {

// Records every line sent and answers whatever the test arms for the next
// poll(); one exchange in flight at a time, same shape as LiveUdpClient.
struct FakeLink : ObserveLink {
  bool busy = false;  // forced "not idle" regardless of in_flight_
  bool in_flight = false;
  std::vector<std::string> sent;
  std::optional<LiveReply> next_reply;

  bool idle() const override { return !busy && !in_flight; }

  bool send_line(const std::string& line, uint64_t, std::string* why) override {
    if (!idle()) {
      if (why) *why = "busy";
      return false;
    }
    sent.push_back(line);
    in_flight = true;
    return true;
  }

  std::optional<LiveReply> poll(uint64_t) override {
    if (!in_flight || !next_reply) return std::nullopt;
    LiveReply r = *next_reply;
    next_reply.reset();
    in_flight = false;
    return r;
  }

  void answer(bool ok, std::string text) { next_reply = LiveReply{ok, std::move(text)}; }
};

}  // namespace

TEST(open_sends_observe_3000) {
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  REQUIRE(link.sent.size() == 1);
  CHECK(link.sent[0] == "observe 3000");
}

TEST(renews_every_second_while_open) {
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  link.answer(true, "observe none");
  obs.tick(true, 500);
  CHECK(link.sent.size() == 1);  // too soon
  obs.tick(true, 1000);
  REQUIRE(link.sent.size() == 2);
  CHECK(link.sent[1] == "observe 3000");
  link.answer(true, "observe none");
  obs.tick(true, 1500);
  CHECK(link.sent.size() == 2);
  obs.tick(true, 2000);
  REQUIRE(link.sent.size() == 3);
  CHECK(link.sent[2] == "observe 3000");
}

TEST(close_sends_observe_0_and_clears_state) {
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  link.answer(true, "observe one_card");
  obs.tick(true, 100);
  CHECK(obs.state() == "one_card");
  obs.tick(false, 200);
  REQUIRE(link.sent.size() == 2);
  CHECK(link.sent[1] == "observe 0");
  CHECK(obs.state().empty());
}

TEST(menu_closing_with_picker_open_releases) {
  // The whole menu can close (bank.poll's own escape gesture, menu_session.h)
  // while the Channel picker is still open on it; from here that looks
  // exactly like the picker being cancelled -- open_choices_row() reads ""
  // either way, so the same tick(false, ...) call covers both.
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  link.answer(true, "observe one_card");
  obs.tick(true, 50);  // frees the link
  obs.tick(false, 100);
  REQUIRE(link.sent.size() == 2);
  CHECK(link.sent[1] == "observe 0");
}

TEST(reply_sets_state) {
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  link.answer(true, "observe one_card");
  obs.tick(true, 10);
  CHECK(obs.state() == "one_card");
}

TEST(renewal_waits_for_the_link) {
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  link.answer(true, "observe none");
  obs.tick(true, 999);  // consumes the open reply, no send due yet
  link.busy = true;     // the link is tied up elsewhere at the renew mark
  obs.tick(true, 1000);
  CHECK(link.sent.size() == 1);  // refused: still busy
  link.busy = false;
  obs.tick(true, 1200);
  REQUIRE(link.sent.size() == 2);
  CHECK(link.sent[1] == "observe 3000");
}

TEST(reopen_while_release_in_flight_ends_observing) {
  FakeLink link;
  ChannelObserve obs(link);
  obs.tick(true, 0);
  link.answer(true, "observe one_card");
  obs.tick(true, 10);
  obs.tick(false, 20);  // "observe 0" goes out, now in flight
  REQUIRE(link.sent.size() == 2);
  CHECK(link.sent[1] == "observe 0");
  obs.tick(true, 30);  // reopen before that reply ever lands
  CHECK(link.sent.size() == 2);  // the link is still busy with "observe 0"
  link.answer(true, "observe none");
  obs.tick(true, 40);  // the reply frees the link; the next send is the reopen's
  REQUIRE(link.sent.size() == 3);
  CHECK(link.sent[2] == "observe 3000");
}

MTEST_MAIN
