// MenuSession: button events in, sets out through the MenuApplier, and the
// GS live view refreshed over the same one-exchange-at-a-time client the
// GS setter uses -- never while one of a row's steps is in flight there.
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gs_live_view.h"
#include "live_udp_client.h"
#include "menu_apply.h"
#include "menu_model.h"
#include "menu_session.h"
#include "mtest.h"

using maburplay::Button;
using maburplay::GsLiveView;
using maburplay::GsViewSource;
using maburplay::KeyEvent;
using maburplay::KeyKind;
using maburplay::LiveReply;
using maburplay::LiveSetter;
using maburplay::MenuApplier;
using maburplay::MenuModel;
using maburplay::MenuNode;
using maburplay::MenuSession;
using maburplay::NodeKind;
using maburplay::Owner;
using maburplay::SetStep;
using maburplay::ViewRow;

namespace {

// maburgs' live-config client as both the GS setter and the view refresh
// see it: one exchange in flight, a second send refused "busy".
struct FakeLink {
  bool in_flight = false;
  bool busy_seen = false;
  std::vector<std::string> sent;
  std::optional<LiveReply> reply;

  bool send(std::string line, std::string* why) {
    if (in_flight) {
      busy_seen = true;
      if (why) *why = "busy";
      return false;
    }
    sent.push_back(std::move(line));
    in_flight = true;
    return true;
  }
  std::optional<LiveReply> poll() {
    if (!in_flight || !reply) return std::nullopt;
    LiveReply r = *reply;
    reply.reset();
    in_flight = false;
    return r;
  }
  void answer(bool ok, std::string text) { reply = LiveReply{ok, std::move(text)}; }
};

struct FakeGsSetter : LiveSetter {
  FakeLink& link;
  explicit FakeGsSetter(FakeLink& l) : link(l) {}
  bool submit(const std::string& key, const std::string& value, uint64_t, std::string* why) override {
    return link.send("set " + key + " = " + value, why);
  }
  std::optional<LiveReply> poll(uint64_t) override { return link.poll(); }
};

struct FakeGsView : GsViewSource {
  FakeLink& link;
  explicit FakeGsView(FakeLink& l) : link(l) {}
  bool idle() const override { return !link.in_flight; }
  bool send_get_all(uint64_t, std::string* why) override { return link.send("get", why); }
  std::optional<LiveReply> poll(uint64_t) override { return link.poll(); }
};

// maburplay's own setter: answers whatever the test says on the next poll.
struct FakePlayer : LiveSetter {
  std::vector<std::string> submitted;
  std::optional<LiveReply> next;
  bool submit(const std::string& key, const std::string& value, uint64_t, std::string*) override {
    submitted.push_back(key + " = " + value);
    return true;
  }
  std::optional<LiveReply> poll(uint64_t) override {
    std::optional<LiveReply> r = next;
    next.reset();
    return r;
  }
};

// The drone setter: never answers on its own unless the test tells it to --
// these session tests are about the one-row-at-a-time locking around the
// applier, not DroneLiveSetter's own sideport settle rule (menu_apply.h).
struct FakeDrone : LiveSetter {
  std::vector<std::string> submitted;
  std::optional<LiveReply> next;
  bool submit(const std::string& key, const std::string& value, uint64_t, std::string*) override {
    submitted.push_back(key + " = " + value);
    return true;
  }
  std::optional<LiveReply> poll(uint64_t) override {
    std::optional<LiveReply> r = next;
    next.reset();
    return r;
  }
};

MenuNode action(std::string id, Owner owner, std::vector<SetStep> steps) {
  MenuNode n;
  n.id = std::move(id);
  n.label = n.id;
  n.kind = NodeKind::kAction;
  n.owner = owner;
  n.action = [steps] { return steps; };
  return n;
}

// Root rows, in cursor order: a two-step GS row, a player row, a drone
// row, a one-step GS row, a player number row, and a GS-player-GS row.
std::vector<MenuNode> fixture_tree() {
  MenuNode num;
  num.id = "row.n";
  num.label = "row.n";
  num.kind = NodeKind::kNumber;
  num.owner = Owner::kPlayer;
  num.number.min = 0;
  num.number.max = 10;
  num.number.step = 1;
  num.number.get = [] { return 5.0; };
  num.steps = [](const std::string& v) { return std::vector<SetStep>{{Owner::kPlayer, "n", v}}; };
  return {action("row.g", Owner::kGs, {{Owner::kGs, "a", "1"}, {Owner::kGs, "b", "2"}}),
          action("row.p", Owner::kPlayer, {{Owner::kPlayer, "p", "1"}}),
          action("row.d", Owner::kDrone, {{Owner::kDrone, "x", "1"}}),
          action("row.g1", Owner::kGs, {{Owner::kGs, "c", "3"}}),
          num,
          action("row.mix", Owner::kGs,
                 {{Owner::kGs, "a", "1"}, {Owner::kPlayer, "p", "1"}, {Owner::kGs, "c", "3"}})};
}

constexpr KeyEvent kOpen{Button::kRight, KeyKind::kHold};
constexpr KeyEvent kClose{Button::kLeft, KeyKind::kHold};
constexpr KeyEvent kRight{Button::kRight, KeyKind::kTap};
constexpr KeyEvent kDown{Button::kDown, KeyKind::kTap};
constexpr KeyEvent kUp{Button::kUp, KeyKind::kTap};

// A bare get's reply as LiveUdpClient hands it over: the dump after "ok\n".
const std::string kDump = "link.nack.enable = true\n";

struct Fx {
  FakeLink link;
  FakeGsSetter gs{link};
  FakeGsView gv{link};
  FakePlayer player;
  FakeDrone drone;
  MenuModel model{fixture_tree};
  MenuApplier applier{model, player, gs, drone};
  GsLiveView view;
  MenuSession s{model, applier, gv, view};
  std::vector<std::string> logs;
  uint64_t t = 0;

  Fx() {
    model.set_linked(true);
    s.log = [this](const std::string& l) { logs.push_back(l); };
  }
  std::optional<bool> key(KeyEvent e) { return s.key(e, ++t); }
  void tick() { s.tick(++t); }
  void tick_at(uint64_t ms) {
    t = ms;
    s.tick(t);
  }
  // Opens the menu and lands the refresh it sends.
  void open_and_read() {
    key(kOpen);
    tick();
    REQUIRE(link.sent == std::vector<std::string>{"get"});
    link.answer(true, kDump);
    tick();
    REQUIRE(view.have);
  }
  // Settings read once, then the menu reopened: its refresh is in flight.
  void reopen_with_a_refresh_in_flight() {
    open_and_read();
    key(kClose);
    key(kOpen);
    tick();
    REQUIRE((link.sent == std::vector<std::string>{"get", "get"}));
    REQUIRE(s.refresh_in_flight());
  }
  ViewRow row(const std::string& id) {
    for (const ViewRow& r : model.view(t).rows)
      if (r.id == id) return r;
    REQUIRE(false);
    return ViewRow{};
  }
  std::string toast() { return model.view(t).toast; }
};

}  // namespace

TEST(opening_the_menu_sends_one_bare_get_and_lands_its_reply) {
  Fx f;
  f.key(kOpen);
  CHECK(f.model.is_open());
  CHECK(f.logs == std::vector<std::string>{"maburplay: menu: open"});
  CHECK(f.s.refresh_pending());
  CHECK(f.link.sent.empty());   // sent from the tick, not from the key
  f.tick();
  CHECK(f.link.sent == std::vector<std::string>{"get"});
  CHECK(f.s.refresh_in_flight());
  f.tick();
  CHECK(f.link.sent.size() == 1);
  f.link.answer(true, kDump);
  f.tick();
  CHECK(f.view.have);
  CHECK(f.view.nack);
  CHECK(!f.s.refresh_in_flight());
  f.tick();
  CHECK(f.link.sent.size() == 1);
}

TEST(a_gs_set_that_lands_refreshes_only_once_the_row_has_finished) {
  Fx f;
  f.open_and_read();
  f.key(kRight);   // row.g: two GS steps
  CHECK(f.link.sent.back() == "set a = 1");
  f.link.answer(true, "ok");
  f.tick();
  // The first step's ok asks for a refresh; the second step goes first.
  CHECK(f.link.sent.back() == "set b = 2");
  CHECK(f.s.refresh_pending());
  f.link.answer(true, "ok");
  f.tick();
  CHECK((f.link.sent == std::vector<std::string>{"get", "set a = 1", "set b = 2", "get"}));
  CHECK(!f.link.busy_seen);
  CHECK(f.row("row.g").status_ok);
}

// The client is idle while a row's player step runs, but the row's next
// GS step still needs it: the refresh waits for the whole row.
TEST(the_refresh_waits_through_a_player_step_between_two_gs_steps) {
  Fx f;
  f.open_and_read();
  for (int i = 0; i < 5; ++i) f.key(kDown);
  f.key(kRight);   // row.mix
  CHECK(f.link.sent.back() == "set a = 1");
  f.link.answer(true, "ok");
  f.tick();
  CHECK(f.player.submitted == std::vector<std::string>{"p = 1"});
  f.tick();
  CHECK(f.link.sent.size() == 2);   // no get while the row runs
  f.player.next = LiveReply{true, ""};
  f.tick();
  CHECK(f.link.sent.back() == "set c = 3");
  f.link.answer(true, "ok");
  f.tick();
  CHECK((f.link.sent == std::vector<std::string>{"get", "set a = 1", "set c = 3", "get"}));
  CHECK(!f.link.busy_seen);
  CHECK(f.row("row.mix").status_ok);
}

TEST(a_gs_row_pressed_while_the_refresh_is_in_flight_waits_for_its_reply) {
  Fx f;
  f.reopen_with_a_refresh_in_flight();
  f.key(kRight);
  CHECK(f.s.holding());
  // Shown applying while it waits.
  CHECK(f.row("row.g").busy);
  CHECK(f.row("row.g").status == "applying");
  CHECK((f.link.sent == std::vector<std::string>{"get", "get"}));
  f.tick();
  CHECK(f.link.sent.size() == 2);
  f.link.answer(true, kDump);
  f.tick();
  CHECK(!f.s.holding());
  CHECK((f.link.sent == std::vector<std::string>{"get", "get", "set a = 1"}));
  CHECK(f.row("row.g").busy);
  CHECK(!f.link.busy_seen);
  f.link.answer(true, "ok");
  f.tick();
  f.link.answer(true, "ok");
  f.tick();
  CHECK(f.row("row.g").status_ok);
  CHECK(!f.model.busy(Owner::kGs));
}

// maburgs silent: the refresh times out, and the row held behind it ends
// there instead of waiting out a second timeout of its own.
TEST(a_held_row_ends_at_no_answer_when_the_refresh_times_out) {
  Fx f;
  f.reopen_with_a_refresh_in_flight();
  f.key(kRight);
  REQUIRE(f.s.holding());
  f.link.answer(false, "no answer");
  f.tick();
  CHECK(!f.s.holding());
  CHECK(!f.applier.running());
  CHECK(f.row("row.g").status_err);
  CHECK(f.model.view(f.t).help == "No answer — it may not have applied");
  CHECK((f.link.sent == std::vector<std::string>{"get", "get"}));
  CHECK(!f.model.busy(Owner::kGs));
}

TEST(a_held_row_the_running_applier_refuses_ends_in_the_row) {
  Fx f;
  f.reopen_with_a_refresh_in_flight();
  f.key(kDown);
  f.key(kRight);   // row.p runs, no answer yet
  f.key(kUp);
  f.key(kRight);   // row.g, held
  f.link.answer(true, kDump);
  f.tick();
  CHECK(!f.s.holding());
  CHECK(f.row("row.g").status_err);
  CHECK(f.toast() == "One GS change at a time");
  CHECK(!f.model.busy(Owner::kGs));
  CHECK(f.model.busy(Owner::kPlayer));
}

TEST(a_player_row_does_not_wait_behind_the_refresh) {
  Fx f;
  f.key(kOpen);
  f.tick();
  f.key(kDown);
  f.key(kRight);   // row.p
  CHECK(!f.s.holding());
  CHECK(f.player.submitted == std::vector<std::string>{"p = 1"});
  CHECK(f.link.sent == std::vector<std::string>{"get"});
}

TEST(a_second_gs_row_while_one_is_held_is_refused_with_a_toast) {
  Fx f;
  f.reopen_with_a_refresh_in_flight();
  f.key(kRight);   // row.g, held
  f.key(kDown);
  f.key(kDown);
  f.key(kDown);
  f.key(kRight);   // row.g1
  CHECK(f.toast() == "One GS change at a time");
  f.link.answer(true, kDump);
  f.tick();
  CHECK((f.link.sent == std::vector<std::string>{"get", "get", "set a = 1"}));
}

TEST(a_row_the_running_applier_refuses_toasts_one_gs_change_at_a_time) {
  Fx f;
  f.open_and_read();
  f.key(kDown);
  f.key(kRight);   // row.p runs, no answer yet
  CHECK(f.applier.running());
  f.key(kUp);
  f.key(kRight);   // row.g: the model has no GS lock, the applier is busy
  CHECK(f.toast() == "One GS change at a time");
  CHECK(f.link.sent == std::vector<std::string>{"get"});
  CHECK(!f.row("row.g").busy);
}

// Drone rows now run through the drone setter like any other owner --
// the "One drone change at a time" wording fires the same way "One GS
// change at a time" does, when a second row is attempted while the
// applier is already busy with a first one.
TEST(a_drone_row_is_refused_with_the_drone_wording) {
  Fx f;
  f.open_and_read();
  f.key(kDown);
  f.key(kRight);   // row.p runs, no answer yet
  CHECK(f.applier.running());
  f.key(kDown);
  f.key(kRight);   // row.d: the applier is busy with row.p
  CHECK(f.toast() == "One drone change at a time");
  CHECK(f.applier.running());        // row.p is still the one running
  CHECK(f.drone.submitted.empty());  // row.d never reached its setter
}

TEST(key_hands_back_the_repeat_setting_and_logs_the_close) {
  Fx f;
  f.open_and_read();
  for (int i = 0; i < 4; ++i) CHECK(!f.key(kDown).has_value());
  CHECK(f.key(kRight) == std::optional<bool>(true));   // row.n's number picker
  CHECK(!f.key(kUp).has_value());
  CHECK(f.key(kClose) == std::optional<bool>(false));
  CHECK(!f.model.is_open());
  CHECK((f.logs == std::vector<std::string>{"maburplay: menu: open", "maburplay: menu: close"}));
}

TEST(an_unread_view_is_asked_again_every_2_s_while_the_menu_is_open) {
  Fx f;
  f.key(kOpen);
  f.tick_at(10);
  CHECK(f.link.sent == std::vector<std::string>{"get"});
  f.link.answer(false, "no answer");
  f.tick_at(11);
  CHECK(!f.view.have);
  CHECK(!f.s.refresh_in_flight());
  f.tick_at(2009);
  CHECK(f.link.sent.size() == 1);
  f.tick_at(2010);
  CHECK((f.link.sent == std::vector<std::string>{"get", "get"}));
  f.link.answer(true, kDump);
  f.tick_at(2011);
  CHECK(f.view.have);
  f.tick_at(9000);   // read: no more asking
  CHECK(f.link.sent.size() == 2);
}

TEST(a_closed_menu_does_not_ask_again) {
  Fx f;
  f.key(kOpen);
  f.tick_at(10);
  f.link.answer(false, "no answer");
  f.tick_at(11);
  f.s.key(kClose, 12);
  f.tick_at(9000);
  CHECK(f.link.sent.size() == 1);
}

TEST(gs_rows_are_refused_until_the_first_get_lands) {
  Fx f;
  f.key(kOpen);
  f.tick();
  f.key(kRight);   // row.g
  CHECK(f.toast() == "Reading GS settings…");
  CHECK(!f.s.holding());
  CHECK(f.link.sent == std::vector<std::string>{"get"});
  f.link.answer(true, kDump);
  f.tick();
  f.key(kRight);
  CHECK(f.link.sent.back() == "set a = 1");
}

TEST(the_generation_moves_on_keys_replies_and_row_progress) {
  Fx f;
  uint64_t g = f.s.generation();
  f.key(kOpen);
  CHECK(f.s.generation() != g);
  g = f.s.generation();
  f.tick();
  f.tick();
  CHECK(f.s.generation() == g);   // nothing landed
  f.link.answer(true, kDump);
  f.tick();
  CHECK(f.s.generation() != g);
  f.key(kRight);   // row.g starts
  g = f.s.generation();
  f.link.answer(true, "ok");
  f.tick();
  CHECK(f.s.generation() != g);   // step 1 landed
  g = f.s.generation();
  f.link.answer(true, "ok");
  f.tick();
  CHECK(f.s.generation() != g);   // the row finished
}

TEST(reopening_the_menu_refreshes_again) {
  Fx f;
  f.open_and_read();
  f.key(kClose);
  f.key(kOpen);
  f.tick();
  CHECK((f.link.sent == std::vector<std::string>{"get", "get"}));
  CHECK((f.logs == std::vector<std::string>{"maburplay: menu: open", "maburplay: menu: close",
                                            "maburplay: menu: open"}));
}

MTEST_MAIN
