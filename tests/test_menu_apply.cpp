// MenuApplier: runs one row's SetSteps against a FakeSetter per owner,
// driving the MenuModel the same way main will drive it with the real
// PlayerLiveSetter/GsLiveSetter.
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "live_udp_client.h"
#include "menu_apply.h"
#include "menu_model.h"
#include "mtest.h"

using maburplay::LiveReply;
using maburplay::LiveSetter;
using maburplay::MenuApplier;
using maburplay::MenuModel;
using maburplay::MenuNode;
using maburplay::NodeKind;
using maburplay::Owner;
using maburplay::SetStep;
using maburplay::ViewRow;

namespace {

// Records every submit and answers whatever the test tells it to on the
// next poll().
struct FakeSetter : LiveSetter {
  std::vector<std::pair<std::string, std::string>> submitted;
  bool busy_ = false;
  std::optional<LiveReply> next_reply;

  bool submit(const std::string& key, const std::string& value, uint64_t, std::string* why) override {
    if (busy_) {
      if (why) *why = "busy";
      return false;
    }
    submitted.emplace_back(key, value);
    busy_ = true;
    return true;
  }
  std::optional<LiveReply> poll(uint64_t) override {
    if (!next_reply) return std::nullopt;
    LiveReply r = *next_reply;
    next_reply.reset();
    busy_ = false;
    return r;
  }
  void answer(bool ok, std::string text) { next_reply = LiveReply{ok, std::move(text)}; }
};

std::vector<MenuNode> fixture_tree() {
  MenuNode p;
  p.id = "row.p";
  p.kind = NodeKind::kAction;
  p.owner = Owner::kPlayer;

  MenuNode g;
  g.id = "row.g";
  g.kind = NodeKind::kAction;
  g.owner = Owner::kGs;

  MenuNode mix;
  mix.id = "row.mix";
  mix.kind = NodeKind::kAction;
  mix.owner = Owner::kPlayer;

  return {p, g, mix};
}

ViewRow row(const MenuModel& model, uint64_t now_ms, const std::string& id) {
  for (const ViewRow& r : model.view(now_ms).rows)
    if (r.id == id) return r;
  REQUIRE(false);
  return ViewRow{};
}

}  // namespace

TEST(steps_run_in_order_and_stop_at_the_first_err) {
  MenuModel model(fixture_tree);
  model.open();
  FakeSetter player, gs;
  MenuApplier app(model, player, gs);
  const std::vector<SetStep> steps = {
      {Owner::kPlayer, "k1", "v1"},
      {Owner::kPlayer, "k2", "v2"},
      {Owner::kPlayer, "k3", "v3"},
      {Owner::kPlayer, "k4", "v4"},
  };
  REQUIRE(app.start("row.p", steps, 0));
  CHECK(app.running());
  REQUIRE(player.submitted.size() == 1);
  CHECK(player.submitted[0] == std::make_pair(std::string("k1"), std::string("v1")));

  player.answer(true, "");
  app.tick(10);
  REQUIRE(player.submitted.size() == 2);
  CHECK(player.submitted[1] == std::make_pair(std::string("k2"), std::string("v2")));

  player.answer(false, "boom");
  app.tick(20);
  CHECK(!app.running());
  // The 3rd and 4th steps were never sent.
  CHECK(player.submitted.size() == 2);

  // The bare reason reaches the model untouched; it dresses it up.
  const auto v = model.view(20);
  CHECK(v.help_is_error);
  CHECK(v.help == "err boom");
}

TEST(the_model_sees_started_each_step_and_done) {
  MenuModel model(fixture_tree);
  model.open();
  FakeSetter player, gs;
  MenuApplier app(model, player, gs);
  const std::vector<SetStep> steps = {{Owner::kPlayer, "k1", "v1"}, {Owner::kPlayer, "k2", "v2"}};

  CHECK(!model.busy(Owner::kPlayer));
  REQUIRE(app.start("row.p", steps, 0));
  CHECK(model.busy(Owner::kPlayer));
  CHECK(row(model, 0, "row.p").status == "applying 1/2");

  player.answer(true, "");
  app.tick(10);
  CHECK(model.busy(Owner::kPlayer));
  CHECK(row(model, 10, "row.p").status == "applying 2/2");

  player.answer(true, "");
  app.tick(20);
  CHECK(!app.running());
  CHECK(!model.busy(Owner::kPlayer));
  CHECK(row(model, 20, "row.p").status_ok);
}

TEST(a_player_step_and_a_gs_step_go_to_their_own_setter) {
  MenuModel model(fixture_tree);
  FakeSetter player, gs;
  MenuApplier app(model, player, gs);
  const std::vector<SetStep> steps = {{Owner::kPlayer, "k1", "v1"}, {Owner::kGs, "k2", "v2"}};

  REQUIRE(app.start("row.mix", steps, 0));
  REQUIRE(player.submitted.size() == 1);
  CHECK(gs.submitted.empty());

  player.answer(true, "");
  app.tick(10);
  REQUIRE(gs.submitted.size() == 1);
  CHECK(gs.submitted[0] == std::make_pair(std::string("k2"), std::string("v2")));
  CHECK(player.submitted.size() == 1);

  gs.answer(true, "");
  app.tick(20);
  CHECK(!app.running());
}

TEST(a_timeout_reports_no_answer) {
  // A single row, selected by the default cursor, so its help text (only
  // shown for the selected row) is the one checked below.
  auto one_gs_row = [] {
    MenuNode g;
    g.id = "row.g";
    g.kind = NodeKind::kAction;
    g.owner = Owner::kGs;
    return std::vector<MenuNode>{g};
  };
  MenuModel model(one_gs_row);
  model.open();
  FakeSetter player, gs;
  MenuApplier app(model, player, gs);
  REQUIRE(app.start("row.g", {{Owner::kGs, "k", "v"}}, 0));

  gs.answer(false, "no answer");  // what GsLiveSetter::poll gives for a LiveUdpClient timeout
  app.tick(10);
  CHECK(!app.running());
  CHECK(!model.busy(Owner::kGs));

  const auto v = model.view(10);
  REQUIRE(v.rows.size() == 1);
  CHECK(v.rows[0].id == "row.g");
  CHECK(v.rows[0].status_err);
  CHECK(v.help_is_error);
  CHECK(v.help == "No answer — it may not have applied");
}

TEST(a_second_row_cannot_start_while_one_is_running) {
  MenuModel model(fixture_tree);
  model.open();
  FakeSetter player, gs;
  MenuApplier app(model, player, gs);

  REQUIRE(app.start("row.p", {{Owner::kPlayer, "k1", "v1"}}, 0));
  CHECK(app.running());

  // A second row is refused outright while the first is still in
  // flight -- it never reaches the model or either setter.
  CHECK(!app.start("row.g", {{Owner::kGs, "k2", "v2"}}, 0));
  CHECK(gs.submitted.empty());
  CHECK(!model.busy(Owner::kGs));
  CHECK(row(model, 0, "row.g").status.empty());

  // The first row still completes normally.
  player.answer(true, "");
  app.tick(10);
  CHECK(!app.running());
  CHECK(player.submitted.size() == 1);
  CHECK(row(model, 10, "row.p").status_ok);

  // Now that it's done, a new row can start.
  REQUIRE(app.start("row.g", {{Owner::kGs, "k2", "v2"}}, 10));
  CHECK(app.running());
  REQUIRE(gs.submitted.size() == 1);
}

TEST(drone_steps_are_refused_in_this_build) {
  MenuModel model(fixture_tree);
  FakeSetter player, gs;
  MenuApplier app(model, player, gs);
  CHECK(!app.start("row.drone", {{Owner::kDrone, "k", "v"}}, 0));
  CHECK(!app.running());
  CHECK(player.submitted.empty());
  CHECK(gs.submitted.empty());
}

MTEST_MAIN
