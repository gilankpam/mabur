// MenuApplier: runs one row's SetSteps against a FakeSetter per owner,
// driving the MenuModel the same way main will drive it with the real
// PlayerLiveSetter/GsLiveSetter/DroneLiveSetter. DroneSetWatch is tested
// separately, directly against GsSnapshot fixtures, since it never
// touches a socket; DroneLiveSetter's own test drives a real loopback
// listener, reusing test_live_udp_client.cpp's fixture shape.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gs_snapshot.h"
#include "live_udp_client.h"
#include "menu_apply.h"
#include "menu_model.h"
#include "mtest.h"
#include "player_config.h"
#include "player_config_apply.h"

using maburplay::DroneLiveSetter;
using maburplay::DroneSetWatch;
using maburplay::DroneSnapshotView;
using maburplay::GsSnapshot;
using maburplay::LiveReply;
using maburplay::LiveSetter;
using maburplay::LiveUdpClient;
using maburplay::MenuApplier;
using maburplay::MenuModel;
using maburplay::MenuNode;
using maburplay::NodeKind;
using maburplay::Owner;
using maburplay::SetStep;
using maburplay::VtxRow;
using maburplay::VtxState;
using maburplay::ViewRow;

namespace {

// Records every submit and answers whatever the test tells it to on the
// next poll().
struct FakeSetter : LiveSetter {
  std::vector<std::pair<std::string, std::string>> submitted;
  bool busy_ = false;
  std::string busy_why_ = "busy";
  std::optional<LiveReply> next_reply;

  bool submit(const std::string& key, const std::string& value, uint64_t, std::string* why) override {
    if (busy_) {
      if (why) *why = busy_why_;
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

  MenuNode d;
  d.id = "row.d";
  d.kind = NodeKind::kAction;
  d.owner = Owner::kDrone;

  return {p, g, mix, d};
}

ViewRow row(const MenuModel& model, uint64_t now_ms, const std::string& id) {
  for (const ViewRow& r : model.view(now_ms).rows)
    if (r.id == id) return r;
  REQUIRE(false);
  return ViewRow{};
}

// A snapshot carrying one vtx row, linked, with the given state/age/t_ms.
GsSnapshot vtx_snap(bool linked, const std::string& id, const std::string& st,
                    std::optional<int64_t> age_ms, std::optional<uint64_t> t_ms,
                    std::string status = "") {
  GsSnapshot s;
  s.t_ms = t_ms;
  VtxState vs;
  vs.linked = linked;
  VtxRow r;
  r.id = id;
  r.st = st;
  r.age_ms = age_ms;
  r.status = std::move(status);
  vs.rows.push_back(r);
  s.vtx = vs;
  return s;
}

}  // namespace

TEST(steps_run_in_order_and_stop_at_the_first_err) {
  MenuModel model(fixture_tree);
  model.open();
  FakeSetter player, gs, drone;
  MenuApplier app(model, player, gs, drone);
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
  FakeSetter player, gs, drone;
  MenuApplier app(model, player, gs, drone);
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
  FakeSetter player, gs, drone;
  MenuApplier app(model, player, gs, drone);
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
  FakeSetter player, gs, drone;
  MenuApplier app(model, player, gs, drone);
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
  FakeSetter player, gs, drone;
  MenuApplier app(model, player, gs, drone);

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

TEST(the_applier_runs_a_drone_step_through_the_drone_setter) {
  MenuModel model(fixture_tree);
  model.open();
  FakeSetter player, gs, drone;
  MenuApplier app(model, player, gs, drone);

  REQUIRE(app.start("row.d", {{Owner::kDrone, "VENC_MIRROR", "1"}}, 0));
  CHECK(app.running());
  REQUIRE(drone.submitted.size() == 1);
  CHECK(drone.submitted[0] == std::make_pair(std::string("VENC_MIRROR"), std::string("1")));

  drone.answer(true, "");
  app.tick(10);
  CHECK(!app.running());
  CHECK(!model.busy(Owner::kDrone));
  CHECK(row(model, 10, "row.d").status_ok);
}

TEST(a_drone_refusal_at_submit_fails_the_row_with_the_reason) {
  MenuModel model(fixture_tree);
  model.open();
  FakeSetter player, gs, drone;
  drone.busy_ = true;
  drone.busy_why_ = "apply in progress";
  MenuApplier app(model, player, gs, drone);

  REQUIRE(app.start("row.d", {{Owner::kDrone, "VENC_MIRROR", "1"}}, 0));
  CHECK(!app.running());
  CHECK(!model.busy(Owner::kDrone));
  CHECK(row(model, 0, "row.d").status_err);
}

// --- DroneSetWatch: the settle rule on its own, no socket involved -------

TEST(a_fresh_terminal_state_settles_even_without_seeing_sending) {
  GsSnapshot before = vtx_snap(true, "VENC_MODE", "accepted", 5000, std::nullopt);
  DroneSetWatch w;
  w.begin("VENC_MODE", &before);

  // Same age as at submit: the old outcome, not a new one.
  CHECK(!w.check(DroneSnapshotView{&before, false}).has_value());

  GsSnapshot after = vtx_snap(true, "VENC_MODE", "accepted", 120, std::nullopt);
  std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
  REQUIRE(r.has_value());
  CHECK(r->ok);
  CHECK(r->text.empty());
}

TEST(a_failed_set_carries_the_status_or_refused) {
  {
    DroneSetWatch w;
    w.begin("VENC_MODE", nullptr);
    GsSnapshot after = vtx_snap(true, "VENC_MODE", "failed", 10, std::nullopt, "venc.size: must fit");
    std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
    REQUIRE(r.has_value());
    CHECK(!r->ok);
    CHECK(r->text == "venc.size: must fit");
  }
  {
    DroneSetWatch w;
    w.begin("VENC_MODE", nullptr);
    GsSnapshot after = vtx_snap(true, "VENC_MODE", "unsupported", 10, std::nullopt, "");
    std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
    REQUIRE(r.has_value());
    CHECK(!r->ok);
    CHECK(r->text == "refused");
  }
  {
    DroneSetWatch w;
    w.begin("VENC_MODE", nullptr);
    GsSnapshot after = vtx_snap(true, "VENC_MODE", "timeout", 10, std::nullopt, "");
    std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
    REQUIRE(r.has_value());
    CHECK(!r->ok);
    CHECK(r->text == "no answer");
  }
}

TEST(a_row_with_no_prior_age_settles_on_its_first_terminal_state) {
  GsSnapshot before = vtx_snap(true, "VENC_MODE", "idle", std::nullopt, std::nullopt);
  DroneSetWatch w;
  w.begin("VENC_MODE", &before);

  GsSnapshot after = vtx_snap(true, "VENC_MODE", "accepted", 40, std::nullopt);
  std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
  REQUIRE(r.has_value());
  CHECK(r->ok);
}

TEST(a_stale_or_unlinked_snapshot_ends_the_step_with_link_lost) {
  GsSnapshot before = vtx_snap(true, "VENC_MODE", "idle", std::nullopt, std::nullopt);
  DroneSetWatch w;
  w.begin("VENC_MODE", &before);
  std::optional<LiveReply> r1 = w.check(DroneSnapshotView{&before, /*stale=*/true});
  REQUIRE(r1.has_value());
  CHECK(!r1->ok);
  CHECK(r1->text == "link lost");
  CHECK(!w.active());

  DroneSetWatch w2;
  w2.begin("VENC_MODE", &before);
  GsSnapshot unlinked = vtx_snap(false, "VENC_MODE", "idle", std::nullopt, std::nullopt);
  std::optional<LiveReply> r2 = w2.check(DroneSnapshotView{&unlinked, false});
  REQUIRE(r2.has_value());
  CHECK(!r2->ok);
  CHECK(r2->text == "link lost");
}

// The settle rule is keyed on each side's absolute result time
// (t_ms - age_ms), not the raw age, so a genuinely new result that
// happens to read an older age than a very recent previous one still
// settles.
TEST(a_new_outcome_older_than_the_previous_age_still_settles_by_result_time) {
  GsSnapshot before = vtx_snap(true, "VENC_MODE", "accepted", 50, uint64_t{10000});  // result 9950
  DroneSetWatch w;
  w.begin("VENC_MODE", &before);

  GsSnapshot after = vtx_snap(true, "VENC_MODE", "accepted", 150, uint64_t{10200});  // result 10050
  std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
  REQUIRE(r.has_value());
  CHECK(r->ok);
}

TEST(the_same_datagram_seen_again_never_settles) {
  GsSnapshot before = vtx_snap(true, "VENC_MODE", "accepted", 50, uint64_t{10000});
  DroneSetWatch w;
  w.begin("VENC_MODE", &before);

  CHECK(!w.check(DroneSnapshotView{&before, false}).has_value());
}

TEST(a_terminal_after_seeing_in_flight_settles) {
  GsSnapshot before = vtx_snap(true, "VENC_MODE", "accepted", 50, uint64_t{10000});  // result 9950
  DroneSetWatch w;
  w.begin("VENC_MODE", &before);

  GsSnapshot sending = vtx_snap(true, "VENC_MODE", "sending", std::nullopt, uint64_t{10050});
  CHECK(!w.check(DroneSnapshotView{&sending, false}).has_value());

  // Same result time as before (10600 - 650 = 9950): by the result-time
  // rule alone this would never settle, but having been seen in flight
  // it settles anyway.
  GsSnapshot after = vtx_snap(true, "VENC_MODE", "accepted", 650, uint64_t{10600});
  std::optional<LiveReply> r = w.check(DroneSnapshotView{&after, false});
  REQUIRE(r.has_value());
  CHECK(r->ok);
}

// --- DroneLiveSetter: a real loopback listener ----------------------------
// Minimal fixture copied from tests/test_live_udp_client.cpp: a real
// PlayerConfigControl/PlayerConfigApplier pair stands in for maburgs'
// listener (same mabur::LiveConfigControl<Applier> template), answering
// the unknown "vtx" verb through its extra_verb hook -- exactly the seam
// a real GS applier would use for the same verb.

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

maburplay::Config bundle() { return maburplay::load_config(kBundlePath); }

struct ScratchPlayerBundle {
  std::string dir, file;
  ScratchPlayerBundle() {
    static int seq = 0;
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    dir = std::string(MABUR_TEST_SCRATCH_DIR) + "/test_menu_apply-" + std::to_string(::getpid()) +
          "-" + std::to_string(seq++);
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

struct PFake {
  maburplay::PlayerApplyHooks hooks() {
    maburplay::PlayerApplyHooks h;
    h.reacquire_display = [](const maburplay::Config&, std::string*) { return true; };
    h.osd_layers = [](const maburplay::Config&, std::string*) { return true; };
    h.next_start = [](const maburplay::Config&) {};
    h.menu = [](const maburplay::Config&) {};
    return h;
  }
};

template <class Done>
void drive_until(maburplay::PlayerConfigControl& ctl, maburplay::PlayerConfigApplier& a,
                 Done done) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!done()) {
    REQUIRE(std::chrono::steady_clock::now() < deadline);
    ctl.poll(a);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

TEST(drone_live_setter_submits_the_vtx_set_line) {
  constexpr int kPort = 18440;
  ScratchPlayerBundle sb;
  PFake f;
  maburplay::PlayerConfigApplier a(sb.path(), bundle(), maburplay::BuildCaps{true, false},
                                   f.hooks(), nullptr);
  maburplay::PlayerConfigControl ctl;
  REQUIRE(ctl.open(kPort));

  std::vector<std::string> seen_lines;
  std::optional<std::string> next_reply;
  ctl.extra_verb = [&](const std::string& verb,
                       const std::string& rest) -> std::optional<std::string> {
    seen_lines.push_back(verb + " " + rest);
    if (!next_reply) return std::nullopt;
    std::string r = *next_reply;
    next_reply.reset();
    return r;
  };

  LiveUdpClient client;
  std::string err;
  REQUIRE(client.open(kPort, &err));

  // Linked, with VENC_FLIP present but idle: DroneLiveSetter's view of
  // the sideport as it stood right before the submit.
  GsSnapshot snap = vtx_snap(true, "VENC_FLIP", "idle", std::nullopt, std::nullopt);
  DroneLiveSetter s(client, [&] { return DroneSnapshotView{&snap, false}; });

  std::string why;
  REQUIRE(s.submit("VENC_FLIP", "0", 0, &why));

  next_reply = "ok";
  std::optional<LiveReply> r;
  drive_until(ctl, a, [&] {
    r = s.poll(1);
    return !seen_lines.empty();
  });
  CHECK(seen_lines == std::vector<std::string>{"vtx set VENC_FLIP 0"});
  CHECK(!r.has_value());  // the verb was sent; the row has not answered yet

  // The row answers, fresh: the watch settles on the next poll.
  snap.vtx->rows[0].st = "accepted";
  snap.vtx->rows[0].age_ms = 10;
  snap.t_ms = 1000;
  std::optional<LiveReply> settled = s.poll(2);
  REQUIRE(settled.has_value());
  CHECK(settled->ok);

  // A fresh exchange: maburgs refuses outright, no watch involved.
  REQUIRE(s.submit("VENC_FLIP", "1", 100, &why));
  next_reply = "err not linked";
  std::optional<LiveReply> refused;
  drive_until(ctl, a, [&] {
    refused = s.poll(101);
    return refused.has_value();
  });
  REQUIRE(refused.has_value());
  CHECK(!refused->ok);
  CHECK(refused->text == "not linked");
}

MTEST_MAIN
