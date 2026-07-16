#include "mtest.h"
#include "vectors.h"
#include "controller.h"
using namespace maburgs;

TEST(replay_matches_python_decisions) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/controller_replay.json");
  LinkTable lt;
  ControllerConfig cfg;
  cfg.target = j["cfg"]["target"];
  cfg.allow_shed = j["cfg"]["allow_shed"];
  cfg.src_bitrate_bps = j["cfg"]["src_bitrate_bps"];
  Controller ctrl(lt, cfg);
  for (auto& e : j["trace"]) {
    std::optional<OpPoint> op;
    if (e["kind"] == "tick") op = ctrl.on_tick(e["now"]);
    else op = ctrl.update(e["snr"], e["pwr_offset_qdb"], e["now"]);
    if (e["out"].is_null()) { CHECK(!op.has_value()); continue; }
    REQUIRE(op.has_value());
    CHECK(op->vht == e["out"]["vht"].get<bool>());
    CHECK(op->mcs == e["out"]["mcs"].get<int>());
    CHECK(op->bw == e["out"]["bw"].get<int>());
    CHECK(op->pwr_offset_qdb == e["out"]["pwr_offset_qdb"].get<int>());
    CHECK(op->overhead == e["out"]["ov"].get<double>());
  }
}

TEST(shed_layer_returns_null_when_nothing_clears) {
  LinkTable lt;
  ControllerConfig cfg;
  cfg.allow_shed = true;
  Controller ctrl(lt, cfg);
  // Path loss so deep no row clears even at max offset -> shed (nullopt).
  auto op = ctrl.update(-60.0, 0, 0.0);
  CHECK(!op.has_value());
  CHECK(ctrl.shed());
}

TEST(controller_clamps_offset_at_rails) {
  // Path loss so bad that even max_offset_qdb can't close any row at target
  // -> controller must fall to MAX_RANGE (offset = cfg.max_offset_qdb),
  // never emit an offset above max.
  LinkTable lt;
  ControllerConfig cfg;
  cfg.allow_shed = false;
  Controller ctrl(lt, cfg);
  // reported_snr very low, reported offset at floor -> path_loss huge.
  auto op = ctrl.update(-80.0, cfg.min_offset_qdb, 0.0);
  REQUIRE(op.has_value());
  CHECK(op->pwr_offset_qdb == cfg.max_offset_qdb);
}

TEST(strong_link_rides_offset_floor) {
  // Huge margin -> cheapest row picks min_offset_qdb, never below it.
  LinkTable lt;
  ControllerConfig cfg;
  Controller ctrl(lt, cfg);
  double now = 0.0;
  std::optional<OpPoint> op;
  // Drive several updates at a very strong reported SNR so the EMA settles
  // and the controller has time to walk off MAX_RANGE onto the cheapest row.
  for (int i = 0; i < 20; ++i) {
    op = ctrl.update(80.0, 0, now);
    now += 200.0;
  }
  REQUIRE(op.has_value());
  CHECK(op->pwr_offset_qdb >= cfg.min_offset_qdb);
  CHECK(op->pwr_offset_qdb <= cfg.max_offset_qdb);
}
MTEST_MAIN
