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
    else op = ctrl.update(e["snr"], e["txagc"], e["now"]);
    if (e["out"].is_null()) { CHECK(!op.has_value()); continue; }
    REQUIRE(op.has_value());
    CHECK(op->vht == e["out"]["vht"].get<bool>());
    CHECK(op->mcs == e["out"]["mcs"].get<int>());
    CHECK(op->bw == e["out"]["bw"].get<int>());
    CHECK(op->txagc == e["out"]["txagc"].get<int>());
    CHECK(op->overhead == e["out"]["ov"].get<double>());
  }
}

TEST(shed_layer_returns_null_when_nothing_clears) {
  LinkTable lt;
  ControllerConfig cfg;
  cfg.allow_shed = true;
  Controller ctrl(lt, cfg);
  // Path loss so deep no row clears even at txagc 63 -> shed (nullopt).
  auto op = ctrl.update(-60.0, 63, 0.0);
  CHECK(!op.has_value());
  CHECK(ctrl.shed());
}
MTEST_MAIN
