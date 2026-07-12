#include "mtest.h"
#include "vectors.h"
#include "score.h"
using namespace maburgs;

TEST(score_window_matches_python) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/score.json");
  for (auto& c : j["score"]) {
    ScoreWindow w;
    for (auto& f : c["frames"])
      w.add_frame(f[0], f[1], f[2], f[3].get<uint16_t>(), f[4]);
    CHECK(w.n() == c["n"].get<size_t>());
    if (c["snr_est"].is_null()) CHECK(!w.snr_estimate().has_value());
    else CHECK(*w.snr_estimate() == c["snr_est"].get<double>());
    CHECK(w.fcs_loss() == c["fcs_loss"].get<double>());
    CHECK(w.seq_gap_loss() == c["seq_gap_loss"].get<double>());
    CHECK(w.ack_seq() == c["ack_seq"].get<int>());
    CHECK(w.score() == c["score_none"].get<int>());
    if (!c["residual"].is_null())
      CHECK(w.score(c["residual"].get<double>()) == c["score_residual"].get<int>());
  }
}

TEST(rung_window_matches_python) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/score.json");
  RungWindow rw({20, 40});
  for (auto& s : j["rung"]["seqs"]) rw.add_seq(s.get<uint16_t>());
  auto stats = rw.stats();
  for (auto& [bw, dn] : j["rung"]["stats"].items()) {
    int b = std::stoi(bw);
    REQUIRE(stats.count(b) == 1);
    CHECK(stats[b].first == dn[0].get<double>());
    CHECK(stats[b].second == dn[1].get<int>());
  }
}
MTEST_MAIN
