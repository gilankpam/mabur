#include "mtest.h"
#include "vectors.h"
#include "mabur/sbi.h"
using namespace mabur;

TEST(unpack_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/sbi_unpack.json");
  for (auto& c : j["cases"]) {
    auto body = mtest::unhex(c["body"].get<std::string>());
    auto r = sbi_unpack(body.data(), body.size(), c["block_payload"]);
    CHECK(r.n_blocks == c["n_blocks"].get<int>());
    CHECK(r.n_failed == c["n_failed"].get<int>());
    CHECK(r.header_ok == c["header_ok"].get<bool>());
    CHECK(r.stream_id == c["stream_id"].get<int>());
    REQUIRE(r.survivors.size() == c["survivors"].size());
    for (size_t i = 0; i < r.survivors.size(); ++i)
      CHECK(mtest::hex(r.survivors[i]) == c["survivors"][i].get<std::string>());
  }
}

TEST(peek_stream_id) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/sbi_unpack.json");
  for (auto& c : j["cases"]) {
    auto body = mtest::unhex(c["body"].get<std::string>());
    int sid = sbi_peek_stream_id(body.data(), body.size());
    if (c["name"] == "clean" || c["name"] == "one_subblock_corrupt")
      CHECK(sid == c["stream_id"].get<int>());
    else
      CHECK(sid == -1);  // header_corrupt (bad magic) and short both peek -1
  }
}
MTEST_MAIN
