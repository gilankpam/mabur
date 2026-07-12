#include "mtest.h"
#include "vectors.h"
#include "mabur/frag_reassembler.h"
#include <algorithm>
using namespace mabur;

// frag.json cases: "in" = original packet, "out" = its FRAG-headed chunks.
TEST(reassemble_frag_vectors_in_order_and_reversed) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/frag.json");
  for (auto& c : j["cases"]) {
    for (bool reversed_order : {false, true}) {
      FragReassembler r;
      std::vector<std::vector<uint8_t>> chunks;
      for (auto& ch : c["out"]) chunks.push_back(mtest::unhex(ch.get<std::string>()));
      if (reversed_order) std::reverse(chunks.begin(), chunks.end());
      std::vector<FragCompleted> done;
      for (auto& ch : chunks)
        for (auto& d : r.add(ch.data(), ch.size())) done.push_back(std::move(d));
      REQUIRE(done.size() == 1);
      CHECK(mtest::hex(done[0].pkt) == c["in"].get<std::string>());
      CHECK(r.pending() == 0);
    }
  }
}

TEST(missing_chunk_never_emits) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/frag.json");
  for (auto& c : j["cases"]) {
    if (c["out"].size() < 2) continue;  // single-chunk packets can't lose one
    FragReassembler r;
    for (size_t i = 1; i < c["out"].size(); ++i) {  // drop chunk 0
      auto ch = mtest::unhex(c["out"][i].get<std::string>());
      CHECK(r.add(ch.data(), ch.size()).empty());
    }
    CHECK(r.completed() == 0);
    CHECK(r.pending() == 1);
  }
}

TEST(eviction_bounds_pending) {
  FragReassembler r(2);  // max_pending = 2
  // Three incomplete entries (seq 0,1,2), each chunk 0-of-2.
  for (uint16_t s = 0; s < 3; ++s) {
    uint8_t pkt[5] = {static_cast<uint8_t>(s & 0xFF), static_cast<uint8_t>(s >> 8), 0, 2, 0xAA};
    CHECK(r.add(pkt, sizeof(pkt)).empty());
  }
  CHECK(r.pending() == 2);   // oldest (seq 0) evicted
  CHECK(r.evicted() == 1);
}

TEST(malformed_dropped) {
  FragReassembler r;
  uint8_t count0[4] = {0, 0, 0, 0};          // count == 0
  CHECK(r.add(count0, sizeof(count0)).empty());
  uint8_t short3[3] = {0, 0, 0};             // shorter than the header
  CHECK(r.add(short3, sizeof(short3)).empty());
  // idx beyond count: {seq=5, idx=7, count=2} then the real 2 chunks -> the
  // poisoned entry is discarded when completion finds index 1 missing... it
  // fills size==count with idx {7,0}: non-contiguous -> dropped as malformed.
  uint8_t bad[5] = {5, 0, 7, 2, 0x11};
  uint8_t c0[5] = {5, 0, 0, 2, 0x22};
  CHECK(r.add(bad, sizeof(bad)).empty());
  CHECK(r.add(c0, sizeof(c0)).empty());      // size reaches 2, idx 1 missing -> drop
  CHECK(r.completed() == 0);
  CHECK(r.pending() == 0);
  CHECK(r.evicted() == 1);
}
MTEST_MAIN
