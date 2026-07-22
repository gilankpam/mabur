#include "mtest.h"
#include "vectors.h"
#include "mabur/frag.h"
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

TEST(age_eviction_expires_stale_entries) {
  FragReassembler r(512, /*max_age_ms=*/200);
  // Incomplete entry (seq 0, chunk 0-of-2) at t=1000.
  uint8_t a0[5] = {0, 0, 0, 2, 0xAA};
  CHECK(r.add(a0, sizeof(a0), 1000).empty());
  // Within horizon: still pending.
  uint8_t b0[5] = {1, 0, 0, 2, 0xBB};
  CHECK(r.add(b0, sizeof(b0), 1100).empty());
  CHECK(r.pending() == 2);
  // Past the horizon for seq 0 only: swept, seq 1 (age 150) survives.
  uint8_t c0[5] = {2, 0, 0, 2, 0xCC};
  CHECK(r.add(c0, sizeof(c0), 1250).empty());
  CHECK(r.pending() == 2);
  CHECK(r.evicted() == 1);
  // Late second chunk of the evicted seq 0 starts a fresh (incomplete)
  // entry rather than completing with stale data.
  uint8_t a1[5] = {0, 0, 1, 2, 0xAD};
  CHECK(r.add(a1, sizeof(a1), 1251).empty());
  CHECK(r.completed() == 0);
}

TEST(age_eviction_disabled_without_clock) {
  FragReassembler r(512, /*max_age_ms=*/200);
  uint8_t a0[5] = {0, 0, 0, 2, 0xAA};
  CHECK(r.add(a0, sizeof(a0)).empty());  // now_ms = 0: age checks off
  uint8_t b0[5] = {1, 0, 0, 2, 0xBB};
  CHECK(r.add(b0, sizeof(b0)).empty());
  CHECK(r.pending() == 2);
  CHECK(r.evicted() == 0);
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
TEST(wide_reassembler_roundtrip_large_unit) {
  Fragmenter frag(/*wide=*/true);
  FragReassembler reasm(512, 0, /*wide=*/true);
  std::vector<uint8_t> p(100 * 1024);
  for (size_t i = 0; i < p.size(); ++i) p[i] = static_cast<uint8_t>(i * 7);
  auto frags = frag.fragment(p.data(), p.size(), 158);
  REQUIRE(frags.size() > 255);
  std::vector<FragCompleted> done;
  for (auto& f : frags) {
    auto out = reasm.add(f.data(), f.size());
    for (auto& c : out) done.push_back(std::move(c));
  }
  REQUIRE(done.size() == 1);
  CHECK(done[0].pkt == p);
}

TEST(wide_reassembler_rejects_short_header) {
  FragReassembler reasm(512, 0, /*wide=*/true);
  uint8_t buf[5] = {0, 0, 0, 0, 0};  // < 6-byte wide header
  CHECK(reasm.add(buf, sizeof buf).empty());
}

MTEST_MAIN
