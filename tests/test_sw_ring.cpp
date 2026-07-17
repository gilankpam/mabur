// Characterization tests locking SwEncoder behavior across the window-
// storage refactor (deque -> flat ring). Recovery through repairs exercises
// every window row: a wrong ring slot or eviction corrupts repair payloads
// and the decoder fails to recover the dropped sources byte-exactly.
#include <map>
#include <random>
#include <vector>

#include "mabur/sw_decoder.h"
#include "mabur/sw_encoder.h"
#include "mabur/sw_wire.h"
#include "mtest.h"

using namespace mabur;

// Feed n max-size packets, drop every 3rd SOURCE envelope, keep all repairs:
// every packet must still arrive byte-exactly. Runs many window wraps.
static void roundtrip(int symbol_size, int window, int n_pkts) {
  SwConfig cfg{symbol_size, window, 1.0};  // one repair per seal
  SwEncoder enc(cfg);
  SwDecoder dec(cfg);
  std::mt19937 rng(1234);
  std::map<std::vector<uint8_t>, int> want;  // payload -> pending count
  uint64_t now = 0;
  int src_i = 0;
  size_t delivered = 0;
  auto pump = [&](std::vector<std::vector<uint8_t>> envs) {
    for (auto& e : envs) {
      sw::SwHeader h;
      REQUIRE(sw::parse_header(e.data(), e.size(), &h));
      if (!h.repair && (src_i++ % 3 == 2)) continue;  // drop this source
      for (auto& p : dec.add_symbol(e.data(), e.size(), ++now)) {
        auto it = want.find(p);
        CHECK(it != want.end());
        if (it != want.end() && --it->second == 0) want.erase(it);
        ++delivered;
      }
    }
  };
  const size_t maxp = static_cast<size_t>(cfg.max_packet_size());
  for (int i = 0; i < n_pkts; ++i) {
    std::vector<uint8_t> pkt(maxp);
    for (auto& v : pkt) v = static_cast<uint8_t>(rng());
    ++want[pkt];
    pump(enc.add_packet(pkt.data(), pkt.size()));
  }
  pump(enc.flush());
  CHECK(delivered == static_cast<size_t>(n_pkts));
  CHECK(want.empty());
}

TEST(ring_roundtrip_small_window_many_wraps) { roundtrip(64, 8, 400); }
TEST(ring_roundtrip_prod_scalar_geometry) { roundtrip(164, 64, 800); }
TEST(ring_roundtrip_big_symbols) { roundtrip(1312, 128, 600); }

TEST(repair_window_len_never_exceeds_config_window) {
  SwConfig cfg{64, 8, 1.0};
  SwEncoder enc(cfg);
  std::mt19937 rng(5);
  for (int i = 0; i < 100; ++i) {
    std::vector<uint8_t> pkt(static_cast<size_t>(cfg.max_packet_size()));
    for (auto& v : pkt) v = static_cast<uint8_t>(rng());
    for (auto& e : enc.add_packet(pkt.data(), pkt.size())) {
      sw::SwHeader h;
      REQUIRE(sw::parse_header(e.data(), e.size(), &h));
      if (h.repair) CHECK(static_cast<int>(h.window_len) <= cfg.window);
    }
  }
}

MTEST_MAIN
