#include <cstdint>
#include <random>
#include <vector>

#include "mtest.h"
#include "vectors.h"
#include "mabur/crc16.h"

namespace {
// Bit-serial CRC-16/CCITT-FALSE: the reference the table implementation
// must match byte-for-byte. Kept here, in the test, as the oracle.
uint16_t crc16_bitwise(const uint8_t* data, size_t len, uint16_t init) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}
}  // namespace

TEST(crc16_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/crc16.json");
  int n = 0;
  for (auto& c : j["cases"]) {
    auto in = mtest::unhex(c["in"].get<std::string>());
    CHECK(mabur::crc16_ccitt(in.data(), in.size()) == c["crc"].get<uint16_t>());
    ++n;
  }
  CHECK(n > 0);
}

TEST(crc16_matches_bitwise_reference_random_lengths_and_inits) {
  std::mt19937 gen(0x5bd1e995);
  std::uniform_int_distribution<int> len_d(0, 700);
  std::uniform_int_distribution<int> byte_d(0, 255);
  std::uniform_int_distribution<int> init_d(0, 65535);
  for (int it = 0; it < 2000; ++it) {
    std::vector<uint8_t> buf(static_cast<size_t>(len_d(gen)));
    for (auto& b : buf) b = static_cast<uint8_t>(byte_d(gen));
    const uint16_t init = static_cast<uint16_t>(init_d(gen));
    CHECK(mabur::crc16_ccitt(buf.data(), buf.size(), init) ==
          crc16_bitwise(buf.data(), buf.size(), init));
  }
}

TEST(crc16_init_chains_across_a_split) {
  // crc(a ++ b) == crc(b, init = crc(a)): the init parameter is a real
  // running state, which a table implementation has to preserve.
  std::vector<uint8_t> buf(332);
  for (size_t i = 0; i < buf.size(); ++i) buf[i] = static_cast<uint8_t>(i * 7 + 3);
  for (size_t split : {size_t{0}, size_t{1}, size_t{100}, size_t{331}, size_t{332}}) {
    const uint16_t whole = mabur::crc16_ccitt(buf.data(), buf.size());
    const uint16_t head = mabur::crc16_ccitt(buf.data(), split);
    const uint16_t chained = mabur::crc16_ccitt(buf.data() + split, buf.size() - split, head);
    CHECK(whole == chained);
  }
}

MTEST_MAIN
