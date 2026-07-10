#include "mtest.h"
#include "vectors.h"
#include "mabur/sbi.h"
#include "mabur/crc16.h"
using namespace mabur;

TEST(sbi_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/sbi.json");
  int block_payload = j["block_payload"];
  int blocks_per_body = j["blocks_per_body"];
  uint8_t stream_id = static_cast<uint8_t>(j["stream_id"].get<int>());

  SbiPacker packer(block_payload, blocks_per_body, stream_id);

  std::vector<std::string> stream;
  for (auto& eh : j["envelopes"]) {
    auto env = mtest::unhex(eh.get<std::string>());
    for (auto& body : packer.add(env.data(), env.size()))
      stream.push_back(mtest::hex(body));
  }

  REQUIRE(stream.size() == j["stream"].size());
  for (size_t i = 0; i < stream.size(); ++i)
    CHECK(stream[i] == j["stream"][i].get<std::string>());

  size_t fi = 0;
  for (auto& body : packer.flush())
    CHECK(mtest::hex(body) == j["flush"][fi++].get<std::string>());
  CHECK(fi == j["flush"].size());
}

TEST(sbi_layout_and_crc_isolation) {
  int block_payload = 8;
  int blocks_per_body = 2;
  uint8_t stream_id = 5;
  SbiPacker packer(block_payload, blocks_per_body, stream_id);

  std::vector<uint8_t> env0 = {0, 1, 2, 3, 4, 5, 6, 7};
  std::vector<uint8_t> env1 = {8, 9, 10, 11, 12, 13, 14, 15};

  auto out0 = packer.add(env0.data(), env0.size());
  CHECK(out0.empty());
  auto out1 = packer.add(env1.data(), env1.size());
  REQUIRE(out1.size() == 1);
  auto& body = out1[0];

  auto flushed = packer.flush();
  CHECK(flushed.empty());  // nothing pending after the full body was emitted

  int block_stride = packer.block_stride();
  CHECK(block_stride == 2 + block_payload);

  size_t expected_len = static_cast<size_t>(SBI_HDR_LEN) +
                        static_cast<size_t>(blocks_per_body) * block_stride;
  REQUIRE(body.size() == expected_len);

  // Header fields: <u16 MAGIC LE, u8 ver, u8 stream_id, u16 block_payload LE, u8 n_blocks>
  uint16_t magic = static_cast<uint16_t>(body[0]) | (static_cast<uint16_t>(body[1]) << 8);
  uint8_t ver = body[2];
  uint8_t sid = body[3];
  uint16_t bp = static_cast<uint16_t>(body[4]) | (static_cast<uint16_t>(body[5]) << 8);
  uint8_t n_blocks = body[6];

  CHECK(magic == SBI_MAGIC);
  CHECK(ver == 0);
  CHECK(sid == stream_id);
  CHECK(bp == block_payload);
  CHECK(n_blocks == blocks_per_body);

  // Block layout: each block is <u16 crc LE, payload[block_payload]>
  std::vector<std::vector<uint8_t>> envs = {env0, env1};
  for (int i = 0; i < blocks_per_body; ++i) {
    size_t off = static_cast<size_t>(SBI_HDR_LEN) + i * block_stride;
    uint16_t crc_field = static_cast<uint16_t>(body[off]) |
                          (static_cast<uint16_t>(body[off + 1]) << 8);
    uint16_t expect_crc = crc16_ccitt(envs[i].data(), envs[i].size());
    CHECK(crc_field == expect_crc);
    for (int b = 0; b < block_payload; ++b)
      CHECK(body[off + 2 + b] == envs[i][b]);
  }

  // Flip a byte in block 0's payload: only block 0's CRC should fail.
  for (int i = 0; i < blocks_per_body; ++i) {
    std::vector<uint8_t> corrupted = body;
    size_t off = static_cast<size_t>(SBI_HDR_LEN) + i * block_stride;
    corrupted[off + 2] ^= 0xFF;  // flip first payload byte of block i

    for (int k = 0; k < blocks_per_body; ++k) {
      size_t koff = static_cast<size_t>(SBI_HDR_LEN) + k * block_stride;
      uint16_t crc_field = static_cast<uint16_t>(corrupted[koff]) |
                            (static_cast<uint16_t>(corrupted[koff + 1]) << 8);
      uint16_t recompute = crc16_ccitt(&corrupted[koff + 2], block_payload);
      bool matches = (crc_field == recompute);
      if (k == i) CHECK(!matches);
      else CHECK(matches);
    }
  }
}

TEST(sbi_wrong_length_add_returns_empty) {
  SbiPacker packer(8, 2, 0);
  std::vector<uint8_t> wrong(5, 0xAB);
  auto out = packer.add(wrong.data(), wrong.size());
  CHECK(out.empty());
}

MTEST_MAIN
