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
  CHECK(ver == SBI_VER);
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

TEST(sbi_ver2_header_and_patch) {
  mabur::SbiPacker p(8, 2, 0);
  std::vector<std::vector<uint8_t>> bodies;
  const uint8_t env[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  for (int i = 0; i < 2; ++i)
    for (auto& b : p.add(env, sizeof(env))) bodies.push_back(std::move(b));
  CHECK(bodies.size() == 1);
  auto& b = bodies[0];
  CHECK(mabur::SBI_HDR_LEN == 13);
  CHECK(b.size() == static_cast<size_t>(mabur::SBI_HDR_LEN + 2 * (2 + 8)));
  CHECK(b[2] == mabur::SBI_VER && mabur::SBI_VER == 2);
  CHECK(b[7] == 0 && b[8] == 0);          // q_ms placeholder zeroed
  CHECK(b[9] == 0 && b[10] == 0);         // enc_us placeholder zeroed
  CHECK(b[11] == 0 && b[12] == 0);        // air_ms placeholder zeroed (ver 2)
  mabur::sbi_set_q_ms(b.data(), b.size(), 0x1234);
  mabur::sbi_set_enc_us(b.data(), b.size(), 0xBEEF);
  mabur::sbi_set_air_ms(b.data(), b.size(), 0x0A0B);
  auto r = mabur::sbi_unpack(b.data(), b.size(), 8);
  CHECK(r.header_ok);
  CHECK(r.q_ms == 0x1234);
  CHECK(r.enc_us == 0xBEEF);
  CHECK(r.air_ms == 0x0A0B);
  CHECK(static_cast<int>(r.survivors.size()) == 2);  // patch broke no CRC
  CHECK(mabur::sbi_peek_stream_id(b.data(), b.size()) == 0);
}

TEST(sbi_stale_versions_rejected) {
  mabur::SbiPacker p(8, 1, 3);
  const uint8_t env[8] = {9, 9, 9, 9, 9, 9, 9, 9};
  auto bodies = p.add(env, sizeof(env));
  CHECK(bodies.size() == 1);
  for (uint8_t stale : {uint8_t{0}, uint8_t{1}}) {
    auto b = bodies[0];
    b[2] = stale;  // forge a ver-0 (7-byte) / ver-1 (11-byte) header
    CHECK(!mabur::sbi_unpack(b.data(), b.size(), 8).header_ok);
    CHECK(mabur::sbi_peek_stream_id(b.data(), b.size()) == -1);
  }
}

TEST(sbi_q_ms_saturates) {
  mabur::SbiPacker p(8, 1, 0);
  const uint8_t env[8] = {0};
  auto b = p.add(env, sizeof(env))[0];
  mabur::sbi_set_q_ms(b.data(), b.size(), 65535);
  CHECK(mabur::sbi_unpack(b.data(), b.size(), 8).q_ms == 65535);
}

MTEST_MAIN
