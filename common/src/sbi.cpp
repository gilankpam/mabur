#include "mabur/sbi.h"
#include "mabur/crc16.h"

namespace mabur {

SbiPacker::SbiPacker(int block_payload, int blocks_per_body, uint8_t stream_id)
    : block_payload_(block_payload),
      blocks_per_body_(blocks_per_body),
      stream_id_(stream_id) {}

int SbiPacker::block_stride() const { return 2 + block_payload_; }

std::vector<std::vector<uint8_t>> SbiPacker::add(const uint8_t* env, size_t len) {
  std::vector<std::vector<uint8_t>> out;
  if (static_cast<int>(len) != block_payload_) return out;

  pending_.emplace_back(env, env + len);
  while (static_cast<int>(pending_.size()) >= blocks_per_body_) {
    std::vector<std::vector<uint8_t>> batch(
        pending_.begin(), pending_.begin() + blocks_per_body_);
    pending_.erase(pending_.begin(), pending_.begin() + blocks_per_body_);
    out.push_back(build_body(batch));
  }
  return out;
}

std::vector<std::vector<uint8_t>> SbiPacker::flush() {
  std::vector<std::vector<uint8_t>> out;
  if (pending_.empty()) return out;
  out.push_back(build_body(pending_));
  pending_.clear();
  return out;
}

std::vector<uint8_t> SbiPacker::build_body(
    const std::vector<std::vector<uint8_t>>& batch) {
  std::vector<uint8_t> out;
  out.reserve(SBI_HDR_LEN + batch.size() * block_stride());

  // Header: <u16 MAGIC LE, u8 ver, u8 stream_id, u16 block_payload LE, u8 n_blocks, u16 q_ms LE, u16 enc_us LE, u16 air_ms LE>
  out.push_back(static_cast<uint8_t>(SBI_MAGIC & 0xFF));
  out.push_back(static_cast<uint8_t>((SBI_MAGIC >> 8) & 0xFF));
  out.push_back(SBI_VER);
  out.push_back(stream_id_);
  out.push_back(static_cast<uint8_t>(block_payload_ & 0xFF));
  out.push_back(static_cast<uint8_t>((block_payload_ >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(batch.size()));
  // q_ms placeholder (bytes 7-8)
  out.push_back(0);
  out.push_back(0);
  // enc_us placeholder (bytes 9-10)
  out.push_back(0);
  out.push_back(0);
  // air_ms placeholder (bytes 11-12), patched by the hot thread's sink
  out.push_back(0);
  out.push_back(0);

  for (auto& env : batch) {
    uint16_t crc = crc16_ccitt(env.data(), env.size());
    out.push_back(static_cast<uint8_t>(crc & 0xFF));
    out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    out.insert(out.end(), env.begin(), env.end());
  }
  return out;
}

namespace {
uint16_t sbi_rd_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
}  // namespace

SbiUnpackResult sbi_unpack(const uint8_t* body, size_t len, int block_payload) {
  SbiUnpackResult r;
  // Memory-safety guard: negative block_payload would wrap stride to a huge value,
  // causing unbounded reads in crc16_ccitt(). Python is safe via slice semantics.
  if (block_payload <= 0) return r;
  const size_t stride = 2 + static_cast<size_t>(block_payload);
  if (len >= static_cast<size_t>(SBI_HDR_LEN)) {
    const uint16_t magic = sbi_rd_u16(body);
    const uint8_t ver = body[2];
    r.stream_id = body[3];
    const uint16_t hdr_bp = sbi_rd_u16(body + 4);
    r.header_ok = magic == SBI_MAGIC && ver == SBI_VER && hdr_bp == block_payload;
    if (r.header_ok) {
      r.q_ms = sbi_rd_u16(body + SBI_Q_MS_OFF);
      r.enc_us = sbi_rd_u16(body + SBI_ENC_US_OFF);
      r.air_ms = sbi_rd_u16(body + SBI_AIR_MS_OFF);
    }
  } else {
    return r;  // Python: empty region -> zero blocks, header_ok false
  }
  const uint8_t* region = body + SBI_HDR_LEN;
  const size_t region_len = len - static_cast<size_t>(SBI_HDR_LEN);
  r.n_blocks = static_cast<int>(region_len / stride);
  for (int i = 0; i < r.n_blocks; ++i) {
    const uint8_t* off = region + static_cast<size_t>(i) * stride;
    const uint16_t crc_field = sbi_rd_u16(off);
    if (crc16_ccitt(off + 2, static_cast<size_t>(block_payload)) == crc_field)
      r.survivors.emplace_back(off + 2, off + 2 + block_payload);
    else
      ++r.n_failed;
  }
  return r;
}

int sbi_peek_stream_id(const uint8_t* body, size_t len) {
  if (len < static_cast<size_t>(SBI_HDR_LEN)) return -1;
  if (sbi_rd_u16(body) != SBI_MAGIC || body[2] != SBI_VER) return -1;
  return body[3];
}

void sbi_set_q_ms(uint8_t* body, size_t len, uint16_t ms) {
  if (len < static_cast<size_t>(SBI_HDR_LEN)) return;
  body[SBI_Q_MS_OFF] = static_cast<uint8_t>(ms & 0xFF);
  body[SBI_Q_MS_OFF + 1] = static_cast<uint8_t>(ms >> 8);
}

void sbi_set_enc_us(uint8_t* body, size_t len, uint16_t us) {
  if (len < static_cast<size_t>(SBI_HDR_LEN)) return;
  body[SBI_ENC_US_OFF] = static_cast<uint8_t>(us & 0xFF);
  body[SBI_ENC_US_OFF + 1] = static_cast<uint8_t>(us >> 8);
}

void sbi_set_air_ms(uint8_t* body, size_t len, uint16_t ms) {
  if (len < static_cast<size_t>(SBI_HDR_LEN)) return;
  body[SBI_AIR_MS_OFF] = static_cast<uint8_t>(ms & 0xFF);
  body[SBI_AIR_MS_OFF + 1] = static_cast<uint8_t>(ms >> 8);
}

}  // namespace mabur
