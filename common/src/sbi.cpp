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

  // Header: <u16 MAGIC LE, u8 ver=0, u8 stream_id, u16 block_payload LE, u8 n_blocks>
  out.push_back(static_cast<uint8_t>(SBI_MAGIC & 0xFF));
  out.push_back(static_cast<uint8_t>((SBI_MAGIC >> 8) & 0xFF));
  out.push_back(0);  // ver
  out.push_back(stream_id_);
  out.push_back(static_cast<uint8_t>(block_payload_ & 0xFF));
  out.push_back(static_cast<uint8_t>((block_payload_ >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(batch.size()));

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
  const size_t stride = 2 + static_cast<size_t>(block_payload);
  if (len >= static_cast<size_t>(SBI_HDR_LEN)) {
    const uint16_t magic = sbi_rd_u16(body);
    const uint8_t ver = body[2];
    r.stream_id = body[3];
    const uint16_t hdr_bp = sbi_rd_u16(body + 4);
    r.header_ok = magic == SBI_MAGIC && ver == 0 && hdr_bp == block_payload;
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
  if (sbi_rd_u16(body) != SBI_MAGIC || body[2] != 0) return -1;
  return body[3];
}

}  // namespace mabur
