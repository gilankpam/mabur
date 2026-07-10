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

}  // namespace mabur
