#include "mabur/rs_encoder.h"

#include <cstring>

#include "mabur/gf256.h"

namespace mabur {
namespace {
constexpr uint16_t kRsMagic = 0xF540;
constexpr size_t kRsHeaderLen = 11;
constexpr size_t kPacketLenPrefix = 2;

void pack_header(std::vector<uint8_t>& out, uint8_t k, uint8_t kreal, uint16_t symbol_size,
                  uint16_t block_id, uint8_t esi, uint8_t n) {
  // <HBBBHHBB> little-endian: MAGIC, flags, k, kreal, symbol_size, block_id, esi, n
  out.push_back(static_cast<uint8_t>(kRsMagic & 0xFF));
  out.push_back(static_cast<uint8_t>((kRsMagic >> 8) & 0xFF));
  out.push_back(0);  // flags
  out.push_back(k);
  out.push_back(kreal);
  out.push_back(static_cast<uint8_t>(symbol_size & 0xFF));
  out.push_back(static_cast<uint8_t>((symbol_size >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(block_id & 0xFF));
  out.push_back(static_cast<uint8_t>((block_id >> 8) & 0xFF));
  out.push_back(esi);
  out.push_back(n);
}
}  // namespace

RsEncoder::RsEncoder(const RsConfig& cfg) : cfg_(cfg), n_(cfg.n()) {}

void RsEncoder::maybe_apply_pending_overhead() {
  if (has_pending_overhead_ && pending_symbols_.empty()) {
    cfg_.overhead = pending_overhead_;
    n_ = cfg_.n();
    has_pending_overhead_ = false;
  }
}

void RsEncoder::set_overhead(double overhead) {
  pending_overhead_ = overhead;
  has_pending_overhead_ = true;
  maybe_apply_pending_overhead();
}

bool RsEncoder::has_pending() const { return !current_symbol_.empty() || !pending_symbols_.empty(); }

void RsEncoder::append_to_current(const uint8_t* data, size_t len) {
  uint16_t ln = static_cast<uint16_t>(len);
  current_symbol_.push_back(static_cast<uint8_t>(ln & 0xFF));
  current_symbol_.push_back(static_cast<uint8_t>((ln >> 8) & 0xFF));
  current_symbol_.insert(current_symbol_.end(), data, data + len);
}

void RsEncoder::seal_current_symbol() {
  if (current_symbol_.empty()) return;
  size_t pad = static_cast<size_t>(cfg_.symbol_size) - current_symbol_.size();
  if (pad) current_symbol_.insert(current_symbol_.end(), pad, 0);
  pending_symbols_.push_back(std::move(current_symbol_));
  current_symbol_.clear();
}

std::vector<std::vector<uint8_t>> RsEncoder::maybe_encode_full_block() {
  if (static_cast<int>(pending_symbols_.size()) >= cfg_.k) return encode_block(cfg_.k);
  return {};
}

std::vector<std::vector<uint8_t>> RsEncoder::add_packet(const uint8_t* data, size_t len) {
  if (static_cast<int>(len) > cfg_.max_packet_size()) {
    ++oversize_drops_;
    return {};
  }
  size_t needed = kPacketLenPrefix + len;
  size_t remaining = static_cast<size_t>(cfg_.symbol_size) - current_symbol_.size();
  if (needed > remaining) {
    seal_current_symbol();
    auto ready = maybe_encode_full_block();
    if (!ready.empty()) {
      append_to_current(data, len);
      return ready;
    }
  }
  append_to_current(data, len);
  return {};
}

std::vector<std::vector<uint8_t>> RsEncoder::flush() {
  seal_current_symbol();
  if (pending_symbols_.empty()) return {};
  int kreal = static_cast<int>(pending_symbols_.size());
  while (static_cast<int>(pending_symbols_.size()) < cfg_.k) {
    pending_symbols_.emplace_back(static_cast<size_t>(cfg_.symbol_size), 0);
  }
  return encode_block(kreal);
}

std::vector<std::vector<uint8_t>> RsEncoder::encode_block(int kreal) {
  std::vector<std::vector<uint8_t>> src(pending_symbols_.begin(), pending_symbols_.begin() + cfg_.k);
  pending_symbols_.erase(pending_symbols_.begin(), pending_symbols_.begin() + cfg_.k);

  int k = cfg_.k;
  int n = n_;
  int ss = cfg_.symbol_size;
  const auto& A = gf::encoding_matrix(k, n);
  uint16_t bid = block_id_;
  block_id_ = static_cast<uint16_t>(block_id_ + 1);

  std::vector<std::vector<uint8_t>> out;
  out.reserve(static_cast<size_t>(n));
  for (int esi = 0; esi < n; ++esi) {
    std::vector<uint8_t> env;
    env.reserve(kRsHeaderLen + static_cast<size_t>(ss));
    pack_header(env, static_cast<uint8_t>(k), static_cast<uint8_t>(kreal), static_cast<uint16_t>(ss), bid,
                static_cast<uint8_t>(esi), static_cast<uint8_t>(n));
    if (esi < k) {
      env.insert(env.end(), src[static_cast<size_t>(esi)].begin(), src[static_cast<size_t>(esi)].end());
    } else {
      size_t off = env.size();
      env.insert(env.end(), static_cast<size_t>(ss), 0);
      const auto& row = A[static_cast<size_t>(esi)];
      for (int j = 0; j < k; ++j) {
        gf::lincomb(env.data() + off, src[static_cast<size_t>(j)].data(), row[static_cast<size_t>(j)],
                    static_cast<size_t>(ss));
      }
    }
    out.push_back(std::move(env));
  }

  // Block boundary reached (pending_symbols_ has k fewer symbols than
  // before; if it's now empty, a pending overhead change takes effect).
  maybe_apply_pending_overhead();
  return out;
}

}  // namespace mabur
