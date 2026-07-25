#include "mabur/frag.h"

#include <algorithm>

namespace mabur {

std::vector<std::vector<uint8_t>> Fragmenter::fragment(const uint8_t* pkt, size_t len, int usable) {
  size_t span = len > 0 ? len : 1;  // a zero-length unit still emits one chunk
  size_t n_chunks = (span + static_cast<size_t>(usable) - 1) / static_cast<size_t>(usable);

  uint16_t seq = seq_;
  seq_ = static_cast<uint16_t>(seq_ + 1);  // wraps at u16 automatically

  std::vector<std::vector<uint8_t>> out;
  out.reserve(n_chunks);
  for (size_t i = 0; i < n_chunks; ++i) {
    size_t off = i * static_cast<size_t>(usable);
    size_t chunk_len = off < len ? std::min(static_cast<size_t>(usable), len - off) : 0;

    // {seq_lo, seq_hi, idx_lo, idx_hi, count_lo, count_hi}
    std::vector<uint8_t> frag;
    frag.reserve(kHdrLen + chunk_len);
    const uint16_t idx16 = static_cast<uint16_t>(i);
    const uint16_t count16 = static_cast<uint16_t>(n_chunks);
    frag.push_back(static_cast<uint8_t>(seq & 0xFF));
    frag.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
    frag.push_back(static_cast<uint8_t>(idx16 & 0xFF));
    frag.push_back(static_cast<uint8_t>((idx16 >> 8) & 0xFF));
    frag.push_back(static_cast<uint8_t>(count16 & 0xFF));
    frag.push_back(static_cast<uint8_t>((count16 >> 8) & 0xFF));
    if (chunk_len > 0) frag.insert(frag.end(), pkt + off, pkt + off + chunk_len);
    out.push_back(std::move(frag));
  }
  return out;
}

}  // namespace mabur
