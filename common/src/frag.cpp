#include "mabur/frag.h"

#include <algorithm>

namespace mabur {

std::vector<std::vector<uint8_t>> Fragmenter::fragment(const uint8_t* pkt, size_t len, int usable) {
  size_t span = len > 0 ? len : 1;  // Python: range(0, max(len,1), usable)
  size_t n_chunks = (span + static_cast<size_t>(usable) - 1) / static_cast<size_t>(usable);

  uint16_t seq = seq_;
  seq_ = static_cast<uint16_t>(seq_ + 1);  // wraps at u16 automatically

  std::vector<std::vector<uint8_t>> out;
  out.reserve(n_chunks);
  for (size_t i = 0; i < n_chunks; ++i) {
    size_t off = i * static_cast<size_t>(usable);
    size_t chunk_len = off < len ? std::min(static_cast<size_t>(usable), len - off) : 0;

    std::vector<uint8_t> frag;
    frag.reserve(4 + chunk_len);
    frag.push_back(static_cast<uint8_t>(seq & 0xFF));
    frag.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
    frag.push_back(static_cast<uint8_t>(i));
    frag.push_back(static_cast<uint8_t>(n_chunks));
    if (chunk_len > 0) frag.insert(frag.end(), pkt + off, pkt + off + chunk_len);
    out.push_back(std::move(frag));
  }
  return out;
}

}  // namespace mabur
