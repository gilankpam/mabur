#include "mabur/hevc_bits.h"

namespace mabur::hevc {

std::vector<uint8_t> unescape(const uint8_t* p, size_t n) {
  std::vector<uint8_t> o;
  o.reserve(n);
  int zeros = 0;
  for (size_t i = 0; i < n; ++i) {
    if (zeros >= 2 && p[i] == 0x03) { zeros = 0; continue; }
    o.push_back(p[i]);
    zeros = p[i] == 0 ? zeros + 1 : 0;
  }
  return o;
}

std::vector<uint8_t> escape(const uint8_t* p, size_t n) {
  std::vector<uint8_t> o;
  o.reserve(n + n / 64 + 4);
  int zeros = 0;
  for (size_t i = 0; i < n; ++i) {
    if (zeros >= 2 && p[i] <= 0x03) { o.push_back(0x03); zeros = 0; }
    o.push_back(p[i]);
    zeros = p[i] == 0 ? zeros + 1 : 0;
  }
  return o;
}

uint32_t ceil_log2(uint32_t x) {
  uint32_t c = 0;
  while ((1ull << c) < x) ++c;
  return c;
}

uint32_t BitReader::u(int bits) {
  uint32_t v = 0;
  for (int i = 0; i < bits; ++i) {
    const size_t byte = pos_ >> 3;
    if (byte >= n_) { bad_ = true; return 0; }
    v = (v << 1) | ((d_[byte] >> (7 - (pos_ & 7))) & 1u);
    ++pos_;
  }
  return v;
}

uint32_t BitReader::ue() {
  int lz = 0;
  while (u(1) == 0) {
    if (bad_ || ++lz > 31) { bad_ = true; return 0; }
  }
  return ((1u << lz) - 1) + u(lz);
}

int32_t BitReader::se() {
  const uint32_t k = ue();
  return (k & 1) ? static_cast<int32_t>((k + 1) / 2) : -static_cast<int32_t>(k / 2);
}

void BitWriter::put(int bit) {
  if ((bits_ & 7) == 0) buf_.push_back(0);
  if (bit) buf_.back() |= static_cast<uint8_t>(0x80u >> (bits_ & 7));
  ++bits_;
}

void BitWriter::u(uint32_t v, int bits) {
  for (int i = bits - 1; i >= 0; --i) put((v >> i) & 1);
}

void BitWriter::ue(uint32_t v) {
  const uint64_t x = static_cast<uint64_t>(v) + 1;
  int len = 0;
  while ((x >> len) > 1) ++len;  // len = floor(log2(x))
  for (int i = 0; i < len; ++i) put(0);
  for (int i = len; i >= 0; --i) put(static_cast<int>((x >> i) & 1));
}

void BitWriter::copy(const uint8_t* src, size_t from_bit, size_t to_bit) {
  for (size_t b = from_bit; b < to_bit; ++b) put((src[b >> 3] >> (7 - (b & 7))) & 1);
}

void BitWriter::align_zero() {
  while (bits_ & 7) put(0);
}

}  // namespace mabur::hevc
