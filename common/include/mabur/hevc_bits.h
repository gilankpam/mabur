#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mabur::hevc {

// H.265 7.4.2: emulation_prevention_three_byte. unescape() turns a NAL
// payload into its RBSP; escape() the reverse (a 0x03 before any byte <= 3
// that follows two zero bytes).
std::vector<uint8_t> unescape(const uint8_t* p, size_t n);
std::vector<uint8_t> escape(const uint8_t* p, size_t n);

// Smallest c with (1 << c) >= x (Ceil(Log2(x)) in the spec); 0 for x <= 1.
uint32_t ceil_log2(uint32_t x);

// MSB-first reader over an RBSP; bad() latches on overrun or an Exp-Golomb
// code longer than 31 leading zeros.
class BitReader {
 public:
  BitReader(const uint8_t* d, size_t n) : d_(d), n_(n) {}
  uint32_t u(int bits);
  uint32_t ue();
  int32_t se();
  size_t pos() const { return pos_; }
  bool bad() const { return bad_; }

 private:
  const uint8_t* d_;
  size_t n_;
  size_t pos_ = 0;
  bool bad_ = false;
};

// MSB-first writer. copy() appends src bits [from_bit, to_bit).
class BitWriter {
 public:
  void put(int bit);
  void u(uint32_t v, int bits);
  void ue(uint32_t v);
  void copy(const uint8_t* src, size_t from_bit, size_t to_bit);
  void align_zero();
  size_t bits() const { return bits_; }
  const std::vector<uint8_t>& bytes() const { return buf_; }

 private:
  std::vector<uint8_t> buf_;
  size_t bits_ = 0;
};

}  // namespace mabur::hevc
