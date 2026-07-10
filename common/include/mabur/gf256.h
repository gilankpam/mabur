#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur::gf {
// GF(2^8) with primitive polynomial 0x11D (matches zfec/wfb-ng and
// devourer's stream_fec_rs.py).
uint8_t mul(uint8_t a, uint8_t b);

// acc[i] ^= coeff * sym[i] for i in [0, len) — GF(2^8) linear combination
// accumulation, mirrors the inner loop of Python's _lincomb.
void lincomb(uint8_t* acc, const uint8_t* sym, uint8_t coeff, size_t len);

// N×K systematic MDS encoding matrix: top K rows = identity, any K rows
// invertible. Built once per (k, n) and cached (thread-safe).
const std::vector<std::vector<uint8_t>>& encoding_matrix(int k, int n);
}  // namespace mabur::gf
