#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur::gf {
// GF(2^8) with primitive polynomial 0x11D (matches zfec/wfb-ng and
// devourer's stream_fec_rs.py).
uint8_t mul(uint8_t a, uint8_t b);

// Multiplicative inverse; inv(0) = 0 (no caller passes 0 — repair
// coefficients are drawn from [1,255]).
uint8_t inv(uint8_t a);

// acc[i] ^= coeff * sym[i] for i in [0, len) — GF(2^8) linear combination
// accumulation, mirrors the inner loop of Python's _lincomb.
void lincomb(uint8_t* acc, const uint8_t* sym, uint8_t coeff, size_t len);

// N×K systematic MDS encoding matrix: top K rows = identity, any K rows
// invertible. Built once per (k, n) and cached (thread-safe).
const std::vector<std::vector<uint8_t>>& encoding_matrix(int k, int n);

using Matrix = std::vector<std::vector<uint8_t>>;

// Gauss-Jordan inverse over GF(2^8) — mirror of stream_fec_rs.py's _mat_inv.
// Throws std::runtime_error on a singular matrix (cannot happen for any K
// rows of encoding_matrix; the decoder relies on that MDS property).
Matrix mat_inv(const Matrix& m);

// Compile-time-selected lincomb backend, for startup logs: "neon-vqtbl"
// (aarch64 ASIMD), "neon-vtbl2" (ARMv7 NEON), or "scalar". Lets a bench log
// prove the SIMD path is actually compiled in (a build-flag regression
// otherwise shows up only as mysterious CPU load).
const char* backend();
}  // namespace mabur::gf
