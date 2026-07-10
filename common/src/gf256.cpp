#include "mabur/gf256.h"

#include <array>
#include <map>
#include <mutex>
#include <stdexcept>

namespace mabur::gf {
namespace {

constexpr int kGfPoly = 0x11D;

struct Tables {
  std::array<uint8_t, 512> exp{};
  std::array<uint8_t, 256> log{};
  Tables() {
    int x = 1;
    for (int i = 0; i < 255; ++i) {
      exp[static_cast<size_t>(i)] = static_cast<uint8_t>(x);
      log[static_cast<size_t>(x)] = static_cast<uint8_t>(i);
      x <<= 1;
      if (x & 0x100) x ^= kGfPoly;
    }
    for (int i = 255; i < 512; ++i) exp[static_cast<size_t>(i)] = exp[static_cast<size_t>(i - 255)];
  }
};

const Tables& tables() {
  static const Tables t;
  return t;
}

uint8_t gf_inv(uint8_t a) {
  const Tables& t = tables();
  // a == 0 has no inverse; callers only invoke this on nonzero pivots.
  return t.exp[static_cast<size_t>(255 - t.log[a])];
}

uint8_t gf_pow(uint8_t a, int e) {
  if (e == 0) return 1;
  if (a == 0) return 0;
  const Tables& t = tables();
  int idx = (static_cast<int>(t.log[a]) * e) % 255;
  return t.exp[static_cast<size_t>(idx)];
}

using Matrix = std::vector<std::vector<uint8_t>>;

Matrix mat_mul(const Matrix& a, const Matrix& b) {
  const Tables& t = tables();
  size_t n = a.size();
  size_t m = b.size();
  size_t p = b.empty() ? 0 : b[0].size();
  Matrix out(n, std::vector<uint8_t>(p, 0));
  for (size_t i = 0; i < n; ++i) {
    for (size_t k = 0; k < m; ++k) {
      uint8_t av = a[i][k];
      if (av == 0) continue;
      uint8_t la = t.log[av];
      const auto& bk = b[k];
      auto& oi = out[i];
      for (size_t j = 0; j < p; ++j) {
        uint8_t bv = bk[j];
        if (bv) oi[j] = static_cast<uint8_t>(oi[j] ^ t.exp[static_cast<size_t>(la) + t.log[bv]]);
      }
    }
  }
  return out;
}

Matrix mat_inv(const Matrix& m) {
  const Tables& t = tables();
  size_t n = m.size();
  Matrix a(n, std::vector<uint8_t>(2 * n, 0));
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) a[i][j] = m[i][j];
    a[i][n + i] = 1;
  }
  for (size_t col = 0; col < n; ++col) {
    size_t piv = n;
    for (size_t r = col; r < n; ++r) {
      if (a[r][col] != 0) {
        piv = r;
        break;
      }
    }
    if (piv == n) throw std::runtime_error("gf256: singular matrix (should not happen for K rows)");
    if (piv != col) std::swap(a[col], a[piv]);
    uint8_t inv_p = gf_inv(a[col][col]);
    for (auto& v : a[col]) v = mul(v, inv_p);
    for (size_t r = 0; r < n; ++r) {
      if (r == col) continue;
      uint8_t f = a[r][col];
      if (!f) continue;
      uint8_t lf = t.log[f];
      auto& ac = a[col];
      auto& ar = a[r];
      for (size_t j = 0; j < 2 * n; ++j) {
        uint8_t v = ac[j];
        if (v) ar[j] = static_cast<uint8_t>(ar[j] ^ t.exp[static_cast<size_t>(lf) + t.log[v]]);
      }
    }
  }
  Matrix inv(n, std::vector<uint8_t>(n));
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < n; ++j) inv[i][j] = a[i][n + j];
  return inv;
}

std::mutex& matrix_cache_mutex() {
  static std::mutex m;
  return m;
}

std::map<std::pair<int, int>, Matrix>& matrix_cache() {
  static std::map<std::pair<int, int>, Matrix> cache;
  return cache;
}

}  // namespace

uint8_t mul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) return 0;
  const Tables& t = tables();
  return t.exp[static_cast<size_t>(t.log[a]) + t.log[b]];
}

void lincomb(uint8_t* acc, const uint8_t* sym, uint8_t coeff, size_t len) {
  if (coeff == 0) return;
  const Tables& t = tables();
  uint8_t lc = t.log[coeff];
  for (size_t i = 0; i < len; ++i) {
    uint8_t s = sym[i];
    if (s) acc[i] = static_cast<uint8_t>(acc[i] ^ t.exp[static_cast<size_t>(lc) + t.log[s]]);
  }
}

const std::vector<std::vector<uint8_t>>& encoding_matrix(int k, int n) {
  std::lock_guard<std::mutex> lock(matrix_cache_mutex());
  auto key = std::make_pair(k, n);
  auto it = matrix_cache().find(key);
  if (it != matrix_cache().end()) return it->second;
  if (n > 256) throw std::runtime_error("RS needs K+repair = N <= 256");

  Matrix v(static_cast<size_t>(n), std::vector<uint8_t>(static_cast<size_t>(k)));
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < k; ++j) v[static_cast<size_t>(i)][static_cast<size_t>(j)] = gf_pow(static_cast<uint8_t>(i), j);

  Matrix top(v.begin(), v.begin() + k);
  Matrix top_inv = mat_inv(top);
  Matrix a = mat_mul(v, top_inv);

  auto [ins_it, _] = matrix_cache().emplace(key, std::move(a));
  return ins_it->second;
}

}  // namespace mabur::gf
