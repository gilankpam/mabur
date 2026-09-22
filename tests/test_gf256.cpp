#include "mtest.h"
#include "mabur/gf256.h"
#include <random>
#include <string>
#include <vector>
using namespace mabur;

// Multiply two matrices with the public mul() (mat_mul is internal).
static gf::Matrix mm(const gf::Matrix& a, const gf::Matrix& b) {
  size_t n = a.size(), m = b.size(), p = b[0].size();
  gf::Matrix out(n, std::vector<uint8_t>(p, 0));
  for (size_t i = 0; i < n; ++i)
    for (size_t k = 0; k < m; ++k)
      for (size_t j = 0; j < p; ++j)
        out[i][j] = static_cast<uint8_t>(out[i][j] ^ gf::mul(a[i][k], b[k][j]));
  return out;
}

TEST(mat_inv_identity) {
  gf::Matrix I = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  CHECK(gf::mat_inv(I) == I);
}

TEST(mat_inv_times_original_is_identity) {
  // Any K rows of the systematic encoding matrix are invertible — take a
  // mixed systematic/parity subset, exactly what the decoder inverts.
  const auto& A = gf::encoding_matrix(4, 8);
  gf::Matrix sub = {A[1], A[3], A[5], A[7]};
  gf::Matrix inv = gf::mat_inv(sub);
  gf::Matrix prod = mm(inv, sub);
  for (size_t i = 0; i < 4; ++i)
    for (size_t j = 0; j < 4; ++j)
      CHECK(prod[i][j] == (i == j ? 1 : 0));
}

TEST(gf256_backend_reports_known_value) {
  const char* b = gf::backend();
  REQUIRE(b != nullptr);
  std::string s = b;
  CHECK(s == "neon-vqtbl" || s == "neon-vtbl2-q16" || s == "scalar");
}

TEST(inv_roundtrip_all_nonzero) {
  for (int a = 1; a < 256; ++a) {
    const uint8_t ia = gf::inv(static_cast<uint8_t>(a));
    CHECK(ia != 0);
    CHECK(gf::mul(static_cast<uint8_t>(a), ia) == 1);
  }
  CHECK(gf::inv(0) == 0);
}
// lincomb_rows(out, rows, coeffs, n, len) must equal n accumulated lincomb
// calls over a zeroed out — same bytes for any row count (incl. 0), any
// length with a non-multiple-of-16 tail, and coefficients including 0.
// Would fail if the rows kernel skipped a row, mishandled the tail, or
// accumulated instead of overwriting.
TEST(lincomb_rows_matches_accumulated_lincomb) {
  std::mt19937 rng(7);
  for (int it = 0; it < 300; ++it) {
    const int n = static_cast<int>(rng() % 40);
    const size_t len = 1 + rng() % 400;
    std::vector<std::vector<uint8_t>> rows(static_cast<size_t>(n), std::vector<uint8_t>(len));
    std::vector<const uint8_t*> ptrs;
    std::vector<uint8_t> coeffs;
    for (auto& r : rows) {
      for (auto& v : r) v = static_cast<uint8_t>(rng());
      ptrs.push_back(r.data());
      coeffs.push_back(static_cast<uint8_t>(it % 7 == 0 ? 0 : rng()));
    }
    std::vector<uint8_t> want(len, 0);
    for (int i = 0; i < n; ++i) gf::lincomb(want.data(), ptrs[static_cast<size_t>(i)], coeffs[static_cast<size_t>(i)], len);
    std::vector<uint8_t> got(len, 0xA5);  // non-zero: overwrite, not accumulate
    gf::lincomb_rows(got.data(), ptrs.data(), coeffs.data(), n, len);
    CHECK(got == want);
    if (got != want) break;
  }
}
MTEST_MAIN
