#include "mtest.h"
#include "mabur/gf256.h"
#include <string>
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
  CHECK(s == "neon-vqtbl" || s == "neon-vtbl2" || s == "scalar");
}

TEST(inv_roundtrip_all_nonzero) {
  for (int a = 1; a < 256; ++a) {
    const uint8_t ia = gf::inv(static_cast<uint8_t>(a));
    CHECK(ia != 0);
    CHECK(gf::mul(static_cast<uint8_t>(a), ia) == 1);
  }
  CHECK(gf::inv(0) == 0);
}
MTEST_MAIN
