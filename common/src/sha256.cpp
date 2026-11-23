#include "mabur/sha256.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#define MAVLINK_HELPER static inline
#include "mavlink/mavlink_sha256.h"

namespace mabur {

struct Sha256Ctx::Impl {
  mavlink_sha256_ctx c;
};

Sha256Ctx::Sha256Ctx() : impl_(new Impl) { mavlink_sha256_init(&impl_->c); }
Sha256Ctx::~Sha256Ctx() = default;

void Sha256Ctx::update(const void* p, size_t n) {
  mavlink_sha256_update(&impl_->c, p, static_cast<uint32_t>(n));
}

Sha256 Sha256Ctx::final() {
  mavlink_sha256_ctx* m = &impl_->c;
  // Same padding as mavlink_sha256_final_48, then all eight words out.
  unsigned char zeros[72];
  unsigned offset = (m->sz[0] / 8) % 64;
  unsigned dstart = (120 - offset - 1) % 64 + 1;
  std::memset(zeros, 0, sizeof(zeros));
  zeros[0] = 0x80;
  for (int i = 0; i < 4; i++) {
    zeros[dstart + 7 - i] = (m->sz[0] >> (8 * i)) & 0xff;
    zeros[dstart + 3 - i] = (m->sz[1] >> (8 * i)) & 0xff;
  }
  mavlink_sha256_update(m, zeros, dstart + 8);
  Sha256 out;
  for (int w = 0; w < 8; w++)
    for (int b = 0; b < 4; b++) out[w * 4 + b] = (m->counter[w] >> (24 - 8 * b)) & 0xff;
  return out;
}

std::optional<Sha256> sha256_file(const std::string& path) {
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return std::nullopt;
  Sha256Ctx ctx;
  static thread_local unsigned char buf[64 * 1024];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) ctx.update(buf, n);
  const bool err = std::ferror(f);
  std::fclose(f);
  if (err) return std::nullopt;
  return ctx.final();
}

std::string sha256_hex(const Sha256& h) {
  static const char d[] = "0123456789abcdef";
  std::string s;
  for (uint8_t b : h) {
    s += d[b >> 4];
    s += d[b & 15];
  }
  return s;
}

std::optional<Sha256> sha256_from_hex(std::string_view s) {
  if (s.size() != 64) return std::nullopt;
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  Sha256 out;
  for (size_t i = 0; i < 32; i++) {
    int hi = nib(s[2 * i]), lo = nib(s[2 * i + 1]);
    if (hi < 0 || lo < 0) return std::nullopt;
    out[i] = static_cast<uint8_t>(hi << 4 | lo);
  }
  return out;
}

}  // namespace mabur
