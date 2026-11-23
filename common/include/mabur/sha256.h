#pragma once
// SHA-256 over the compression function already vendored for MAVLink signing.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace mabur {

using Sha256 = std::array<uint8_t, 32>;

class Sha256Ctx {
 public:
  Sha256Ctx();
  ~Sha256Ctx();
  Sha256Ctx(const Sha256Ctx&) = delete;
  Sha256Ctx& operator=(const Sha256Ctx&) = delete;
  void update(const void* p, size_t n);
  Sha256 final();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// nullopt when the file cannot be opened or read.
std::optional<Sha256> sha256_file(const std::string& path);
std::string sha256_hex(const Sha256& h);
// Exactly 64 hex characters, either case.
std::optional<Sha256> sha256_from_hex(std::string_view s);

}  // namespace mabur
