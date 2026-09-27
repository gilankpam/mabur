// The LP AU record format of maburgs --dry-run --out-aus (gs/src/main.cpp
// AuFileOut): u32 len | u8 sid | u8 flags(|0x04 complete) | u32 pts_us |
// Annex-B bytes. Host-endian, like maburgs. webgs replay writes it so the
// web_au_parity gate can cmp the two byte-for-byte.
#pragma once
#include <cstdint>
#include <cstdio>

#include "web_gs.h"

namespace webgs {
class AuFileWriter {
 public:
  AuFileWriter() = default;
  AuFileWriter(const AuFileWriter&) = delete;
  AuFileWriter& operator=(const AuFileWriter&) = delete;
  ~AuFileWriter() { if (f_) std::fclose(f_); }
  bool open(const char* path) { f_ = std::fopen(path, "wb"); return f_ != nullptr; }
  void write(const Au& a) {
    if (!f_) return;
    const uint32_t len = static_cast<uint32_t>(a.data.size());
    const uint8_t flags = static_cast<uint8_t>(a.flags | (a.complete ? 0x04 : 0));
    std::fwrite(&len, 4, 1, f_);
    std::fwrite(&a.sid, 1, 1, f_);
    std::fwrite(&flags, 1, 1, f_);
    std::fwrite(&a.pts_us, 4, 1, f_);
    if (len) std::fwrite(a.data.data(), 1, len, f_);
    ++written_;
  }
  uint64_t written() const { return written_; }

 private:
  FILE* f_ = nullptr;
  uint64_t written_ = 0;
};
}  // namespace webgs
