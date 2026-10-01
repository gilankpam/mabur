#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
namespace mabur {
// SipHash-2-4 (Aumasson & Bernstein), 128-bit key, 64-bit output. The keyed
// MAC on every GS->drone RC frame (spec 2026-10-01 link-pairing §3). Pure,
// allocation-free, identical on armv7 / aarch64 / wasm32.
uint64_t siphash24(const std::array<uint8_t, 16>& key, const uint8_t* in, size_t len);
}  // namespace mabur
